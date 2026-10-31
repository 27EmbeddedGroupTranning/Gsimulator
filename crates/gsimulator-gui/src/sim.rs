//! Simulation control for the front end.
//!
//! The GUI owns one [`Simulation`] on the UI thread: stepping happens inside
//! `eframe::App::update` with a bounded instruction budget, so the window stays
//! responsive without threads, channels or locks.
//!
//! Two sources can drive the device:
//!
//! * a real ELF firmware image executed on the Cortex-M3 core, or
//! * a built-in stimulus that toggles `PA5` through `GPIOA->BSRR` the way a
//!   firmware would, so wiring can be tried out without compiling anything.
//!   It starts by default and can be switched off again, which leaves the
//!   simulation idle and releases `PA5`.

use std::cell::{Cell, RefCell};
use std::path::{Path, PathBuf};
use std::rc::Rc;

use gsim_core::{AccessWidth, Clocks, Event, MemoryBus};
use gsim_cpu::{Cpu, CpuError, ElfImage, RunError, StopReason};
use gsim_devices::{
    BusRecord, CanBus, CanBusStats, CanTransceiver, ChipState, Max7219, TransceiverMode,
    TransceiverStats,
};
use gsim_stm32f1::status::PeripheralStatus;
use gsim_stm32f1::{map, Soc, Stats};

use crate::board::PinLevels;
use crate::waveform::Expression;

/// Default half period of the built-in square wave (50 % duty).
const STIMULUS_HALF_PERIOD_NS: u64 = 120_000_000;
/// Granularity the stimulus uses when advancing simulated time.
const STIMULUS_STEP_NS: u64 = 20_000;
/// Sampling interval of a custom stimulus expression.
const STIMULUS_FUNCTION_STEP_NS: u64 = 1_000_000;
/// Instructions between two device servicing points while a firmware runs.
const SERVICE_CHUNK: u32 = 20_000;
/// High/low dwell times of a square wave; both are at least 1 ns.
fn square_durations(half_period_ns: u64, duty: f32) -> (u64, u64) {
    let period = half_period_ns.saturating_mul(2);
    let high = ((period as f64) * f64::from(duty.clamp(0.0, 1.0))).round() as u64;
    let high = high.clamp(1, period.saturating_sub(1).max(1));
    let low = period.saturating_sub(high).max(1);
    (high, low)
}
/// Maximum number of bus frames handed to the controller in one service call.
const MAX_CAN_FRAMES_PER_STEP: usize = 64;

/// What drives the pins.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Source {
    /// Nothing loaded yet.
    Idle,
    /// The configurable built-in stimulus.
    Stimulus,
    /// A firmware image running on the emulated core.
    Firmware(PathBuf),
}

/// Waveform produced by the built-in stimulus.
#[derive(Clone, Debug, PartialEq)]
pub enum StimulusWaveform {
    /// Square wave: `half_period_ns` is half of the period at 50 % duty.
    Square { half_period_ns: u64, duty: f32 },
    /// A level that never changes.
    Level(bool),
    /// Expression in `t` (simulated seconds); high while the value is > 0.5.
    Function { expression: String },
}

impl StimulusWaveform {
    pub fn label(&self) -> String {
        match self {
            StimulusWaveform::Square {
                half_period_ns,
                duty,
            } => {
                let hz = 1e9 / (2.0 * *half_period_ns as f64);
                format!("{hz:.2} Hz 方波（占空比 {:.0} %）", duty * 100.0)
            }
            StimulusWaveform::Level(true) => "恒定高电平".to_string(),
            StimulusWaveform::Level(false) => "恒定低电平".to_string(),
            StimulusWaveform::Function { .. } => "自定义函数".to_string(),
        }
    }
}

/// Everything the built-in stimulus can be configured with.
#[derive(Clone, Debug, PartialEq)]
pub struct StimulusConfig {
    /// Index into [`gsim_stm32f1::map::gpio::PORTS`] (0 = A, 1 = B, 2 = C).
    pub port: u8,
    /// Pin number inside the port.
    pub pin: u8,
    pub waveform: StimulusWaveform,
}

impl Default for StimulusConfig {
    fn default() -> Self {
        StimulusConfig {
            port: 0,
            pin: 5,
            waveform: StimulusWaveform::Square {
                half_period_ns: STIMULUS_HALF_PERIOD_NS,
                duty: 0.5,
            },
        }
    }
}

impl StimulusConfig {
    /// `PA5`-style pin name.
    pub fn pin_label(&self) -> String {
        let port = gsim_stm32f1::PORT_NAMES
            .get(usize::from(self.port))
            .copied()
            .unwrap_or("?");
        format!("P{port}{}", self.pin)
    }

    pub fn label(&self) -> String {
        format!("内置激励：{} {}", self.pin_label(), self.waveform.label())
    }

    /// Rejects pins the board does not expose and impossible timing.
    pub fn validate(&self) -> Result<(), String> {
        let pin_ok = match self.port {
            0 | 1 => self.pin <= 15,
            2 => (13..=15).contains(&self.pin),
            _ => false,
        };
        if !pin_ok {
            return Err(format!(
                "引脚 {} 不可用（只有 PA0–PA15、PB0–PB15、PC13–PC15）",
                self.pin_label()
            ));
        }
        match &self.waveform {
            StimulusWaveform::Square {
                half_period_ns,
                duty,
            } => {
                if !(1_000..=3_600_000_000_000).contains(half_period_ns) {
                    return Err("半周期必须在 1 µs 到 1 小时之间".to_string());
                }
                if !(0.0..=1.0).contains(duty) {
                    return Err("占空比必须在 0 % 到 100 % 之间".to_string());
                }
            }
            StimulusWaveform::Function { expression } => {
                Expression::parse(expression)?;
            }
            StimulusWaveform::Level(_) => {}
        }
        Ok(())
    }
}

/// Progress of the built-in stimulus, for the status panel.
#[derive(Clone, Debug)]
pub struct StimulusState {
    pub config: StimulusConfig,
    pub high: bool,
    /// `None` when the waveform has no further edge (constant level/function).
    pub next_toggle_ns: Option<u64>,
}

/// Live state of the MAX7219 LED matrix module.
#[derive(Clone, Debug, Default)]
pub struct MatrixSnapshot {
    pub chips: Vec<ChipState>,
    pub frames: u64,
    pub words: u64,
}

impl MatrixSnapshot {
    /// Pixels of the first die, `rows[row][column]`.
    pub fn pixels(&self) -> [[bool; 8]; 8] {
        self.chips
            .first()
            .map(ChipState::pixels)
            .unwrap_or([[false; 8]; 8])
    }

    pub fn intensity(&self) -> u8 {
        self.chips.first().map(ChipState::intensity).unwrap_or(0)
    }

    pub fn enabled(&self) -> bool {
        self.chips.first().map(ChipState::enabled).unwrap_or(false)
    }

    pub fn display_test(&self) -> bool {
        self.chips
            .first()
            .map(ChipState::display_test)
            .unwrap_or(false)
    }
}

/// Live state of the CAN controller, the transceiver and the bus.
#[derive(Clone, Debug)]
pub struct CanSnapshot {
    /// Decoded controller registers, straight from the SoC status list.
    pub controller: Vec<(&'static str, String)>,
    pub records: Vec<BusRecord>,
    pub bus: CanBusStats,
    pub transceiver: TransceiverStats,
    pub mode: TransceiverMode,
    pub node: u32,
    pub node_name: String,
    pub pending: usize,
}

impl Default for CanSnapshot {
    fn default() -> Self {
        CanSnapshot {
            controller: Vec::new(),
            records: Vec::new(),
            bus: CanBusStats::default(),
            transceiver: TransceiverStats::default(),
            mode: TransceiverMode::Normal,
            node: 0,
            node_name: String::new(),
            pending: 0,
        }
    }
}

/// Everything the UI needs to draw one frame.
#[derive(Clone, Debug)]
pub struct Snapshot {
    pub running: bool,
    pub source: Source,
    /// Human readable name of the active source, including its settings.
    pub source_label: String,
    pub now_ns: u64,
    pub instructions: u64,
    pub interrupts: u64,
    pub resets: u32,
    pub clocks: Clocks,
    pub stats: Stats,
    pub levels: PinLevels,
    pub uart: String,
    pub halt: Option<String>,
    pub pc: Option<u32>,
    pub stimulus: Option<StimulusState>,
    pub peripherals: Vec<PeripheralStatus>,
    pub matrix: MatrixSnapshot,
    pub can: CanSnapshot,
}

impl Snapshot {
    pub fn elapsed_ms(&self) -> f64 {
        self.now_ns as f64 / 1_000_000.0
    }
}

/// One simulated STM32F103 plus the optional core executing a firmware.
pub struct Simulation {
    soc: Rc<RefCell<Soc>>,
    cpu: Option<Cpu>,
    image: Option<ElfImage>,
    source: Source,
    running: bool,
    halt: Option<String>,
    uart: Rc<RefCell<String>>,
    stimulus: StimulusConfig,
    /// Parsed `StimulusWaveform::Function` expression, if that mode is active.
    stimulus_expr: Option<Expression>,
    stimulus_high: bool,
    stimulus_next_ns: u64,
    /// 8x8 LED matrix wired to SPI1 (`PA5`/`PA7`) with `PA4` as LOAD.
    matrix: Rc<RefCell<Max7219>>,
    /// CAN transceiver between CAN1 and the virtual bus.
    transceiver: Rc<RefCell<CanTransceiver>>,
    bus: Rc<RefCell<CanBus>>,
    /// Approximate timestamp handed to events while the SoC is stepping.
    event_clock_ns: Rc<Cell<u64>>,
    /// Events produced since the front end last drained them.
    events: Rc<RefCell<Vec<Event>>>,
}

impl Default for Simulation {
    fn default() -> Self {
        Simulation::new()
    }
}

impl Simulation {
    pub fn new() -> Self {
        let soc = Rc::new(RefCell::new(Soc::with_hse(8_000_000)));
        let uart = Rc::new(RefCell::new(String::new()));
        let events: Rc<RefCell<Vec<Event>>> = Rc::new(RefCell::new(Vec::new()));

        let sink_text = Rc::clone(&uart);
        let sink_events = Rc::clone(&events);
        soc.borrow_mut()
            .add_event_sink(Box::new(move |event: &Event| {
                sink_events.borrow_mut().push(event.clone());
                if let Event::UartTx { unit: 0, byte } = event {
                    let mut text = sink_text.borrow_mut();
                    if text.len() >= 16_384 {
                        let cut = text.len() - 8_192;
                        text.replace_range(0..cut, "");
                    }
                    text.push(*byte as char);
                }
            }));

        let matrix = Rc::new(RefCell::new(Max7219::firmware_default()));
        let bus = Rc::new(RefCell::new(CanBus::new()));
        let transceiver = Rc::new(RefCell::new(CanTransceiver::attach(
            Rc::clone(&bus),
            "CAN1 节点",
            0,
        )));
        let event_clock_ns = Rc::new(Cell::new(0u64));

        let sink_matrix = Rc::clone(&matrix);
        let sink_can = Rc::clone(&transceiver);
        let sink_clock = Rc::clone(&event_clock_ns);
        soc.borrow_mut()
            .add_event_sink(Box::new(move |event: &Event| {
                let now = sink_clock.get();
                sink_matrix.borrow_mut().on_event(event);
                sink_can.borrow_mut().on_event(event, now);
            }));

        let mut simulation = Simulation {
            soc,
            cpu: None,
            image: None,
            source: Source::Idle,
            running: false,
            halt: None,
            uart,
            stimulus: StimulusConfig::default(),
            stimulus_expr: None,
            stimulus_high: false,
            stimulus_next_ns: 0,
            matrix,
            transceiver,
            bus,
            event_clock_ns,
            events,
        };
        simulation.start_stimulus();
        simulation
    }

    pub fn source(&self) -> &Source {
        &self.source
    }

    /// Current stimulus settings, whether or not it is running.
    pub fn stimulus(&self) -> &StimulusConfig {
        &self.stimulus
    }

    /// Replaces the stimulus settings.
    ///
    /// `Err` leaves the previous settings (and the running waveform) untouched.
    /// When the stimulus is the active source it is restarted with the new
    /// settings, keeping the run/pause state.
    pub fn set_stimulus_config(&mut self, config: StimulusConfig) -> Result<(), String> {
        config.validate()?;
        let expression = match &config.waveform {
            StimulusWaveform::Function { expression } => Some(Expression::parse(expression)?),
            _ => None,
        };

        let active = self.source == Source::Stimulus;
        let was_running = self.running;
        if active {
            self.stop_stimulus();
        }
        self.stimulus = config;
        self.stimulus_expr = expression;
        if active {
            self.start_stimulus();
            self.running = was_running;
        }
        Ok(())
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    pub fn set_running(&mut self, running: bool) {
        self.running = running && self.source != Source::Idle;
    }

    pub fn toggle_running(&mut self) {
        self.set_running(!self.running);
    }

    pub fn clear_uart(&mut self) {
        self.uart.borrow_mut().clear();
    }

    pub fn now_ns(&self) -> u64 {
        self.soc.borrow().now_ns()
    }

    /// Clock tree of the device, for the bridge's slice arithmetic.
    pub fn clocks(&self) -> gsim_core::Clocks {
        self.soc.borrow().clocks()
    }

    /// Drains the peripheral events produced since the last call.
    pub fn take_events(&self) -> Vec<Event> {
        std::mem::take(&mut *self.events.borrow_mut())
    }

    /// Drives an MCU pin from the outside, as a wired device would.
    pub fn drive_pin(&mut self, pin: (u8, u8), high: bool) {
        self.soc
            .borrow_mut()
            .gpio_set_input(usize::from(pin.0), pin.1, high);
    }

    /// Hands one byte to a USART receiver.
    ///
    /// Returns `false` when the port is not ready for it (receiver disabled, or
    /// a byte still unread), so callers can retry later instead of losing data.
    pub fn send_uart(&mut self, unit: usize, byte: u8) -> bool {
        let Some(base) = gsim_stm32f1::map::usart::BASES.get(unit).copied() else {
            return false;
        };
        let mut soc = self.soc.borrow_mut();
        let control = soc
            .read(base + gsim_stm32f1::map::usart::CR1, AccessWidth::Word)
            .unwrap_or(0);
        // UE | RE - the receiver only takes bytes when it is switched on.
        if control & ((1 << 13) | (1 << 2)) != ((1 << 13) | (1 << 2)) {
            return false;
        }
        let status = soc
            .read(base + gsim_stm32f1::map::usart::SR, AccessWidth::Word)
            .unwrap_or(0);
        if status & (1 << 5) != 0 {
            return false;
        }
        soc.usart_receive(unit, byte);
        true
    }

    /// Reads and clears one received byte of a USART, for scripting and tests.
    pub fn usart_read(&mut self, unit: usize) -> Option<u8> {
        let base = *gsim_stm32f1::map::usart::BASES.get(unit)?;
        self.with_soc(|soc| {
            let status = soc
                .read(base + gsim_stm32f1::map::usart::SR, AccessWidth::Word)
                .ok()?;
            if status & (1 << 5) == 0 {
                return None;
            }
            let data = soc
                .read(base + gsim_stm32f1::map::usart::DR, AccessWidth::Word)
                .ok()?;
            Some((data & 0xFF) as u8)
        })
    }

    /// Runs `f` against the simulated device.
    ///
    /// Useful for scripting a scenario (poke a register, read a peripheral)
    /// without going through the CPU.
    pub fn with_soc<R>(&mut self, f: impl FnOnce(&mut Soc) -> R) -> R {
        f(&mut self.soc.borrow_mut())
    }

    /// Starts the built-in stimulus with its current [`StimulusConfig`].
    ///
    /// The stimulus drives the real GPIO registers through the bus (`APB2ENR`
    /// clock, `CRL`/`CRH` configuration, `BSRR` toggling), exactly like a
    /// firmware would, so the wiring sees ordinary pin activity.
    pub fn start_stimulus(&mut self) {
        self.cpu = None;
        self.image = None;
        self.halt = None;
        self.stimulus_high = false;
        self.stimulus_next_ns = 0;

        let port = usize::from(self.stimulus.port);
        let pin = self.stimulus.pin;
        let gpio = map::gpio::PORTS[port];
        {
            let mut soc = self.soc.borrow_mut();
            soc.set_step_ns(STIMULUS_STEP_NS);
            // GPIOAEN = bit 2, GPIOBEN = bit 3, GPIOCEN = bit 4.
            let _ = soc.write(
                map::rcc::BASE + map::rcc::APB2ENR,
                AccessWidth::Word,
                1 << (2 + port),
            );
            let (register, shift) = if pin < 8 {
                (map::gpio::CRL, u32::from(pin) * 4)
            } else {
                (map::gpio::CRH, u32::from(pin - 8) * 4)
            };
            let value = soc.read(gpio + register, AccessWidth::Word).unwrap_or(0);
            // Output, push-pull, 2 MHz (MODE = 0b10, CNF = 0b00).
            let value = (value & !(0xF << shift)) | (0b0010 << shift);
            let _ = soc.write(gpio + register, AccessWidth::Word, value);
        }

        self.source = Source::Stimulus;
        self.running = true;
        self.begin_stimulus_waveform();
    }

    /// Switches the built-in stimulus off and leaves the simulation idle.
    ///
    /// The configured pin is driven low and handed back as a floating input,
    /// so the wiring stops seeing activity instead of freezing at the last
    /// level. Calling this while a firmware is loaded does nothing.
    pub fn stop_stimulus(&mut self) {
        if self.source != Source::Stimulus {
            return;
        }

        let pin = self.stimulus.pin;
        let gpio = map::gpio::PORTS[usize::from(self.stimulus.port)];
        {
            let mut soc = self.soc.borrow_mut();
            // BRR is the high half of BSRR.
            let _ = soc.write(gpio + map::gpio::BSRR, AccessWidth::Word, 1 << (pin + 16));
            let (register, shift) = if pin < 8 {
                (map::gpio::CRL, u32::from(pin) * 4)
            } else {
                (map::gpio::CRH, u32::from(pin - 8) * 4)
            };
            let value = soc.read(gpio + register, AccessWidth::Word).unwrap_or(0);
            // Input, floating (MODE = 0b00, CNF = 0b01).
            let value = (value & !(0xF << shift)) | (0b0100 << shift);
            let _ = soc.write(gpio + register, AccessWidth::Word, value);
        }

        self.source = Source::Idle;
        self.running = false;
        self.halt = None;
        self.stimulus_high = false;
        self.stimulus_next_ns = 0;
    }

    /// Puts the pin at the waveform's value for `now`.
    fn begin_stimulus_waveform(&mut self) {
        match self.stimulus.waveform.clone() {
            StimulusWaveform::Square {
                half_period_ns,
                duty,
            } => {
                let (high_ns, _) = square_durations(half_period_ns, duty);
                self.stimulus_high = true;
                self.apply_stimulus_level();
                self.stimulus_next_ns = self.now_ns() + high_ns;
            }
            StimulusWaveform::Level(high) => {
                self.stimulus_high = high;
                self.apply_stimulus_level();
                self.stimulus_next_ns = 0;
            }
            StimulusWaveform::Function { .. } => {
                self.stimulus_high = self.eval_stimulus_function();
                self.apply_stimulus_level();
                self.stimulus_next_ns = 0;
            }
        }
    }

    /// Loads a firmware image and boots the core from its vector table.
    pub fn load_elf(&mut self, path: &Path) -> Result<(), String> {
        let image = ElfImage::open(path).map_err(|error| format!("无法读取 ELF：{error}"))?;
        let mut cpu = Cpu::new(Rc::clone(&self.soc)).map_err(|error| error.to_string())?;
        cpu.load(&image)
            .map_err(|error| describe(&error, Some(&image)))?;

        self.cpu = Some(cpu);
        self.image = Some(image);
        self.source = Source::Firmware(path.to_path_buf());
        self.running = true;
        self.halt = None;
        self.uart.borrow_mut().clear();
        Ok(())
    }

    /// Restarts whatever is currently driving the pins.
    pub fn reset(&mut self) {
        match self.source.clone() {
            Source::Firmware(_) => {
                self.uart.borrow_mut().clear();
                match self.cpu.as_mut().map(Cpu::reset) {
                    Some(Ok(())) => {
                        self.halt = None;
                        self.running = true;
                    }
                    Some(Err(error)) => {
                        self.halt = Some(describe(&error, self.image.as_ref()));
                        self.running = false;
                    }
                    None => self.start_stimulus(),
                }
            }
            Source::Stimulus => {
                self.soc.borrow_mut().system_reset(0);
                self.start_stimulus();
            }
            Source::Idle => {
                // Nothing was driving the pins, so a reset must not silently
                // switch the built-in stimulus back on.
                self.soc.borrow_mut().system_reset(0);
                self.running = false;
                self.halt = None;
            }
        }
    }

    /// Advances one frame: `budget` instructions, or the equivalent amount of
    /// simulated time when the built-in stimulus is driving.
    pub fn advance_frame(&mut self, budget: u32) {
        if !self.running || budget == 0 {
            return;
        }
        self.step_once(budget);
    }
    /// Advances by `budget` instructions regardless of the run/pause state, as
    /// the single-step button does.
    pub fn step_once(&mut self, budget: u32) {
        if budget == 0 {
            return;
        }
        match self.source.clone() {
            Source::Firmware(_) => self.run_firmware(budget),
            Source::Stimulus => {
                let sysclk = u64::from(self.soc.borrow().clocks().sysclk_hz.max(1));
                let ns = u64::from(budget).saturating_mul(1_000_000_000) / sysclk;
                self.advance_stimulus(ns);
                self.advance_devices();
            }
            Source::Idle => {}
        }
    }

    /// Pushes the external devices one step forward: primes the `MISO` line of
    /// the LED matrix and hands every frame the bus delivered to the CAN
    /// controller.
    fn advance_devices(&mut self) {
        self.event_clock_ns.set(self.now_ns());
        let (unit, miso) = {
            let matrix = self.matrix.borrow();
            (usize::from(matrix.unit()), matrix.miso())
        };
        self.soc.borrow_mut().spi_set_miso(unit, miso);
        for _ in 0..MAX_CAN_FRAMES_PER_STEP {
            let frame = self.transceiver.borrow_mut().poll();
            let Some(frame) = frame else {
                break;
            };
            self.soc.borrow_mut().can_receive(0, frame);
        }
    }

    fn run_firmware(&mut self, budget: u32) {
        let mut remaining = budget;
        while remaining > 0 {
            let chunk = remaining.min(SERVICE_CHUNK);
            let result = {
                let Some(cpu) = self.cpu.as_mut() else {
                    self.source = Source::Idle;
                    self.running = false;
                    return;
                };
                cpu.run(u64::from(chunk))
            };
            remaining -= chunk;
            match result {
                Ok(StopReason::InstructionLimit) => {}
                Ok(reason) => {
                    self.running = false;
                    self.halt = Some(format!("核心已停止：{reason:?}"));
                    return;
                }
                Err(error) => {
                    let message = describe(&error, self.image.as_ref());
                    self.running = false;
                    self.halt = Some(message);
                    return;
                }
            }
            self.advance_devices();
        }
    }

    fn advance_stimulus(&mut self, mut remaining_ns: u64) {
        match self.stimulus.waveform.clone() {
            StimulusWaveform::Square {
                half_period_ns,
                duty,
            } => {
                let (high_ns, low_ns) = square_durations(half_period_ns, duty);
                while remaining_ns > 0 {
                    let now = self.now_ns();
                    let until_toggle = self.stimulus_next_ns.saturating_sub(now);
                    if until_toggle == 0 {
                        self.stimulus_high = !self.stimulus_high;
                        let dwell = if self.stimulus_high { high_ns } else { low_ns };
                        self.stimulus_next_ns = now + dwell;
                        self.apply_stimulus_level();
                        continue;
                    }
                    let chunk = until_toggle.min(remaining_ns);
                    self.soc.borrow_mut().advance_ns(chunk);
                    remaining_ns -= chunk;
                }
            }
            StimulusWaveform::Level(_) => {
                self.soc.borrow_mut().advance_ns(remaining_ns);
            }
            StimulusWaveform::Function { .. } => {
                while remaining_ns > 0 {
                    let chunk = remaining_ns.min(STIMULUS_FUNCTION_STEP_NS);
                    self.soc.borrow_mut().advance_ns(chunk);
                    remaining_ns -= chunk;
                    let level = self.eval_stimulus_function();
                    if level != self.stimulus_high {
                        self.stimulus_high = level;
                        self.apply_stimulus_level();
                    }
                }
            }
        }
    }

    fn eval_stimulus_function(&self) -> bool {
        let Some(expression) = &self.stimulus_expr else {
            return false;
        };
        let value = expression.eval(self.now_ns() as f64 / 1e9);
        value.is_finite() && value > 0.5
    }

    fn apply_stimulus_level(&mut self) {
        let gpio = map::gpio::PORTS[usize::from(self.stimulus.port)];
        let bit = 1u32 << self.stimulus.pin;
        // BSRR sets the low half, BRR is its high half.
        let value = if self.stimulus_high { bit } else { bit << 16 };
        let _ = self
            .soc
            .borrow_mut()
            .write(gpio + map::gpio::BSRR, AccessWidth::Word, value);
    }

    /// Current pin levels, read straight out of the GPIO models.
    pub fn pin_levels(&self) -> PinLevels {
        let soc = self.soc.borrow();
        let mut levels = PinLevels::default();
        for (port, gpio) in soc.gpio.iter().enumerate() {
            for pin in 0..16u8 {
                levels.level[port][pin as usize] = gpio.input_level(pin);
                levels.output[port][pin as usize] = gpio.is_output(pin);
            }
        }
        levels
    }

    pub fn snapshot(&self) -> Snapshot {
        let levels = self.pin_levels();
        let soc = self.soc.borrow();
        let matrix = self.matrix.borrow();
        let transceiver = self.transceiver.borrow();
        let bus = self.bus.borrow();
        let controller = soc
            .can
            .status(&soc.clocks())
            .into_iter()
            .collect::<Vec<_>>();
        Snapshot {
            running: self.running,
            source: self.source.clone(),
            now_ns: soc.now_ns(),
            instructions: self.cpu.as_ref().map(Cpu::instructions).unwrap_or(0),
            interrupts: self.cpu.as_ref().map(Cpu::interrupts).unwrap_or(0),
            resets: soc.reset_count(),
            clocks: soc.clocks(),
            stats: soc.stats(),
            levels,
            uart: self.uart.borrow().clone(),
            halt: self.halt.clone(),
            pc: self.cpu.as_ref().map(Cpu::pc),
            stimulus: (self.source == Source::Stimulus).then_some(StimulusState {
                high: self.stimulus_high,
                next_toggle_ns: self.stimulus_next_ns,
            }),
            peripherals: {
                let mut list = soc.peripheral_status();
                list.push(PeripheralStatus {
                    name: "MAX7219".to_string(),
                    kind: "点阵显示（外部）",
                    base: 0,
                    clock_hz: 0,
                    rows: matrix_rows(&matrix),
                });
                list.push(PeripheralStatus {
                    name: "CAN 收发器".to_string(),
                    kind: "总线收发器（外部）",
                    base: 0,
                    clock_hz: 0,
                    rows: vec![
                        ("节点", transceiver.name().to_string()),
                        ("模式", transceiver.mode().label().to_string()),
                        (
                            "引脚",
                            format!(
                                "TXD = P{}{}，RXD = P{}{}",
                                gsim_stm32f1::PORT_NAMES[usize::from(transceiver.pins().0 .0)],
                                transceiver.pins().0 .1,
                                gsim_stm32f1::PORT_NAMES[usize::from(transceiver.pins().1 .0)],
                                transceiver.pins().1 .1
                            ),
                        ),
                        ("已发送帧", format!("{}", transceiver.stats().tx_frames)),
                        ("已接收帧", format!("{}", transceiver.stats().rx_frames)),
                        ("回环帧", format!("{}", transceiver.stats().loopback_frames)),
                        ("总线帧数", format!("{}", bus.stats().frames)),
                        ("待投递", format!("{}", bus.pending(transceiver.node()))),
                    ],
                });
                list
            },
            matrix: MatrixSnapshot {
                chips: matrix.chips().to_vec(),
                frames: matrix.frames(),
                words: matrix.words(),
            },
            can: CanSnapshot {
                controller,
                records: bus.records().iter().cloned().collect(),
                bus: bus.stats(),
                transceiver: transceiver.stats(),
                mode: transceiver.mode(),
                node: transceiver.node(),
                node_name: transceiver.name().to_string(),
                pending: bus.pending(transceiver.node()),
            },
        }
    }

    /// Puts a frame on the virtual bus as if another ECU had sent it.
    pub fn inject_can_frame(&mut self, frame: gsim_core::CanFrame) {
        let now = self.now_ns();
        self.bus.borrow_mut().inject(frame, now);
    }

    /// Switches the transceiver between normal, listen-only and standby.
    pub fn set_transceiver_mode(&mut self, mode: TransceiverMode) {
        self.transceiver.borrow_mut().set_mode(mode);
    }

    pub fn transceiver_mode(&self) -> TransceiverMode {
        self.transceiver.borrow().mode()
    }

    pub fn clear_can_log(&mut self) {
        self.bus.borrow_mut().clear_records();
    }
}

/// Rows describing the LED matrix module in the peripheral browser.
fn matrix_rows(matrix: &Max7219) -> Vec<(&'static str, String)> {
    let (port, pin) = matrix.cs_pin();
    let mut rows = vec![
        ("级联片数", format!("{}", matrix.chips().len())),
        (
            "LOAD 引脚",
            format!(
                "P{}{}",
                gsim_stm32f1::PORT_NAMES
                    .get(usize::from(port))
                    .copied()
                    .unwrap_or("?"),
                pin
            ),
        ),
        ("SPI 单元", format!("SPI{}", matrix.unit() + 1)),
        ("已接收字节", format!("{}", matrix.frames())),
        ("已锁存字", format!("{}", matrix.words())),
    ];
    if let Some(chip) = matrix.chips().first() {
        rows.push(("亮度", format!("{}/15", chip.intensity())));
        rows.push(("扫描行", format!("{}", chip.scanned_rows())));
        rows.push(("关断", if chip.enabled() { "否" } else { "是" }.to_string()));
        rows.push((
            "显示测试",
            if chip.display_test() { "是" } else { "否" }.to_string(),
        ));
    }
    rows
}

/// Turns an emulation failure into a message that names the offending address
/// and, when the symbol table knows it, the firmware function that caused it.
fn describe(error: &CpuError, image: Option<&ElfImage>) -> String {
    let (pc, address, access) = match error {
        CpuError::Emulation {
            pc,
            error: RunError::MemRead { address, .. },
        } => (*pc, *address, "读"),
        CpuError::Emulation {
            pc,
            error: RunError::MemWrite { address, .. },
        } => (*pc, *address, "写"),
        other => return other.to_string(),
    };
    let function = image
        .and_then(|image| image.function_at(pc))
        .map(|symbol| format!(" ({})", symbol.name))
        .unwrap_or_default();
    format!("总线错误：{access} 0x{address:08X} 处没有外设模型，PC=0x{pc:08X}{function}")
}
