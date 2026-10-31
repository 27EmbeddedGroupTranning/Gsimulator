//! The STM32F103 system on chip: address decoding, the simulation loop,
//! interrupt routing and the DMA engine.

use gsim_core::bus::{check_alignment, AccessWidth, BusError, MemoryBus};
use gsim_core::ctx::{Clocks, Ctx, Event, EventSink};
use gsim_core::irq::{exception, lines};
use gsim_core::nvic::Nvic;
use gsim_core::peripheral::Peripheral;
use gsim_core::scb::Scb;
use gsim_core::systick::SysTick;

use crate::adc::Adc;
use crate::afio::Afio;
use crate::bitband;
use crate::can::Can;
use crate::crc::{Crc, DbgMcu};
use crate::dma::{self, Dma, Request};
use crate::exti::{Exti, LINE_COUNT};
use crate::gpio::{GpioPort, PORT_COUNT};
use crate::i2c::I2c;
use crate::map;
use crate::memory::{Erase, FlashInterface, Memory};
use crate::pwr::{Bkp, Pwr};
use crate::rcc::{self, Rcc};
use crate::rtc::Rtc;
use crate::spi::Spi;
use crate::tim::Tim;
use crate::usart::Usart;
use crate::watchdog::{Iwdg, Wwdg};

/// Default simulation granularity: every step advances one microsecond.
pub const DEFAULT_STEP_NS: u64 = 1_000;
/// Maximum number of DMA transfers serviced in one simulation step.
const MAX_DMA_TRANSFERS_PER_STEP: usize = 1024;

/// A block attached at run time through [`Soc::attach_peripheral`].
struct ExtraRegion {
    base: u32,
    size: u32,
    block: Box<dyn Peripheral>,
}

/// Access counters, useful for diagnostics and for the CLI.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    pub reads: u64,
    pub writes: u64,
    pub steps: u64,
    pub dma_transfers: u64,
    pub bus_errors: u64,
}

/// The simulated device.
pub struct Soc {
    pub memory: Memory,
    pub flash_if: FlashInterface,
    pub rcc: Rcc,
    pub gpio: [GpioPort; PORT_COUNT],
    pub afio: Afio,
    pub exti: Exti,
    pub usart: [Usart; 3],
    pub spi: [Spi; 2],
    pub i2c: [I2c; 2],
    pub can: Can,
    pub tim: [Tim; 4],
    pub adc: [Adc; 2],
    pub dma1: Dma,
    pub dma2: Dma,
    pub pwr: Pwr,
    pub bkp: Bkp,
    pub rtc: Rtc,
    pub iwdg: Iwdg,
    pub wwdg: Wwdg,
    pub crc: Crc,
    pub dbgmcu: DbgMcu,
    pub nvic: Nvic,
    pub scb: Scb,
    pub systick: SysTick,

    /// Crystal fitted to the board, used by [`Soc::new`].
    hse_hz: u32,
    now_ns: u64,
    step_ns: u64,
    sinks: Vec<Box<dyn EventSink>>,
    extra: Vec<ExtraRegion>,
    stats: Stats,
    strict_clock_gating: bool,
    reset_count: u32,
    systick_external: bool,
}

impl Default for Soc {
    fn default() -> Self {
        Soc::new()
    }
}

impl Soc {
    /// Creates a powered on device with an 8 MHz crystal.
    pub fn new() -> Self {
        Soc::with_hse(8_000_000)
    }

    /// Creates a powered on device with a board specific crystal frequency.
    pub fn with_hse(hse_hz: u32) -> Self {
        let mut rcc = Rcc::new();
        rcc.set_hse_hz(hse_hz);
        let mut soc = Soc {
            memory: Memory::new(),
            flash_if: FlashInterface::new(),
            rcc,
            gpio: std::array::from_fn(|index| GpioPort::new(index as u8)),
            afio: Afio::new(),
            exti: Exti::new(),
            usart: [
                Usart::new(0, lines::USART1),
                Usart::new(1, lines::USART2),
                Usart::new(2, lines::USART3),
            ],
            spi: [Spi::new(0, lines::SPI1), Spi::new(1, lines::SPI2)],
            i2c: [
                I2c::new(0, lines::I2C1_EV, lines::I2C1_ER),
                I2c::new(1, lines::I2C2_EV, lines::I2C2_ER),
            ],
            can: Can::new(
                0,
                lines::CAN1_TX,
                lines::CAN1_RX0,
                lines::CAN1_RX1,
                lines::CAN1_SCE,
            ),
            tim: [
                Tim::new(0, lines::TIM1_UP, lines::TIM1_CC, true),
                Tim::new(1, lines::TIM2, lines::TIM2, false),
                Tim::new(2, lines::TIM3, lines::TIM3, false),
                Tim::new(3, lines::TIM4, lines::TIM4, false),
            ],
            adc: [Adc::new(0, lines::ADC1_2), Adc::new(1, lines::ADC1_2)],
            dma1: Dma::new(7, lines::DMA1_CHANNEL1.number()),
            dma2: Dma::new(5, lines::DMA2_CHANNEL1.number()),
            pwr: Pwr::new(),
            bkp: Bkp::new(),
            rtc: Rtc::new(),
            iwdg: Iwdg::new(),
            wwdg: Wwdg::new(),
            crc: Crc::new(),
            dbgmcu: DbgMcu::new(),
            nvic: Nvic::new(),
            scb: Scb::new(),
            systick: SysTick::new(),
            hse_hz,
            now_ns: 0,
            step_ns: DEFAULT_STEP_NS,
            sinks: Vec::new(),
            extra: Vec::new(),
            stats: Stats::default(),
            strict_clock_gating: false,
            reset_count: 0,
            systick_external: false,
        };
        soc.refresh_backup_protection();
        soc
    }

    /// Crystal frequency configured on OSC_IN/OSC_OUT.
    pub fn hse_hz(&self) -> u32 {
        self.hse_hz
    }

    /// Simulation time in nanoseconds.
    pub fn now_ns(&self) -> u64 {
        self.now_ns
    }

    /// Simulation time in microseconds.
    pub fn now_us(&self) -> u64 {
        self.now_ns / 1_000
    }

    /// Current simulation granularity.
    pub fn step_ns(&self) -> u64 {
        self.step_ns
    }

    /// Changes the simulation granularity. Larger steps simulate faster but
    /// quantise event times to the step boundary.
    pub fn set_step_ns(&mut self, step_ns: u64) {
        self.step_ns = step_ns.max(1);
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// Number of system resets performed so far (watchdogs included).
    pub fn reset_count(&self) -> u32 {
        self.reset_count
    }

    /// Clock tree snapshot.
    pub fn clocks(&self) -> Clocks {
        self.rcc.clocks()
    }

    /// Registers an observer for peripheral events (UART bytes, pin changes...).
    pub fn add_event_sink(&mut self, sink: Box<dyn EventSink>) {
        self.sinks.push(sink);
    }

    /// Attaches a block at an address range that the core decode does not know.
    ///
    /// This is the extension point for controllers added in a later phase: the
    /// block only has to implement [`Peripheral`] and the SoC will route reads,
    /// writes and clock ticks to it. The range must not overlap a peripheral
    /// that is already decoded.
    pub fn attach_peripheral(&mut self, base: u32, size: u32, block: Box<dyn Peripheral>) {
        self.extra.push(ExtraRegion { base, size, block });
    }

    /// Requires peripheral clocks to be enabled before their registers answer.
    pub fn set_strict_clock_gating(&mut self, strict: bool) {
        self.strict_clock_gating = strict;
    }

    /// Hands the SysTick timer to an attached CPU core.
    ///
    /// A core model normally owns the Cortex-M system control space, including
    /// `SysTick`, so the SoC must stop advancing its own copy to avoid counting
    /// the timer twice.
    pub fn set_external_systick(&mut self, external: bool) {
        self.systick_external = external;
    }

    // ----------------------------------------------------------------- memory

    /// Copies an image into flash or SRAM, as a debugger would.
    pub fn load_image(&mut self, addr: u32, data: &[u8]) -> Result<(), BusError> {
        self.memory.load(addr, data)
    }

    // ------------------------------------------------------- input injection

    /// Drives the level sampled by a GPIO input pin.
    pub fn gpio_set_input(&mut self, port: usize, pin: u8, high: bool) {
        if let Some(gpio) = self.gpio.get_mut(port) {
            gpio.set_external_level(pin, high);
        }
    }

    /// Hands a byte to a USART receiver.
    pub fn usart_receive(&mut self, unit: usize, byte: u8) {
        let cx = Ctx::new(self.now_ns, self.rcc.clocks());
        if let Some(usart) = self.usart.get_mut(unit) {
            usart.receive_byte(&cx, byte);
        }
        self.drain_events(&cx);
    }

    /// Sets the value a SPI master shifts in on MISO for the next frame.
    pub fn spi_set_miso(&mut self, unit: usize, value: u16) {
        if let Some(spi) = self.spi.get_mut(unit) {
            spi.set_miso(value);
        }
    }

    /// Sets the voltage present on an ADC channel.
    pub fn adc_set_channel_mv(&mut self, unit: usize, channel: usize, millivolts: u32) {
        if let Some(adc) = self.adc.get_mut(unit) {
            adc.set_channel_mv(channel, millivolts);
        }
    }

    /// Drives a timer channel input, used by input capture.
    pub fn tim_set_channel_input(&mut self, unit: usize, channel: usize, high: bool) {
        if let Some(tim) = self.tim.get_mut(unit) {
            tim.set_channel_input(channel, high);
        }
    }

    /// Hands one frame from the external bus to the CAN controller, which runs
    /// it through its acceptance filters. Returns `true` when a filter accepted
    /// it and it was queued in a receive FIFO.
    pub fn can_receive(&mut self, unit: usize, frame: gsim_core::CanFrame) -> bool {
        let cx = Ctx::new(self.now_ns, self.rcc.clocks());
        let accepted = if unit == 0 {
            self.can.receive(&cx, frame)
        } else {
            false
        };
        self.drain_events(&cx);
        accepted
    }

    // ------------------------------------------------------------ simulation

    /// Advances the simulation by one granularity step.
    pub fn step(&mut self) {
        let step = self.step_ns;
        self.advance_ns(step);
    }

    /// Advances the simulation by `total_ns` nanoseconds.
    pub fn advance_ns(&mut self, total_ns: u64) {
        let mut remaining = total_ns;
        while remaining > 0 {
            let chunk = remaining.min(self.step_ns);
            self.advance_once(chunk);
            remaining -= chunk;
        }
    }

    pub fn run_us(&mut self, microseconds: u64) {
        self.advance_ns(microseconds * 1_000);
    }

    pub fn run_ms(&mut self, milliseconds: u64) {
        self.advance_ns(milliseconds * 1_000_000);
    }

    fn advance_once(&mut self, dt: u64) {
        self.now_ns += dt;
        self.stats.steps += 1;

        // The RCC may change the clock tree, so it runs first.
        let rcc_cx = Ctx::new(self.now_ns, self.rcc.clocks());
        self.rcc.advance(&rcc_cx, dt);
        self.drain_events(&rcc_cx);

        let cx = Ctx::new(self.now_ns, self.rcc.clocks());

        if !self.systick_external {
            self.systick.advance(&cx, dt);
        }

        // EXTI samples the pin levels produced by GPIO + AFIO routing.
        let levels = self.exti_levels();
        self.exti.update(&cx, &levels);

        let clocks = *cx.clocks();
        let rcc_ref = &self.rcc;

        if rcc_ref.tim1_clock() > 0 {
            self.tim[0].advance(&cx, dt);
        }
        // TIM2/TIM3/TIM4 are gated individually by APB1ENR bits 0..2.
        for index in 1..4 {
            let gate = 1 << (index - 1);
            if rcc_ref.apb1_timer_clock(gate) > 0 {
                self.tim[index].advance(&cx, dt);
            }
        }

        for (index, usart) in self.usart.iter_mut().enumerate() {
            let gate = match index {
                0 => 1 << 14,
                1 => 1 << 17,
                _ => 1 << 18,
            };
            if rcc_ref.apb2_clock(gate) > 0 || rcc_ref.apb1_clock(gate) > 0 {
                usart.advance(&cx, dt);
            }
        }
        for (index, spi) in self.spi.iter_mut().enumerate() {
            let gate = if index == 0 { 1 << 12 } else { 1 << 14 };
            if rcc_ref.apb2_clock(gate) > 0 || rcc_ref.apb1_clock(gate) > 0 {
                spi.advance(&cx, dt);
            }
        }
        for (index, i2c) in self.i2c.iter_mut().enumerate() {
            let gate = if index == 0 { 1 << 21 } else { 1 << 22 };
            if rcc_ref.apb1_clock(gate) > 0 {
                i2c.advance(&cx, dt);
            }
        }
        if rcc_ref.apb1_clock(1 << 25) > 0 {
            self.can.advance(&cx, dt);
        }
        for (index, adc) in self.adc.iter_mut().enumerate() {
            let gate = if index == 0 { 1 << 9 } else { 1 << 10 };
            if rcc_ref.adc_clock(gate) > 0 {
                adc.advance(&cx, dt);
            }
        }
        let _ = clocks;

        self.rtc.advance(&cx, dt);
        self.iwdg.advance(&cx, dt);
        self.wwdg.advance(&cx, dt);

        self.service_dma(&cx);
        self.drain_events(&cx);
        self.check_resets(&cx);
    }

    fn exti_levels(&self) -> [bool; LINE_COUNT] {
        let mut levels = [false; LINE_COUNT];
        for (line, level) in levels.iter_mut().enumerate() {
            let port = usize::from(self.afio.exti_port(line as u8));
            if let Some(gpio) = self.gpio.get(port) {
                *level = gpio.input_level(line as u8);
            }
        }
        levels
    }

    fn check_resets(&mut self, cx: &Ctx) {
        let mut cause = 0u32;
        if self.scb.take_reset_request() {
            cause |= rcc::csr::SFTRSTF;
        }
        if self.iwdg.take_expired() {
            cause |= rcc::csr::IWDGRSTF;
        }
        if self.wwdg.take_expired() {
            cause |= rcc::csr::WWDGRSTF;
        }
        if cause != 0 {
            cx.note("SoC", "system reset", cause);
            self.system_reset(cause);
        }
    }

    /// System reset: peripherals return to their power-on state, memory keeps
    /// its contents, and the reset cause is recorded in `RCC_CSR`.
    pub fn system_reset(&mut self, cause: u32) {
        self.flash_if.reset();
        self.rcc.reset();
        for gpio in &mut self.gpio {
            gpio.reset();
        }
        self.afio.reset();
        self.exti.reset();
        for usart in &mut self.usart {
            usart.reset();
        }
        for spi in &mut self.spi {
            spi.reset();
        }
        for i2c in &mut self.i2c {
            i2c.reset();
        }
        self.can.reset();
        for tim in &mut self.tim {
            tim.reset();
        }
        for adc in &mut self.adc {
            adc.reset();
        }
        self.dma1.reset();
        self.dma2.reset();
        self.pwr.reset();
        self.bkp.reset();
        self.rtc.reset();
        self.iwdg.reset();
        self.wwdg.reset();
        self.crc.reset();
        self.dbgmcu.reset();
        self.nvic.reset();
        self.scb.reset();
        self.systick.reset();
        self.rcc.set_hse_hz(self.hse_hz);
        if cause != 0 {
            self.rcc.set_reset_flag(cause);
        }
        self.reset_count += 1;
        self.refresh_backup_protection();
    }

    fn refresh_backup_protection(&mut self) {
        let enabled = self.pwr.backup_access_enabled();
        self.bkp.set_write_enabled(enabled);
    }

    // -------------------------------------------------------------- interrupts

    /// Takes the highest priority pending external interrupt, clearing its
    /// pending bit as exception entry does on real hardware.
    pub fn take_pending_irq(&mut self) -> Option<gsim_core::Irq> {
        let prigroup = self.scb.prigroup();
        let exception = self.nvic.pending_exception(prigroup)?;
        if exception < 16 {
            self.nvic.set_system_pending(exception, false);
            None
        } else {
            let irq = gsim_core::Irq::new(exception - 16);
            self.nvic.set_pending(irq, false);
            Some(irq)
        }
    }

    /// Highest priority exception that is ready to be taken.
    pub fn pending_exception(&self) -> Option<u16> {
        self.nvic.pending_exception(self.scb.prigroup())
    }

    /// Marks an exception as entered, as the core does before running the ISR.
    pub fn enter_exception(&mut self, exception_number: u16) {
        self.nvic.enter(exception_number);
    }

    /// Marks an exception as complete, as the core does on exception return.
    pub fn exit_exception(&mut self, exception_number: u16) {
        self.nvic.exit(exception_number);
    }

    fn drain_events(&mut self, cx: &Ctx) {
        for event in cx.take_events() {
            match &event {
                Event::Irq { irq, high } => {
                    // Pending mirrors the peripheral line level: an ISR that
                    // acknowledges its flag also drops the pending request.
                    self.nvic.set_pending(*irq, *high);
                }
                Event::Exception { number } => {
                    self.nvic.set_system_pending(*number, true);
                }
                _ => {}
            }
            for sink in &mut self.sinks {
                sink.on_event(&event);
            }
        }
    }

    // --------------------------------------------------------------------- DMA

    fn dma(&self, controller: u8) -> &Dma {
        if controller == 0 {
            &self.dma1
        } else {
            &self.dma2
        }
    }

    fn dma_mut(&mut self, controller: u8) -> &mut Dma {
        if controller == 0 {
            &mut self.dma1
        } else {
            &mut self.dma2
        }
    }

    fn request_asserted(&self, request: Request) -> bool {
        use Request::*;
        match request {
            Adc1 => self.adc[0].dma_request(),
            Adc2 => self.adc[1].dma_request(),
            Spi1Rx => self.spi[0].dma_rx_request(),
            Spi1Tx => self.spi[0].dma_tx_request(),
            Spi2Rx => self.spi[1].dma_rx_request(),
            Spi2Tx => self.spi[1].dma_tx_request(),
            Usart1Rx => self.usart[0].dma_rx_request(),
            Usart1Tx => self.usart[0].dma_tx_request(),
            Usart2Rx => self.usart[1].dma_rx_request(),
            Usart2Tx => self.usart[1].dma_tx_request(),
            Usart3Rx => self.usart[2].dma_rx_request(),
            Usart3Tx => self.usart[2].dma_tx_request(),
            I2c1Rx | I2c1Tx | I2c2Rx | I2c2Tx => false,
            _ => {
                let (unit, channel, update) = tim_request_parts(request);
                let Some(tim) = self.tim.get(unit) else {
                    return false;
                };
                if update {
                    tim.update_dma_request()
                } else {
                    tim.cc_dma_request(channel)
                }
            }
        }
    }

    fn ack_request(&mut self, request: Request) {
        use Request::*;
        match request {
            I2c1Rx | I2c1Tx | I2c2Rx | I2c2Tx => {}
            Adc1 | Adc2 | Spi1Rx | Spi1Tx | Spi2Rx | Spi2Tx | Usart1Rx | Usart1Tx | Usart2Rx
            | Usart2Tx | Usart3Rx | Usart3Tx => {
                // Reading/writing the data register already cleared the request.
            }
            _ => {
                let (unit, channel, update) = tim_request_parts(request);
                if let Some(tim) = self.tim.get_mut(unit) {
                    if update {
                        tim.clear_update_dma_request();
                    } else {
                        tim.clear_cc_dma_request(channel);
                    }
                }
            }
        }
    }

    fn dma_requests(&self, controller: u8) -> [bool; 7] {
        let mut requests = [false; 7];
        for request in Request::ALL {
            let (unit, channel) = request.route();
            if unit != controller {
                continue;
            }
            if let Some(slot) = requests.get_mut(usize::from(channel)) {
                if self.request_asserted(*request) {
                    *slot = true;
                }
            }
        }
        requests
    }

    fn service_dma(&mut self, cx: &Ctx) {
        for controller in 0..2u8 {
            let gate = if controller == 0 { 1 << 0 } else { 1 << 1 };
            if self.rcc.dma_clock(gate) == 0 {
                continue;
            }
            if !self.dma(controller).any_enabled() {
                continue;
            }
            for _ in 0..MAX_DMA_TRANSFERS_PER_STEP {
                let requests = self.dma_requests(controller);
                let Some(transfer) = self.dma(controller).next_transfer(&requests) else {
                    break;
                };
                let Ok(value) = self.read(transfer.source, transfer.width) else {
                    break;
                };
                if self
                    .write(transfer.destination, transfer.width, value)
                    .is_err()
                {
                    break;
                }
                self.dma_mut(controller).complete(cx, transfer);
                self.stats.dma_transfers += 1;
                let channel = transfer.channel;
                for request in Request::ALL {
                    if request.route() == (controller, channel as u8)
                        && self.request_asserted(*request)
                    {
                        self.ack_request(*request);
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------- addressing

    fn dispatch_read(&mut self, addr: u32, width: AccessWidth) -> Result<u32, BusError> {
        let cx = Ctx::new(self.now_ns, self.rcc.clocks());
        let result = self.dispatch_read_inner(&cx, addr, width);
        self.drain_events(&cx);
        result
    }

    fn dispatch_write(
        &mut self,
        addr: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        let cx = Ctx::new(self.now_ns, self.rcc.clocks());
        let result = self.dispatch_write_inner(&cx, addr, width, value);
        self.drain_events(&cx);
        result
    }

    fn dispatch_read_inner(
        &mut self,
        cx: &Ctx,
        addr: u32,
        width: AccessWidth,
    ) -> Result<u32, BusError> {
        if let Some(bit) = bitband::decode(addr) {
            let word = self.dispatch_read_inner(cx, bit.word, AccessWidth::Word)?;
            return Ok(bit.extract(word));
        }

        if Memory::is_flash(addr) || Memory::is_sram(addr) {
            return self.memory.read(addr, width);
        }

        if let Some(offset) = region_offset(addr, map::rcc::BASE, map::PERIPHERAL_WINDOW) {
            return self.rcc.read_reg(cx, offset, width);
        }
        if let Some(offset) = region_offset(addr, map::flash::BASE, map::PERIPHERAL_WINDOW) {
            return self.flash_if.read_reg(cx, offset, width);
        }
        if let Some(offset) = region_offset(addr, map::crc::BASE, map::PERIPHERAL_WINDOW) {
            return self.crc.read_reg(cx, offset, width);
        }
        if let Some(offset) = region_offset(addr, map::dma1::BASE, map::PERIPHERAL_WINDOW) {
            return self.dma1.read_reg(cx, offset, width);
        }
        if let Some(offset) = region_offset(addr, map::dma2::BASE, map::PERIPHERAL_WINDOW) {
            return self.dma2.read_reg(cx, offset, width);
        }
        if let Some(offset) = region_offset(addr, map::dbgmcu::BASE, 0x1000) {
            return self.dbgmcu.read_reg(cx, offset, width);
        }

        if let Some(result) = self.dispatch_apb1_read(cx, addr, width) {
            return result;
        }
        if let Some(result) = self.dispatch_apb2_read(cx, addr, width) {
            return result;
        }
        if let Some(result) = self.dispatch_cortex_read(cx, addr, width) {
            return result;
        }

        for region in &mut self.extra {
            if (region.base..region.base + region.size).contains(&addr) {
                return region.block.read(cx, addr - region.base, width);
            }
        }

        Err(BusError::Unmapped {
            addr,
            width,
            write: false,
        })
    }

    fn dispatch_write_inner(
        &mut self,
        cx: &Ctx,
        addr: u32,
        width: AccessWidth,
        value: u32,
    ) -> Result<(), BusError> {
        if let Some(bit) = bitband::decode(addr) {
            let word = self.dispatch_read_inner(cx, bit.word, AccessWidth::Word)?;
            let word = bit.apply(word, value & 1 != 0);
            return self.dispatch_write_inner(cx, bit.word, AccessWidth::Word, word);
        }

        if Memory::is_flash(addr) {
            return self.write_flash(addr, width, value);
        }
        if Memory::is_sram(addr) {
            return self.memory.write(addr, width, value);
        }

        if let Some(offset) = region_offset(addr, map::rcc::BASE, map::PERIPHERAL_WINDOW) {
            self.rcc.write_reg(cx, offset, width, value)?;
            self.apply_rcc_side_effects(cx);
            return Ok(());
        }
        if let Some(offset) = region_offset(addr, map::flash::BASE, map::PERIPHERAL_WINDOW) {
            self.flash_if.write_reg(cx, offset, width, value)?;
            if let Some(erase) = self.flash_if.take_erase() {
                match erase {
                    Erase::Page(page) => self.memory.erase_page(page),
                    Erase::Mass => self.memory.erase_all(),
                }
                self.flash_if.set_end_of_operation();
            }
            self.flash_if.refresh_irq(cx);
            return Ok(());
        }
        if let Some(offset) = region_offset(addr, map::crc::BASE, map::PERIPHERAL_WINDOW) {
            return self.crc.write_reg(cx, offset, width, value);
        }
        if let Some(offset) = region_offset(addr, map::dma1::BASE, map::PERIPHERAL_WINDOW) {
            return self.dma1.write_reg(cx, offset, width, value);
        }
        if let Some(offset) = region_offset(addr, map::dma2::BASE, map::PERIPHERAL_WINDOW) {
            return self.dma2.write_reg(cx, offset, width, value);
        }
        if let Some(offset) = region_offset(addr, map::dbgmcu::BASE, 0x1000) {
            return self.dbgmcu.write_reg(cx, offset, width, value);
        }

        if let Some(result) = self.dispatch_apb1_write(cx, addr, width, value) {
            return result;
        }
        if let Some(result) = self.dispatch_apb2_write(cx, addr, width, value) {
            return result;
        }
        if let Some(result) = self.dispatch_cortex_write(cx, addr, width, value) {
            return result;
        }

        for region in &mut self.extra {
            if (region.base..region.base + region.size).contains(&addr) {
                return region.block.write(cx, addr - region.base, width, value);
            }
        }

        Err(BusError::Unmapped {
            addr,
            width,
            write: true,
        })
    }

    fn write_flash(&mut self, addr: u32, width: AccessWidth, value: u32) -> Result<(), BusError> {
        if !self.flash_if.programming_enabled() {
            self.flash_if.set_write_error();
            return Ok(());
        }
        for half in 0..(width.bytes().div_ceil(2)) {
            let target = addr + half * 2;
            let halfword = ((value >> (half * 16)) & 0xFFFF) as u16;
            self.memory.program_flash(target, halfword);
        }
        self.flash_if.set_end_of_operation();
        Ok(())
    }

    fn apply_rcc_side_effects(&mut self, cx: &Ctx) {
        if let Some((apb1, apb2)) = self.rcc.take_reset_requests() {
            self.reset_apb1(cx, apb1);
            self.reset_apb2(cx, apb2);
        }
        if self.rcc.take_backup_reset() {
            self.bkp.reset();
            self.rtc.reset();
            cx.note("RCC", "backup domain reset", 0);
        }
        self.refresh_backup_protection();
    }

    fn reset_apb1(&mut self, cx: &Ctx, mask: u32) {
        if mask & (1 << 0) != 0 {
            self.tim[1].reset();
        }
        if mask & (1 << 1) != 0 {
            self.tim[2].reset();
        }
        if mask & (1 << 2) != 0 {
            self.tim[3].reset();
        }
        if mask & (1 << 11) != 0 {
            self.wwdg.reset();
        }
        if mask & (1 << 14) != 0 {
            self.spi[1].reset();
        }
        if mask & (1 << 17) != 0 {
            self.usart[1].reset();
        }
        if mask & (1 << 18) != 0 {
            self.usart[2].reset();
        }
        if mask & (1 << 21) != 0 {
            self.i2c[0].reset();
        }
        if mask & (1 << 22) != 0 {
            self.i2c[1].reset();
        }
        if mask & (1 << 25) != 0 {
            self.can.reset();
        }
        let _ = cx;
    }

    fn reset_apb2(&mut self, _cx: &Ctx, mask: u32) {
        if mask & (1 << 0) != 0 {
            self.afio.reset();
        }
        for (bit, port) in [(2u32, 0usize), (3, 1), (4, 2), (5, 3), (6, 4)] {
            if mask & (1 << bit) != 0 {
                self.gpio[port].reset();
            }
        }
        if mask & (1 << 9) != 0 {
            self.adc[0].reset();
        }
        if mask & (1 << 10) != 0 {
            self.adc[1].reset();
        }
        if mask & (1 << 11) != 0 {
            self.tim[0].reset();
        }
        if mask & (1 << 12) != 0 {
            self.spi[0].reset();
        }
        if mask & (1 << 14) != 0 {
            self.usart[0].reset();
        }
    }

    fn dispatch_apb1_read(
        &mut self,
        cx: &Ctx,
        addr: u32,
        width: AccessWidth,
    ) -> Option<Result<u32, BusError>> {
        let base = map::APB1_BASE;
        let size = map::PERIPHERAL_WINDOW;
        if let Some(offset) = region_offset(addr, map::tim::BASE_2, size) {
            return Some(self.tim[1].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::tim::BASE_3, size) {
            return Some(self.tim[2].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::tim::BASE_4, size) {
            return Some(self.tim[3].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::rtc::BASE, size) {
            return Some(self.rtc.read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::wwdg::BASE, size) {
            return Some(self.wwdg.read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::iwdg::BASE, size) {
            return Some(self.iwdg.read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::spi::BASE_2, size) {
            return Some(self.spi[1].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::usart::BASE_2, size) {
            return Some(self.usart[1].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::usart::BASE_3, size) {
            return Some(self.usart[2].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::i2c::BASE_1, size) {
            return Some(self.i2c[0].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::i2c::BASE_2, size) {
            return Some(self.i2c[1].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::bkp::BASE, size) {
            return Some(self.bkp.read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::pwr::BASE, size) {
            return Some(self.pwr.read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::can::BASE_1, size) {
            return Some(self.can.read_reg(cx, offset, width));
        }
        if addr >= base && addr < base + 0x8000 {
            return Some(Err(BusError::Unmapped {
                addr,
                width,
                write: false,
            }));
        }
        None
    }

    fn dispatch_apb1_write(
        &mut self,
        cx: &Ctx,
        addr: u32,
        width: AccessWidth,
        value: u32,
    ) -> Option<Result<(), BusError>> {
        let base = map::APB1_BASE;
        let size = map::PERIPHERAL_WINDOW;
        if let Some(offset) = region_offset(addr, map::tim::BASE_2, size) {
            return Some(self.tim[1].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::tim::BASE_3, size) {
            return Some(self.tim[2].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::tim::BASE_4, size) {
            return Some(self.tim[3].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::rtc::BASE, size) {
            return Some(self.rtc.write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::wwdg::BASE, size) {
            return Some(self.wwdg.write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::iwdg::BASE, size) {
            return Some(self.iwdg.write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::spi::BASE_2, size) {
            return Some(self.spi[1].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::usart::BASE_2, size) {
            return Some(self.usart[1].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::usart::BASE_3, size) {
            return Some(self.usart[2].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::i2c::BASE_1, size) {
            return Some(self.i2c[0].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::i2c::BASE_2, size) {
            return Some(self.i2c[1].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::bkp::BASE, size) {
            return Some(self.bkp.write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::pwr::BASE, size) {
            let result = self.pwr.write_reg(cx, offset, width, value);
            self.refresh_backup_protection();
            return Some(result);
        }
        if let Some(offset) = region_offset(addr, map::can::BASE_1, size) {
            return Some(self.can.write_reg(cx, offset, width, value));
        }
        if addr >= base && addr < base + 0x8000 {
            return Some(Err(BusError::Unmapped {
                addr,
                width,
                write: true,
            }));
        }
        None
    }

    fn dispatch_apb2_read(
        &mut self,
        cx: &Ctx,
        addr: u32,
        width: AccessWidth,
    ) -> Option<Result<u32, BusError>> {
        let size = map::PERIPHERAL_WINDOW;
        if let Some(offset) = region_offset(addr, map::afio::BASE, size) {
            return Some(self.afio.read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::exti::BASE, size) {
            return Some(self.exti.read_reg(cx, offset, width));
        }
        for port in 0..PORT_COUNT {
            if let Some(offset) = region_offset(addr, map::gpio::PORTS[port], size) {
                return Some(self.gpio[port].read_reg(cx, offset, width));
            }
        }
        for (index, base) in map::adc::BASES.iter().enumerate() {
            if let Some(offset) = region_offset(addr, *base, size) {
                return Some(self.adc[index].read_reg(cx, offset, width));
            }
        }
        if let Some(offset) = region_offset(addr, map::tim::BASE_1, size) {
            return Some(self.tim[0].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::spi::BASE_1, size) {
            return Some(self.spi[0].read_reg(cx, offset, width));
        }
        if let Some(offset) = region_offset(addr, map::usart::BASE_1, size) {
            return Some(self.usart[0].read_reg(cx, offset, width));
        }
        if (map::APB2_BASE..map::APB2_BASE + 0x8000).contains(&addr) {
            return Some(Err(BusError::Unmapped {
                addr,
                width,
                write: false,
            }));
        }
        None
    }

    fn dispatch_apb2_write(
        &mut self,
        cx: &Ctx,
        addr: u32,
        width: AccessWidth,
        value: u32,
    ) -> Option<Result<(), BusError>> {
        let size = map::PERIPHERAL_WINDOW;
        if let Some(offset) = region_offset(addr, map::afio::BASE, size) {
            return Some(self.afio.write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::exti::BASE, size) {
            let result = self.exti.write_reg(cx, offset, width, value);
            let levels = self.exti_levels();
            self.exti.prime(&levels);
            return Some(result);
        }
        for port in 0..PORT_COUNT {
            if let Some(offset) = region_offset(addr, map::gpio::PORTS[port], size) {
                return Some(self.gpio[port].write_reg(cx, offset, width, value));
            }
        }
        for (index, base) in map::adc::BASES.iter().enumerate() {
            if let Some(offset) = region_offset(addr, *base, size) {
                return Some(self.adc[index].write_reg(cx, offset, width, value));
            }
        }
        if let Some(offset) = region_offset(addr, map::tim::BASE_1, size) {
            return Some(self.tim[0].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::spi::BASE_1, size) {
            return Some(self.spi[0].write_reg(cx, offset, width, value));
        }
        if let Some(offset) = region_offset(addr, map::usart::BASE_1, size) {
            return Some(self.usart[0].write_reg(cx, offset, width, value));
        }
        if (map::APB2_BASE..map::APB2_BASE + 0x8000).contains(&addr) {
            return Some(Err(BusError::Unmapped {
                addr,
                width,
                write: true,
            }));
        }
        None
    }

    fn dispatch_cortex_read(
        &mut self,
        _cx: &Ctx,
        addr: u32,
        width: AccessWidth,
    ) -> Option<Result<u32, BusError>> {
        if let Some(offset) = region_offset(addr, 0xE000_ED00, 0x40) {
            return Some(self.scb.read_reg(offset, width, &self.nvic));
        }
        if let Some(offset) = region_offset(addr, 0xE000_E010, 0x10) {
            return Some(self.systick.read_reg(offset, width));
        }
        if let Some(offset) = region_offset(addr, 0xE000_E100, 0xF00) {
            return Some(self.nvic.read_reg(offset, width));
        }
        if (map::CORTEX_BASE..map::CORTEX_BASE + 0x0010_0000).contains(&addr) {
            return Some(Err(BusError::Unmapped {
                addr,
                width,
                write: false,
            }));
        }
        None
    }

    fn dispatch_cortex_write(
        &mut self,
        _cx: &Ctx,
        addr: u32,
        width: AccessWidth,
        value: u32,
    ) -> Option<Result<(), BusError>> {
        if let Some(offset) = region_offset(addr, 0xE000_ED00, 0x40) {
            return Some(self.scb.write_reg(offset, width, value, &mut self.nvic));
        }
        if let Some(offset) = region_offset(addr, 0xE000_E010, 0x10) {
            return Some(self.systick.write_reg(offset, width, value));
        }
        if let Some(offset) = region_offset(addr, 0xE000_E100, 0xF00) {
            return Some(self.nvic.write_reg(offset, width, value));
        }
        if (map::CORTEX_BASE..map::CORTEX_BASE + 0x0010_0000).contains(&addr) {
            return Some(Err(BusError::Unmapped {
                addr,
                width,
                write: true,
            }));
        }
        None
    }

    fn strict_gate(&self, clock_hz: u32, addr: u32) -> Result<(), BusError> {
        if self.strict_clock_gating && clock_hz == 0 {
            return Err(BusError::Peripheral {
                addr,
                reason: "peripheral clock is disabled",
            });
        }
        Ok(())
    }

    /// Clock feeding a peripheral register window, used by strict gating.
    pub fn peripheral_clock(&self, addr: u32) -> u32 {
        let size = map::PERIPHERAL_WINDOW;
        let rcc = &self.rcc;
        let ip = |base| region_offset(addr, base, size).is_some();
        if ip(map::usart::BASE_1) {
            rcc.apb2_clock(1 << 14)
        } else if ip(map::usart::BASE_2) {
            rcc.apb1_clock(1 << 17)
        } else if ip(map::usart::BASE_3) {
            rcc.apb1_clock(1 << 18)
        } else if ip(map::spi::BASE_1) {
            rcc.apb2_clock(1 << 12)
        } else if ip(map::spi::BASE_2) {
            rcc.apb1_clock(1 << 14)
        } else if ip(map::tim::BASE_1) {
            rcc.tim1_clock()
        } else if ip(map::tim::BASE_2) || ip(map::tim::BASE_3) || ip(map::tim::BASE_4) {
            rcc.apb1_timer_clock(1 << 0)
        } else if ip(map::i2c::BASE_1) {
            rcc.apb1_clock(1 << 21)
        } else if ip(map::i2c::BASE_2) {
            rcc.apb1_clock(1 << 22)
        } else if ip(map::can::BASE_1) {
            rcc.apb1_clock(1 << 25)
        } else if ip(map::afio::BASE) {
            rcc.apb2_clock(1 << 0)
        } else if let Some(port) = map::gpio::PORTS.iter().position(|base| ip(*base)) {
            rcc.apb2_clock(1 << (port + 2))
        } else if ip(map::adc::BASE_1) {
            rcc.adc_clock(1 << 9)
        } else if ip(map::adc::BASE_2) {
            rcc.adc_clock(1 << 10)
        } else if ip(map::dma1::BASE) {
            rcc.dma_clock(1 << 0)
        } else if ip(map::dma2::BASE) {
            rcc.dma_clock(1 << 1)
        } else {
            1
        }
    }

    /// Checks strict clock gating before an access.
    fn gate(&self, addr: u32) -> Result<(), BusError> {
        self.strict_gate(self.peripheral_clock(addr), addr)
    }
}

impl MemoryBus for Soc {
    fn read(&mut self, addr: u32, width: AccessWidth) -> Result<u32, BusError> {
        if let Err(error) = check_alignment(addr, width) {
            self.stats.bus_errors += 1;
            return Err(error);
        }
        self.stats.reads += 1;
        match self
            .gate(addr)
            .and_then(|_| self.dispatch_read(addr, width))
        {
            Ok(value) => Ok(value),
            Err(error) => {
                self.stats.bus_errors += 1;
                Err(error)
            }
        }
    }

    fn write(&mut self, addr: u32, width: AccessWidth, value: u32) -> Result<(), BusError> {
        if let Err(error) = check_alignment(addr, width) {
            self.stats.bus_errors += 1;
            return Err(error);
        }
        self.stats.writes += 1;
        match self
            .gate(addr)
            .and_then(|_| self.dispatch_write(addr, width, value))
        {
            Ok(()) => Ok(()),
            Err(error) => {
                self.stats.bus_errors += 1;
                Err(error)
            }
        }
    }
}

/// Returns the offset of `addr` inside `[base, base + size)`.
fn region_offset(addr: u32, base: u32, size: u32) -> Option<u32> {
    if addr >= base && addr < base + size {
        Some(addr - base)
    } else {
        None
    }
}

/// Splits a timer DMA request into `(timer index, channel, is_update)`.
fn tim_request_parts(request: Request) -> (usize, usize, bool) {
    use Request::*;
    match request {
        Tim1Ch1 => (0, 0, false),
        Tim1Ch2 => (0, 1, false),
        Tim1Ch3 => (0, 2, false),
        Tim1Ch4 => (0, 3, false),
        Tim1Up | Tim1Trig | Tim1Com => (0, 0, true),
        Tim2Ch1 => (1, 0, false),
        Tim2Ch2 => (1, 1, false),
        Tim2Ch3 => (1, 2, false),
        Tim2Ch4 => (1, 3, false),
        Tim2Up => (1, 0, true),
        Tim3Ch1 => (2, 0, false),
        Tim3Ch3 => (2, 2, false),
        Tim3Ch4 => (2, 3, false),
        Tim3Up => (2, 0, true),
        Tim4Ch1 => (3, 0, false),
        Tim4Ch2 => (3, 1, false),
        Tim4Ch3 => (3, 2, false),
        Tim4Up => (3, 0, true),
        _ => (0, 0, true),
    }
}

/// Convenience: the interrupt line of a watchdog or DMA channel, for tests.
pub fn dma_channel_irq(controller: u8, channel: u8) -> gsim_core::Irq {
    gsim_core::Irq::new(dma::base_irq(controller) + u16::from(channel))
}

const _: () = {
    // Compile time guard: the SysTick exception number must stay consistent
    // with the NVIC contract used by `Event::Exception`.
    assert!(exception::SYSTICK == 15);
};
