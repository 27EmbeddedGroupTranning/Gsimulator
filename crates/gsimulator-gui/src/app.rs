//! Application shell: window layout, toolbar, status panel and the glue
//! between the simulation and the schematic.

use std::path::PathBuf;

use eframe::egui;
use egui::{Color32, RichText};

use crate::board::{
    Board, BoardState, DeviceKind, PinLevels, PinRef, Supply, JUNCTION_PIN, POWER_PIN,
};
use crate::bridge;
use crate::canbus::{CanAction, CanBusWindow};
use crate::canvas::Schematic;
use crate::debugger::DebugWindow;
use crate::library::DeviceLibrary;
use crate::motor_doc::{MotorLive, MotorProtocolWindow};
use crate::peripherals::PeripheralBrowser;
use crate::project;
use crate::scope::{ScopeAction, ScopeWindow};
use crate::sim::{Simulation, Snapshot, Source};
use crate::stimulus::{StimulusAction, StimulusWindow};
use gsim_devices::Direction;

/// Instructions executed per UI frame; the slider scales this.
const MIN_BUDGET: u32 = 5_000;
const MAX_BUDGET: u32 = 2_000_000;
const DEFAULT_BUDGET: u32 = 400_000;
/// Length of one device-servicing slice of simulated time.
const SLICE_NS: u64 = 1_000_000;
/// Instruction budget per frame while a debugger is attached: small enough that
/// a reply never waits long, large enough that the firmware still moves.
const DEBUG_BUDGET: u32 = 20_000;

pub struct App {
    sim: Simulation,
    board: Board,
    schematic: Schematic,
    snapshot: Snapshot,
    levels: PinLevels,
    board_state: BoardState,
    budget: u32,
    elf_path: Option<PathBuf>,
    /// Project file the schematic was last saved to or opened from.
    project_path: Option<PathBuf>,
    last_frame: Option<std::time::Instant>,
    /// Simulated seconds per real second, smoothed.
    speed: f64,
    /// The drawing is only auto-fitted once, so the user's pan/zoom survives.
    fitted: bool,
    show_status: bool,
    /// Window with the live state of every peripheral.
    peripherals: PeripheralBrowser,
    /// Dedicated window for the CAN transceiver and the bus it sits on.
    can_bus: CanBusWindow,
    /// Window showing the traces of the probes on the drawing.
    scope: ScopeWindow,
    /// Window with the pin/waveform settings of the built-in stimulus.
    stimulus: StimulusWindow,
    /// Window listing every part that can be placed on the canvas.
    library: DeviceLibrary,
    /// On/off switch and port of the built-in virtual ST-Link.
    debug: DebugWindow,
    /// Wire format document of the CAN motor it was opened from.
    motor_doc: MotorProtocolWindow,
    /// Text typed into each UART terminal, keyed by device id.
    uart_input: std::collections::BTreeMap<u32, String>,
    /// Speed typed for each CAN motor, keyed by device id.
    motor_rpm: std::collections::BTreeMap<u32, f32>,
}

impl App {
    /// Builds the app for `eframe`, loading the CJK font into the context.
    ///
    /// `gdb_port` starts the built-in virtual ST-Link right away, so a debugger
    /// can attach without touching the window first; `None` leaves it off until
    /// the 🐞 window switches it on.
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        elf_path: Option<PathBuf>,
        gdb_port: Option<u16>,
    ) -> Self {
        let mut app = App::with_context(&cc.egui_ctx, elf_path);
        if let Some(port) = gdb_port {
            match app.debug.start(port) {
                Ok(()) => eprintln!(
                    "虚拟 ST-Link 已在 127.0.0.1:{} 上等待调试器",
                    app.debug.port()
                ),
                Err(error) => eprintln!("{error}"),
            }
        }
        app
    }

    /// Same as [`App::new`] but with a bare [`egui::Context`], so tests can
    /// drive the whole UI headlessly.
    pub fn with_context(ctx: &egui::Context, elf_path: Option<PathBuf>) -> Self {
        install_fonts(ctx);
        ctx.set_visuals(egui::Visuals::dark());

        // A new project starts with the HSE crystal already placed and wired;
        // the other parts come from the library (示例电路 loads a demo).
        let board = Board::new_project();
        let mut sim = Simulation::new();
        sim.set_crystals(board.crystals());
        if let Some(path) = elf_path {
            if let Err(error) = sim.load_elf(&path) {
                eprintln!("{error}");
            }
        }
        let levels = sim.pin_levels();
        let board_state = board.evaluate(&levels);
        let mut snapshot = sim.snapshot();
        board.fill_snapshot(&mut snapshot, &levels);
        let stimulus = StimulusWindow::from_config(sim.stimulus());

        App {
            sim,
            board,
            schematic: Schematic::default(),
            snapshot,
            levels,
            board_state,
            budget: DEFAULT_BUDGET,
            elf_path: None,
            project_path: None,
            last_frame: None,
            speed: 0.0,
            fitted: false,
            show_status: true,
            peripherals: PeripheralBrowser::new(),
            can_bus: CanBusWindow::default(),
            scope: ScopeWindow::default(),
            stimulus,
            library: DeviceLibrary::new(),
            debug: DebugWindow::new(),
            motor_doc: MotorProtocolWindow::default(),
            uart_input: std::collections::BTreeMap::new(),
            motor_rpm: std::collections::BTreeMap::new(),
        }
    }

    /// The debug window, so tools and tests can start the service without
    /// clicking the toolbar.
    pub fn debug_window_mut(&mut self) -> &mut DebugWindow {
        &mut self.debug
    }

    fn load_firmware(&mut self) {
        let mut dialog = rfd::FileDialog::new().add_filter("ELF firmware", &["elf", "axf"]);
        if let Some(start) = self
            .elf_path
            .as_ref()
            .and_then(|path| path.parent().map(|parent| parent.to_path_buf()))
        {
            dialog = dialog.set_directory(start);
        }
        let Some(path) = dialog.pick_file() else {
            return;
        };
        match self.sim.load_elf(&path) {
            Ok(()) => {
                self.schematic
                    .say(format!("已加载 {}", path.display()), false);
                self.elf_path = Some(path);
            }
            Err(error) => self.schematic.say(error, true),
        }
    }

    /// Writes the schematic to a `.gsim` file chosen by the user.
    fn save_project(&mut self) {
        let mut dialog = rfd::FileDialog::new()
            .add_filter("Gsimulator 工程", &[project::EXTENSION])
            .set_file_name(format!("schematic.{}", project::EXTENSION));
        if let Some(directory) = self
            .project_path
            .as_ref()
            .and_then(|path| path.parent().map(|parent| parent.to_path_buf()))
        {
            dialog = dialog.set_directory(directory);
        }
        let Some(path) = dialog.save_file() else {
            return;
        };
        match self.save_project_to(&path) {
            Ok(()) => self
                .schematic
                .say(format!("已保存 {}", path.display()), false),
            Err(error) => self.schematic.say(error, true),
        }
    }

    /// Replaces the schematic with a `.gsim` file chosen by the user.
    fn open_project(&mut self) {
        let mut dialog = rfd::FileDialog::new()
            .add_filter("Gsimulator 工程", &[project::EXTENSION])
            .add_filter("所有文件", &["*"]);
        if let Some(directory) = self
            .project_path
            .as_ref()
            .and_then(|path| path.parent().map(|parent| parent.to_path_buf()))
        {
            dialog = dialog.set_directory(directory);
        }
        let Some(path) = dialog.pick_file() else {
            return;
        };
        match self.open_project_from(&path) {
            Ok(()) => self
                .schematic
                .say(format!("已打开 {}", path.display()), false),
            Err(error) => self.schematic.say(error, true),
        }
    }

    /// Saves the schematic to `path` and remembers it as the current project.
    pub fn save_project_to(&mut self, path: &std::path::Path) -> Result<(), String> {
        project::save(path, &self.board).map_err(|error| format!("保存失败：{error}"))?;
        self.project_path = Some(path.to_path_buf());
        Ok(())
    }

    /// Loads `path` over the current schematic; the board stays untouched when
    /// the file cannot be parsed.
    pub fn open_project_from(&mut self, path: &std::path::Path) -> Result<(), String> {
        let board = project::open(path)?;
        self.board = board;
        self.schematic.reset_interaction();
        self.uart_input.clear();
        // Crystals of the loaded board take effect immediately.
        self.sim.set_crystals(self.board.crystals());
        self.schematic.fit(self.schematic.last_rect, &self.board);
        self.project_path = Some(path.to_path_buf());
        Ok(())
    }

    fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            let running = self.snapshot.running;
            if ui
                .button(if running { "⏸ 暂停" } else { "▶ 运行" })
                .on_hover_text("空格键也可以切换")
                .clicked()
            {
                self.sim.toggle_running();
            }
            if ui.button("⟲ 复位").clicked() {
                self.sim.reset();
            }
            if ui
                .button("⏭ 单步")
                .on_hover_text("推进一小步（暂停时也可用）")
                .clicked()
            {
                self.sim.step_once(10_000);
            }
            ui.separator();
            if ui.button("📂 加载 ELF…").clicked() {
                self.load_firmware();
            }
            let stimulus_on = matches!(self.sim.source(), Source::Stimulus);
            let stimulus_label = self.sim.stimulus().label();
            if ui
                .selectable_label(stimulus_on, "内置激励")
                .on_hover_text(if stimulus_on {
                    format!("{stimulus_label}，点击关闭并释放引脚")
                } else {
                    format!("{stimulus_label}，不需要固件")
                })
                .clicked()
            {
                if stimulus_on {
                    let pin = self.sim.stimulus().pin_label();
                    self.sim.stop_stimulus();
                    self.schematic
                        .say(format!("已关闭内置激励，{pin} 已释放"), false);
                } else {
                    self.sim.start_stimulus();
                }
            }
            if ui
                .button("⚙ 激励")
                .on_hover_text("配置内置激励的引脚、波形、半周期、占空比或自定义函数")
                .clicked()
            {
                self.stimulus.toggle();
            }
            ui.separator();
            if ui
                .selectable_label(self.library.open, "＋ 添加器件")
                .on_hover_text("打开器件库：挑选 LED、串口终端、旋转编码器后放到画布上")
                .clicked()
            {
                self.library.toggle();
            }
            if ui
                .button("💾 保存工程…")
                .on_hover_text(
                    "把当前原理图（器件、连线、拐点、器件选项）存成 .gsim 文件（Ctrl+S）",
                )
                .clicked()
            {
                self.save_project();
            }
            if ui
                .button("📂 打开工程…")
                .on_hover_text("读取 .gsim 工程文件，替换当前原理图（Ctrl+O）")
                .clicked()
            {
                self.open_project();
            }
            if ui.button("清空连线").clicked() {
                self.board.clear_wires();
                self.schematic.cancel_wire();
            }
            ui.separator();
            if ui
                .selectable_label(self.peripherals.open, "🔍 外设浏览")
                .on_hover_text("查看所有片上外设与两个外部器件的实时状态")
                .clicked()
            {
                self.peripherals.toggle();
            }
            if ui
                .selectable_label(self.scope.open, "📈 示波器")
                .on_hover_text("查看探头抓到的 CANH/CANL 波形")
                .clicked()
            {
                self.scope.toggle();
            }
            if ui
                .selectable_label(self.can_bus.open, "🚌 CAN 总线")
                .on_hover_text("CAN 收发器与总线状态、帧记录和注入")
                .clicked()
            {
                self.can_bus.toggle();
            }
            let debug_label = if self.debug.is_listening() {
                format!("🐞 调试服务 :{} ●", self.debug.port())
            } else {
                "🐞 调试服务".to_string()
            };
            if ui
                .selectable_label(self.debug.open, debug_label)
                .on_hover_text(
                    "虚拟 ST-Link 的开关与端口（启动时已在 3333 监听）：让 GDB / VS Code 调试这份正在运行的仿真",
                )
                .clicked()
            {
                self.debug.toggle();
            }
            ui.separator();
            ui.spacing_mut().slider_width = 90.0;
            ui.add(
                egui::Slider::new(&mut self.budget, MIN_BUDGET..=MAX_BUDGET)
                    .logarithmic(true)
                    .custom_formatter(|value, _| format!("{} 指令/帧", value as u32)),
            );
        });
    }

    fn status_panel(&mut self, ui: &mut egui::Ui) {
        let snapshot = self.snapshot.clone();
        egui::ScrollArea::vertical()
            .id_salt("status-panel")
            .show(ui, |ui| {
                ui.heading("仿真状态");
                // An idle simulator has no "source" line at all.
                if !matches!(snapshot.source, Source::Idle) {
                    ui.label(
                        RichText::new(snapshot.source_label.as_str())
                            .color(if snapshot.running {
                                Color32::from_rgb(120, 220, 140)
                            } else {
                                Color32::from_rgb(220, 180, 90)
                            })
                            .strong(),
                    );
                }
                if let Some(halt) = &snapshot.halt {
                    ui.colored_label(Color32::from_rgb(240, 120, 120), halt);
                }
                ui.add_space(4.0);

                egui::Grid::new("sim-stats")
                    .num_columns(2)
                    .spacing([12.0, 3.0])
                    .show(ui, |ui| {
                        let mut row = |name: &str, value: String| {
                            ui.label(RichText::new(name).color(Color32::GRAY));
                            ui.monospace(value);
                            ui.end_row();
                        };
                        row("仿真时间", format!("{:.3} ms", snapshot.elapsed_ms()));
                        row(
                            "实时倍率",
                            if self.speed > 0.0 {
                                format!("{:.2} ×", self.speed)
                            } else {
                                "—".to_string()
                            },
                        );
                        row("已执行指令", format!("{}", snapshot.instructions));
                        if let Some(pc) = snapshot.pc {
                            row("PC", format!("0x{pc:08X}"));
                        }
                        row("中断次数", format!("{}", snapshot.interrupts));
                        row("复位次数", format!("{}", snapshot.resets));
                        row(
                            "总线访问",
                            format!("{}", snapshot.stats.reads + snapshot.stats.writes),
                        );
                        row("总线错误", format!("{}", snapshot.stats.bus_errors));
                    });

                ui.add_space(12.0);
                ui.heading("时钟树");
                egui::Grid::new("clocks")
                    .num_columns(2)
                    .spacing([12.0, 3.0])
                    .show(ui, |ui| {
                        let mut row = |name: &str, hz: u32| {
                            ui.label(RichText::new(name).color(Color32::GRAY));
                            ui.monospace(format_hz(hz));
                            ui.end_row();
                        };
                        row("SYSCLK", snapshot.clocks.sysclk_hz);
                        row("HCLK", snapshot.clocks.hclk_hz);
                        row("PCLK1", snapshot.clocks.pclk1_hz);
                        row("PCLK2", snapshot.clocks.pclk2_hz);
                        row("TIM2", snapshot.clocks.tim2_hz);
                        row("ADC", snapshot.clocks.adc_hz);
                    });

                if let Some(stimulus) = &snapshot.stimulus {
                    ui.add_space(12.0);
                    ui.heading("内置激励");
                    ui.horizontal(|ui| {
                        ui.monospace(stimulus.config.pin_label());
                        let (text, color) = if stimulus.high {
                            ("高电平", Color32::from_rgb(90, 226, 120))
                        } else {
                            ("低电平", Color32::from_rgb(140, 150, 164))
                        };
                        ui.colored_label(color, text);
                    });
                    ui.label(
                        RichText::new(stimulus.config.waveform.label())
                            .small()
                            .color(Color32::GRAY),
                    );
                    if let Some(next) = stimulus.next_toggle_ns {
                        let remaining = next.saturating_sub(snapshot.now_ns) as f64 / 1e6;
                        ui.label(format!("{remaining:.1} ms 后翻转"));
                    }
                }

                ui.add_space(12.0);
                ui.heading("引脚电平");
                self.pin_matrix(ui, &snapshot.levels);

                ui.add_space(12.0);
                self.device_list(ui);

                ui.add_space(12.0);
                ui.horizontal(|ui| {
                    ui.heading("USART1 输出");
                    if ui.small_button("清空").clicked() {
                        self.sim.clear_uart();
                    }
                });
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_min_height(90.0);
                    ui.set_min_width(260.0);
                    egui::ScrollArea::vertical()
                        .id_salt("usart1-console")
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            let text = if snapshot.uart.is_empty() {
                                "（固件把字节写到 USART1 时会显示在这里）".to_string()
                            } else {
                                snapshot.uart.clone()
                            };
                            ui.monospace(text);
                        });
                });
                ui.add_space(8.0);
                ui.label(
                    RichText::new(
                        "提示：＋ 添加器件 打开器件库（也可把卡片拖到画布）；点击引脚 → 点击另一个引脚 完成连线；\
                     拖动线段 拉出拐点；双击连线 加接线节点（双击节点删除）；Delete 删除选中；中键拖动平移；滚轮缩放。",
                    )
                    .color(Color32::GRAY)
                    .small(),
                );
            });
    }

    fn pin_matrix(&mut self, ui: &mut egui::Ui, levels: &PinLevels) {
        for (port, name) in gsim_stm32f1::PORT_NAMES.iter().enumerate() {
            let pins: Vec<u8> = match port {
                0 | 1 => (0..16).collect(),
                2 => vec![13, 14, 15],
                // OSC_IN / OSC_OUT of the HSE crystal.
                3 => vec![0, 1],
                _ => Vec::new(),
            };
            if pins.is_empty() {
                continue;
            }
            ui.horizontal(|ui| {
                ui.label(RichText::new(format!("P{name}")).color(Color32::GRAY));
                ui.horizontal_wrapped(|ui| {
                    ui.spacing_mut().item_spacing.x = 2.0;
                    for pin in &pins {
                        let level = levels.level_of(port as u8, *pin);
                        let output = levels.is_output(port as u8, *pin);
                        let color = if level {
                            Color32::from_rgb(90, 226, 120)
                        } else {
                            Color32::from_rgb(64, 70, 80)
                        };
                        let text_color = if level {
                            Color32::BLACK
                        } else {
                            Color32::from_rgb(150, 158, 170)
                        };
                        let label = RichText::new(format!("{pin:>2}"))
                            .monospace()
                            .color(text_color)
                            .background_color(color);
                        let response = ui
                            .label(if output { label.strong() } else { label })
                            .on_hover_text(format!(
                                "P{name}{pin}：{}，{}",
                                if level { "高电平" } else { "低电平" },
                                if output { "输出" } else { "输入" }
                            ));
                        if response.clicked() {
                            self.start_wire_from(PinRef::Mcu {
                                port: port as u8,
                                pin: *pin,
                            });
                        }
                    }
                });
            });
        }
        ui.horizontal(|ui| {
            ui.label(RichText::new("电源").color(Color32::GRAY));
            for (kind, color) in [
                (DeviceKind::Vcc, Color32::from_rgb(200, 90, 90)),
                (DeviceKind::Gnd, Color32::from_rgb(110, 135, 175)),
            ] {
                let Some(device) = self.board.first_of(kind) else {
                    ui.label(
                        RichText::new(format!("未放置 {}", kind.label()))
                            .small()
                            .color(Color32::GRAY),
                    );
                    continue;
                };
                if ui
                    .add(egui::Button::new(RichText::new(kind.label()).small()).fill(color))
                    .on_hover_text("从这个电源符号开始连线")
                    .clicked()
                {
                    self.start_wire_from(PinRef::Device {
                        device: device.id,
                        pin: POWER_PIN,
                    });
                }
            }
        });
    }

    fn start_wire_from(&mut self, pin: PinRef) {
        self.schematic.say(
            format!("已选择 {}，点击第二个引脚完成连线", pin.label(&self.board)),
            false,
        );
        self.schematic.request_wire_from(pin);
    }

    fn device_list(&mut self, ui: &mut egui::Ui) {
        ui.heading("虚拟外设");
        if self.board.devices().is_empty() {
            ui.label(
                RichText::new("还没有外设，点工具栏的「＋ 添加器件」从器件库放置")
                    .color(Color32::GRAY),
            );
            return;
        }
        let ids: Vec<u32> = self
            .board
            .devices()
            .iter()
            .map(|device| device.id)
            .collect();
        let mut remove = None;
        for id in ids {
            let Some(device) = self.board.device(id) else {
                continue;
            };
            let kind = device.kind;
            let accent = device.accent.color32();
            let name = device.name.clone();
            let wiring = |pin: u8| {
                PinRef::Device { device: id, pin }
                    .label(&self.board)
                    .split('.')
                    .nth(1)
                    .map(str::to_string)
                    .unwrap_or_default()
            };
            let connected_to = |pin: u8| -> Option<String> {
                let pin_ref = PinRef::Device { device: id, pin };
                self.board
                    .wires()
                    .iter()
                    .find(|wire| wire.a == pin_ref || wire.b == pin_ref)
                    .map(|wire| {
                        let other = if wire.a == pin_ref { wire.b } else { wire.a };
                        other.label(&self.board)
                    })
            };

            ui.horizontal(|ui| {
                match kind {
                    DeviceKind::Led => {
                        let brightness = self.board_state.led_brightness(id);
                        let (text, color) = match (brightness * 100.0).round() as u32 {
                            0 => ("熄灭".to_string(), Color32::from_rgb(110, 118, 130)),
                            100 => ("点亮".to_string(), accent),
                            percent => (format!("调光 {percent}%"), accent),
                        };
                        ui.label(RichText::new(format!("● {name}")).color(color));
                        ui.label(RichText::new(text).color(Color32::GRAY));
                    }
                    DeviceKind::Uart => {
                        let unit = device.hardware.uart().map(|uart| uart.unit()).unwrap_or(0);
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        ui.label(RichText::new(format!("USART{}", unit + 1)).color(Color32::GRAY));
                    }
                    DeviceKind::Encoder => {
                        let detents = device
                            .hardware
                            .encoder()
                            .map(|encoder| encoder.detents())
                            .unwrap_or(0);
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        ui.label(RichText::new(format!("位置 {detents:+}")).color(Color32::GRAY));
                    }
                    DeviceKind::Vcc => {
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        ui.label(RichText::new("高电平").color(Color32::GRAY));
                    }
                    DeviceKind::Gnd => {
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        ui.label(RichText::new("低电平").color(Color32::GRAY));
                    }
                    DeviceKind::Can => {
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        let (mode, frames) = device
                            .hardware
                            .can()
                            .map(|transceiver| {
                                let transceiver = transceiver.borrow();
                                (transceiver.mode().label(), transceiver.stats().tx_frames)
                            })
                            .unwrap_or(("正常", 0));
                        ui.label(
                            RichText::new(format!("{mode} · 已发 {frames}")).color(Color32::GRAY),
                        );
                    }
                    DeviceKind::Matrix => {
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        let words = device
                            .hardware
                            .matrix()
                            .map(|matrix| matrix.borrow().words())
                            .unwrap_or(0);
                        ui.label(RichText::new(format!("已锁存 {words} 字")).color(Color32::GRAY));
                    }
                    DeviceKind::HseCrystal | DeviceKind::LseCrystal => {
                        let crystals = self.board.crystals();
                        let (wired, running) = match kind {
                            DeviceKind::HseCrystal => {
                                (crystals.hse_hz.is_some(), self.sim.hse_ready())
                            }
                            _ => (crystals.lse, self.sim.lse_ready()),
                        };
                        let frequency = device
                            .hardware
                            .crystal()
                            .map(|crystal| crystal.frequency_label())
                            .unwrap_or_default();
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        let text = if running {
                            format!("{frequency} · 起振中")
                        } else if wired {
                            format!("{frequency} · 已接到振荡脚")
                        } else {
                            format!("{frequency} · 未接到振荡脚")
                        };
                        ui.label(RichText::new(text).color(Color32::GRAY));
                    }
                    DeviceKind::Resistor => {
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        let ohms = device
                            .hardware
                            .resistor()
                            .map(|resistor| resistor.ohms())
                            .unwrap_or_default();
                        let network = self.board.can_network();
                        let text = if network.terminator_ids.contains(&id) {
                            format!(
                                "{} · 终端 · 并联 {}",
                                gsim_devices::format_ohms(ohms),
                                crate::board::termination_label(network.termination_ohm)
                            )
                        } else {
                            format!("{} · 未跨接 CANH/CANL", gsim_devices::format_ohms(ohms))
                        };
                        ui.label(RichText::new(text).color(Color32::GRAY));
                    }
                    DeviceKind::Scope => {
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        let (samples, window) = device
                            .hardware
                            .scope()
                            .map(|scope| {
                                let scope = scope.borrow();
                                (scope.samples().len(), scope.window_ns())
                            })
                            .unwrap_or((0, 0));
                        ui.label(
                            RichText::new(format!("{samples} 点 · {:.2} ms", window as f64 / 1e6))
                                .color(Color32::GRAY),
                        );
                    }
                    DeviceKind::Motor => {
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        let (spin, rpm, target, commands) = device
                            .hardware
                            .motor()
                            .map(|motor| {
                                let motor = motor.borrow();
                                (
                                    motor.spin().label(),
                                    motor.rpm(),
                                    motor.target_rpm(),
                                    motor.commands(),
                                )
                            })
                            .unwrap_or(("停转", 0.0, 0.0, 0));
                        let color = if spin == "停转" {
                            Color32::from_rgb(140, 148, 160)
                        } else {
                            accent
                        };
                        ui.label(
                            RichText::new(format!(
                                "{spin} · {rpm:+.0} rpm（目标 {target:+.0}）· 指令 {commands}"
                            ))
                            .color(color),
                        );
                    }
                    DeviceKind::Junction => {
                        let pin = PinRef::Device {
                            device: id,
                            pin: JUNCTION_PIN,
                        };
                        let wires = self
                            .board
                            .wires()
                            .iter()
                            .filter(|wire| wire.a == pin || wire.b == pin)
                            .count();
                        ui.label(RichText::new(format!("● {name}")).color(accent));
                        ui.label(
                            RichText::new(format!("接线节点 · {wires} 根连线"))
                                .color(Color32::GRAY),
                        );
                    }
                }
                if let Some(supply) = self.board_state.supply(id) {
                    let color = match supply {
                        Supply::Powered => Color32::from_rgb(120, 200, 130),
                        Supply::Unwired => Color32::from_rgb(240, 190, 90),
                        Supply::Fault => Color32::from_rgb(240, 120, 110),
                    };
                    ui.label(RichText::new(supply.label()).small().color(color));
                }
                if ui.small_button("🗑").clicked() {
                    remove = Some(id);
                }
            });

            let pins: Vec<u8> = (0..kind.pins().len() as u8).collect();
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                for pin in pins {
                    let label = kind.pins()[pin as usize].name;
                    let text = match connected_to(pin) {
                        Some(other) => format!("{label}←{other}"),
                        None => format!("{label} 未连接"),
                    };
                    ui.label(RichText::new(text).small().color(Color32::GRAY));
                }
                let _ = wiring(0);
            });

            match kind {
                DeviceKind::Uart => self.uart_controls(ui, id),
                DeviceKind::Encoder => self.encoder_controls(ui, id),
                DeviceKind::Resistor => self.resistor_controls(ui, id),
                DeviceKind::Scope => self.scope_controls(ui, id),
                DeviceKind::Motor => self.motor_controls(ui, id),
                DeviceKind::Led
                | DeviceKind::Vcc
                | DeviceKind::Gnd
                | DeviceKind::Can
                | DeviceKind::Matrix
                | DeviceKind::HseCrystal
                | DeviceKind::LseCrystal
                | DeviceKind::Junction => {}
            }
        }
        if let Some(id) = remove {
            if let Some(device) = self.board.device(id) {
                let name = device.name.clone();
                self.board.remove_device(id);
                self.uart_input.remove(&id);
                self.schematic.say(format!("已删除 {name}"), false);
            }
        }

        if !self.board_state.shorts.is_empty() {
            ui.colored_label(
                Color32::from_rgb(240, 160, 70),
                format!(
                    "⚠ {} 个网络短路（VCC 与 GND 或两个输出连在一起）",
                    self.board_state.shorts.len()
                ),
            );
        }
    }

    /// Port choice plus the send console of one UART terminal.
    fn uart_controls(&mut self, ui: &mut egui::Ui, id: u32) {
        let Some(device) = self.board.device(id) else {
            return;
        };
        let Some(uart) = device.hardware.uart() else {
            return;
        };
        let unit = uart.unit();
        let rx_bytes = uart.rx_bytes();
        let tx_bytes = uart.tx_bytes();
        let pending = uart.pending_tx();
        let log = uart.rx_text().to_string();

        ui.horizontal(|ui| {
            ui.label(RichText::new("端口").small().color(Color32::GRAY));
            for candidate in 0..3u8 {
                if ui
                    .selectable_label(unit == candidate, format!("USART{}", candidate + 1))
                    .clicked()
                {
                    if let Some(uart) = self
                        .board
                        .device_mut(id)
                        .and_then(|device| device.hardware.uart_mut())
                    {
                        uart.set_unit(candidate);
                    }
                }
            }
            ui.label(
                RichText::new(format!("↓{rx_bytes} ↑{tx_bytes}"))
                    .small()
                    .color(Color32::GRAY),
            );
        });

        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            egui::ScrollArea::vertical()
                .max_height(72.0)
                .stick_to_bottom(true)
                .id_salt(("uart-log", id))
                .show(ui, |ui| {
                    let text = if log.is_empty() {
                        "（MCU 通过 TX 发来的字节会显示在这里）".to_string()
                    } else {
                        log
                    };
                    ui.monospace(RichText::new(text).size(11.0));
                });
        });

        let buffer = self.uart_input.entry(id).or_default();
        let mut send = false;
        ui.horizontal(|ui| {
            let response = ui.add(
                egui::TextEdit::singleline(buffer)
                    .hint_text("输入要发给 MCU 的内容")
                    .desired_width(150.0),
            );
            if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                send = true;
            }
            if ui.button("发送").clicked() {
                send = true;
            }
            if ui.small_button("清空").clicked() {
                if let Some(uart) = self
                    .board
                    .device_mut(id)
                    .and_then(|device| device.hardware.uart_mut())
                {
                    uart.clear();
                }
            }
        });
        if send && !buffer.is_empty() {
            let text = format!("{buffer}\r\n");
            if let Some(uart) = self
                .board
                .device_mut(id)
                .and_then(|device| device.hardware.uart_mut())
            {
                uart.send(&text);
                buffer.clear();
            }
        }
        if pending > 0 {
            ui.label(
                RichText::new(format!("{pending} 字节待发送（等待连线）"))
                    .small()
                    .color(Color32::from_rgb(240, 190, 90)),
            );
        }
    }

    /// Resistance of one resistor, and what it does to the CAN pair.
    fn resistor_controls(&mut self, ui: &mut egui::Ui, id: u32) {
        let Some(ohms) = self
            .board
            .device(id)
            .and_then(|device| device.hardware.resistor())
            .map(|resistor| resistor.ohms())
        else {
            return;
        };
        let network = self.board.can_network();
        let on_pair = network.terminator_ids.contains(&id);

        let mut value = ohms;
        let mut changed = false;
        ui.horizontal(|ui| {
            ui.label(RichText::new("阻值").small().color(Color32::GRAY));
            for preset in gsim_devices::Resistor::PRESETS {
                let label = gsim_devices::format_ohms(preset);
                if ui
                    .selectable_label((ohms - preset).abs() < 0.5, label)
                    .clicked()
                {
                    value = preset;
                    changed = true;
                }
            }
            if ui
                .add(
                    egui::DragValue::new(&mut value)
                        .speed(1.0)
                        .range(0.1..=1_000_000.0)
                        .suffix(" Ω"),
                )
                .changed()
            {
                changed = true;
            }
        });
        if changed {
            self.set_resistor_value(id, value);
        }

        let (text, color) = if !on_pair {
            (
                "1、2 两端没有跨接在 CANH 与 CANL 之间：它只是普通电阻，不影响总线".to_string(),
                Color32::from_rgb(150, 158, 170),
            )
        } else {
            let quality = network.quality();
            let text = format!(
                "并联终端 {} → 总线质量 {:.0}% · 位错误率 {:.3}%/位{}",
                crate::board::termination_label(network.termination_ohm),
                quality * 100.0,
                gsim_devices::bit_error_rate(quality) * 100.0,
                if quality > 0.99 {
                    "（教科书接法）"
                } else if quality > 0.5 {
                    "（能用，但偶发振铃丢帧）"
                } else {
                    "（严重失配，帧会被判错）"
                }
            );
            let color = if quality > 0.9 {
                Color32::from_rgb(120, 200, 130)
            } else if quality > 0.4 {
                Color32::from_rgb(240, 190, 90)
            } else {
                Color32::from_rgb(240, 120, 110)
            };
            (text, color)
        };
        ui.label(RichText::new(text).small().color(color));
    }

    /// Sets the resistance of one resistor and refreshes the snapshot.
    fn set_resistor_value(&mut self, id: u32, ohms: f64) {
        let Some(resistor) = self
            .board
            .device_mut(id)
            .and_then(|device| device.hardware.resistor_mut())
        else {
            return;
        };
        resistor.set_ohms(ohms);
        self.refresh_snapshot();
    }

    /// Shortcut from a probe row to its window.
    fn scope_controls(&mut self, ui: &mut egui::Ui, id: u32) {
        let Some(scope) = self
            .board
            .device(id)
            .and_then(|device| device.hardware.scope())
            .cloned()
        else {
            return;
        };
        ui.horizontal(|ui| {
            if ui
                .button("打开示波器")
                .on_hover_text("查看这个探头抓到的 CANH/CANL 波形")
                .clicked()
            {
                self.scope.open = true;
            }
            let mut frozen = scope.borrow().frozen();
            if ui.checkbox(&mut frozen, "冻结").changed() {
                scope.borrow_mut().set_frozen(frozen);
            }
            if ui.small_button("清空").clicked() {
                scope.borrow_mut().clear();
            }
        });
    }

    /// Command identifier plus a shortcut that puts a real command frame on
    /// the bus, so the motor is driven the same way the firmware drives it.
    fn motor_controls(&mut self, ui: &mut egui::Ui, id: u32) {
        let Some(motor) = self
            .board
            .device(id)
            .and_then(|device| device.hardware.motor())
            .cloned()
        else {
            return;
        };
        let command_id = motor.borrow().command_id();
        let on_pair = self
            .board
            .can_network()
            .motors
            .iter()
            .any(|(motor_id, _)| *motor_id == id);
        let mut rpm = self.motor_rpm.get(&id).copied().unwrap_or(1200.0);

        let mut command = command_id;
        let mut send = None;
        ui.horizontal(|ui| {
            ui.label(RichText::new("指令 ID").small().color(Color32::GRAY));
            if ui
                .add(
                    egui::DragValue::new(&mut command)
                        .range(0..=0x7FF)
                        .speed(1.0)
                        .custom_formatter(|value, _| format!("0x{:03X}", value as u32)),
                )
                .changed()
            {
                motor.borrow_mut().set_command_id(command);
            }
            ui.separator();
            ui.label(RichText::new("转速").small().color(Color32::GRAY));
            ui.add(
                egui::DragValue::new(&mut rpm)
                    .speed(20.0)
                    .range(-gsim_devices::motor::MAX_RPM..=gsim_devices::motor::MAX_RPM)
                    .suffix(" rpm"),
            );
            if ui.small_button("正转 ▶").clicked() {
                send = Some(rpm.abs());
            }
            if ui.small_button("◀ 反转").clicked() {
                send = Some(-rpm.abs());
            }
            if ui.small_button("停止").clicked() {
                send = Some(0.0);
            }
        });
        self.motor_rpm.insert(id, rpm);
        if let Some(speed) = send {
            self.send_motor_command(id, speed);
        }
        let (text, color) = if on_pair {
            (
                format!(
                    "正在听 0x{command_id:03X}：data[0..2] 是大端有符号转速；上面的按钮让外部节点真的发一帧"
                ),
                Color32::from_rgb(120, 200, 130),
            )
        } else {
            (
                "CANH/CANL 没有和收发器接在同一对线上：收不到任何指令".to_string(),
                Color32::from_rgb(240, 190, 90),
            )
        };
        ui.horizontal_wrapped(|ui| {
            if ui
                .small_button("📖 通讯协议")
                .on_hover_text("在应用内查看这台电机收发的 CAN 报文格式")
                .clicked()
            {
                self.motor_doc.open_for(id);
            }
            ui.label(RichText::new(text).small().color(color));
        });
    }

    /// State the protocol document shows for the part it was opened from.
    fn motor_live(&self) -> Option<MotorLive> {
        let id = self.motor_doc.device()?;
        let device = self.board.device(id)?;
        let motor = device.hardware.motor()?.clone();
        let motor = motor.borrow();
        Some(MotorLive {
            name: device.name.clone(),
            command_id: motor.command_id(),
            target_rpm: motor.target_rpm(),
            rpm: motor.rpm(),
            frames: motor.frames(),
            commands: motor.commands(),
            enabled: motor.enabled(),
        })
    }

    /// Puts a speed command on the wire as the external node.
    fn send_motor_command(&mut self, id: u32, rpm: f32) {
        let Some(motor) = self
            .board
            .device(id)
            .and_then(|device| device.hardware.motor())
            .cloned()
        else {
            return;
        };
        let command_id = motor.borrow().command_id();
        let speed = rpm.round().clamp(-32767.0, 32767.0) as i16;
        let [high, low] = speed.to_be_bytes();
        let frame = gsim_core::CanFrame::standard(command_id, &[high, low]);
        let now = self.sim.now_ns();
        self.board
            .can_bus()
            .borrow_mut()
            .inject_bit_frame(frame, now);
        self.schematic.say(
            format!("外部节点发送电机指令 0x{command_id:03X}：{speed} rpm"),
            false,
        );
    }

    /// Manual control of one rotary encoder.
    fn encoder_controls(&mut self, ui: &mut egui::Ui, id: u32) {
        let Some(device) = self.board.device(id) else {
            return;
        };
        let Some(encoder) = device.hardware.encoder() else {
            return;
        };
        let detents = encoder.detents();
        let pressed = encoder.button();
        let pending = encoder.pending_edges();
        let now_ns = self.sim.now_ns();

        ui.horizontal(|ui| {
            if ui.small_button("◀ 逆时针").clicked() {
                self.rotate_encoder(id, Direction::CounterClockwise, now_ns);
            }
            if ui.small_button("顺时针 ▶").clicked() {
                self.rotate_encoder(id, Direction::Clockwise, now_ns);
            }
            if ui
                .small_button(if pressed { "松开轴" } else { "按下轴" })
                .clicked()
            {
                if let Some(encoder) = self
                    .board
                    .device_mut(id)
                    .and_then(|device| device.hardware.encoder_mut())
                {
                    encoder.press(!pressed);
                }
            }
            if pending > 0 {
                ui.label(
                    RichText::new(format!("{pending} 个边沿排队"))
                        .small()
                        .color(Color32::GRAY),
                );
            }
            let _ = detents;
        });
        ui.label(
            RichText::new("画布上：滚轮转旋钮，点击按下轴")
                .small()
                .color(Color32::GRAY),
        );
    }

    fn rotate_encoder(&mut self, id: u32, direction: Direction, now_ns: u64) {
        if let Some(encoder) = self
            .board
            .device_mut(id)
            .and_then(|device| device.hardware.encoder_mut())
        {
            encoder.rotate(direction, 1, now_ns);
        }
    }

    /// Takes a fresh frame and completes it with the parts placed on the board.
    fn refresh_snapshot(&mut self) {
        self.snapshot = self.sim.snapshot();
        let levels = self.snapshot.levels.clone();
        self.board.fill_snapshot(&mut self.snapshot, &levels);
    }

    /// Applies a command issued from the CAN bus window.
    fn apply_can_action(&mut self, action: CanAction) {
        match action {
            CanAction::Inject(frame) => {
                let now = self.sim.now_ns();
                // The frame really travels on the wire, bit by bit, so it
                // arbitrates against the controller instead of being pasted
                // into its FIFO.
                self.board
                    .can_bus()
                    .borrow_mut()
                    .inject_bit_frame(frame, now);
                self.schematic.say(
                    format!(
                        "外部节点开始发送 {} {}",
                        frame.id_label(),
                        frame.data_label()
                    ),
                    false,
                );
            }
            CanAction::ClearLog => self.board.can_bus().borrow_mut().clear_records(),
            CanAction::OpenScope => {
                self.scope.open = true;
                self.refresh_snapshot();
            }
            CanAction::SetMode(mode) => {
                let mut changed = false;
                for transceiver in self.board.transceivers() {
                    transceiver.borrow_mut().set_mode(mode);
                    changed = true;
                }
                if changed {
                    self.schematic
                        .say(format!("CAN 收发器切换到 {}", mode.label()), false);
                } else {
                    self.schematic
                        .say("画布上还没有 CAN 收发器，先从器件库放置一个", true);
                }
            }
        }
        self.refresh_snapshot();
    }

    /// Applies a command issued from the oscilloscope window.
    fn apply_scope_action(&mut self, action: ScopeAction) {
        let scopes = self.board.scopes();
        match action {
            ScopeAction::SetWindow(index, window_ns) => {
                if let Some(scope) = scopes.get(index) {
                    scope.borrow_mut().set_window_ns(window_ns);
                }
            }
            ScopeAction::SetFrozen(index, frozen) => {
                if let Some(scope) = scopes.get(index) {
                    scope.borrow_mut().set_frozen(frozen);
                }
            }
            ScopeAction::Clear(index) => {
                if let Some(scope) = scopes.get(index) {
                    scope.borrow_mut().clear();
                }
            }
        }
        self.refresh_snapshot();
    }

    fn apply_stimulus_action(&mut self, action: StimulusAction) {
        match action {
            StimulusAction::Apply { config, run } => match self.sim.set_stimulus_config(config) {
                Ok(()) => {
                    if run {
                        self.sim.start_stimulus();
                    }
                    self.refresh_snapshot();
                    self.schematic.say("已应用内置激励设置", false);
                }
                Err(error) => self.schematic.say(error, true),
            },
            StimulusAction::Stop => {
                self.sim.stop_stimulus();
                self.refresh_snapshot();
                self.schematic
                    .say("已停止内置激励，引脚已释放（仿真时钟继续运行）", false);
            }
        }
    }

    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        let (space, reset, save, open) = ctx.input(|i| {
            (
                i.key_pressed(egui::Key::Space),
                i.key_pressed(egui::Key::R) && !i.modifiers.ctrl,
                i.key_pressed(egui::Key::S) && i.modifiers.command,
                i.key_pressed(egui::Key::O) && i.modifiers.command,
            )
        });
        if space {
            self.sim.toggle_running();
        }
        if reset {
            self.sim.reset();
        }
        if save {
            self.save_project();
        }
        if open {
            self.open_project();
        }
    }

    /// Simulated time of the last frame, exposed for the headless UI test.
    pub fn sim_time_ns(&self) -> u64 {
        self.snapshot.now_ns
    }

    /// Pin levels drawn in the last frame, exposed for the headless UI tests.
    pub fn levels(&self) -> &PinLevels {
        &self.levels
    }

    /// The schematic currently on the canvas.
    pub fn board(&self) -> &Board {
        &self.board
    }

    /// Mutable access to the circuit, so tests can place and tweak parts.
    pub fn board_mut(&mut self) -> &mut Board {
        &mut self.board
    }

    /// Refreshes the frame snapshot from the simulation and the drawing.
    pub fn refresh(&mut self) {
        self.refresh_snapshot();
    }

    /// Device library window, exposed for the headless UI test.
    pub fn library_mut(&mut self) -> &mut DeviceLibrary {
        &mut self.library
    }

    /// Protocol document of the CAN motor, exposed for the headless UI test.
    pub fn motor_protocol_mut(&mut self) -> &mut MotorProtocolWindow {
        &mut self.motor_doc
    }

    fn update_speed(&mut self, now: std::time::Instant, before_ns: u64) {
        if let Some(last) = self.last_frame {
            let real = now.duration_since(last).as_secs_f64();
            let simulated = self.snapshot.now_ns.saturating_sub(before_ns) as f64 / 1e9;
            if real > 1e-4 {
                let instant = simulated / real;
                self.speed = if self.speed == 0.0 {
                    instant
                } else {
                    self.speed * 0.85 + instant * 0.15
                };
            }
        }
        self.last_frame = Some(now);
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.draw(ctx);
    }
}

impl App {
    /// Draws one frame and advances the simulation. Kept free of `eframe::Frame`
    /// so the whole UI can be rendered from a test.
    pub fn draw(&mut self, ctx: &egui::Context) {
        self.handle_shortcuts(ctx);
        // Soldering or removing a crystal takes effect immediately; the RCC
        // decides what to do with it, not the front end.
        self.sim.set_crystals(self.board.crystals());
        // Serve the debugger first: a halt takes effect before this frame runs.
        self.debug.poll(&mut self.sim);
        if self.debug.needs_frames() {
            // The debugger's socket is only serviced by this frame loop, so the
            // window has to keep drawing even when the canvas is paused or has
            // nothing to animate - otherwise GDB waits, times out and reports
            // "Ignoring packet error".
            ctx.request_repaint();
        }
        if self.snapshot.running {
            // A debugger waits for a reply after every packet, so the frame has
            // to stay short while one is attached: a full budget in a debug
            // build takes long enough for GDB to give up on the request.
            let budget = if self.debug.needs_frames() {
                self.budget.min(DEBUG_BUDGET)
            } else {
                self.budget
            };
            // Service the parts first, then let the core run a millisecond at a
            // time, so encoder edges and terminal bytes land between
            // instructions instead of only at frame boundaries.
            bridge::run_frame(&mut self.board, &mut self.sim, budget, SLICE_NS);
            ctx.request_repaint();
        } else {
            // Devices stay live while paused: a knob turn or a typed line is
            // still queued for the moment the simulation resumes.
            bridge::service(&mut self.board, &mut self.sim);
        }

        let before_ns = self.snapshot.now_ns;
        self.refresh_snapshot();
        self.update_speed(std::time::Instant::now(), before_ns);
        self.levels = self.snapshot.levels.clone();
        self.board_state = self.board.evaluate(&self.levels);

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Gsimulator");
                ui.label(RichText::new("STM32F103 片上外设仿真").color(Color32::GRAY));
            });
            self.toolbar(ui);
        });

        egui::TopBottomPanel::bottom("footer").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new(format!(
                        "{} 个外设 · {} 条连线 · 缩放 {:.0}%",
                        self.board.devices().len(),
                        self.board.wires().len(),
                        self.schematic.zoom * 100.0
                    ))
                    .small()
                    .color(Color32::GRAY),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("适应视图").clicked() {
                        self.schematic.fit(self.schematic.last_rect, &self.board);
                    }
                    if ui.selectable_label(self.show_status, "状态面板").clicked() {
                        self.show_status = !self.show_status;
                    }
                });
            });
        });

        egui::SidePanel::right("status")
            .resizable(true)
            .default_width(panel_width(ctx))
            .show_animated(ctx, self.show_status, |ui| self.status_panel(ui));

        egui::CentralPanel::default().show(ctx, |ui| {
            let response = self.schematic.show(
                ui,
                &mut self.board,
                &self.levels,
                &self.board_state,
                self.snapshot.now_ns,
            );
            if let Some((message, error)) = response.message {
                if error {
                    eprintln!("{message}");
                }
            }
        });

        self.peripherals.show(ctx, &self.snapshot);
        if let Some(action) = self.can_bus.show(ctx, &self.snapshot) {
            self.apply_can_action(action);
        }
        if let Some(action) = self.scope.show(ctx, &self.snapshot) {
            self.apply_scope_action(action);
        }
        if let Some(action) = self.stimulus.show(ctx, &self.snapshot) {
            self.apply_stimulus_action(action);
        }
        self.debug.show(ctx, &mut self.sim);
        let motor_live = self.motor_live();
        self.motor_doc.show(ctx, motor_live.as_ref());
        // Picking a part in the library arms the canvas: the next click on the
        // drawing drops it (dragging the card there works too).
        if let Some(kind) = self.library.show(ctx, &self.board) {
            self.schematic.place_pending(kind);
        }

        if !self.fitted {
            self.schematic.fit(self.schematic.last_rect, &self.board);
            self.fitted = true;
        }

        if std::env::var_os("GSIM_DEBUG_LAYOUT").is_some() {
            eprintln!(
                "screen={:?} canvas={:?} zoom={:.2} pan={:?} ppp={:.2} native_ppp={:?}",
                ctx.screen_rect(),
                self.schematic.last_rect,
                self.schematic.zoom,
                self.schematic.pan,
                ctx.pixels_per_point(),
                ctx.input(|i| i.viewport().native_pixels_per_point),
            );
        }
    }
}

/// Width of the status panel, kept usable on small windows.
fn panel_width(ctx: &egui::Context) -> f32 {
    (ctx.screen_rect().width() * 0.30).clamp(250.0, 360.0)
}

fn format_hz(hz: u32) -> String {
    if hz >= 1_000_000 {
        format!("{:.3} MHz", hz as f64 / 1e6)
    } else if hz >= 1_000 {
        format!("{:.1} kHz", hz as f64 / 1e3)
    } else {
        format!("{hz} Hz")
    }
}

/// Loads a CJK-capable font so the Chinese labels render, falling back to the
/// built-in fonts when the platform does not ship one where we expect it.
fn install_fonts(ctx: &egui::Context) {
    const CANDIDATES: [&str; 6] = [
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\simsun.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    ];

    let mut override_path = std::env::var_os("GSIM_FONT").map(PathBuf::from);
    let path = override_path
        .take()
        .filter(|path| path.exists())
        .or_else(|| {
            CANDIDATES
                .iter()
                .map(PathBuf::from)
                .find(|path| path.exists())
        });
    let Some(path) = path else {
        eprintln!("未找到中文字体，界面中的中文可能显示为方块；可用 GSIM_FONT 指定字体文件");
        return;
    };
    let Ok(bytes) = std::fs::read(&path) else {
        return;
    };

    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "cjk".to_owned(),
        std::sync::Arc::new(egui::FontData::from_owned(bytes)),
    );
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts
            .families
            .entry(family)
            .or_default()
            .push("cjk".to_owned());
    }
    ctx.set_fonts(fonts);
}
