//! Application shell: window layout, toolbar, status panel and the glue
//! between the simulation and the schematic.

use std::path::PathBuf;

use eframe::egui;
use egui::{Color32, RichText};

use crate::board::{Board, BoardState, DeviceKind, PinLevels, PinRef, Rail};
use crate::bridge;
use crate::canbus::{CanAction, CanBusWindow};
use crate::canvas::Schematic;
use crate::library::DeviceLibrary;
use crate::peripherals::PeripheralBrowser;
use crate::sim::{Simulation, Snapshot, Source};
use gsim_devices::Direction;

/// Instructions executed per UI frame; the slider scales this.
const MIN_BUDGET: u32 = 5_000;
const MAX_BUDGET: u32 = 2_000_000;
const DEFAULT_BUDGET: u32 = 400_000;
/// Length of one device-servicing slice of simulated time.
const SLICE_NS: u64 = 1_000_000;

pub struct App {
    sim: Simulation,
    board: Board,
    schematic: Schematic,
    snapshot: Snapshot,
    levels: PinLevels,
    board_state: BoardState,
    budget: u32,
    elf_path: Option<PathBuf>,
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
    /// Window listing every part that can be placed on the canvas.
    library: DeviceLibrary,
    /// Text typed into each UART terminal, keyed by device id.
    uart_input: std::collections::BTreeMap<u32, String>,
}

impl App {
    /// Builds the app for `eframe`, loading the CJK font into the context.
    pub fn new(cc: &eframe::CreationContext<'_>, elf_path: Option<PathBuf>) -> Self {
        App::with_context(&cc.egui_ctx, elf_path)
    }

    /// Same as [`App::new`] but with a bare [`egui::Context`], so tests can
    /// drive the whole UI headlessly.
    pub fn with_context(ctx: &egui::Context, elf_path: Option<PathBuf>) -> Self {
        install_fonts(ctx);
        ctx.set_visuals(egui::Visuals::dark());

        let mut sim = Simulation::new();
        if let Some(path) = elf_path {
            if let Err(error) = sim.load_elf(&path) {
                eprintln!("{error}");
            }
        }
        let levels = sim.pin_levels();
        let board = Board::demo();
        let board_state = board.evaluate(&levels);
        let snapshot = sim.snapshot();

        App {
            sim,
            board,
            schematic: Schematic::default(),
            snapshot,
            levels,
            board_state,
            budget: DEFAULT_BUDGET,
            elf_path: None,
            last_frame: None,
            speed: 0.0,
            fitted: false,
            show_status: true,
            peripherals: PeripheralBrowser::new(),
            can_bus: CanBusWindow::default(),
            library: DeviceLibrary::new(),
            uart_input: std::collections::BTreeMap::new(),
        }
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

    fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            let running = self.snapshot.running;
            let idle = matches!(self.sim.source(), Source::Idle);
            if ui
                .add_enabled(
                    !idle,
                    egui::Button::new(if running { "⏸ 暂停" } else { "▶ 运行" }),
                )
                .on_hover_text(if idle {
                    "没有驱动源：先点击「内置激励」或加载 ELF"
                } else {
                    "空格键也可以切换"
                })
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
            if ui
                .selectable_label(stimulus_on, "内置激励")
                .on_hover_text(if stimulus_on {
                    "PA5 方波运行中，点击关闭并释放 PA5"
                } else {
                    "PA5 方波，不需要固件"
                })
                .clicked()
            {
                if stimulus_on {
                    self.sim.stop_stimulus();
                    self.schematic.say("已关闭内置激励，PA5 已释放", false);
                } else {
                    self.sim.start_stimulus();
                }
            }
            ui.separator();
            if ui
                .selectable_label(self.library.open, "＋ 添加器件")
                .on_hover_text("打开器件库：挑选 LED、串口终端、旋转编码器后放到画布上")
                .clicked()
            {
                self.library.toggle();
            }
            if ui.button("示例电路").clicked() {
                self.board = Board::demo();
                self.schematic.say(
                    "已恢复示例电路（LED/PA5、UART1/PA9+PA10、编码器/PA0-PA2）",
                    false,
                );
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
                .selectable_label(self.can_bus.open, "🚌 CAN 总线")
                .on_hover_text("CAN 收发器与总线状态、帧记录和注入")
                .clicked()
            {
                self.can_bus.toggle();
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
                ui.label(
                    RichText::new(snapshot.source.label())
                        .color(if snapshot.running {
                            Color32::from_rgb(120, 220, 140)
                        } else {
                            Color32::from_rgb(220, 180, 90)
                        })
                        .strong(),
                );
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

                if let Some(stimulus) = snapshot.stimulus {
                    ui.add_space(12.0);
                    ui.heading("内置激励");
                    ui.horizontal(|ui| {
                        ui.label("PA5");
                        let (text, color) = if stimulus.high {
                            ("高电平", Color32::from_rgb(90, 226, 120))
                        } else {
                            ("低电平", Color32::from_rgb(140, 150, 164))
                        };
                        ui.colored_label(color, text);
                    });
                    let remaining =
                        stimulus.next_toggle_ns.saturating_sub(snapshot.now_ns) as f64 / 1e6;
                    ui.label(format!("{remaining:.0} ms 后翻转"));
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
                     Delete 删除选中；中键拖动平移；滚轮缩放。",
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
            for rail in Rail::ALL {
                let color = if rail.is_high() {
                    Color32::from_rgb(200, 90, 90)
                } else {
                    Color32::from_rgb(110, 135, 175)
                };
                if ui
                    .add(egui::Button::new(RichText::new(rail.label()).small()).fill(color))
                    .clicked()
                {
                    self.start_wire_from(PinRef::Rail(rail));
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
                RichText::new("还没有外设，点工具栏的“＋ LED / ＋ UART / ＋ 编码器”添加")
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
                        let lit = self.board_state.led_lit(id);
                        let (text, color) = if lit {
                            ("点亮", accent)
                        } else {
                            ("熄灭", Color32::from_rgb(110, 118, 130))
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
                DeviceKind::Led => {}
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

    /// Applies a command issued from the CAN bus window.
    fn apply_can_action(&mut self, action: CanAction) {
        match action {
            CanAction::Inject(frame) => {
                self.sim.inject_can_frame(frame);
                self.schematic.say(
                    format!("已向总线注入 {} {}", frame.id_label(), frame.data_label()),
                    false,
                );
            }
            CanAction::ClearLog => self.sim.clear_can_log(),
            CanAction::SetMode(mode) => {
                self.sim.set_transceiver_mode(mode);
                self.schematic
                    .say(format!("CAN 收发器切换到 {}", mode.label()), false);
            }
        }
        self.snapshot = self.sim.snapshot();
    }

    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        let (space, reset) = ctx.input(|i| {
            (
                i.key_pressed(egui::Key::Space),
                i.key_pressed(egui::Key::R) && !i.modifiers.ctrl,
            )
        });
        if space {
            self.sim.toggle_running();
        }
        if reset {
            self.sim.reset();
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

    /// Device library window, exposed for the headless UI test.
    pub fn library_mut(&mut self) -> &mut DeviceLibrary {
        &mut self.library
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
        if self.snapshot.running {
            // Service the parts first, then let the core run a millisecond at a
            // time, so encoder edges and terminal bytes land between
            // instructions instead of only at frame boundaries.
            bridge::run_frame(&mut self.board, &mut self.sim, self.budget, SLICE_NS);
            ctx.request_repaint();
        } else {
            // Devices stay live while paused: a knob turn or a typed line is
            // still queued for the moment the simulation resumes.
            bridge::service(&mut self.board, &mut self.sim);
        }

        let before_ns = self.snapshot.now_ns;
        self.snapshot = self.sim.snapshot();
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
