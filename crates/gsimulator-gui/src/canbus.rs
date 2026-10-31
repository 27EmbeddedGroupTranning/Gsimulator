//! CAN bus status window.
//!
//! The transceiver is the only part that sees the wire, so its window shows the
//! whole bus: controller state, transceiver state, every frame in order and a
//! small injection panel for playing the role of a second ECU.
//!
//! The window is a pure view plus a few commands it returns to the application
//! as [`CanAction`], which keeps `app.rs` in charge of mutating the simulation.

use eframe::egui;
use egui::{Color32, RichText};
use gsim_core::{CanDirection, CanFrame};
use gsim_devices::TransceiverMode;

use crate::sim::Snapshot;

/// Commands the window sends back to the application.
#[derive(Clone, Debug, PartialEq)]
pub enum CanAction {
    /// Put a frame on the bus as an external node.
    Inject(CanFrame),
    /// Drop the frame log.
    ClearLog,
    /// Change the electrical mode of the transceiver.
    SetMode(TransceiverMode),
}

/// State of the CAN bus window.
#[derive(Debug)]
pub struct CanBusWindow {
    /// `true` while the window is open.
    pub open: bool,
    id_text: String,
    data_text: String,
    extended: bool,
    remote: bool,
    /// Only show frames this node transmitted.
    only_tx: bool,
    /// Follow the newest frame in the table.
    follow: bool,
    error: Option<String>,
}

impl Default for CanBusWindow {
    fn default() -> Self {
        CanBusWindow {
            open: false,
            id_text: "0x321".to_string(),
            data_text: "11 22 33".to_string(),
            extended: false,
            remote: false,
            only_tx: false,
            follow: true,
            error: None,
        }
    }
}

impl CanBusWindow {
    pub fn toggle(&mut self) {
        self.open = !self.open;
    }

    /// Number of frames currently listed, for the headless UI test.
    pub fn visible_frames(&self, snapshot: &Snapshot) -> usize {
        snapshot
            .can
            .records
            .iter()
            .filter(|record| !self.only_tx || record.direction == CanDirection::Tx)
            .count()
    }

    pub fn show(&mut self, ctx: &egui::Context, snapshot: &Snapshot) -> Option<CanAction> {
        if !self.open {
            return None;
        }
        let mut action = None;
        let mut open = self.open;
        egui::Window::new("CAN 总线状态")
            .id(egui::Id::new("can-bus-browser"))
            .open(&mut open)
            .default_size([820.0, 560.0])
            .default_pos([320.0, 150.0])
            .resizable(true)
            .show(ctx, |ui| {
                self.body(ui, snapshot, &mut action);
            });
        self.open = open;
        action
    }

    fn body(&mut self, ui: &mut egui::Ui, snapshot: &Snapshot, action: &mut Option<CanAction>) {
        self.header(ui, snapshot, action);
        ui.separator();
        self.controller_panel(ui, snapshot);
        ui.separator();
        self.log_panel(ui, snapshot, action);
    }

    fn header(&mut self, ui: &mut egui::Ui, snapshot: &Snapshot, action: &mut Option<CanAction>) {
        let can = &snapshot.can;
        ui.horizontal(|ui| {
            ui.heading("CAN 总线");
            ui.label(RichText::new(&can.node_name).color(Color32::GRAY));
            ui.separator();
            ui.label(RichText::new("收发器模式").color(Color32::GRAY));
            let mut mode = can.mode;
            egui::ComboBox::from_id_salt("can-transceiver-mode")
                .selected_text(mode.label())
                .show_ui(ui, |ui| {
                    for candidate in [
                        TransceiverMode::Normal,
                        TransceiverMode::Silent,
                        TransceiverMode::Standby,
                    ] {
                        if ui
                            .selectable_label(mode == candidate, candidate.label())
                            .clicked()
                        {
                            mode = candidate;
                        }
                    }
                });
            if mode != can.mode {
                *action = Some(CanAction::SetMode(mode));
            }
            ui.separator();
            ui.checkbox(&mut self.follow, "跟随最新帧");
            ui.checkbox(&mut self.only_tx, "只看发送");
            if ui.small_button("清空记录").clicked() {
                *action = Some(CanAction::ClearLog);
            }
        });
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!(
                    "总线帧 {} · 投递 {} · 回环 {} · 拒发 {} · 待投递 {}",
                    can.bus.frames,
                    can.bus.deliveries,
                    can.bus.loopbacks,
                    can.bus.refused,
                    can.pending
                ))
                .small()
                .color(Color32::GRAY),
            );
            ui.separator();
            ui.label(
                RichText::new(format!(
                    "收发器 发送 {} · 接收 {} · 回环 {}",
                    can.transceiver.tx_frames,
                    can.transceiver.rx_frames,
                    can.transceiver.loopback_frames
                ))
                .small()
                .color(Color32::GRAY),
            );
            if can.mode == TransceiverMode::Standby {
                ui.colored_label(
                    Color32::from_rgb(230, 170, 80),
                    "收发器待机：控制器已与总线隔离",
                );
            }
        });
    }

    fn controller_panel(&mut self, ui: &mut egui::Ui, snapshot: &Snapshot) {
        egui::CollapsingHeader::new("控制器寄存器")
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("can-controller-rows")
                    .num_columns(4)
                    .striped(true)
                    .spacing([14.0, 3.0])
                    .show(ui, |ui| {
                        for (index, (label, value)) in snapshot.can.controller.iter().enumerate() {
                            ui.label(RichText::new(*label).color(Color32::GRAY));
                            ui.monospace(value);
                            if index % 2 == 1 {
                                ui.end_row();
                            }
                        }
                        ui.end_row();
                    });
            });
    }

    fn log_panel(
        &mut self,
        ui: &mut egui::Ui,
        snapshot: &Snapshot,
        action: &mut Option<CanAction>,
    ) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("帧记录").strong());
            ui.label(
                RichText::new(format!("{} 条", self.visible_frames(snapshot)))
                    .small()
                    .color(Color32::GRAY),
            );
        });

        let height = (ui.available_height() - 96.0).max(160.0);
        egui::ScrollArea::both()
            .id_salt("can-frame-log")
            .max_height(height)
            .stick_to_bottom(self.follow)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                egui::Grid::new("can-frame-grid")
                    .num_columns(8)
                    .striped(true)
                    .spacing([12.0, 3.0])
                    .show(ui, |ui| {
                        for header in [
                            "时间(ms)",
                            "节点",
                            "方向",
                            "ID",
                            "帧型",
                            "DLC",
                            "数据",
                            "ACK",
                        ] {
                            ui.label(RichText::new(header).small().color(Color32::GRAY));
                        }
                        ui.end_row();

                        for record in
                            snapshot.can.records.iter().filter(|record| {
                                !self.only_tx || record.direction == CanDirection::Tx
                            })
                        {
                            let (direction, color) = match record.direction {
                                CanDirection::Tx => ("发送", Color32::from_rgb(110, 210, 130)),
                                CanDirection::Rx => ("接收", Color32::from_rgb(120, 170, 240)),
                                CanDirection::Loopback => ("回环", Color32::from_rgb(230, 180, 90)),
                            };
                            ui.monospace(format!("{:.3}", record.time_ns as f64 / 1e6));
                            ui.monospace(&record.node_name);
                            ui.label(RichText::new(direction).small().color(color));
                            ui.monospace(record.frame.id_label());
                            ui.label(
                                RichText::new(if record.frame.extended {
                                    "扩展"
                                } else {
                                    "标准"
                                })
                                .small()
                                .color(Color32::GRAY),
                            );
                            ui.monospace(format!("{}", record.frame.dlc));
                            ui.monospace(if record.frame.remote {
                                "远程帧".to_string()
                            } else {
                                record.frame.data_label()
                            });
                            ui.label(if record.acknowledged {
                                RichText::new("✓").color(Color32::from_rgb(110, 210, 130))
                            } else {
                                RichText::new("—").color(Color32::from_rgb(220, 120, 120))
                            });
                            ui.end_row();
                        }
                    });
                if snapshot.can.records.is_empty() {
                    ui.label(
                        RichText::new("总线上还没有帧：运行固件，或在下面注入一帧")
                            .small()
                            .color(Color32::GRAY),
                    );
                }
            });

        ui.separator();
        ui.horizontal(|ui| {
            ui.label(RichText::new("注入帧").strong());
            ui.label(RichText::new("ID").small().color(Color32::GRAY));
            ui.add(egui::TextEdit::singleline(&mut self.id_text).desired_width(90.0));
            ui.checkbox(&mut self.extended, "扩展帧");
            ui.checkbox(&mut self.remote, "远程帧");
            ui.label(RichText::new("数据").small().color(Color32::GRAY));
            ui.add(
                egui::TextEdit::singleline(&mut self.data_text)
                    .desired_width(180.0)
                    .hint_text("11 22 33"),
            );
            if ui.button("发送到总线").clicked() {
                match parse_frame(&self.id_text, &self.data_text, self.extended, self.remote) {
                    Ok(frame) => {
                        self.error = None;
                        *action = Some(CanAction::Inject(frame));
                    }
                    Err(message) => self.error = Some(message),
                }
            }
        });
        if let Some(message) = &self.error {
            ui.colored_label(Color32::from_rgb(235, 130, 130), message);
        }
    }
}

/// Parses the injection widgets into a frame.
fn parse_frame(
    id_text: &str,
    data_text: &str,
    extended: bool,
    remote: bool,
) -> Result<CanFrame, String> {
    let id_text = id_text
        .trim()
        .trim_start_matches("0x")
        .trim_start_matches("0X");
    let id =
        u32::from_str_radix(id_text, 16).map_err(|_| format!("ID `{id_text}` 不是十六进制数字"))?;
    if extended && id > 0x1FFF_FFFF {
        return Err("扩展帧 ID 最大 0x1FFFFFFF".to_string());
    }
    if !extended && id > 0x7FF {
        return Err("标准帧 ID 最大 0x7FF".to_string());
    }
    let mut data = Vec::new();
    for token in data_text
        .split(|c: char| c == ' ' || c == ',' || c == '-')
        .filter(|token| !token.is_empty())
    {
        let cleaned = token.trim_start_matches("0x").trim_start_matches("0X");
        let byte = u8::from_str_radix(cleaned, 16)
            .map_err(|_| format!("数据 `{token}` 不是十六进制字节"))?;
        data.push(byte);
    }
    if data.len() > 8 {
        return Err("一帧最多 8 个字节".to_string());
    }
    let mut frame = if extended {
        CanFrame::extended(id, &data)
    } else {
        CanFrame::standard(id as u16, &data)
    };
    if remote {
        frame = CanFrame::remote(frame);
    }
    Ok(frame)
}
