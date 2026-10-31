//! Peripheral status browser: one window that shows the live state of every
//! peripheral of the simulated chip, plus the two off-chip parts.
//!
//! The window is data driven: [`crate::sim::Simulation::snapshot`] collects a
//! [`PeripheralStatus`] list from the SoC and from the device models, and this
//! module only renders it. That keeps the register knowledge in the models.

use eframe::egui;
use egui::{Color32, RichText};
use gsim_stm32f1::status::PeripheralStatus;

use crate::sim::Snapshot;

/// State of the floating window.
#[derive(Debug, Default)]
pub struct PeripheralBrowser {
    /// `true` while the window is open.
    pub open: bool,
    /// Name of the peripheral whose details are shown.
    selected: Option<String>,
    /// Case insensitive filter over name and group.
    filter: String,
    /// Draw the 8x8 matrix preview for the LED matrix entry.
    show_matrix: bool,
}

impl PeripheralBrowser {
    pub fn new() -> Self {
        PeripheralBrowser {
            open: false,
            selected: None,
            filter: String::new(),
            show_matrix: true,
        }
    }

    pub fn toggle(&mut self) {
        self.open = !self.open;
    }

    /// Currently selected peripheral name, for the headless UI test.
    pub fn selected(&self) -> Option<&str> {
        self.selected.as_deref()
    }

    pub fn show(&mut self, ctx: &egui::Context, snapshot: &Snapshot) {
        if !self.open {
            return;
        }
        let mut open = self.open;
        egui::Window::new("外设状态浏览")
            .id(egui::Id::new("peripheral-browser"))
            .open(&mut open)
            .default_size([760.0, 520.0])
            .default_pos([240.0, 110.0])
            .resizable(true)
            .vscroll(false)
            .show(ctx, |ui| self.body(ui, snapshot));
        self.open = open;
    }

    fn body(&mut self, ui: &mut egui::Ui, snapshot: &Snapshot) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("过滤").color(Color32::GRAY));
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .desired_width(160.0)
                    .hint_text("名称 / 类别"),
            );
            ui.label(
                RichText::new(format!("共 {} 个外设", snapshot.peripherals.len()))
                    .color(Color32::GRAY),
            );
            if ui.small_button("清除选择").clicked() {
                self.selected = None;
            }
        });
        ui.separator();

        let filter = self.filter.to_lowercase();
        let matches = |status: &PeripheralStatus| {
            filter.is_empty()
                || status.name.to_lowercase().contains(&filter)
                || status.kind.to_lowercase().contains(&filter)
        };

        if self.selected.is_none() {
            self.selected = snapshot
                .peripherals
                .iter()
                .find(|status| matches(status))
                .map(|status| status.name.clone());
        }

        ui.horizontal_top(|ui| {
            let height = ui.available_height().max(220.0);
            ui.allocate_ui_with_layout(
                egui::vec2(200.0, height),
                egui::Layout::top_down(egui::Align::LEFT),
                |ui| {
                    egui::ScrollArea::vertical()
                        .id_salt("peripheral-list")
                        .max_height(height)
                        .show(ui, |ui| {
                            for status in snapshot.peripherals.iter().filter(|s| matches(s)) {
                                let selected =
                                    self.selected.as_deref() == Some(status.name.as_str());
                                let clock = if status.clock_hz == 0 && status.base != 0 {
                                    "时钟关闭".to_string()
                                } else {
                                    String::new()
                                };
                                let label = format!("{} · {}", status.name, status.kind);
                                let response = ui.selectable_label(selected, label);
                                if !clock.is_empty() {
                                    ui.label(
                                        RichText::new(clock)
                                            .small()
                                            .color(Color32::from_rgb(220, 160, 90)),
                                    );
                                }
                                if response.clicked() {
                                    self.selected = Some(status.name.clone());
                                }
                            }
                        });
                },
            );
            ui.separator();
            ui.vertical(|ui| {
                let Some(status) = snapshot
                    .peripherals
                    .iter()
                    .find(|status| self.selected.as_deref() == Some(status.name.as_str()))
                else {
                    ui.label(RichText::new("选择一个外设查看实时状态").color(Color32::GRAY));
                    return;
                };
                self.details(ui, status, snapshot);
            });
        });
    }

    fn details(&mut self, ui: &mut egui::Ui, status: &PeripheralStatus, snapshot: &Snapshot) {
        ui.horizontal(|ui| {
            ui.heading(&status.name);
            ui.label(RichText::new(status.kind).color(Color32::GRAY));
        });
        ui.horizontal(|ui| {
            if status.base != 0 {
                ui.monospace(format!("基址 0x{:08X}", status.base));
                ui.separator();
            }
            ui.monospace(format!("时钟 {}", clock_label(status.clock_hz)));
        });
        ui.add_space(6.0);

        if status.rows.is_empty() {
            ui.label(RichText::new("暂无状态").color(Color32::GRAY));
        }
        egui::ScrollArea::vertical()
            .id_salt("peripheral-details")
            .max_height(240.0)
            .show(ui, |ui| {
                egui::Grid::new("peripheral-rows")
                    .num_columns(2)
                    .striped(true)
                    .spacing([16.0, 4.0])
                    .show(ui, |ui| {
                        for (label, value) in &status.rows {
                            ui.label(RichText::new(*label).color(Color32::GRAY));
                            ui.monospace(value);
                            ui.end_row();
                        }
                    });
            });

        if status.kind.contains("MAX7219") {
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                ui.heading("点阵画面");
                ui.checkbox(&mut self.show_matrix, "显示 8×8 预览");
            });
            if self.show_matrix {
                matrix_preview(ui, snapshot);
            }
        }
    }
}

/// `12.000 MHz` style text, `—` when the block is not clocked.
fn clock_label(hz: u32) -> String {
    if hz == 0 {
        return "—".to_string();
    }
    if hz >= 1_000_000 {
        format!("{:.3} MHz", f64::from(hz) / 1e6)
    } else if hz >= 1_000 {
        format!("{:.1} kHz", f64::from(hz) / 1e3)
    } else {
        format!("{hz} Hz")
    }
}

/// Draws the visible pixels of the first MAX7219 die.
fn matrix_preview(ui: &mut egui::Ui, snapshot: &Snapshot) {
    let pixels = snapshot.matrix.pixels();
    let intensity = snapshot.matrix.intensity();
    let lit = |level: u8| {
        let base = 60 + u32::from(level) * 12;
        Color32::from_rgb(
            (base + 60).min(255) as u8,
            (base / 2).min(255) as u8,
            (base / 4).min(255) as u8,
        )
    };

    let cell = 22.0;
    let size = egui::vec2(cell * 8.0 + 8.0, cell * 8.0 + 8.0);
    ui.horizontal(|ui| {
        let (response, painter) = ui.allocate_painter(size, egui::Sense::hover());
        let origin = response.rect.min + egui::vec2(4.0, 4.0);
        for (row, line) in pixels.iter().enumerate() {
            for (column, on) in line.iter().enumerate() {
                let rect = egui::Rect::from_min_size(
                    origin + egui::vec2(column as f32 * cell, row as f32 * cell),
                    egui::vec2(cell - 2.0, cell - 2.0),
                );
                painter.rect_filled(
                    rect,
                    3.0,
                    if *on {
                        lit(intensity)
                    } else {
                        Color32::from_rgb(30, 33, 39)
                    },
                );
                painter.rect_stroke(
                    rect,
                    3.0,
                    egui::Stroke::new(1.0, Color32::from_rgb(48, 52, 60)),
                    egui::StrokeKind::Inside,
                );
            }
        }

        ui.vertical(|ui| {
            ui.label(
                RichText::new(format!(
                    "亮度 {intensity}/15 · 已锁存 {} 帧 · 显示{}",
                    snapshot.matrix.words,
                    if snapshot.matrix.enabled() {
                        "开启"
                    } else {
                        "关断"
                    }
                ))
                .small()
                .color(Color32::GRAY),
            );
            if snapshot.matrix.display_test() {
                ui.label(
                    RichText::new("显示测试模式：全部点亮")
                        .small()
                        .color(Color32::from_rgb(240, 190, 90)),
                );
            }
            ui.label(
                RichText::new("点阵由 SPI1（PA5/PA7）+ PA4 片选驱动")
                    .small()
                    .color(Color32::GRAY),
            );
        });
    });
}
