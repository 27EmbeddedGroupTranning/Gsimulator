//! Oscilloscope window.
//!
//! The probes are ordinary parts on the drawing (they have to be wired to
//! `CANH`/`CANL` to see anything), so this window is a pure view over the
//! samples the bridge pushed into them, plus the controls that belong to the
//! instrument itself: the time window, freezing the trace and clearing it.

use eframe::egui;
use egui::{Color32, Pos2, Rect, RichText, Stroke, Vec2};
use gsim_core::can::{BitField, TraceBit, TraceFrame};

use crate::sim::Snapshot;

/// Window widths offered by the panel, in nanoseconds.
const WINDOWS_NS: [u64; 5] = [200_000, 500_000, 1_000_000, 2_000_000, 5_000_000];

/// Colour of the `CANH` channel.
const COLOR_HIGH: Color32 = Color32::from_rgb(250, 210, 90);
/// Colour of the `CANL` channel.
const COLOR_LOW: Color32 = Color32::from_rgb(90, 200, 240);

/// Commands the window sends back to the application.
#[derive(Clone, Debug, PartialEq)]
pub enum ScopeAction {
    /// Change the time window of probe `index`.
    SetWindow(usize, u64),
    /// Freeze or release probe `index`.
    SetFrozen(usize, bool),
    /// Drop the trace of probe `index`.
    Clear(usize),
}

/// State of the oscilloscope window.
#[derive(Debug, Default)]
pub struct ScopeWindow {
    pub open: bool,
    /// Which probe is on screen.
    selected: usize,
    /// Draw the differential pair instead of the two wires.
    differential: bool,
    /// Volts per division of the vertical axis.
    volts_per_div: f32,
    /// How many frames back the bit ruler is pinned, counted from the newest.
    frame_back: usize,
}

impl ScopeWindow {
    pub fn toggle(&mut self) {
        self.open = !self.open;
    }

    pub fn show(&mut self, ctx: &egui::Context, snapshot: &Snapshot) -> Option<ScopeAction> {
        if !self.open {
            return None;
        }
        let mut action = None;
        let mut open = self.open;
        egui::Window::new("示波器")
            .id(egui::Id::new("scope-browser"))
            .open(&mut open)
            .default_size([880.0, 460.0])
            .default_pos([360.0, 200.0])
            .resizable(true)
            .show(ctx, |ui| {
                self.body(ui, snapshot, &mut action);
            });
        self.open = open;
        action
    }

    fn body(&mut self, ui: &mut egui::Ui, snapshot: &Snapshot, action: &mut Option<ScopeAction>) {
        if snapshot.scopes.is_empty() {
            ui.label(
                RichText::new("还没有示波器：从器件库放一个，并把 H/L 接到 CANH/CANL 上。")
                    .color(Color32::GRAY),
            );
            return;
        }
        self.selected = self.selected.min(snapshot.scopes.len() - 1);
        self.toolbar(ui, snapshot, action);
        ui.separator();
        self.plot(ui, snapshot);
    }

    fn toolbar(
        &mut self,
        ui: &mut egui::Ui,
        snapshot: &Snapshot,
        action: &mut Option<ScopeAction>,
    ) {
        let index = self.selected;
        let scope = &snapshot.scopes[index];
        ui.horizontal(|ui| {
            ui.label(RichText::new(&scope.name).strong());
            ui.separator();
            ui.label(RichText::new("时基").color(Color32::GRAY));
            let mut window = scope.window_ns;
            egui::ComboBox::from_id_salt("scope-window")
                .selected_text(format!("{:.2} ms", window as f64 / 1e6))
                .show_ui(ui, |ui| {
                    for candidate in WINDOWS_NS {
                        let label = format!("{:.2} ms", candidate as f64 / 1e6);
                        if ui.selectable_label(window == candidate, label).clicked() {
                            window = candidate;
                        }
                    }
                });
            if window != scope.window_ns {
                *action = Some(ScopeAction::SetWindow(index, window));
            }
            ui.separator();
            ui.label(RichText::new("档位").color(Color32::GRAY));
            ui.add(
                egui::DragValue::new(&mut self.volts_per_div)
                    .speed(0.1)
                    .range(0.5..=5.0)
                    .suffix(" V/格"),
            );
            ui.checkbox(&mut self.differential, "差分");
            ui.separator();
            let mut frozen = scope.frozen;
            if ui.checkbox(&mut frozen, "冻结").changed() {
                *action = Some(ScopeAction::SetFrozen(index, frozen));
            }
            if ui.button("清空").clicked() {
                *action = Some(ScopeAction::Clear(index));
            }
            ui.label(
                RichText::new(format!(
                    "{} 点 · 尖峰 {} · 峰峰 {:.2} V",
                    scope.samples.len(),
                    scope.glitches,
                    scope.peak
                ))
                .small()
                .color(Color32::GRAY),
            );
        });
        if snapshot.scopes.len() > 1 {
            ui.horizontal(|ui| {
                for (candidate, scope) in snapshot.scopes.iter().enumerate() {
                    if ui
                        .selectable_label(candidate == self.selected, &scope.name)
                        .clicked()
                    {
                        self.selected = candidate;
                    }
                }
            });
        }
    }

    fn plot(&mut self, ui: &mut egui::Ui, snapshot: &Snapshot) {
        let scope = &snapshot.scopes[self.selected];
        let frames: Vec<&TraceFrame> = scope.visible_frames().collect();
        let decode_height = if frames.is_empty() { 0.0 } else { 168.0 };
        let height = (ui.available_height() - decode_height - 8.0).max(140.0);
        let (response, painter) = ui.allocate_painter(
            Vec2::new(ui.available_width(), height),
            egui::Sense::hover(),
        );
        let rect = response.rect;
        painter.rect_filled(rect, 4.0, Color32::from_rgb(14, 20, 24));

        let Some(first) = scope.samples.first().map(|sample| sample.time_ns) else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "还没有采样：把探头接到 CANH/CANL 上并开始仿真",
                egui::FontId::proportional(13.0),
                Color32::GRAY,
            );
            return;
        };
        let last = scope
            .samples
            .last()
            .map(|sample| sample.time_ns)
            .unwrap_or(first);
        let span = (last.saturating_sub(first)).max(1);
        // Voltage axis: the pair swings between 1.5 V and 3.5 V.
        let top = 4.5_f32;
        let bottom = 0.5_f32;
        let to_screen = |time_ns: u64, volts: f32| {
            Pos2::new(
                rect.left() + rect.width() * (time_ns - first) as f32 / span as f32,
                rect.top() + rect.height() * (top - volts) / (top - bottom),
            )
        };

        // Graticule.
        let grid = Stroke::new(1.0, Color32::from_rgb(34, 46, 54));
        for step in 0..=4 {
            let fraction = step as f32 / 4.0;
            let x = rect.left() + rect.width() * fraction;
            painter.line_segment(
                [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                grid,
            );
            let y = rect.top() + rect.height() * fraction;
            painter.line_segment(
                [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                grid,
            );
        }

        let mut high = Vec::with_capacity(scope.samples.len());
        let mut low = Vec::with_capacity(scope.samples.len());
        let mut differential = Vec::with_capacity(scope.samples.len());
        for sample in &scope.samples {
            if self.differential {
                // Draw the difference around 2.5 V so it lines up with the wires.
                differential.push(to_screen(sample.time_ns, 2.5 + sample.differential()));
            } else {
                high.push(to_screen(sample.time_ns, sample.can_h));
                low.push(to_screen(sample.time_ns, sample.can_l));
            }
        }
        if self.differential {
            painter.add(egui::Shape::line(
                differential,
                Stroke::new(1.6, Color32::from_rgb(140, 240, 160)),
            ));
        } else {
            painter.add(egui::Shape::line(high, Stroke::new(1.6, COLOR_HIGH)));
            painter.add(egui::Shape::line(low, Stroke::new(1.6, COLOR_LOW)));
        }

        // Ringing is visible as a spike in the trace, mark it on the raster.
        let spike = Stroke::new(1.0, Color32::from_rgb(220, 90, 90));
        for sample in scope.samples.iter().filter(|sample| sample.glitch) {
            let x = rect.left() + rect.width() * (sample.time_ns - first) as f32 / span as f32;
            painter.line_segment(
                [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                spike,
            );
        }

        // Axis labels.
        let font = egui::FontId::proportional(11.0);
        painter.text(
            Pos2::new(rect.left() + 4.0, rect.top() + 2.0),
            egui::Align2::LEFT_TOP,
            format!(
                "CANH {:.1} V · CANL {:.1} V · {:.2} µs/格",
                scope.samples.last().map(|s| s.can_h).unwrap_or(0.0),
                scope.samples.last().map(|s| s.can_l).unwrap_or(0.0),
                span as f64 / 4000.0
            ),
            font.clone(),
            Color32::GRAY,
        );
        painter.text(
            rect.left_bottom() + Vec2::new(4.0, -2.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{:.3} ms", first as f64 / 1e6),
            font.clone(),
            Color32::GRAY,
        );
        painter.text(
            rect.right_bottom() + Vec2::new(-4.0, -2.0),
            egui::Align2::RIGHT_BOTTOM,
            format!("{:.3} ms", last as f64 / 1e6),
            font,
            Color32::GRAY,
        );
        let _ = self.volts_per_div;

        if !frames.is_empty() {
            ui.separator();
            self.bit_ruler(ui, first, span, &frames);
            self.decode_details(ui, &frames);
        }
    }

    /// One cell per bit under the trace, on the same time axis, so a bit can be
    /// matched with the waveform above it. Hovering a cell names the bit.
    fn bit_ruler(&self, ui: &mut egui::Ui, first: u64, span: u64, frames: &[&TraceFrame]) {
        let (response, painter) = ui.allocate_painter(
            Vec2::new(ui.available_width(), 46.0),
            egui::Sense::hover(),
        );
        let rect = response.rect;
        painter.rect_filled(rect, 3.0, Color32::from_rgb(14, 20, 24));
        let cells = Rect::from_min_max(
            Pos2::new(rect.left() + 1.0, rect.top() + 13.0),
            Pos2::new(rect.right() - 1.0, rect.bottom() - 1.0),
        );
        let x_of = |time_ns: u64| {
            rect.left() + rect.width() * time_ns.saturating_sub(first) as f32 / span as f32
        };
        // Bit boundaries come from the sample times, so a cell ends where the
        // next sample was taken.
        let cell_end = |frame: &TraceFrame, index: usize| {
            let step = (frame.end_ns - frame.start_ns) / frame.bits.len().max(1) as u64;
            frame
                .bits
                .get(index + 1)
                .map(|next| next.time_ns)
                .unwrap_or(frame.bits[index].time_ns.saturating_add(step.max(1)))
        };

        let caption_font = egui::FontId::proportional(9.0);
        for frame in frames {
            for (index, bit) in frame.bits.iter().enumerate() {
                let x0 = x_of(bit.time_ns).max(cells.left());
                let x1 = x_of(cell_end(frame, index)).min(cells.right()).max(x0 + 0.5);
                if x0 >= cells.right() || x1 <= cells.left() {
                    continue;
                }
                let color = field_color(bit.field);
                let fill = if bit.dominant {
                    color
                } else {
                    color.gamma_multiply(0.28)
                };
                painter.rect_filled(
                    Rect::from_min_max(Pos2::new(x0, cells.top()), Pos2::new(x1, cells.bottom())),
                    0.0,
                    fill,
                );
            }
            // Field names above the cells, whenever the field is wide enough.
            let mut index = 0;
            while index < frame.bits.len() {
                let field = frame.bits[index].field;
                let mut last = index;
                while last + 1 < frame.bits.len() && frame.bits[last + 1].field == field {
                    last += 1;
                }
                let x0 = x_of(frame.bits[index].time_ns).max(cells.left());
                let x1 = x_of(cell_end(frame, last)).min(cells.right());
                if x1 - x0 >= 22.0 {
                    painter.text(
                        Pos2::new((x0 + x1) / 2.0, rect.top() + 1.0),
                        egui::Align2::CENTER_TOP,
                        field.label(),
                        caption_font.clone(),
                        field_color(field),
                    );
                }
                index = last + 1;
            }
        }

        let Some(pointer) = response.hover_pos().filter(|pos| cells.contains(*pos)) else {
            return;
        };
        let mut hit = None;
        for frame in frames.iter().rev() {
            for (index, bit) in frame.bits.iter().enumerate() {
                let x0 = x_of(bit.time_ns);
                let x1 = x_of(cell_end(frame, index)).max(x0 + 0.5);
                if pointer.x >= x0 && pointer.x < x1 {
                    hit = Some((index, *frame, *bit));
                    break;
                }
            }
            if hit.is_some() {
                break;
            }
        }
        let Some((index, frame, bit)) = hit else {
            return;
        };
        response.on_hover_ui(|ui| {
            bit_tooltip(ui, index, frame, bit);
        });
    }

    /// The newest frame written out bit by bit, so every bit can be read even
    /// when the waveform is far too dense to label each cell.
    fn decode_details(&mut self, ui: &mut egui::Ui, frames: &[&TraceFrame]) {
        let count = frames.len();
        self.frame_back = self.frame_back.min(count - 1);
        let index = count - 1 - self.frame_back;
        let trace = frames[index];

        ui.horizontal(|ui| {
            ui.label(RichText::new("CAN 帧解码").strong());
            ui.label(
                RichText::new(format!("第 {} / {} 帧", index + 1, count))
                    .small()
                    .color(Color32::GRAY),
            );
            if ui
                .add_enabled(index > 0, egui::Button::new("◀").small())
                .clicked()
            {
                self.frame_back += 1;
            }
            if ui
                .add_enabled(index + 1 < count, egui::Button::new("▶").small())
                .clicked()
            {
                self.frame_back = self.frame_back.saturating_sub(1);
            }
            if !trace.crc_ok {
                ui.colored_label(Color32::from_rgb(230, 110, 110), "CRC 错误");
            }
            if !trace.acknowledged {
                ui.colored_label(Color32::from_rgb(230, 170, 80), "无应答");
            }
            ui.label(RichText::new(frame_summary(trace)).small().color(Color32::GRAY));
        });

        egui::ScrollArea::vertical()
            .id_salt("scope-frame-bits")
            .max_height(104.0)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    for (position, bit) in trace.bits.iter().enumerate() {
                        ui.label(
                            RichText::new(format!("{position}·{}={}", bit.caption(), bit.value()))
                                .monospace()
                                .size(10.0)
                                .color(field_color(bit.field)),
                        );
                    }
                });
            });
    }
}

/// Colour of one field on the bit ruler.
fn field_color(field: BitField) -> Color32 {
    match field {
        BitField::StartOfFrame => Color32::from_rgb(235, 120, 120),
        BitField::Identifier => Color32::from_rgb(110, 170, 240),
        BitField::SubstituteRemoteRequest | BitField::IdentifierExtension => {
            Color32::from_rgb(150, 140, 230)
        }
        BitField::RemoteRequest => Color32::from_rgb(120, 150, 225),
        BitField::Reserved => Color32::from_rgb(130, 140, 160),
        BitField::DataLength => Color32::from_rgb(230, 180, 90),
        BitField::Data => Color32::from_rgb(120, 210, 160),
        BitField::Crc => Color32::from_rgb(200, 140, 220),
        BitField::CrcDelimiter | BitField::AcknowledgeDelimiter => {
            Color32::from_rgb(150, 150, 160)
        }
        BitField::Acknowledge => Color32::from_rgb(240, 200, 110),
        BitField::EndOfFrame => Color32::from_rgb(150, 160, 180),
        BitField::Intermission => Color32::from_rgb(110, 120, 135),
        BitField::Stuff => Color32::from_rgb(105, 105, 115),
    }
}

/// Text of the hover box of one bit.
fn bit_tooltip(ui: &mut egui::Ui, index: usize, frame: &TraceFrame, bit: TraceBit) {
    ui.label(
        RichText::new(format!("位 #{index} · {}", bit.caption()))
            .strong()
            .color(field_color(bit.field)),
    );
    ui.label(format!("电平 {}", bit.level_label()));
    ui.label(format!("逻辑值 {}", bit.value()));
    ui.label(
        RichText::new(format!("{:.3} ms", bit.time_ns as f64 / 1e6))
            .small()
            .color(Color32::GRAY),
    );
    if let Some(decoded) = frame.frame {
        ui.label(
            RichText::new(format!(
                "{} ID={}",
                if decoded.extended { "扩展帧" } else { "标准帧" },
                decoded.id_label()
            ))
            .small()
            .color(Color32::GRAY),
        );
    }
}

/// One line summary of a decoded frame.
fn frame_summary(trace: &TraceFrame) -> String {
    let mut parts = Vec::new();
    match trace.frame {
        Some(frame) => {
            parts.push(format!(
                "{}{}",
                if frame.extended { "扩展" } else { "标准" },
                if frame.remote { "远程帧" } else { "数据帧" }
            ));
            parts.push(format!("ID={}", frame.id_label()));
            parts.push(format!("DLC={}", frame.dlc));
            parts.push(format!("数据 {}", frame.data_label()));
        }
        None => parts.push("帧头未完整".to_string()),
    }
    parts.push(format!(
        "{:.3} – {:.3} ms",
        trace.start_ns as f64 / 1e6,
        trace.end_ns as f64 / 1e6
    ));
    parts.push(format!(
        "{} 位（{} 填充）",
        trace.bits.len(),
        trace.stuff_bits()
    ));
    parts.push(if trace.crc_ok {
        "CRC ✓".to_string()
    } else {
        "CRC ✗".to_string()
    });
    parts.push(if trace.acknowledged {
        "已应答".to_string()
    } else {
        "无应答".to_string()
    });
    parts.join(" · ")
}

/// Rectangle a probe occupies on the canvas, used by the symbol painter.
pub fn screen_rect(rect: Rect) -> Rect {
    rect.shrink(rect.width() * 0.12)
}
