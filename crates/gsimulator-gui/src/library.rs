//! Device library window: the parts palette of the schematic editor.
//!
//! Instead of parking one toolbar button per part, the window lists every
//! [`DeviceKind`] with a symbol preview, sorted into categories and searchable
//! by name. A part is placed either by clicking **放置** and then clicking the
//! canvas, or by dragging its card straight onto the canvas.

use eframe::egui;
use egui::{Align2, Color32, Pos2, Rect, RichText, Sense, Shape, Stroke, StrokeKind, Vec2};

use crate::board::{Board, DeviceKind, Side};

/// Width of one part card, in points.
const CARD_WIDTH: f32 = 170.0;
/// Height reserved for one part card, so a row of cards lines up.
const CARD_HEIGHT: f32 = 156.0;
/// Height of the symbol preview at the top of a card.
const THUMB_HEIGHT: f32 = 50.0;

const COLOR_PREVIEW: Color32 = Color32::from_rgb(26, 30, 36);
const COLOR_BODY: Color32 = Color32::from_rgb(42, 48, 58);
const COLOR_EDGE: Color32 = Color32::from_rgb(96, 108, 124);
const COLOR_PIN: Color32 = Color32::from_rgb(120, 196, 240);
const COLOR_DIM: Color32 = Color32::from_rgb(140, 148, 160);

/// Payload of a library card dragged onto the schematic.
#[derive(Clone, Copy, Debug)]
pub struct DeviceDrag(pub DeviceKind);

/// State of the device library window.
#[derive(Debug, Default)]
pub struct DeviceLibrary {
    /// `true` while the window is open.
    pub open: bool,
    /// Case insensitive filter typed by the user.
    filter: String,
    /// Category tab; `None` shows every part.
    category: Option<&'static str>,
}

impl DeviceLibrary {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn toggle(&mut self) {
        self.open = !self.open;
    }

    /// Draws the window and returns the part the user asked to place.
    pub fn show(&mut self, ctx: &egui::Context, board: &Board) -> Option<DeviceKind> {
        if !self.open {
            return None;
        }
        let mut place = None;
        let mut open = self.open;
        egui::Window::new("器件库")
            .id(egui::Id::new("device-library"))
            .open(&mut open)
            .default_size([600.0, 470.0])
            .default_pos([260.0, 120.0])
            .resizable(true)
            .vscroll(false)
            .show(ctx, |ui| place = self.body(ui, board));
        self.open = open;
        place
    }

    fn body(&mut self, ui: &mut egui::Ui, board: &Board) -> Option<DeviceKind> {
        let mut place = None;

        ui.horizontal(|ui| {
            ui.label(RichText::new("🔍").color(COLOR_DIM));
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .desired_width(170.0)
                    .hint_text("LED / 串口 / 编码器…"),
            );
            if ui.small_button("清除").clicked() {
                self.filter.clear();
            }
            ui.label(
                RichText::new(format!("画布上已有 {} 个器件", board.devices().len()))
                    .small()
                    .color(COLOR_DIM),
            );
        });
        ui.label(
            RichText::new("单击“放置”后点画布确定位置，或直接把卡片拖到画布上")
                .small()
                .color(COLOR_DIM),
        );
        ui.separator();

        ui.horizontal_wrapped(|ui| {
            if ui
                .selectable_label(self.category.is_none(), "全部")
                .clicked()
            {
                self.category = None;
            }
            for category in categories() {
                if ui
                    .selectable_label(self.category == Some(category), category)
                    .clicked()
                {
                    self.category = Some(category);
                }
            }
        });
        ui.separator();

        let filter = self.filter.trim().to_lowercase();
        let kinds: Vec<DeviceKind> = DeviceKind::ALL
            .into_iter()
            .filter(|kind| self.category.is_none_or(|shown| kind.category() == shown))
            .filter(|kind| filter.is_empty() || kind.matches(&filter))
            .collect();

        if kinds.is_empty() {
            ui.label(RichText::new("没有匹配的器件").color(COLOR_DIM));
            return None;
        }

        egui::ScrollArea::vertical()
            .id_salt("device-library-list")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                let spacing = ui.spacing().item_spacing.x;
                let columns = (((ui.available_width() + spacing) / (CARD_WIDTH + spacing))
                    .floor()
                    .max(1.0)) as usize;
                for row in kinds.chunks(columns) {
                    ui.horizontal_top(|ui| {
                        for kind in row {
                            let placed = board
                                .devices()
                                .iter()
                                .filter(|device| device.kind == *kind)
                                .count();
                            let picked = ui
                                .allocate_ui_with_layout(
                                    Vec2::new(CARD_WIDTH, CARD_HEIGHT),
                                    egui::Layout::top_down(egui::Align::Min),
                                    |ui| self.card(ui, *kind, placed),
                                )
                                .inner;
                            place = picked.or(place);
                        }
                    });
                }
            });

        place
    }

    /// One draggable card; returns the kind when its 放置 button is pressed.
    fn card(&self, ui: &mut egui::Ui, kind: DeviceKind, placed: usize) -> Option<DeviceKind> {
        let mut place = None;
        ui.dnd_drag_source(
            egui::Id::new(("device-card", kind.slug())),
            DeviceDrag(kind),
            |ui| {
                egui::Frame::group(ui.style()).show(ui, |ui| {
                    ui.set_width(CARD_WIDTH - 12.0);
                    thumbnail(ui, kind);
                    ui.add_space(2.0);
                    ui.label(RichText::new(kind.label()).strong());
                    ui.label(RichText::new(kind.summary()).small().color(COLOR_DIM));
                    ui.label(
                        RichText::new(format!("引脚 {}", kind.pin_summary()))
                            .monospace()
                            .small()
                            .color(COLOR_DIM),
                    );
                    ui.add_space(2.0);
                    ui.horizontal(|ui| {
                        if ui
                            .button("放置")
                            .on_hover_text("点一下，再到画布上单击落点")
                            .clicked()
                        {
                            place = Some(kind);
                        }
                        if placed > 0 {
                            ui.label(
                                RichText::new(format!("已有 {placed}"))
                                    .small()
                                    .color(COLOR_DIM),
                            );
                        }
                    });
                });
            },
        );
        place
    }
}

/// Categories in the order they appear in the tab row.
fn categories() -> Vec<&'static str> {
    const PREFERRED: [&str; 6] = ["电源", "时钟", "指示", "交互", "通信", "显示"];
    let mut categories: Vec<&'static str> = Vec::new();
    for category in PREFERRED {
        if DeviceKind::ALL
            .iter()
            .any(|kind| kind.category() == category)
        {
            categories.push(category);
        }
    }
    for kind in DeviceKind::ALL {
        if !categories.contains(&kind.category()) {
            categories.push(kind.category());
        }
    }
    categories
}

/// Index of a side in the `[left, right, top, bottom]` arrays.
fn side_index(side: Side) -> usize {
    match side {
        Side::Left => 0,
        Side::Right => 1,
        Side::Top => 2,
        Side::Bottom => 3,
    }
}

/// Symbol preview drawn at the top of a card, in the spirit of the canvas.
fn thumbnail(ui: &mut egui::Ui, kind: DeviceKind) {
    let width = ui.available_width().max(80.0);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, THUMB_HEIGHT), Sense::hover());
    let painter = ui.painter().with_clip_rect(rect);
    painter.rect_filled(rect, 5.0, COLOR_PREVIEW);

    let body = Rect::from_center_size(
        rect.center(),
        Vec2::new(rect.width() * 0.46, rect.height() * 0.56),
    );
    painter.rect_filled(body, 4.0, COLOR_BODY);
    painter.rect_stroke(body, 4.0, Stroke::new(1.0, COLOR_EDGE), StrokeKind::Inside);

    // Pins sit on the edge their role assigns them: signals on the left, bus
    // and outputs on the right, `VCC`/`GND` along the top.
    let pins = kind.pins();
    let mut totals = [0usize; 4];
    for def in pins {
        totals[side_index(def.side)] += 1;
    }
    let mut seen = [0usize; 4];
    for def in pins {
        let side = side_index(def.side);
        seen[side] += 1;
        let fraction = seen[side] as f32 / (totals[side] + 1) as f32;
        let (base, tip) = match def.side {
            Side::Left => {
                let y = body.top() + body.height() * fraction;
                (Pos2::new(body.left(), y), Pos2::new(body.left() - 12.0, y))
            }
            Side::Right => {
                let y = body.top() + body.height() * fraction;
                (
                    Pos2::new(body.right(), y),
                    Pos2::new(body.right() + 12.0, y),
                )
            }
            Side::Top => {
                let x = body.left() + body.width() * fraction;
                (Pos2::new(x, body.top()), Pos2::new(x, body.top() - 7.0))
            }
            Side::Bottom => {
                let x = body.left() + body.width() * fraction;
                (
                    Pos2::new(x, body.bottom()),
                    Pos2::new(x, body.bottom() + 7.0),
                )
            }
        };
        painter.line_segment([base, tip], Stroke::new(1.4, COLOR_PIN));
        painter.circle_filled(tip, 2.0, COLOR_PIN);
    }

    let accent = match kind {
        DeviceKind::Led => Color32::from_rgb(230, 60, 50),
        DeviceKind::Uart => COLOR_PIN,
        DeviceKind::Encoder => Color32::from_rgb(90, 226, 120),
        DeviceKind::Vcc => Color32::from_rgb(238, 96, 96),
        DeviceKind::Gnd => Color32::from_rgb(120, 150, 200),
        DeviceKind::Can => Color32::from_rgb(245, 200, 60),
        DeviceKind::Matrix => Color32::from_rgb(70, 200, 255),
        DeviceKind::HseCrystal | DeviceKind::LseCrystal => Color32::from_rgb(190, 150, 255),
        DeviceKind::Resistor => Color32::from_rgb(210, 170, 120),
        DeviceKind::Scope => Color32::from_rgb(120, 230, 220),
        DeviceKind::Motor => Color32::from_rgb(110, 205, 165),
        DeviceKind::Junction => Color32::from_rgb(120, 196, 240),
    };
    match kind {
        DeviceKind::Led => {
            let center = body.center();
            let s = (body.height() * 0.22).max(4.0);
            painter.add(Shape::convex_polygon(
                vec![
                    Pos2::new(center.x - s, center.y - s),
                    Pos2::new(center.x - s, center.y + s),
                    Pos2::new(center.x + s * 0.7, center.y),
                ],
                accent,
                Stroke::new(1.0, accent),
            ));
            painter.line_segment(
                [
                    Pos2::new(center.x + s * 0.7, center.y - s),
                    Pos2::new(center.x + s * 0.7, center.y + s),
                ],
                Stroke::new(2.0, accent),
            );
        }
        DeviceKind::Uart => {
            let center = body.center();
            let half = body.width() * 0.26;
            for (offset, forward) in [(-5.0f32, true), (5.0, false)] {
                let y = center.y + offset;
                let (from, tip) = if forward {
                    (center.x - half, center.x + half)
                } else {
                    (center.x + half, center.x - half)
                };
                painter.line_segment(
                    [Pos2::new(from, y), Pos2::new(tip, y)],
                    Stroke::new(1.4, accent),
                );
                let back = tip + if forward { -5.0 } else { 5.0 };
                painter.line_segment(
                    [Pos2::new(back, y - 3.5), Pos2::new(tip, y)],
                    Stroke::new(1.4, accent),
                );
                painter.line_segment(
                    [Pos2::new(back, y + 3.5), Pos2::new(tip, y)],
                    Stroke::new(1.4, accent),
                );
            }
        }
        DeviceKind::Encoder => {
            let center = body.center();
            let radius = (body.height() * 0.30).max(5.0);
            painter.circle_stroke(center, radius, Stroke::new(1.6, accent));
            painter.line_segment(
                [
                    Pos2::new(center.x, center.y - radius),
                    Pos2::new(center.x, center.y - radius * 1.9),
                ],
                Stroke::new(1.6, accent),
            );
            painter.text(
                center,
                Align2::CENTER_CENTER,
                "A B",
                egui::FontId::monospace(7.0),
                COLOR_DIM,
            );
        }
        DeviceKind::Vcc => {
            let center = body.center();
            let len = body.height() * 0.28;
            painter.line_segment(
                [
                    Pos2::new(center.x, center.y + len),
                    Pos2::new(center.x, center.y - len),
                ],
                Stroke::new(1.8, accent),
            );
            painter.line_segment(
                [
                    Pos2::new(center.x - len * 0.7, center.y - len * 0.2),
                    Pos2::new(center.x, center.y - len),
                ],
                Stroke::new(1.8, accent),
            );
            painter.line_segment(
                [
                    Pos2::new(center.x + len * 0.7, center.y - len * 0.2),
                    Pos2::new(center.x, center.y - len),
                ],
                Stroke::new(1.8, accent),
            );
        }
        DeviceKind::Gnd => {
            let center = body.center();
            let len = body.height() * 0.32;
            for (index, width) in [1.2f32, 0.8, 0.4].into_iter().enumerate() {
                let y = center.y - len * 0.5 + len * 0.5 * index as f32;
                painter.line_segment(
                    [
                        Pos2::new(center.x - len * width, y),
                        Pos2::new(center.x + len * width, y),
                    ],
                    Stroke::new(1.8, accent),
                );
            }
        }
        DeviceKind::Can => {
            let center = body.center();
            let half = body.width() * 0.24;
            for (offset, forward) in [(-4.0f32, true), (4.0, false)] {
                let y = center.y + offset;
                let (from, tip) = if forward {
                    (center.x - half, center.x + half)
                } else {
                    (center.x + half, center.x - half)
                };
                painter.line_segment(
                    [Pos2::new(from, y), Pos2::new(tip, y)],
                    Stroke::new(1.4, accent),
                );
                let back = tip + if forward { -4.5 } else { 4.5 };
                painter.line_segment(
                    [Pos2::new(back, y - 3.0), Pos2::new(tip, y)],
                    Stroke::new(1.4, accent),
                );
                painter.line_segment(
                    [Pos2::new(back, y + 3.0), Pos2::new(tip, y)],
                    Stroke::new(1.4, accent),
                );
            }
        }
        DeviceKind::Matrix => {
            let grid = body.shrink(body.width() * 0.14);
            let cell = grid.width().min(grid.height()) / 8.0;
            for row in 0..8 {
                for column in 0..8 {
                    // A checker board reads as "matrix" at thumbnail size.
                    if (row + column) % 3 != 0 {
                        continue;
                    }
                    let min = Pos2::new(
                        grid.left() + column as f32 * cell + cell * 0.12,
                        grid.top() + row as f32 * cell + cell * 0.12,
                    );
                    painter.rect_filled(
                        Rect::from_min_size(min, Vec2::splat(cell * 0.76)),
                        cell * 0.2,
                        accent,
                    );
                }
            }
        }
        DeviceKind::HseCrystal | DeviceKind::LseCrystal => {
            // Quartz: two plates around the body, leads to both terminals.
            let center = body.center();
            let plate = body.height() * 0.26;
            let spacing = body.width() * 0.14;
            let stroke = Stroke::new(1.4, accent);
            painter.line_segment(
                [
                    Pos2::new(body.left() + 8.0, center.y),
                    Pos2::new(center.x - spacing, center.y),
                ],
                stroke,
            );
            painter.line_segment(
                [
                    Pos2::new(center.x + spacing, center.y),
                    Pos2::new(body.right() - 8.0, center.y),
                ],
                stroke,
            );
            for x in [center.x - spacing, center.x + spacing] {
                painter.line_segment(
                    [
                        Pos2::new(x, center.y - plate),
                        Pos2::new(x, center.y + plate),
                    ],
                    stroke,
                );
            }
            painter.rect_filled(
                Rect::from_center_size(center, Vec2::new(spacing * 0.9, plate * 1.4)),
                2.0,
                accent,
            );
        }
        DeviceKind::Resistor => {
            // Zig-zag between the two leads.
            let center = body.center();
            let half = body.width() * 0.16;
            let amplitude = body.height() * 0.18;
            let stroke = Stroke::new(1.3, accent);
            painter.line_segment(
                [
                    Pos2::new(body.left() + 6.0, center.y),
                    Pos2::new(center.x - half, center.y),
                ],
                stroke,
            );
            let mut points = vec![Pos2::new(center.x - half, center.y)];
            for step in 0..6 {
                let x = center.x - half + half * 2.0 * (step as f32 + 0.5) / 6.0;
                let y = center.y + if step % 2 == 0 { -amplitude } else { amplitude };
                points.push(Pos2::new(x, y));
            }
            points.push(Pos2::new(center.x + half, center.y));
            painter.add(Shape::line(points, stroke));
            painter.line_segment(
                [
                    Pos2::new(center.x + half, center.y),
                    Pos2::new(body.right() - 6.0, center.y),
                ],
                stroke,
            );
        }
        DeviceKind::Scope => {
            // A little screen with a trace on it.
            let screen = body.shrink(body.width() * 0.12);
            painter.rect_filled(screen, 2.0, Color32::from_rgb(18, 26, 24));
            let stroke = Stroke::new(1.2, accent);
            let mut points = Vec::new();
            for step in 0..=16 {
                let t = step as f32 / 16.0;
                points.push(Pos2::new(
                    screen.left() + screen.width() * t,
                    screen.center().y - (t * std::f32::consts::TAU).sin() * screen.height() * 0.28,
                ));
            }
            painter.add(Shape::line(points, stroke));
        }
        DeviceKind::Motor => {
            // A body with an `M` and a mark that follows the shaft.
            let center = body.center();
            let radius = (body.height() * 0.3).max(4.0);
            painter.circle_stroke(center, radius, Stroke::new(1.4, accent));
            let tip = Pos2::new(center.x + radius * 0.8, center.y - radius * 0.6);
            painter.line_segment([center, tip], Stroke::new(1.6, accent));
        }
        DeviceKind::Junction => {
            // A tap point: the dot that sits on a wire, with a short lead.
            let center = body.center();
            painter.line_segment(
                [
                    Pos2::new(center.x - body.width() * 0.4, center.y),
                    Pos2::new(center.x + body.width() * 0.4, center.y),
                ],
                Stroke::new(1.6, accent),
            );
            painter.circle_filled(center, 4.0, accent);
        }
    }
}
