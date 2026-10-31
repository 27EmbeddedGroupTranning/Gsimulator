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
            .default_size([600.0, 330.0])
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
            if ui.selectable_label(self.category.is_none(), "全部").clicked() {
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
            .filter(|kind| self.category.map_or(true, |shown| kind.category() == shown))
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
                            if picked.is_some() {
                                place = picked;
                            }
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
    let mut categories: Vec<&'static str> = Vec::new();
    for kind in DeviceKind::ALL {
        if !categories.contains(&kind.category()) {
            categories.push(kind.category());
        }
    }
    categories
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
    painter.rect_stroke(
        body,
        4.0,
        Stroke::new(1.0, COLOR_EDGE),
        StrokeKind::Inside,
    );

    let pins = kind.pins();
    let totals = [
        pins.iter().filter(|def| def.side == Side::Left).count(),
        pins.iter().filter(|def| def.side == Side::Right).count(),
    ];
    let mut seen = [0usize; 2];
    for def in pins.iter() {
        let column = match def.side {
            Side::Left => 0,
            Side::Right => 1,
        };
        seen[column] += 1;
        let fraction = seen[column] as f32 / (totals[column] + 1) as f32;
        let y = body.top() + body.height() * fraction;
        let tip = match def.side {
            Side::Left => Pos2::new(body.left() - 13.0, y),
            Side::Right => Pos2::new(body.right() + 13.0, y),
        };
        let base = match def.side {
            Side::Left => Pos2::new(body.left(), y),
            Side::Right => Pos2::new(body.right(), y),
        };
        painter.line_segment([base, tip], Stroke::new(1.4, COLOR_PIN));
        painter.circle_filled(tip, 2.0, COLOR_PIN);
    }

    let accent = match kind {
        DeviceKind::Led => Color32::from_rgb(230, 60, 50),
        DeviceKind::Uart => COLOR_PIN,
        DeviceKind::Encoder => Color32::from_rgb(90, 226, 120),
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
    }
}
