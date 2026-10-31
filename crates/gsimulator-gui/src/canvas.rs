//! The schematic canvas.
//!
//! Everything is drawn in "board space" and mapped to the screen with a pan and
//! a zoom factor. Interaction follows the usual EDA flow: click a pin to start a
//! wire, click a second pin to finish it, `Delete` removes the selected wire or
//! device, and the wheel zooms around the pointer. Over a rotary encoder the
//! wheel turns the knob instead, and a click presses its shaft.

use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Ui, Vec2};
use gsim_devices::Direction;

use crate::board::{
    Board, BoardState, DeviceId, DeviceKind, NetLevel, PinLevels, PinRef, Rail, Side, STUB,
};
use crate::library::DeviceDrag;

/// Size of the MCU symbol.
const MCU_SIZE: Vec2 = Vec2::new(158.0, 22.0 + 19.0 * PIN_PITCH + 20.0);
/// Top-left corner of the MCU symbol in board space.
const MCU_ORIGIN: Pos2 = Pos2::new(70.0, 60.0);
/// Distance between two pin rows.
const PIN_PITCH: f32 = 17.0;
/// How far the MCU pin stubs stick out.
const MCU_STUB: f32 = 26.0;
/// First pin row offset below the symbol's top edge.
const FIRST_ROW: f32 = 42.0;

const COLOR_BG: Color32 = Color32::from_rgb(24, 27, 32);
const COLOR_GRID: Color32 = Color32::from_rgb(38, 43, 51);
const COLOR_MCU_BODY: Color32 = Color32::from_rgb(42, 48, 58);
const COLOR_MCU_EDGE: Color32 = Color32::from_rgb(96, 108, 124);
const COLOR_TEXT: Color32 = Color32::from_rgb(206, 214, 224);
const COLOR_DIM: Color32 = Color32::from_rgb(130, 140, 152);
const COLOR_HIGH: Color32 = Color32::from_rgb(90, 226, 120);
const COLOR_LOW: Color32 = Color32::from_rgb(92, 100, 112);
const COLOR_WIRE: Color32 = Color32::from_rgb(120, 196, 240);
const COLOR_WIRE_SELECTED: Color32 = Color32::from_rgb(255, 196, 80);
/// Fill of the translucent symbol preview shown while placing a part
/// (the wire blue at 10% alpha, premultiplied).
const COLOR_GHOST_FILL: Color32 = Color32::from_rgba_premultiplied(12, 20, 24, 26);
const COLOR_VCC: Color32 = Color32::from_rgb(238, 96, 96);
const COLOR_GND: Color32 = Color32::from_rgb(120, 150, 200);
const COLOR_ERROR: Color32 = Color32::from_rgb(240, 100, 100);

/// What is selected on the canvas.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Selection {
    Wire(usize),
    Device(DeviceId),
}

#[derive(Clone, Copy, Debug)]
enum Drag {
    Device { id: DeviceId, grab: Vec2 },
    Rail { rail: Rail, grab: Vec2 },
}

/// Persistent view and interaction state of the canvas.
pub struct Schematic {
    pub zoom: f32,
    pub pan: Vec2,
    wire_from: Option<PinRef>,
    selected: Option<Selection>,
    drag: Option<Drag>,
    message: Option<(String, bool)>,
    /// Set when the user asks to clear the board from the canvas overlay.
    pub request_demo: bool,
    /// Device picked in the library window; the next click drops it.
    pending: Option<DeviceKind>,
    /// Encoder whose shaft is held down, and when to release it.
    press: Option<(DeviceId, std::time::Instant)>,
    /// Rectangle the canvas occupied in the last frame (debug/tests).
    pub last_rect: Rect,
}

impl Default for Schematic {
    fn default() -> Self {
        Schematic {
            zoom: 1.0,
            pan: Vec2::new(24.0, 0.0),
            wire_from: None,
            selected: None,
            drag: None,
            message: None,
            request_demo: false,
            pending: None,
            press: None,
            last_rect: Rect::NOTHING,
        }
    }
}

/// What the canvas wants the application to do after this frame.
#[derive(Default)]
pub struct SchematicResponse {
    pub hover_pin: Option<PinRef>,
    pub message: Option<(String, bool)>,
}

impl Schematic {
    pub fn wire_source(&self) -> Option<PinRef> {
        self.wire_from
    }

    /// Starts a wire from `pin`, used by the pin matrix in the status panel.
    pub fn request_wire_from(&mut self, pin: PinRef) {
        self.wire_from = Some(pin);
    }

    pub fn cancel_wire(&mut self) {
        self.wire_from = None;
    }

    /// Arms the canvas so the next click drops a `kind` symbol there, the
    /// "pick a part, then click to place it" flow of an EDA tool.
    pub fn place_pending(&mut self, kind: DeviceKind) {
        self.pending = Some(kind);
        self.wire_from = None;
    }

    /// Kind waiting for its placement click, if any.
    pub fn pending_device(&self) -> Option<DeviceKind> {
        self.pending
    }

    pub fn cancel_pending(&mut self) {
        self.pending = None;
    }

    /// Drops the armed device so its body is centred on `point`.
    ///
    /// Returns the new device id; `None` when nothing was armed.
    pub fn place_pending_at(&mut self, board: &mut Board, point: Pos2) -> Option<DeviceId> {
        let kind = self.pending.take()?;
        let id = board.add_device(kind, point - kind.body_size() * 0.5);
        self.selected = Some(Selection::Device(id));
        self.wire_from = None;
        let name = board
            .device(id)
            .map(|device| device.name.clone())
            .unwrap_or_default();
        self.say(format!("已放置 {name}，点击引脚开始连线"), false);
        Some(id)
    }

    /// Centres and scales the drawing so the whole board is visible.
    pub fn fit(&mut self, canvas: Rect, board: &Board) {
        if !canvas.is_finite() || canvas.width() < 32.0 || canvas.height() < 32.0 {
            return;
        }
        let bounds = content_bounds(board);
        let zoom = (canvas.width() / bounds.width())
            .min(canvas.height() / bounds.height())
            .clamp(0.35, 1.4)
            * 0.92;
        self.zoom = zoom;
        let canvas_center = canvas.center() - canvas.min;
        self.pan = canvas_center - bounds.center().to_vec2() * zoom;
    }

    pub fn say(&mut self, text: impl Into<String>, error: bool) {
        self.message = Some((text.into(), error));
    }

    fn to_screen(&self, point: Pos2, origin: Pos2) -> Pos2 {
        (point.to_vec2() * self.zoom + self.pan).to_pos2() + origin.to_vec2()
    }

    fn to_board(&self, point: Pos2, origin: Pos2) -> Pos2 {
        (((point - origin) - self.pan) / self.zoom).to_pos2()
    }

    fn font(&self, size: f32) -> FontId {
        FontId::proportional((size * self.zoom).max(6.0))
    }

    fn stroke(&self, width: f32) -> f32 {
        (width * self.zoom).max(1.0)
    }

    /// Draws the schematic and handles the interaction for this frame.
    pub fn show(
        &mut self,
        ui: &mut Ui,
        board: &mut Board,
        levels: &PinLevels,
        state: &BoardState,
        now_ns: u64,
    ) -> SchematicResponse {
        let mut out = SchematicResponse::default();
        let rect = ui.available_rect_before_wrap();
        let (response, painter) = ui.allocate_painter(rect.size(), Sense::click_and_drag());
        let origin = response.rect.min;
        self.last_rect = response.rect;
        painter.rect_filled(response.rect, 0.0, COLOR_BG);

        // Releases a knob that was clicked in an earlier frame.
        if let Some((id, deadline)) = self.press {
            if std::time::Instant::now() >= deadline {
                if let Some(encoder) = board
                    .device_mut(id)
                    .and_then(|device| device.hardware.encoder_mut())
                {
                    encoder.press(false);
                }
                self.press = None;
            }
        }

        let pointer = response
            .hover_pos()
            .or_else(|| response.interact_pointer_pos());
        let pointer_board = pointer.map(|point| self.to_board(point, origin));

        // ---------------------------------------------------------- zoom/pan
        let knob = pointer_board.and_then(|point| self.encoder_at(board, point));
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.01 {
            if let (Some(id), true) = (knob, response.hovered()) {
                // The wheel turns the knob instead of zooming the canvas.
                let direction = if scroll > 0.0 {
                    Direction::Clockwise
                } else {
                    Direction::CounterClockwise
                };
                let detents = ((scroll.abs() / 40.0).round() as u32).clamp(1, 4);
                if let Some(encoder) = board
                    .device_mut(id)
                    .and_then(|device| device.hardware.encoder_mut())
                {
                    encoder.rotate(direction, detents, now_ns);
                }
            } else if response.hovered() {
                let pointer = ui
                    .input(|i| i.pointer.hover_pos())
                    .unwrap_or_else(|| response.rect.center());
                let before = self.to_board(pointer, origin);
                self.zoom = (self.zoom * (1.0 + scroll * 0.0015)).clamp(0.4, 2.5);
                let after = self.to_board(pointer, origin);
                self.pan += (after - before) * self.zoom;
            }
        }
        if response.dragged_by(egui::PointerButton::Middle) {
            self.pan += response.drag_delta();
        }

        // ------------------------------------------------------- hit testing
        let hover_pin = pointer_board.and_then(|point| self.pin_at(board, point));
        out.hover_pin = hover_pin;

        if response.clicked() {
            if let Some(point) = pointer_board {
                if self.pending.is_some() {
                    self.place_pending_at(board, point);
                } else {
                    match self.pin_at(board, point) {
                        Some(pin) => self.click_pin(board, pin),
                        None => {
                            if let Some(index) = self.wire_at(board, point) {
                                self.selected = Some(Selection::Wire(index));
                                self.wire_from = None;
                            } else if let Some(id) = self.encoder_at(board, point) {
                                // Clicking the shaft is a button press: a short
                                // pulse, so polling firmware still catches it.
                                if let Some(encoder) = board
                                    .device_mut(id)
                                    .and_then(|device| device.hardware.encoder_mut())
                                {
                                    encoder.press(true);
                                }
                                self.press = Some((
                                    id,
                                    std::time::Instant::now()
                                        + std::time::Duration::from_millis(160),
                                ));
                                self.selected = Some(Selection::Device(id));
                                self.wire_from = None;
                            } else if let Some(id) = self.device_at(board, point) {
                                self.selected = Some(Selection::Device(id));
                                self.wire_from = None;
                            } else {
                                self.selected = None;
                                self.wire_from = None;
                            }
                        }
                    }
                }
            }
        }

        // A card dragged out of the library window lands under the pointer.
        if let Some(dragged) = response.dnd_release_payload::<DeviceDrag>() {
            let drop_point = ui
                .input(|i| i.pointer.latest_pos())
                .map(|point| self.to_board(point, origin));
            if let Some(point) = drop_point {
                let id = board.add_device(dragged.0, point - dragged.0.body_size() * 0.5);
                self.selected = Some(Selection::Device(id));
                self.wire_from = None;
                self.pending = None;
                let name = board
                    .device(id)
                    .map(|device| device.name.clone())
                    .unwrap_or_default();
                self.say(format!("已放置 {name}，点击引脚开始连线"), false);
            }
        }

        if response.drag_started() && self.pending.is_none() {
            if let Some(point) = pointer_board {
                if let Some(pin) = self.pin_at(board, point) {
                    // Dragging from a pin works like the second half of a
                    // click-click: the release on another pin completes it.
                    self.wire_from = Some(pin);
                } else if let Some(id) = self.device_at(board, point) {
                    let device = board.device(id).expect("hit device exists");
                    self.drag = Some(Drag::Device {
                        id,
                        grab: point - device.pos,
                    });
                    self.selected = Some(Selection::Device(id));
                } else if let Some(rail) = self.rail_at(board, point) {
                    self.drag = Some(Drag::Rail {
                        rail,
                        grab: point - board.rail_pos(rail),
                    });
                }
            }
        }
        if let (Some(drag), Some(point)) = (self.drag, pointer_board) {
            match drag {
                Drag::Device { id, grab } => {
                    if let Some(device) = board.device_mut(id) {
                        device.pos = point - grab;
                    }
                }
                Drag::Rail { rail, grab } => {
                    if let Some(node) = board.rail_mut(rail) {
                        node.pos = point - grab;
                    }
                }
            }
        }
        if response.drag_stopped() {
            if let (Some(_), Some(point)) = (self.wire_from, pointer_board) {
                if let Some(pin) = self.pin_at(board, point) {
                    self.click_pin(board, pin);
                }
            }
            self.drag = None;
        }

        // -------------------------------------------------------- key actions
        let (delete, escape) = ui.input(|i| {
            (
                i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace),
                i.key_pressed(egui::Key::Escape),
            )
        });
        if escape {
            self.wire_from = None;
            if self.pending.take().is_some() {
                self.say("已结束放置", false);
            }
        }
        if delete {
            match self.selected {
                Some(Selection::Wire(index)) => {
                    if let Some(wire) = board.remove_wire(index) {
                        let text = format!(
                            "已删除连线 {} - {}",
                            wire.a.label(board),
                            wire.b.label(board)
                        );
                        self.say(text, false);
                        self.selected = None;
                    }
                }
                Some(Selection::Device(id)) => {
                    if let Some(device) = board.device(id) {
                        let text = format!("已删除 {}", device.name);
                        board.remove_device(id);
                        self.say(text, false);
                        self.selected = None;
                    }
                }
                None => {}
            }
        }

        // ------------------------------------------------------------- paint
        self.paint_grid(&painter, origin, &response.rect);
        self.paint_wires(&painter, origin, board, state);
        for device in board.devices() {
            self.paint_device(&painter, origin, device, state.led_lit(device.id));
        }
        for rail in Rail::ALL {
            self.paint_rail(&painter, origin, board, rail);
        }
        self.paint_mcu(&painter, origin, levels, hover_pin);

        if let (Some(kind), Some(point)) = (self.pending, pointer_board) {
            self.paint_ghost(&painter, origin, kind, point);
        }

        if let (Some(from), Some(pointer)) = (self.wire_from, pointer) {
            if let Some(from_pos) = self.pin_screen_pos(board, origin, from) {
                painter.line_segment(
                    [from_pos, pointer],
                    Stroke::new(self.stroke(2.0), COLOR_WIRE.gamma_multiply(0.85)),
                );
                painter.circle_filled(pointer, self.stroke(3.0), COLOR_WIRE);
            }
        }

        self.paint_overlay(&painter, origin, &response.rect, out.hover_pin, board);
        out.message = self.message.clone();
        out
    }

    // ------------------------------------------------------------- wiring API
    pub fn click_pin(&mut self, board: &mut Board, pin: PinRef) {
        match self.wire_from.take() {
            None => {
                self.wire_from = Some(pin);
                self.say(
                    format!("从 {} 开始连线：点击另一个引脚完成", pin.label(board)),
                    false,
                );
            }
            Some(from) if from == pin => {
                self.say("已取消连线", false);
            }
            Some(from) => {
                if board.connect(from, pin) {
                    self.say(
                        format!("已连接 {} → {}", from.label(board), pin.label(board)),
                        false,
                    );
                } else {
                    self.say("这两个引脚已经连在一起了", true);
                }
            }
        }
    }

    // ---------------------------------------------------------- hit testing
    fn pin_at(&self, board: &Board, point: Pos2) -> Option<PinRef> {
        let radius = 9.0;
        for (port, pin, _, pos) in mcu_pin_layout() {
            if pos.distance(point) <= radius {
                return Some(PinRef::Mcu { port, pin });
            }
        }
        for device in board.devices() {
            for (index, _) in device.kind.pins().iter().enumerate() {
                if device.pin_pos(index as u8).distance(point) <= radius {
                    return Some(PinRef::Device {
                        device: device.id,
                        pin: index as u8,
                    });
                }
            }
        }
        for rail in Rail::ALL {
            if board.rail_pos(rail).distance(point) <= radius {
                return Some(PinRef::Rail(rail));
            }
        }
        None
    }

    fn device_at(&self, board: &Board, point: Pos2) -> Option<DeviceId> {
        board
            .devices()
            .iter()
            .rev()
            .find(|device| device.rect().contains(point))
            .map(|device| device.id)
    }

    /// The knob of an encoder under the pointer, if any.
    fn encoder_at(&self, board: &Board, point: Pos2) -> Option<DeviceId> {
        board
            .devices()
            .iter()
            .rev()
            .find(|device| device.kind == DeviceKind::Encoder && device.rect().contains(point))
            .map(|device| device.id)
    }

    fn rail_at(&self, board: &Board, point: Pos2) -> Option<Rail> {
        Rail::ALL.into_iter().find(|rail| {
            Rect::from_center_size(board.rail_pos(*rail), Vec2::splat(34.0)).contains(point)
        })
    }

    fn wire_at(&self, board: &Board, point: Pos2) -> Option<usize> {
        let tolerance = 6.0;
        board.wires().iter().position(|wire| {
            let a = self.pin_pos(board, wire.a);
            let b = self.pin_pos(board, wire.b);
            match (a, b) {
                (Some(a), Some(b)) => distance_to_segment(point, a, b) <= tolerance,
                _ => false,
            }
        })
    }

    fn pin_pos(&self, board: &Board, pin: PinRef) -> Option<Pos2> {
        match pin {
            PinRef::Mcu { port, pin } => mcu_pin_layout()
                .into_iter()
                .find(|(p, n, _, _)| *p == port && *n == pin)
                .map(|(_, _, _, pos)| pos),
            PinRef::Rail(rail) => Some(board.rail_pos(rail)),
            PinRef::Device { device, pin } => {
                board.device(device).map(|device| device.pin_pos(pin))
            }
        }
    }

    fn pin_screen_pos(&self, board: &Board, origin: Pos2, pin: PinRef) -> Option<Pos2> {
        self.pin_pos(board, pin)
            .map(|pos| self.to_screen(pos, origin))
    }

    // --------------------------------------------------------------- painting
    fn paint_grid(&self, painter: &egui::Painter, origin: Pos2, rect: &Rect) {
        let step = 20.0 * self.zoom;
        if step < 6.0 {
            return;
        }
        let start = Pos2::new(
            origin.x + self.pan.x.rem_euclid(step),
            origin.y + self.pan.y.rem_euclid(step),
        );
        let mut x = start.x;
        while x < rect.right() {
            let mut y = start.y;
            while y < rect.bottom() {
                painter.circle_filled(Pos2::new(x, y), 1.0, COLOR_GRID);
                y += step;
            }
            x += step;
        }
    }

    fn paint_mcu(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        levels: &PinLevels,
        hover: Option<PinRef>,
    ) {
        let rect = Rect::from_min_size(MCU_ORIGIN, MCU_SIZE);
        let screen = Rect::from_min_max(
            self.to_screen(rect.min, origin),
            self.to_screen(rect.max, origin),
        );
        painter.rect_filled(screen, 8.0 * self.zoom, COLOR_MCU_BODY);
        painter.rect_stroke(
            screen,
            8.0 * self.zoom,
            Stroke::new(self.stroke(1.5), COLOR_MCU_EDGE),
            StrokeKind::Inside,
        );

        painter.text(
            Pos2::new(screen.center().x, screen.top() + 14.0 * self.zoom),
            Align2::CENTER_CENTER,
            "STM32F103C8T6",
            self.font(12.0),
            COLOR_TEXT,
        );
        painter.text(
            Pos2::new(screen.center().x, screen.top() + 28.0 * self.zoom),
            Align2::CENTER_CENTER,
            "Cortex-M3 · 72 MHz",
            self.font(9.0),
            COLOR_DIM,
        );

        for (port, pin, side, pos) in mcu_pin_layout() {
            let tip = self.to_screen(pos, origin);
            let base = match side {
                Side::Left => Pos2::new(screen.left(), tip.y),
                Side::Right => Pos2::new(screen.right(), tip.y),
            };
            let level = levels.level_of(port, pin);
            let is_output = levels.is_output(port, pin);
            let is_hovered = hover == Some(PinRef::Mcu { port, pin });
            let color = if level { COLOR_HIGH } else { COLOR_LOW };
            painter.line_segment([base, tip], Stroke::new(self.stroke(1.6), color));
            painter.circle_filled(tip, self.stroke(3.0), color);
            if is_output {
                // Filled ring marks a pin the firmware drives.
                painter.circle_stroke(tip, self.stroke(5.0), Stroke::new(self.stroke(1.0), color));
            }
            if is_hovered {
                painter.circle_stroke(
                    tip,
                    self.stroke(8.0),
                    Stroke::new(self.stroke(1.5), Color32::WHITE),
                );
            }
            let label = format!(
                "P{}{}",
                gsim_stm32f1::PORT_NAMES
                    .get(port as usize)
                    .copied()
                    .unwrap_or("?"),
                pin
            );
            let text_pos = match side {
                Side::Left => Pos2::new(screen.left() + 10.0 * self.zoom, tip.y),
                Side::Right => Pos2::new(screen.right() - 10.0 * self.zoom, tip.y),
            };
            painter.text(
                text_pos,
                if side == Side::Left {
                    Align2::LEFT_CENTER
                } else {
                    Align2::RIGHT_CENTER
                },
                label,
                self.font(9.5),
                if is_hovered {
                    Color32::WHITE
                } else {
                    COLOR_TEXT
                },
            );
        }
    }

    fn paint_wires(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        board: &Board,
        state: &BoardState,
    ) {
        for (index, wire) in board.wires().iter().enumerate() {
            let (Some(a), Some(b)) = (self.pin_pos(board, wire.a), self.pin_pos(board, wire.b))
            else {
                continue;
            };
            let selected = self.selected == Some(Selection::Wire(index));
            let net_level = state
                .nets
                .iter()
                .position(|net| net.contains(&wire.a))
                .and_then(|index| state.levels_of_nets.get(index))
                .copied()
                .unwrap_or(NetLevel::Floating);
            let color = if selected {
                COLOR_WIRE_SELECTED
            } else {
                match net_level {
                    NetLevel::High => COLOR_HIGH.gamma_multiply(0.9),
                    NetLevel::Low => COLOR_WIRE.gamma_multiply(0.7),
                    NetLevel::Shorted => COLOR_ERROR,
                    NetLevel::Floating => COLOR_WIRE.gamma_multiply(0.45),
                }
            };
            let width = if selected { 3.0 } else { 2.0 };
            let points = wire_points(self.to_screen(a, origin), self.to_screen(b, origin));
            painter.add(egui::Shape::line(
                points.clone(),
                Stroke::new(self.stroke(width), color),
            ));
            painter.circle_filled(points[0], self.stroke(2.5), color);
            painter.circle_filled(*points.last().expect("non-empty"), self.stroke(2.5), color);
        }
    }

    fn paint_rail(&self, painter: &egui::Painter, origin: Pos2, board: &Board, rail: Rail) {
        let pos = self.to_screen(board.rail_pos(rail), origin);
        let color = match rail {
            Rail::Vcc => COLOR_VCC,
            Rail::Gnd => COLOR_GND,
        };
        let text = if rail.is_high() { "VCC" } else { "GND" };
        let len = 14.0 * self.zoom;
        match rail {
            Rail::Vcc => {
                painter.line_segment(
                    [
                        Pos2::new(pos.x, pos.y - len),
                        Pos2::new(pos.x, pos.y + len * 0.5),
                    ],
                    Stroke::new(self.stroke(1.8), color),
                );
                painter.line_segment(
                    [
                        Pos2::new(pos.x - len * 0.55, pos.y - len * 0.5),
                        Pos2::new(pos.x, pos.y - len),
                    ],
                    Stroke::new(self.stroke(1.8), color),
                );
                painter.line_segment(
                    [
                        Pos2::new(pos.x + len * 0.55, pos.y - len * 0.5),
                        Pos2::new(pos.x, pos.y - len),
                    ],
                    Stroke::new(self.stroke(1.8), color),
                );
            }
            Rail::Gnd => {
                for (index, width) in [1.0f32, 0.66, 0.33].into_iter().enumerate() {
                    let y = pos.y + len * 0.35 * index as f32;
                    painter.line_segment(
                        [
                            Pos2::new(pos.x - len * width, y),
                            Pos2::new(pos.x + len * width, y),
                        ],
                        Stroke::new(self.stroke(1.8), color),
                    );
                }
            }
        }
        painter.circle_stroke(pos, self.stroke(5.0), Stroke::new(self.stroke(1.2), color));
        painter.text(
            Pos2::new(pos.x, pos.y + len * 1.35),
            Align2::CENTER_TOP,
            text,
            self.font(9.5),
            color,
        );
    }

    fn paint_device(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        device: &crate::board::Device,
        lit: bool,
    ) {
        let rect = Rect::from_min_max(
            self.to_screen(device.rect().min, origin),
            self.to_screen(device.rect().max, origin),
        );
        let selected = self.selected == Some(Selection::Device(device.id));
        let base = device.accent.color32();
        let body = if lit {
            Color32::from_rgb(24 + base.r() / 3, 26 + base.g() / 4, 30 + base.b() / 4)
        } else {
            Color32::from_rgb(38, 42, 50)
        };
        if lit {
            // Soft halo so a lit LED reads as "emitting" at any zoom level.
            let glow = Color32::from_rgba_unmultiplied(base.r(), base.g(), base.b(), 26);
            painter.circle_filled(rect.center(), rect.width() * 0.42, glow);
            let inner = Color32::from_rgba_unmultiplied(base.r(), base.g(), base.b(), 40);
            painter.circle_filled(rect.center(), rect.width() * 0.30, inner);
        }
        painter.rect_filled(rect, 7.0 * self.zoom, body);
        painter.rect_stroke(
            rect,
            7.0 * self.zoom,
            Stroke::new(
                self.stroke(if selected { 1.8 } else { 1.2 }),
                if selected {
                    COLOR_WIRE_SELECTED
                } else {
                    COLOR_MCU_EDGE
                },
            ),
            StrokeKind::Inside,
        );

        match device.kind {
            DeviceKind::Led => self.paint_led_symbol(painter, rect, base, lit),
            DeviceKind::Uart => self.paint_uart_symbol(painter, rect, base, device),
            DeviceKind::Encoder => self.paint_encoder_symbol(painter, rect, base, device),
        }

        painter.text(
            Pos2::new(rect.center().x, rect.top() - 8.0 * self.zoom),
            Align2::CENTER_BOTTOM,
            &device.name,
            self.font(10.0),
            COLOR_TEXT,
        );

        for (index, def) in device.kind.pins().iter().enumerate() {
            let tip = self.to_screen(device.pin_pos(index as u8), origin);
            let base_pos = match def.side {
                Side::Left => Pos2::new(rect.left(), tip.y),
                Side::Right => Pos2::new(rect.right(), tip.y),
            };
            painter.line_segment([base_pos, tip], Stroke::new(self.stroke(1.6), COLOR_WIRE));
            painter.circle_filled(tip, self.stroke(3.0), COLOR_WIRE);
            painter.text(
                Pos2::new(
                    if def.side == Side::Left {
                        rect.left() + 6.0 * self.zoom
                    } else {
                        rect.right() - 6.0 * self.zoom
                    },
                    tip.y - 7.0 * self.zoom,
                ),
                if def.side == Side::Left {
                    Align2::LEFT_BOTTOM
                } else {
                    Align2::RIGHT_BOTTOM
                },
                def.name,
                self.font(8.5),
                COLOR_DIM,
            );
        }
    }

    /// Triangle, bar and emission arrows.
    /// Translucent preview of the part waiting for its placement click.
    fn paint_ghost(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        kind: DeviceKind,
        center: Pos2,
    ) {
        let body = Rect::from_min_size(center - kind.body_size() * 0.5, kind.body_size());
        let rect = Rect::from_min_max(
            self.to_screen(body.min, origin),
            self.to_screen(body.max, origin),
        );
        let outline = Color32::from_rgba_unmultiplied(120, 196, 240, 150);
        painter.rect_filled(rect, 7.0 * self.zoom, COLOR_GHOST_FILL);
        painter.rect_stroke(
            rect,
            7.0 * self.zoom,
            Stroke::new(self.stroke(1.4), outline),
            StrokeKind::Inside,
        );
        painter.text(
            rect.center(),
            Align2::CENTER_CENTER,
            kind.label(),
            self.font(10.0),
            COLOR_TEXT,
        );
        for (index, def) in kind.pins().iter().enumerate() {
            let tip = self.to_screen(kind.pin_pos(body, index as u8), origin);
            let base = match def.side {
                Side::Left => Pos2::new(rect.left(), tip.y),
                Side::Right => Pos2::new(rect.right(), tip.y),
            };
            painter.line_segment(
                [base, tip],
                Stroke::new(self.stroke(1.6), outline.gamma_multiply(0.8)),
            );
            painter.circle_filled(tip, self.stroke(3.0), outline);
            painter.text(
                Pos2::new(
                    if def.side == Side::Left {
                        rect.left() + 6.0 * self.zoom
                    } else {
                        rect.right() - 6.0 * self.zoom
                    },
                    tip.y - 7.0 * self.zoom,
                ),
                if def.side == Side::Left {
                    Align2::LEFT_BOTTOM
                } else {
                    Align2::RIGHT_BOTTOM
                },
                def.name,
                self.font(8.5),
                COLOR_DIM,
            );
        }
    }

    fn paint_led_symbol(&self, painter: &egui::Painter, rect: Rect, base: Color32, lit: bool) {
        let center = rect.center();
        let s = (rect.height() * 0.22).max(5.0);
        let fill = if lit {
            base
        } else {
            Color32::from_rgb(70, 76, 88)
        };
        painter.add(egui::Shape::convex_polygon(
            vec![
                Pos2::new(center.x - s, center.y - s),
                Pos2::new(center.x - s, center.y + s),
                Pos2::new(center.x + s * 0.6, center.y),
            ],
            fill,
            Stroke::new(self.stroke(1.0), fill),
        ));
        painter.line_segment(
            [
                Pos2::new(center.x + s * 0.6, center.y - s),
                Pos2::new(center.x + s * 0.6, center.y + s),
            ],
            Stroke::new(self.stroke(2.0), fill),
        );
        for offset in [0.0f32, 1.0] {
            let from = Pos2::new(
                center.x + s * (0.9 + offset * 0.5),
                center.y - s * (1.1 + offset),
            );
            let to = Pos2::new(from.x + s * 0.7, from.y - s * 0.7);
            painter.line_segment([from, to], Stroke::new(self.stroke(1.0), fill));
            painter.line_segment(
                [to, Pos2::new(to.x - s * 0.35, to.y)],
                Stroke::new(self.stroke(1.0), fill),
            );
            painter.line_segment(
                [to, Pos2::new(to.x, to.y + s * 0.35)],
                Stroke::new(self.stroke(1.0), fill),
            );
        }
    }

    /// A little terminal window: port name, the last received line and counters.
    fn paint_uart_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let Some(uart) = device.hardware.uart() else {
            return;
        };
        let left = rect.left() + 8.0 * self.zoom;
        painter.text(
            Pos2::new(left, rect.top() + 9.0 * self.zoom),
            Align2::LEFT_TOP,
            format!("USART{}", uart.unit() + 1),
            self.font(8.5),
            base,
        );

        // Last line of the log, clipped to what fits in the box.
        let last_line = uart
            .rx_text()
            .rsplit('\n')
            .find(|line| !line.is_empty())
            .unwrap_or("");
        let max_chars = (rect.width() / (6.2 * self.zoom)).max(4.0) as usize;
        let shown: String = if last_line.chars().count() > max_chars {
            let skip = last_line.chars().count() - max_chars;
            last_line.chars().skip(skip).collect()
        } else {
            last_line.to_string()
        };
        let text = if shown.is_empty() {
            "（等待数据…）".to_string()
        } else {
            shown
        };
        painter.text(
            Pos2::new(left, rect.center().y),
            Align2::LEFT_CENTER,
            text,
            self.font(9.5),
            if last_line.is_empty() {
                COLOR_DIM
            } else {
                COLOR_TEXT
            },
        );
        painter.text(
            Pos2::new(left, rect.bottom() - 8.0 * self.zoom),
            Align2::LEFT_BOTTOM,
            format!("↓{} ↑{}", uart.rx_bytes(), uart.tx_bytes()),
            self.font(8.0),
            COLOR_DIM,
        );
    }

    /// Knob with a pointer; the pointer follows the detent counter.
    fn paint_encoder_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let Some(encoder) = device.hardware.encoder() else {
            return;
        };
        let center = rect.center();
        let radius = (rect.width().min(rect.height()) * 0.32).max(8.0);
        let pressed = encoder.button();
        let fill = if pressed {
            base.gamma_multiply(0.55)
        } else {
            Color32::from_rgb(52, 58, 68)
        };
        painter.circle_filled(center, radius, fill);
        painter.circle_stroke(
            center,
            radius,
            Stroke::new(self.stroke(1.5), base.gamma_multiply(0.85)),
        );

        // Pointer turns 30° per detent, so turning the knob is visible.
        let angle = -(encoder.detents() as f32) * 0.52 - std::f32::consts::FRAC_PI_2;
        let tip = Pos2::new(
            center.x + angle.cos() * radius * 0.78,
            center.y + angle.sin() * radius * 0.78,
        );
        painter.line_segment(
            [center, tip],
            Stroke::new(self.stroke(2.2), Color32::from_rgb(220, 226, 236)),
        );

        painter.text(
            Pos2::new(center.x, rect.bottom() - 6.0 * self.zoom),
            Align2::CENTER_BOTTOM,
            format!("{}", encoder.detents()),
            self.font(8.5),
            COLOR_DIM,
        );
    }

    fn paint_overlay(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        rect: &Rect,
        hover: Option<PinRef>,
        board: &Board,
    ) {
        let mut y = rect.top() + 12.0;
        let mut line = |text: String, color: Color32, size: f32| {
            painter.text(
                Pos2::new(rect.left() + 12.0, y),
                Align2::LEFT_TOP,
                text,
                FontId::proportional(size),
                color,
            );
            y += size + 4.0;
        };

        if let Some(pin) = hover {
            line(pin.label(board), Color32::WHITE, 11.0);
        }
        if let Some(from) = self.wire_from {
            line(
                format!("连线中：{} → 点击目标引脚（Esc 取消）", from.label(board)),
                COLOR_WIRE_SELECTED,
                11.0,
            );
        }
        if let Some(kind) = self.pending {
            line(
                format!("放置 {}：点击画布确定位置（Esc 结束）", kind.label()),
                COLOR_WIRE_SELECTED,
                11.0,
            );
        }
        if let Some((text, error)) = &self.message {
            line(
                text.clone(),
                if *error { COLOR_ERROR } else { COLOR_DIM },
                11.0,
            );
        }
        let _ = origin;
    }
}

/// Bounding box of everything drawn on the board, in board space.
pub fn content_bounds(board: &Board) -> Rect {
    let mcu = Rect::from_min_size(MCU_ORIGIN, MCU_SIZE);
    let mut bounds = Rect::from_min_max(
        Pos2::new(mcu.left() - MCU_STUB, mcu.top()),
        Pos2::new(mcu.right() + MCU_STUB, mcu.bottom()),
    );
    for rail in Rail::ALL {
        bounds = bounds.union(Rect::from_center_size(
            board.rail_pos(rail),
            Vec2::splat(60.0),
        ));
    }
    for device in board.devices() {
        let rect = device.rect();
        bounds = bounds.union(Rect::from_min_max(
            Pos2::new(rect.left() - STUB - 8.0, rect.top() - 22.0),
            Pos2::new(rect.right() + STUB + 8.0, rect.bottom() + 8.0),
        ));
    }
    bounds
}

/// Screen positions of every MCU pin, in board space.
fn mcu_pin_layout() -> Vec<(u8, u8, Side, Pos2)> {
    let mut pins = Vec::with_capacity(35);
    let left_x = MCU_ORIGIN.x - MCU_STUB;
    let right_x = MCU_ORIGIN.x + MCU_SIZE.x + MCU_STUB;
    for pin in 0..16u8 {
        let y = MCU_ORIGIN.y + FIRST_ROW + f32::from(pin) * PIN_PITCH;
        pins.push((0u8, pin, Side::Left, Pos2::new(left_x, y)));
    }
    let mut row = 0u8;
    for pin in 0..16u8 {
        let y = MCU_ORIGIN.y + FIRST_ROW + f32::from(row) * PIN_PITCH;
        pins.push((1u8, pin, Side::Right, Pos2::new(right_x, y)));
        row += 1;
    }
    for pin in 13..16u8 {
        let y = MCU_ORIGIN.y + FIRST_ROW + f32::from(row) * PIN_PITCH;
        pins.push((2u8, pin, Side::Right, Pos2::new(right_x, y)));
        row += 1;
    }
    pins
}

/// Orthogonal (EDA style) routing between two points.
fn wire_points(a: Pos2, b: Pos2) -> Vec<Pos2> {
    let mid_x = (a.x + b.x) * 0.5;
    vec![a, Pos2::new(mid_x, a.y), Pos2::new(mid_x, b.y), b]
}

fn distance_to_segment(point: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let length_squared = ab.length_sq();
    if length_squared <= f32::EPSILON {
        return point.distance(a);
    }
    let t = ((point - a).dot(ab) / length_squared).clamp(0.0, 1.0);
    point.distance(a + ab * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mcu_layout_covers_every_bonded_pin() {
        let pins = mcu_pin_layout();
        assert_eq!(pins.len(), 35);
        assert!(pins.iter().any(|(p, n, _, _)| *p == 0 && *n == 5));
        assert!(pins.iter().any(|(p, n, _, _)| *p == 2 && *n == 13));
        // No duplicates.
        let mut seen = pins.iter().map(|(p, n, _, _)| (*p, *n)).collect::<Vec<_>>();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), 35);
    }

    #[test]
    fn routing_uses_orthogonal_segments() {
        let points = wire_points(Pos2::new(0.0, 0.0), Pos2::new(100.0, 50.0));
        assert_eq!(points.len(), 4);
        assert_eq!(points[0].y, points[1].y);
        assert_eq!(points[1].x, points[2].x);
        assert_eq!(points[2].y, points[3].y);
    }

    #[test]
    fn point_to_segment_distance_handles_endpoints() {
        let a = Pos2::new(0.0, 0.0);
        let b = Pos2::new(10.0, 0.0);
        assert!(distance_to_segment(Pos2::new(5.0, 3.0), a, b) - 3.0 < 1e-3);
        assert!(distance_to_segment(Pos2::new(-4.0, 0.0), a, b) - 4.0 < 1e-3);
        assert!(distance_to_segment(Pos2::new(0.0, 0.0), a, a) < 1e-3);
    }

    #[test]
    fn zoom_and_pan_round_trip() {
        let schematic = Schematic {
            zoom: 1.7,
            pan: Vec2::new(-30.0, 12.0),
            ..Default::default()
        };
        let origin = Pos2::new(10.0, 20.0);
        let board_point = Pos2::new(123.0, 456.0);
        let screen = schematic.to_screen(board_point, origin);
        let back = schematic.to_board(screen, origin);
        assert!((back - board_point).length() < 1e-3);
    }

    /// Filled rectangles that use the ghost preview colour.
    fn ghost_rects(output: &egui::FullOutput) -> usize {
        fn walk(shape: &egui::Shape, count: &mut usize) {
            match shape {
                egui::Shape::Rect(rect) if rect.fill == COLOR_GHOST_FILL => *count += 1,
                egui::Shape::Vec(shapes) => {
                    for shape in shapes {
                        walk(shape, count);
                    }
                }
                _ => {}
            }
        }
        let mut count = 0;
        for clipped in &output.shapes {
            walk(&clipped.shape, &mut count);
        }
        count
    }

    #[test]
    fn a_part_picked_from_the_library_is_previewed_under_the_pointer() {
        let ctx = egui::Context::default();
        let mut board = Board::demo();
        let levels = PinLevels::default();
        let state = board.evaluate(&levels);
        let mut schematic = Schematic::default();
        let pointer = Pos2::new(300.0, 600.0);

        let mut frame = |events: Vec<egui::Event>,
                         schematic: &mut Schematic,
                         board: &mut Board| {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(
                    Pos2::ZERO,
                    Vec2::new(900.0, 700.0),
                )),
                events,
                ..Default::default()
            };
            ctx.run(input, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    schematic.show(ui, board, &levels, &state, 0);
                });
            })
        };

        frame(
            vec![egui::Event::PointerMoved(pointer)],
            &mut schematic,
            &mut board,
        );
        let idle = frame(Vec::new(), &mut schematic, &mut board);
        assert_eq!(ghost_rects(&idle), 0, "nothing should be previewed yet");

        schematic.place_pending(DeviceKind::Led);
        frame(Vec::new(), &mut schematic, &mut board);
        let armed = frame(Vec::new(), &mut schematic, &mut board);
        assert_eq!(
            ghost_rects(&armed),
            1,
            "the picked part must be previewed where it will land"
        );
    }
}
