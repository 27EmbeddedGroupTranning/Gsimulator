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
    Board, BoardState, DeviceId, DeviceKind, NetLevel, PinLevels, PinRef, Side, STUB,
};
use crate::library::DeviceDrag;
use crate::route::{self, Dir};

/// Size of the MCU symbol: an LQFP48 package drawn square, twelve pins a side.
const MCU_SIZE: Vec2 = Vec2::new(250.0, 250.0);
/// Top-left corner of the MCU symbol in board space.
const MCU_ORIGIN: Pos2 = Pos2::new(70.0, 60.0);
/// Distance between two pins of the package.
const PIN_PITCH: f32 = 17.0;
/// How far the MCU pin stubs stick out.
const MCU_STUB: f32 = 26.0;
/// Distance from the package edge to the first pin of a side.
const FIRST_PIN: f32 = (MCU_SIZE.x - 11.0 * PIN_PITCH) * 0.5;

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
    /// One user placed corner of a wire.
    Bend {
        wire: usize,
        bend: usize,
    },
    Device(DeviceId),
}

#[derive(Clone, Copy, Debug)]
enum Drag {
    Device { id: DeviceId, grab: Vec2 },
    Bend { wire: usize, bend: usize },
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
    /// Cached orthogonal routes, one per wire, refreshed when the drawing moves.
    routes: Vec<RoutedWire>,
    routes_stamp: u64,
    /// Rectangle the canvas occupied in the last frame (debug/tests).
    pub last_rect: Rect,
}

/// One wire's route: a polyline in board space plus where each leg starts.
#[derive(Clone, Debug, Default)]
struct RoutedWire {
    points: Vec<Pos2>,
    /// Index in `points` where each waypoint-to-waypoint leg starts; there is
    /// one more leg than there are bends.
    legs: Vec<usize>,
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
            routes: Vec::new(),
            routes_stamp: 0,
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

    /// What the canvas has selected, for the headless tests.
    pub fn selected(&self) -> Option<Selection> {
        self.selected
    }

    pub fn cancel_wire(&mut self) {
        self.wire_from = None;
    }

    /// Drops every transient interaction (wire in progress, selection, drag,
    /// held knob, armed part) so a freshly loaded board never inherits the
    /// ids of the previous one.
    pub fn reset_interaction(&mut self) {
        self.wire_from = None;
        self.selected = None;
        self.drag = None;
        self.press = None;
        self.pending = None;
        self.request_demo = false;
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
        self.ensure_routes(board);
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
                            if let Some((wire, bend)) = self.bend_at(board, point) {
                                self.selected = Some(Selection::Bend { wire, bend });
                                self.wire_from = None;
                            } else if let Some(index) = self.wire_at(point) {
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

        // Double clicking a wire splices in a tap point the user can wire from,
        // the way an EDA tool drops a junction; double clicking that point
        // again takes it away.
        if response.double_clicked() && self.pending.is_none() {
            if let Some(point) = pointer_board {
                if let Some(id) = self.junction_at(board, point) {
                    let name = board
                        .device(id)
                        .map(|device| device.name.clone())
                        .unwrap_or_default();
                    board.remove_device(id);
                    self.selected = None;
                    self.wire_from = None;
                    self.say(format!("已删除接线节点 {name}"), false);
                } else if self.pin_at(board, point).is_none()
                    && self.device_at(board, point).is_none()
                {
                    if let Some((wire, leg, at)) = self.wire_hit(point) {
                        // The tap goes exactly where the route runs, so the two
                        // halves stay on the line the user clicked.
                        if let Some(id) = board.add_junction(wire, leg, at) {
                            self.selected = Some(Selection::Device(id));
                            self.wire_from = None;
                            self.say("已添加接线节点：点它可继续连线，双击它删除", false);
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
            // A drag is recognised a few pixels after the press, so hit test at
            // the press origin instead of under the moved pointer.
            let origin_board = ui
                .input(|i| i.pointer.press_origin())
                .map(|point| self.to_board(point, origin))
                .or(pointer_board);
            if let Some(point) = origin_board {
                if let Some(pin) = self.pin_at(board, point) {
                    // Dragging from a pin works like the second half of a
                    // click-click: the release on another pin completes it.
                    self.wire_from = Some(pin);
                } else if let Some((wire, bend)) = self.bend_at(board, point) {
                    self.selected = Some(Selection::Bend { wire, bend });
                    self.drag = Some(Drag::Bend { wire, bend });
                    self.wire_from = None;
                } else if let Some((wire, leg)) = self.wire_leg_at(point) {
                    // Grabbing a segment pulls a new corner out of it, like
                    // dragging a wire in an EDA tool.
                    if board.insert_bend(wire, leg, point) {
                        self.selected = Some(Selection::Bend { wire, bend: leg });
                        self.drag = Some(Drag::Bend { wire, bend: leg });
                        self.wire_from = None;
                    }
                } else if let Some(id) = self.device_at(board, point) {
                    let device = board.device(id).expect("hit device exists");
                    self.drag = Some(Drag::Device {
                        id,
                        grab: point - device.pos,
                    });
                    self.selected = Some(Selection::Device(id));
                }
            }
        }
        match (self.drag, pointer_board) {
            (Some(Drag::Device { id, grab }), Some(point)) => {
                if let Some(device) = board.device_mut(id) {
                    device.pos = point - grab;
                }
            }
            (Some(Drag::Bend { wire, bend }), Some(point)) => {
                board.move_bend(wire, bend, snap(point));
            }
            _ => {}
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
                Some(Selection::Bend { wire, bend }) => {
                    if board.remove_bend(wire, bend) {
                        self.say("已删除拐点", false);
                        self.selected = Some(Selection::Wire(wire));
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
        self.ensure_routes(board);
        self.paint_grid(&painter, origin, &response.rect);
        self.paint_wires(&painter, origin, board, state);
        for device in board.devices() {
            self.paint_device(&painter, origin, device, state.led_brightness(device.id));
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

    fn wire_at(&self, point: Pos2) -> Option<usize> {
        self.wire_leg_at(point).map(|(wire, _)| wire)
    }

    fn pin_pos(&self, board: &Board, pin: PinRef) -> Option<Pos2> {
        match pin {
            PinRef::Mcu { port, pin } => mcu_pin_layout()
                .into_iter()
                .find(|(p, n, _, _)| *p == port && *n == pin)
                .map(|(_, _, _, pos)| pos),
            PinRef::Device { device, pin } => {
                board.device(device).map(|device| device.pin_pos(pin))
            }
        }
    }

    fn pin_screen_pos(&self, board: &Board, origin: Pos2, pin: PinRef) -> Option<Pos2> {
        self.pin_pos(board, pin)
            .map(|pos| self.to_screen(pos, origin))
    }

    // --------------------------------------------------------------- routing
    /// Routes every wire, reusing the cached result while the drawing holds
    /// still (the cache is keyed on the part positions and the waypoints).
    fn ensure_routes(&mut self, board: &Board) {
        let stamp = topology_stamp(board);
        if stamp == self.routes_stamp && self.routes.len() == board.wires().len() {
            return;
        }
        let obstacles = obstacles_of(board);
        self.routes = board
            .wires()
            .iter()
            .map(|wire| self.route_wire(board, wire, &obstacles))
            .collect();
        self.routes_stamp = stamp;
    }

    fn route_wire(
        &self,
        board: &Board,
        wire: &crate::board::Wire,
        obstacles: &[Rect],
    ) -> RoutedWire {
        let Some(a) = self.pin_pos(board, wire.a) else {
            return RoutedWire::default();
        };
        let Some(b) = self.pin_pos(board, wire.b) else {
            return RoutedWire::default();
        };
        let mut waypoints = Vec::with_capacity(wire.bends.len() + 2);
        waypoints.push(a);
        waypoints.extend(wire.bends.iter().copied());
        waypoints.push(b);

        let last = waypoints.len() - 1;
        let mut routed = RoutedWire::default();
        for (leg, pair) in waypoints.windows(2).enumerate() {
            let start_side = (leg == 0).then(|| pin_dir(board, wire.a)).flatten();
            let goal_side = (leg + 1 == last).then(|| pin_dir(board, wire.b)).flatten();
            let points = route::route(pair[0], start_side, pair[1], goal_side, obstacles);
            routed.legs.push(routed.points.len());
            let skip = usize::from(leg > 0);
            routed.points.extend(points.into_iter().skip(skip));
        }
        routed
    }

    /// Nearest wire leg under `point`, as `(wire, leg)`.
    fn wire_leg_at(&self, point: Pos2) -> Option<(usize, usize)> {
        self.wire_hit(point).map(|(wire, leg, _)| (wire, leg))
    }

    /// Nearest wire leg under `point`, as `(wire, leg, point on the route)`.
    fn wire_hit(&self, point: Pos2) -> Option<(usize, usize, Pos2)> {
        let tolerance = 7.0;
        let mut best: Option<(f32, usize, usize, Pos2)> = None;
        for (index, route) in self.routes.iter().enumerate() {
            for leg in 0..route.legs.len() {
                let from = route.legs[leg];
                let to = route
                    .legs
                    .get(leg + 1)
                    .copied()
                    .unwrap_or(route.points.len());
                for pair in route.points[from..to].windows(2) {
                    let (distance, at) = closest_on_segment(point, pair[0], pair[1]);
                    if distance <= tolerance
                        && best.is_none_or(|(best_distance, _, _, _)| distance < best_distance)
                    {
                        best = Some((distance, index, leg, at));
                    }
                }
            }
        }
        best.map(|(_, wire, leg, at)| (wire, leg, at))
    }

    /// Junction under `point`, for the "double click removes the tap" gesture.
    fn junction_at(&self, board: &Board, point: Pos2) -> Option<DeviceId> {
        board
            .devices()
            .iter()
            .rev()
            .find(|device| {
                device.kind == DeviceKind::Junction && device.rect().expand(3.0).contains(point)
            })
            .map(|device| device.id)
    }

    /// Corner (waypoint) under `point`, as `(wire, bend)`.
    fn bend_at(&self, board: &Board, point: Pos2) -> Option<(usize, usize)> {
        let radius = 8.0;
        board
            .wires()
            .iter()
            .enumerate()
            .flat_map(|(index, wire)| {
                wire.bends
                    .iter()
                    .enumerate()
                    .map(move |(bend, at)| (index, bend, *at))
            })
            .filter(|(_, _, at)| at.distance(point) <= radius)
            .min_by(|left, right| left.2.distance(point).total_cmp(&right.2.distance(point)))
            .map(|(index, bend, _)| (index, bend))
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
            Pos2::new(screen.center().x, screen.center().y - 12.0 * self.zoom),
            Align2::CENTER_CENTER,
            "STM32F103C8T6",
            self.font(13.0),
            COLOR_TEXT,
        );
        painter.text(
            Pos2::new(screen.center().x, screen.center().y + 6.0 * self.zoom),
            Align2::CENTER_CENTER,
            "Cortex-M3 · 72 MHz",
            self.font(9.0),
            COLOR_DIM,
        );

        for pin in mcu_pins() {
            let tip = self.to_screen(pin.pos, origin);
            let base = match pin.side {
                Side::Left => Pos2::new(screen.left(), tip.y),
                Side::Right => Pos2::new(screen.right(), tip.y),
                Side::Top => Pos2::new(tip.x, screen.top()),
                Side::Bottom => Pos2::new(tip.x, screen.bottom()),
            };
            // The supply/reset/boot pins are drawn but not modelled.
            let Some((port, index)) = pin.signal else {
                painter.line_segment(
                    [base, tip],
                    Stroke::new(self.stroke(1.3), COLOR_LOW.gamma_multiply(0.75)),
                );
                self.paint_pin_name(painter, origin, &screen, pin, COLOR_DIM);
                self.paint_pin_number(painter, origin, &screen, pin);
                continue;
            };

            let level = levels.level_of(port, index);
            let is_output = levels.is_output(port, index);
            let is_hovered = hover == Some(PinRef::Mcu { port, pin: index });
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
            self.paint_pin_name(
                painter,
                origin,
                &screen,
                pin,
                if is_hovered {
                    Color32::WHITE
                } else {
                    COLOR_TEXT
                },
            );
            self.paint_pin_number(painter, origin, &screen, pin);
        }
    }

    /// Name of a package pin, printed inside the body. The names of the top and
    /// bottom pins are rotated so twelve of them fit next to each other.
    fn paint_pin_name(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        screen: &Rect,
        pin: &McuPin,
        color: Color32,
    ) {
        let tip = self.to_screen(pin.pos, origin);
        let font = self.font(8.5);
        match pin.side {
            Side::Left => {
                painter.text(
                    Pos2::new(screen.left() + 8.0 * self.zoom, tip.y),
                    Align2::LEFT_CENTER,
                    pin.name,
                    font,
                    color,
                );
            }
            Side::Right => {
                painter.text(
                    Pos2::new(screen.right() - 8.0 * self.zoom, tip.y),
                    Align2::RIGHT_CENTER,
                    pin.name,
                    font,
                    color,
                );
            }
            Side::Top | Side::Bottom => {
                let galley = painter.layout_no_wrap(pin.name.to_string(), font, color);
                let size = galley.size();
                let y = match pin.side {
                    Side::Top => screen.top() + 8.0 * self.zoom + size.x,
                    _ => screen.bottom() - 8.0 * self.zoom,
                };
                let pos = Pos2::new(tip.x - size.y * 0.5 + 0.5, y);
                painter.add(
                    egui::epaint::TextShape::new(pos, galley, color)
                        .with_angle(-std::f32::consts::FRAC_PI_2),
                );
            }
        }
    }

    /// Package pin number, printed outside the body next to the stub.
    fn paint_pin_number(&self, painter: &egui::Painter, origin: Pos2, screen: &Rect, pin: &McuPin) {
        let tip = self.to_screen(pin.pos, origin);
        let font = self.font(7.0);
        let (pos, align) = match pin.side {
            Side::Left => (
                Pos2::new(screen.left() - 4.0 * self.zoom, tip.y - 6.0 * self.zoom),
                Align2::RIGHT_BOTTOM,
            ),
            Side::Right => (
                Pos2::new(screen.right() + 4.0 * self.zoom, tip.y - 6.0 * self.zoom),
                Align2::LEFT_BOTTOM,
            ),
            Side::Top => (
                Pos2::new(tip.x + 5.0 * self.zoom, screen.top() - 3.0 * self.zoom),
                Align2::LEFT_BOTTOM,
            ),
            Side::Bottom => (
                Pos2::new(tip.x + 5.0 * self.zoom, screen.bottom() + 3.0 * self.zoom),
                Align2::LEFT_TOP,
            ),
        };
        painter.text(pos, align, pin.number.to_string(), font, COLOR_DIM);
    }

    fn paint_wires(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        board: &Board,
        state: &BoardState,
    ) {
        for (index, wire) in board.wires().iter().enumerate() {
            let Some(route) = self.routes.get(index) else {
                continue;
            };
            if route.points.len() < 2 {
                continue;
            }
            let selected = match self.selected {
                Some(Selection::Wire(selected)) => selected == index,
                Some(Selection::Bend { wire, .. }) => wire == index,
                _ => false,
            };
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
            let points: Vec<Pos2> = route
                .points
                .iter()
                .map(|point| self.to_screen(*point, origin))
                .collect();
            painter.add(egui::Shape::line(
                points.clone(),
                Stroke::new(self.stroke(width), color),
            ));
            painter.circle_filled(points[0], self.stroke(2.5), color);
            painter.circle_filled(*points.last().expect("non-empty"), self.stroke(2.5), color);

            // Corner handles: always visible on the selected wire, bigger while
            // one is being dragged.
            for (bend, at) in wire.bends.iter().enumerate() {
                let selected_bend = self.selected == Some(Selection::Bend { wire: index, bend });
                if !selected && !selected_bend {
                    continue;
                }
                let center = self.to_screen(*at, origin);
                let size = if selected_bend { 8.0 } else { 6.0 } * self.zoom.max(0.6);
                painter.rect_filled(
                    Rect::from_center_size(center, Vec2::splat(size)),
                    1.5,
                    if selected_bend {
                        Color32::WHITE
                    } else {
                        COLOR_WIRE_SELECTED
                    },
                );
                painter.rect_stroke(
                    Rect::from_center_size(center, Vec2::splat(size)),
                    1.5,
                    Stroke::new(1.0, Color32::from_rgb(30, 34, 40)),
                    StrokeKind::Inside,
                );
            }
        }
    }

    /// Power symbol drawn instead of a part body: an up arrow for `VCC`, the
    /// shrinking bars of a ground symbol for `GND`.
    fn paint_power_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        kind: DeviceKind,
        selected: bool,
    ) {
        let base = match kind {
            DeviceKind::Vcc => COLOR_VCC,
            _ => COLOR_GND,
        };
        let center = rect.center();
        let len = (rect.height() * 0.28).max(5.0);
        if selected {
            painter.rect_stroke(
                rect.expand(3.0 * self.zoom),
                5.0 * self.zoom,
                Stroke::new(self.stroke(1.2), COLOR_WIRE_SELECTED),
                StrokeKind::Outside,
            );
        }
        match kind {
            DeviceKind::Vcc => {
                let top = Pos2::new(center.x, center.y - len);
                painter.line_segment(
                    [Pos2::new(center.x, center.y + len * 0.6), top],
                    Stroke::new(self.stroke(2.0), base),
                );
                painter.line_segment(
                    [Pos2::new(center.x - len * 0.6, center.y - len * 0.4), top],
                    Stroke::new(self.stroke(2.0), base),
                );
                painter.line_segment(
                    [Pos2::new(center.x + len * 0.6, center.y - len * 0.4), top],
                    Stroke::new(self.stroke(2.0), base),
                );
            }
            _ => {
                for (index, width) in [1.1f32, 0.75, 0.4].into_iter().enumerate() {
                    let y = center.y - len * 0.4 + len * 0.55 * index as f32;
                    painter.line_segment(
                        [
                            Pos2::new(center.x - len * width, y),
                            Pos2::new(center.x + len * width, y),
                        ],
                        Stroke::new(self.stroke(2.0), base),
                    );
                }
                painter.line_segment(
                    [
                        Pos2::new(center.x, center.y - len * 0.9),
                        Pos2::new(center.x, center.y - len * 0.4),
                    ],
                    Stroke::new(self.stroke(2.0), base),
                );
            }
        }
    }

    /// Transceiver box: the mode plus the frame counters of the part.
    fn paint_can_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let Some(transceiver) = device.hardware.can() else {
            return;
        };
        let transceiver = transceiver.borrow();
        painter.text(
            Pos2::new(rect.left() + 8.0 * self.zoom, rect.top() + 7.0 * self.zoom),
            Align2::LEFT_TOP,
            "CAN",
            self.font(9.5),
            base,
        );
        painter.text(
            rect.center(),
            Align2::CENTER_CENTER,
            transceiver.mode().label(),
            self.font(9.0),
            COLOR_TEXT,
        );
        painter.text(
            Pos2::new(
                rect.right() - 8.0 * self.zoom,
                rect.bottom() - 7.0 * self.zoom,
            ),
            Align2::RIGHT_BOTTOM,
            format!(
                "发 {} 收 {}",
                transceiver.stats().tx_frames,
                transceiver.stats().rx_frames
            ),
            self.font(8.5),
            COLOR_DIM,
        );
    }

    /// 8x8 window of the first die of the chain.
    fn paint_matrix_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let Some(matrix) = device.hardware.matrix() else {
            return;
        };
        let matrix = matrix.borrow();
        let pixels = matrix
            .chips()
            .first()
            .map(|chip| chip.pixels())
            .unwrap_or([[false; 8]; 8]);
        // Keep the pin labels on the left and right readable.
        let side = (rect.width().min(rect.height()) * 0.52).max(20.0);
        let cell = side / 8.0;
        let origin = Pos2::new(rect.center().x - side * 0.5, rect.center().y - side * 0.5);
        painter.rect_filled(
            Rect::from_min_size(origin, Vec2::splat(side)).expand(cell * 0.3),
            cell * 0.3,
            Color32::from_rgb(26, 30, 36),
        );
        for (row, line) in pixels.iter().enumerate() {
            for (column, lit) in line.iter().enumerate() {
                let min = Pos2::new(
                    origin.x + column as f32 * cell + cell * 0.12,
                    origin.y + row as f32 * cell + cell * 0.12,
                );
                let dot = Rect::from_min_size(min, Vec2::splat(cell * 0.76));
                painter.rect_filled(
                    dot,
                    cell * 0.15,
                    if *lit {
                        base
                    } else {
                        Color32::from_rgb(46, 52, 62)
                    },
                );
            }
        }
    }

    /// Zig-zag resistor with its value in the middle.
    fn paint_resistor_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let center = rect.center();
        let half = rect.width() * 0.17;
        let amplitude = rect.height() * 0.22;
        let stroke = Stroke::new(self.stroke(1.5), base);
        painter.line_segment(
            [
                Pos2::new(rect.left() + 6.0, center.y),
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
        painter.add(egui::Shape::line(points, stroke));
        painter.line_segment(
            [
                Pos2::new(center.x + half, center.y),
                Pos2::new(rect.right() - 6.0, center.y),
            ],
            stroke,
        );
        if let Some(resistor) = device.hardware.resistor() {
            painter.text(
                Pos2::new(center.x, center.y - amplitude - 2.0 * self.zoom),
                Align2::CENTER_BOTTOM,
                resistor.label(),
                self.font(9.0),
                base,
            );
        }
    }

    /// Small screen showing what the probe is currently capturing.
    fn paint_scope_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let screen = rect.shrink2(Vec2::new(rect.width() * 0.10, rect.height() * 0.14));
        painter.rect_filled(screen, 3.0, Color32::from_rgb(12, 20, 18));
        painter.rect_stroke(
            screen,
            3.0,
            Stroke::new(self.stroke(1.0), base.gamma_multiply(0.8)),
            StrokeKind::Inside,
        );
        let Some(scope) = device.hardware.scope() else {
            return;
        };
        let scope = scope.borrow();
        let samples = scope.samples();
        let Some(first) = samples.front().map(|sample| sample.time_ns) else {
            painter.text(
                screen.center(),
                Align2::CENTER_CENTER,
                "无信号",
                self.font(8.5),
                COLOR_DIM,
            );
            return;
        };
        let last = samples.back().map(|sample| sample.time_ns).unwrap_or(first);
        let span = last.saturating_sub(first).max(1);
        let trace = |value: fn(&gsim_devices::ScopeSample) -> f32| {
            samples
                .iter()
                .map(|sample| {
                    Pos2::new(
                        screen.left()
                            + screen.width() * (sample.time_ns - first) as f32 / span as f32,
                        // 1 V .. 4 V across the screen height.
                        screen.bottom()
                            - screen.height() * ((value(sample) - 1.0) / 3.0).clamp(0.0, 1.0),
                    )
                })
                .collect::<Vec<_>>()
        };
        painter.add(egui::Shape::line(
            trace(|sample| sample.can_h),
            Stroke::new(self.stroke(1.0), Color32::from_rgb(250, 210, 90)),
        ));
        painter.add(egui::Shape::line(
            trace(|sample| sample.can_l),
            Stroke::new(self.stroke(1.0), Color32::from_rgb(90, 200, 240)),
        ));
        painter.text(
            Pos2::new(screen.center().x, rect.bottom() - 3.0 * self.zoom),
            Align2::CENTER_BOTTOM,
            format!("{:.1} V", scope.peak_differential()),
            self.font(8.0),
            COLOR_DIM,
        );
    }

    /// Quartz symbol: two plates and the frequency, with leads to `X1`/`X2`.
    fn paint_crystal_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let center = rect.center();
        let plate_height = rect.height() * 0.30;
        let spacing = rect.width() * 0.16;
        let left = center.x - spacing;
        let right = center.x + spacing;
        let stroke = Stroke::new(self.stroke(1.5), base);
        painter.line_segment(
            [
                Pos2::new(rect.left() + 6.0, center.y),
                Pos2::new(left, center.y),
            ],
            stroke,
        );
        painter.line_segment(
            [
                Pos2::new(right, center.y),
                Pos2::new(rect.right() - 6.0, center.y),
            ],
            stroke,
        );
        for x in [left, right] {
            painter.line_segment(
                [
                    Pos2::new(x, center.y - plate_height * 0.5),
                    Pos2::new(x, center.y + plate_height * 0.5),
                ],
                stroke,
            );
        }
        let body = Rect::from_center_size(center, Vec2::new(spacing * 0.9, plate_height * 0.8));
        painter.rect_filled(body, 2.0, base.gamma_multiply(0.35));
        painter.rect_stroke(body, 2.0, stroke, StrokeKind::Inside);
        if let Some(crystal) = device.hardware.crystal() {
            painter.text(
                center,
                Align2::CENTER_CENTER,
                crystal.frequency_label(),
                self.font(9.0),
                COLOR_TEXT,
            );
        }
    }

    fn paint_device(
        &self,
        painter: &egui::Painter,
        origin: Pos2,
        device: &crate::board::Device,
        brightness: f32,
    ) {
        let rect = Rect::from_min_max(
            self.to_screen(device.rect().min, origin),
            self.to_screen(device.rect().max, origin),
        );
        let selected = self.selected == Some(Selection::Device(device.id));
        if device.kind == DeviceKind::Junction {
            // A tap point is just a dot on the wire, not a body with pins.
            let center = self.to_screen(device.pin_pos(0), origin);
            let color = if selected {
                COLOR_WIRE_SELECTED
            } else {
                COLOR_WIRE
            };
            painter.circle_filled(center, self.stroke(4.5), COLOR_BG);
            painter.circle_filled(center, self.stroke(3.6), color);
            if selected {
                painter.circle_stroke(
                    center,
                    self.stroke(7.0),
                    Stroke::new(self.stroke(1.4), Color32::WHITE),
                );
            }
            return;
        }
        let base = match device.kind {
            // Keep the bus parts in the colour they have in the library.
            DeviceKind::Can => Color32::from_rgb(245, 200, 60),
            DeviceKind::Matrix => Color32::from_rgb(70, 200, 255),
            DeviceKind::Motor => Color32::from_rgb(110, 205, 165),
            _ => device.accent.color32(),
        };
        let brightness = brightness.clamp(0.0, 1.0);
        let lit = brightness > 0.0;
        if device.kind.is_power() {
            self.paint_power_symbol(painter, rect, device.kind, selected);
        } else {
            let body = if lit {
                Color32::from_rgb(24 + base.r() / 3, 26 + base.g() / 4, 30 + base.b() / 4)
            } else {
                Color32::from_rgb(38, 42, 50)
            };
            if lit {
                // Soft halo so a lit LED reads as "emitting" at any zoom level;
                // its strength follows the PWM duty of a dimmed channel.
                let glow = Color32::from_rgba_unmultiplied(
                    base.r(),
                    base.g(),
                    base.b(),
                    (26.0 * brightness) as u8,
                );
                painter.circle_filled(rect.center(), rect.width() * 0.42, glow);
                let inner = Color32::from_rgba_unmultiplied(
                    base.r(),
                    base.g(),
                    base.b(),
                    (40.0 * brightness) as u8,
                );
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
                DeviceKind::Led => self.paint_led_symbol(painter, rect, base, brightness),
                DeviceKind::Uart => self.paint_uart_symbol(painter, rect, base, device),
                DeviceKind::Encoder => self.paint_encoder_symbol(painter, rect, base, device),
                DeviceKind::Can => self.paint_can_symbol(painter, rect, base, device),
                DeviceKind::Matrix => self.paint_matrix_symbol(painter, rect, base, device),
                DeviceKind::HseCrystal | DeviceKind::LseCrystal => {
                    self.paint_crystal_symbol(painter, rect, base, device)
                }
                DeviceKind::Resistor => self.paint_resistor_symbol(painter, rect, base, device),
                DeviceKind::Scope => self.paint_scope_symbol(painter, rect, base, device),
                DeviceKind::Motor => self.paint_motor_symbol(painter, rect, base, device),
                DeviceKind::Vcc | DeviceKind::Gnd | DeviceKind::Junction => {}
            }
        }

        // The part name sits above the stubs, so the top contacts (VCC/GND) do
        // not run through it.
        let label_y = device
            .kind
            .pins()
            .iter()
            .enumerate()
            .filter(|(_, def)| def.side == Side::Top)
            .map(|(index, _)| {
                self.to_screen(device.pin_pos(index as u8), origin).y - 4.0 * self.zoom
            })
            .reduce(f32::min)
            .unwrap_or(rect.top() - 8.0 * self.zoom);
        painter.text(
            Pos2::new(rect.center().x, label_y),
            Align2::CENTER_BOTTOM,
            match device.kind {
                DeviceKind::Vcc => "VCC",
                DeviceKind::Gnd => "GND",
                _ => device.name.as_str(),
            },
            self.font(10.0),
            COLOR_TEXT,
        );

        for (index, def) in device.kind.pins().iter().enumerate() {
            let tip = self.to_screen(device.pin_pos(index as u8), origin);
            let (base_pos, text_pos, align) = match def.side {
                // Left/right names sit on the pin line, so they never drift
                // into the row of top contacts (`VCC`/`GND`).
                Side::Left => (
                    Pos2::new(rect.left(), tip.y),
                    Pos2::new(rect.left() + 6.0 * self.zoom, tip.y),
                    Align2::LEFT_CENTER,
                ),
                Side::Right => (
                    Pos2::new(rect.right(), tip.y),
                    Pos2::new(rect.right() - 6.0 * self.zoom, tip.y),
                    Align2::RIGHT_CENTER,
                ),
                Side::Top => (
                    Pos2::new(tip.x, rect.top()),
                    Pos2::new(tip.x, rect.top() + 6.0 * self.zoom),
                    Align2::CENTER_TOP,
                ),
                Side::Bottom => (
                    Pos2::new(tip.x, rect.bottom()),
                    Pos2::new(tip.x, rect.bottom() - 6.0 * self.zoom),
                    Align2::CENTER_BOTTOM,
                ),
            };
            painter.line_segment([base_pos, tip], Stroke::new(self.stroke(1.6), COLOR_WIRE));
            painter.circle_filled(tip, self.stroke(3.0), COLOR_WIRE);
            painter.text(text_pos, align, def.name, self.font(8.5), COLOR_DIM);
        }
    }

    /// Translucent preview of the part waiting for its placement click.
    fn paint_ghost(&self, painter: &egui::Painter, origin: Pos2, kind: DeviceKind, center: Pos2) {
        let body = Rect::from_min_size(center - kind.body_size() * 0.5, kind.body_size());
        let rect = Rect::from_min_max(
            self.to_screen(body.min, origin),
            self.to_screen(body.max, origin),
        );
        let outline = Color32::from_rgba_unmultiplied(120, 196, 240, 150);
        if kind == DeviceKind::Junction {
            painter.circle_filled(rect.center(), self.stroke(3.6), outline);
            painter.text(
                rect.center() + Vec2::new(10.0, -10.0) * self.zoom,
                Align2::LEFT_BOTTOM,
                kind.label(),
                self.font(10.0),
                COLOR_TEXT,
            );
            return;
        }
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
                Side::Top => Pos2::new(tip.x, rect.top()),
                Side::Bottom => Pos2::new(tip.x, rect.bottom()),
            };
            painter.line_segment(
                [base, tip],
                Stroke::new(self.stroke(1.6), outline.gamma_multiply(0.8)),
            );
            painter.circle_filled(tip, self.stroke(3.0), outline);
            let (label_pos, align) = match def.side {
                Side::Left => (
                    Pos2::new(rect.left() + 6.0 * self.zoom, tip.y),
                    Align2::LEFT_CENTER,
                ),
                Side::Right => (
                    Pos2::new(rect.right() - 6.0 * self.zoom, tip.y),
                    Align2::RIGHT_CENTER,
                ),
                Side::Top => (
                    Pos2::new(tip.x, rect.top() + 6.0 * self.zoom),
                    Align2::CENTER_TOP,
                ),
                Side::Bottom => (
                    Pos2::new(tip.x, rect.bottom() - 6.0 * self.zoom),
                    Align2::CENTER_BOTTOM,
                ),
            };
            painter.text(label_pos, align, def.name, self.font(8.5), COLOR_DIM);
        }
    }

    /// Triangle, bar and emission arrows.
    fn paint_led_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        brightness: f32,
    ) {
        let center = rect.center();
        let s = (rect.height() * 0.22).max(5.0);
        // A PWM dimmed LED keeps its colour but drops towards the dark body,
        // the way a real diode looks at a lower duty cycle.
        let fill = if brightness > 0.0 {
            base.gamma_multiply(0.35 + 0.65 * brightness.clamp(0.0, 1.0))
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
            // The left edge belongs to the `RX`/`TX` contacts, the top edge to
            // `VCC`/`GND`: the port sits in the free top-right corner.
            Pos2::new(
                rect.right() - 8.0 * self.zoom,
                rect.top() + 20.0 * self.zoom,
            ),
            Align2::RIGHT_TOP,
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
            Pos2::new(
                rect.right() - 8.0 * self.zoom,
                rect.bottom() - 5.0 * self.zoom,
            ),
            Align2::RIGHT_BOTTOM,
            format!("{}", encoder.detents()),
            self.font(8.5),
            COLOR_DIM,
        );
    }

    /// Motor body with a mark that follows the shaft, plus its speed.
    fn paint_motor_symbol(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        base: Color32,
        device: &crate::board::Device,
    ) {
        let Some(motor) = device.hardware.motor() else {
            return;
        };
        let motor = motor.borrow();
        let center = Pos2::new(rect.center().x, rect.center().y - 4.0 * self.zoom);
        let radius = (rect.height() * 0.26).max(8.0);
        let spinning = motor.spin() != gsim_devices::Spin::Stopped;
        painter.circle_filled(
            center,
            radius,
            if spinning {
                base.gamma_multiply(0.28)
            } else {
                Color32::from_rgb(50, 55, 64)
            },
        );
        painter.circle_stroke(
            center,
            radius,
            Stroke::new(self.stroke(1.5), base.gamma_multiply(0.85)),
        );
        painter.text(center, Align2::CENTER_CENTER, "M", self.font(11.0), base);

        // The mark points at the angle the shaft reached.
        let angle = motor.angle_deg().to_radians() - std::f32::consts::FRAC_PI_2;
        let tip = Pos2::new(
            center.x + angle.cos() * radius * 0.82,
            center.y + angle.sin() * radius * 0.82,
        );
        painter.line_segment(
            [center, tip],
            Stroke::new(self.stroke(2.0), Color32::from_rgb(226, 232, 240)),
        );

        painter.text(
            Pos2::new(rect.center().x, rect.bottom() - 5.0 * self.zoom),
            Align2::CENTER_BOTTOM,
            format!("{:+.0} rpm · {:.1} 圈", motor.rpm(), motor.turns()),
            self.font(9.0),
            if spinning { COLOR_TEXT } else { COLOR_DIM },
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
    for device in board.devices() {
        let rect = device.rect();
        bounds = bounds.union(Rect::from_min_max(
            Pos2::new(rect.left() - STUB - 8.0, rect.top() - 22.0),
            Pos2::new(rect.right() + STUB + 8.0, rect.bottom() + 8.0),
        ));
    }
    bounds
}

/// One pin of the LQFP48 package, drawn where the real package puts it.
struct McuPin {
    /// Package pin number, `1..=48`.
    number: u8,
    /// Name printed on the symbol (`PA0`, `VDD`, `BOOT0`, ...).
    name: &'static str,
    /// Modelled port/pin; `None` for the supply, reset and boot pins the
    /// simulator has no register for.
    signal: Option<(u8, u8)>,
    side: Side,
    /// Tip of the stub, in board space.
    pos: Pos2,
}

/// One row of the package table: pin number, silk name, modelled signal.
type PackagePin = (u8, &'static str, Option<(u8, u8)>);

/// The LQFP48 pinout, in package order: pins 1–12 down the left side, 13–24
/// along the bottom, 25–36 up the right side and 37–48 back along the top.
const LQFP48: [PackagePin; 48] = [
    (1, "VBAT", None),
    (2, "PC13", Some((2, 13))),
    (3, "PC14", Some((2, 14))),
    (4, "PC15", Some((2, 15))),
    (5, "PD0", Some((3, 0))),
    (6, "PD1", Some((3, 1))),
    (7, "NRST", None),
    (8, "VSSA", None),
    (9, "VDDA", None),
    (10, "PA0", Some((0, 0))),
    (11, "PA1", Some((0, 1))),
    (12, "PA2", Some((0, 2))),
    (13, "PA3", Some((0, 3))),
    (14, "PA4", Some((0, 4))),
    (15, "PA5", Some((0, 5))),
    (16, "PA6", Some((0, 6))),
    (17, "PA7", Some((0, 7))),
    (18, "PB0", Some((1, 0))),
    (19, "PB1", Some((1, 1))),
    (20, "PB2", Some((1, 2))),
    (21, "PB10", Some((1, 10))),
    (22, "PB11", Some((1, 11))),
    (23, "VSS", None),
    (24, "VDD", None),
    (25, "PB12", Some((1, 12))),
    (26, "PB13", Some((1, 13))),
    (27, "PB14", Some((1, 14))),
    (28, "PB15", Some((1, 15))),
    (29, "PA8", Some((0, 8))),
    (30, "PA9", Some((0, 9))),
    (31, "PA10", Some((0, 10))),
    (32, "PA11", Some((0, 11))),
    (33, "PA12", Some((0, 12))),
    (34, "PA13", Some((0, 13))),
    (35, "VSS", None),
    (36, "VDD", None),
    (37, "PA14", Some((0, 14))),
    (38, "PA15", Some((0, 15))),
    (39, "PB3", Some((1, 3))),
    (40, "PB4", Some((1, 4))),
    (41, "PB5", Some((1, 5))),
    (42, "PB6", Some((1, 6))),
    (43, "PB7", Some((1, 7))),
    (44, "BOOT0", None),
    (45, "PB8", Some((1, 8))),
    (46, "PB9", Some((1, 9))),
    (47, "VSS", None),
    (48, "VDD", None),
];

fn build_mcu_pins() -> Vec<McuPin> {
    let left_x = MCU_ORIGIN.x - MCU_STUB;
    let right_x = MCU_ORIGIN.x + MCU_SIZE.x + MCU_STUB;
    let top_y = MCU_ORIGIN.y - MCU_STUB;
    let bottom_y = MCU_ORIGIN.y + MCU_SIZE.y + MCU_STUB;
    LQFP48
        .iter()
        .enumerate()
        .map(|(index, (number, name, signal))| {
            let (side, pos) = match index {
                0..=11 => (
                    Side::Left,
                    Pos2::new(left_x, MCU_ORIGIN.y + FIRST_PIN + index as f32 * PIN_PITCH),
                ),
                12..=23 => (
                    Side::Bottom,
                    Pos2::new(
                        MCU_ORIGIN.x + FIRST_PIN + (index - 12) as f32 * PIN_PITCH,
                        bottom_y,
                    ),
                ),
                24..=35 => (
                    Side::Right,
                    Pos2::new(
                        right_x,
                        MCU_ORIGIN.y + MCU_SIZE.y - FIRST_PIN - (index - 24) as f32 * PIN_PITCH,
                    ),
                ),
                _ => (
                    Side::Top,
                    Pos2::new(
                        MCU_ORIGIN.x + MCU_SIZE.x - FIRST_PIN - (index - 36) as f32 * PIN_PITCH,
                        top_y,
                    ),
                ),
            };
            McuPin {
                number: *number,
                name,
                signal: *signal,
                side,
                pos,
            }
        })
        .collect()
}

/// All 48 pins of the package, laid out once.
fn mcu_pins() -> &'static [McuPin] {
    static PINS: std::sync::LazyLock<Vec<McuPin>> = std::sync::LazyLock::new(build_mcu_pins);
    &PINS
}

/// The pins the simulator models, as `(port, pin, side, position)`.
fn mcu_pin_layout() -> Vec<(u8, u8, Side, Pos2)> {
    mcu_pins()
        .iter()
        .filter_map(|pin| {
            pin.signal
                .map(|(port, index)| (port, index, pin.side, pin.pos))
        })
        .collect()
}

/// Bodies the wires have to go around, in board space.
///
/// Every symbol (including the MCU and the power flags) is inflated by the
/// router's clearance so the wire keeps a readable distance from the drawing.
pub fn obstacles_of(board: &Board) -> Vec<Rect> {
    let mut obstacles = Vec::with_capacity(board.devices().len() + 1);
    obstacles.push(Rect::from_min_size(MCU_ORIGIN, MCU_SIZE).expand(route::CLEARANCE));
    for device in board.devices() {
        // A junction sits *on* the wire it taps, so it must not block it.
        if device.kind == DeviceKind::Junction {
            continue;
        }
        obstacles.push(device.rect().expand(route::CLEARANCE));
    }
    obstacles
}

/// Direction a pin leaves its body.
fn pin_dir(board: &Board, pin: PinRef) -> Option<Dir> {
    match pin {
        PinRef::Mcu { port, pin } => mcu_pin_layout()
            .into_iter()
            .find(|(layout_port, layout_pin, _, _)| *layout_port == port && *layout_pin == pin)
            .map(|(_, _, side, _)| dir_of(side)),
        PinRef::Device { device, pin } => {
            let device = board.device(device)?;
            if device.kind == DeviceKind::Junction {
                // A tap point has no body, so a wire may leave it any way.
                return None;
            }
            device.pin_def(pin).map(|def| dir_of(def.side))
        }
    }
}

fn dir_of(side: Side) -> Dir {
    match side {
        Side::Left => Dir::Left,
        Side::Right => Dir::Right,
        Side::Top => Dir::Up,
        Side::Bottom => Dir::Down,
    }
}

/// Fingerprint of everything a route depends on: the part bodies, the wire
/// endpoints and the user placed corners. Cheap enough to hash every frame.
fn topology_stamp(board: &Board) -> u64 {
    use std::hash::{Hash, Hasher};

    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    MCU_ORIGIN.x.to_bits().hash(&mut hasher);
    MCU_ORIGIN.y.to_bits().hash(&mut hasher);
    for device in board.devices() {
        let rect = device.rect();
        device.id.hash(&mut hasher);
        device.kind.hash(&mut hasher);
        for value in [rect.left(), rect.top(), rect.right(), rect.bottom()] {
            value.to_bits().hash(&mut hasher);
        }
    }
    for wire in board.wires() {
        wire.a.hash(&mut hasher);
        wire.b.hash(&mut hasher);
        for bend in &wire.bends {
            bend.x.to_bits().hash(&mut hasher);
            bend.y.to_bits().hash(&mut hasher);
        }
    }
    hasher.finish()
}

/// Snaps a board space point to the 2 unit grid the wire corners sit on.
fn snap(point: Pos2) -> Pos2 {
    Pos2::new((point.x / 2.0).round() * 2.0, (point.y / 2.0).round() * 2.0)
}

/// Distance from `point` to the segment `a`-`b`, plus the point on the segment
/// that is closest to it.
fn closest_on_segment(point: Pos2, a: Pos2, b: Pos2) -> (f32, Pos2) {
    let ab = b - a;
    let length_squared = ab.length_sq();
    if length_squared <= f32::EPSILON {
        return (point.distance(a), a);
    }
    let t = ((point - a).dot(ab) / length_squared).clamp(0.0, 1.0);
    let at = a + ab * t;
    (point.distance(at), at)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mcu_layout_covers_every_bonded_pin() {
        let pins = mcu_pin_layout();
        assert_eq!(pins.len(), 37);
        assert!(pins.iter().any(|(p, n, _, _)| *p == 0 && *n == 5));
        assert!(pins.iter().any(|(p, n, _, _)| *p == 2 && *n == 13));
        // OSC_IN / OSC_OUT for the HSE crystal.
        assert!(pins.iter().any(|(p, n, _, _)| *p == 3 && *n == 0));
        assert!(pins.iter().any(|(p, n, _, _)| *p == 3 && *n == 1));
        // No duplicates.
        let mut seen = pins.iter().map(|(p, n, _, _)| (*p, *n)).collect::<Vec<_>>();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), 37);
    }

    #[test]
    fn the_package_pins_follow_the_lqfp48_pinout() {
        let pins = mcu_pins();
        assert_eq!(pins.len(), 48);
        // Numbers run 1..=48 counterclockwise around the package.
        for (index, pin) in pins.iter().enumerate() {
            assert_eq!(pin.number as usize, index + 1, "pin order");
        }
        // Anchors of the datasheet pinout.
        assert_eq!((pins[0].name, pins[0].side), ("VBAT", Side::Left));
        assert_eq!((pins[12].name, pins[12].side), ("PA3", Side::Bottom));
        assert_eq!((pins[24].name, pins[24].side), ("PB12", Side::Right));
        assert_eq!((pins[36].name, pins[36].side), ("PA14", Side::Top));
        assert_eq!(pins[43].name, "BOOT0");
        assert_eq!(pins[47].name, "VDD");
        // Twelve pins per side, running the same way as the real package.
        assert_eq!(pins.iter().filter(|pin| pin.side == Side::Left).count(), 12);
        assert_eq!(
            pins.iter().filter(|pin| pin.side == Side::Bottom).count(),
            12
        );
        assert_eq!(
            pins.iter().filter(|pin| pin.side == Side::Right).count(),
            12
        );
        assert_eq!(pins.iter().filter(|pin| pin.side == Side::Top).count(), 12);
        assert!(pins[0].pos.y < pins[11].pos.y, "pin 12 sits below pin 1");
        assert!(pins[12].pos.x < pins[23].pos.x, "pin 24 sits right of 13");
        assert!(pins[24].pos.y > pins[35].pos.y, "pin 36 sits above pin 25");
        assert!(pins[36].pos.x > pins[47].pos.x, "pin 48 sits left of 37");

        // Every port pin is on the package exactly where the datasheet puts it.
        let place = |name: &str| {
            let pin = pins
                .iter()
                .find(|pin| pin.name == name)
                .expect("pin exists");
            (pin.side, pin.signal)
        };
        assert_eq!(place("PA0"), (Side::Left, Some((0, 0))));
        assert_eq!(place("PA7"), (Side::Bottom, Some((0, 7))));
        assert_eq!(place("PB12"), (Side::Right, Some((1, 12))));
        assert_eq!(place("PA14"), (Side::Top, Some((0, 14))));
        assert_eq!(place("PD0"), (Side::Left, Some((3, 0))));
        assert_eq!(place("NRST"), (Side::Left, None));
    }

    #[test]
    fn routing_uses_orthogonal_segments() {
        let points = route::route_between(Pos2::new(0.0, 0.0), Pos2::new(100.0, 50.0), &[]);
        assert_eq!(points.len(), 3);
        assert_eq!(points[0].y, points[1].y);
        assert_eq!(points[1].x, points[2].x);
    }

    #[test]
    fn point_to_segment_distance_handles_endpoints() {
        let a = Pos2::new(0.0, 0.0);
        let b = Pos2::new(10.0, 0.0);
        let distance = |point| closest_on_segment(point, a, b).0;
        assert!(distance(Pos2::new(5.0, 3.0)) - 3.0 < 1e-3);
        assert!(distance(Pos2::new(-4.0, 0.0)) - 4.0 < 1e-3);
        assert!(closest_on_segment(Pos2::new(0.0, 0.0), a, a).0 < 1e-3);
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

    /// Runs one canvas frame with the given input events.
    fn canvas_frame(
        ctx: &egui::Context,
        schematic: &mut Schematic,
        board: &mut Board,
        events: Vec<egui::Event>,
    ) -> egui::FullOutput {
        canvas_frame_at(ctx, schematic, board, events, None)
    }

    /// Runs one canvas frame at `time` (egui counts double clicks in seconds).
    fn canvas_frame_at(
        ctx: &egui::Context,
        schematic: &mut Schematic,
        board: &mut Board,
        events: Vec<egui::Event>,
        time: Option<f64>,
    ) -> egui::FullOutput {
        let levels = PinLevels::default();
        let state = board.evaluate(&levels);
        let input = egui::RawInput {
            time,
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1200.0, 800.0))),
            events,
            ..Default::default()
        };
        ctx.run(input, |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                schematic.show(ui, board, &levels, &state, 0);
            });
        })
    }

    /// A board whose wire has to go around a part parked between the pins.
    fn blocked_board() -> (Board, DeviceId) {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(700.0, 300.0));
        board.add_uart(Pos2::new(520.0, 250.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        (board, led)
    }

    #[test]
    fn a_wire_goes_around_the_parts_instead_of_through_them() {
        let (mut board, _) = blocked_board();
        let mut schematic = Schematic::default();
        let ctx = egui::Context::default();
        canvas_frame(&ctx, &mut schematic, &mut board, Vec::new());
        canvas_frame(&ctx, &mut schematic, &mut board, Vec::new());

        let route = schematic.routes[0].points.clone();
        assert!(route.len() >= 3, "the wire must have at least one corner");
        assert!(route::orthogonal(&route), "{route:?}");
        let obstacles = obstacles_of(&board);
        assert!(
            !route::collides(&route, &obstacles),
            "the wire runs through a part: {route:?}"
        );
        // It really detours: no straight line would fit between the two pins.
        let start = *route.first().unwrap();
        let goal = *route.last().unwrap();
        let straight = (goal.x - start.x).abs() + (goal.y - start.y).abs();
        assert!(
            route::length(&route) > straight * 1.05,
            "the wire did not detour: {route:?}"
        );
    }

    /// Runs the frames of one primary click at `screen`, at `time` seconds.
    ///
    /// Two clicks closer than `max_double_click_delay` count as one double
    /// click, so the times decide what gesture the canvas sees.
    fn primary_click(
        ctx: &egui::Context,
        schematic: &mut Schematic,
        board: &mut Board,
        screen: Pos2,
        time: f64,
    ) {
        canvas_frame_at(
            ctx,
            schematic,
            board,
            vec![egui::Event::PointerMoved(screen)],
            Some(time),
        );
        canvas_frame_at(
            ctx,
            schematic,
            board,
            vec![
                egui::Event::PointerButton {
                    pos: screen,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::default(),
                },
                egui::Event::PointerButton {
                    pos: screen,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::default(),
                },
            ],
            Some(time),
        );
    }

    #[test]
    fn clicking_a_wire_selects_it_and_double_clicking_adds_a_junction() {
        let (mut board, led) = blocked_board();
        let mut schematic = Schematic::default();
        let ctx = egui::Context::default();
        canvas_frame(&ctx, &mut schematic, &mut board, Vec::new());
        canvas_frame(&ctx, &mut schematic, &mut board, Vec::new());

        // Click the middle of a horizontal run of the route.
        let route = schematic.routes[0].points.clone();
        let segment = route
            .windows(2)
            .find(|pair| (pair[0].y - pair[1].y).abs() < 0.5 && (pair[1] - pair[0]).length() > 30.0)
            .expect("a horizontal run");
        let middle = Pos2::new((segment[0].x + segment[1].x) * 0.5, segment[0].y);
        let origin = schematic.last_rect.min;
        let screen = schematic.to_screen(middle, origin);

        primary_click(&ctx, &mut schematic, &mut board, screen, 1.0);
        assert_eq!(schematic.selected(), Some(Selection::Wire(0)));

        // A double click on the same spot splices in a tap point.
        primary_click(&ctx, &mut schematic, &mut board, screen, 1.05);
        let junction = board.first_of(DeviceKind::Junction).expect("a junction");
        let tap = PinRef::Device {
            device: junction.id,
            pin: 0,
        };
        assert!(
            board
                .wires()
                .iter()
                .all(|wire| wire.a == tap || wire.b == tap),
            "the wire was split into two halves that end on the tap"
        );
        assert!(
            board.same_net(
                PinRef::Mcu { port: 0, pin: 5 },
                PinRef::Device {
                    device: led,
                    pin: 0
                }
            ),
            "the tap must not break the wire it sits on"
        );
        assert_eq!(schematic.selected(), Some(Selection::Device(junction.id)));
        assert!((junction.pin_pos(0) - middle).length() < 2.0);

        // The selected tap is ringed, so the user sees what a double click hits.
        let output = canvas_frame(&ctx, &mut schematic, &mut board, Vec::new());
        assert!(
            circle_strokes(&output, Color32::WHITE) > 0,
            "the tap was not ringed"
        );
        // Both halves keep being routed as orthogonal wires.
        assert_eq!(schematic.routes.len(), 2);
        for routed in &schematic.routes {
            assert!(routed.points.len() >= 2, "a half lost its route");
            assert!(route::orthogonal(&routed.points), "{:?}", routed.points);
        }

        // Double clicking the tap again takes it away and restores one wire.
        primary_click(&ctx, &mut schematic, &mut board, screen, 3.0);
        primary_click(&ctx, &mut schematic, &mut board, screen, 3.05);
        assert!(board.first_of(DeviceKind::Junction).is_none());
        assert_eq!(board.wires().len(), 1, "the halves were fused back");
        assert!(board.is_connected(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0
            }
        ));
    }

    #[test]
    fn dragging_a_segment_pulls_a_corner_out_of_it() {
        let (mut board, _) = blocked_board();
        let mut schematic = Schematic::default();
        let ctx = egui::Context::default();
        canvas_frame(&ctx, &mut schematic, &mut board, Vec::new());
        canvas_frame(&ctx, &mut schematic, &mut board, Vec::new());

        let route = schematic.routes[0].points.clone();
        let segment = route
            .windows(2)
            .find(|pair| (pair[0].y - pair[1].y).abs() < 0.5 && (pair[1] - pair[0]).length() > 30.0)
            .expect("a horizontal run");
        let middle = Pos2::new((segment[0].x + segment[1].x) * 0.5, segment[0].y);
        let origin = schematic.last_rect.min;
        let grab = schematic.to_screen(middle, origin);
        let target = grab + Vec2::new(0.0, 60.0);

        for events in [
            vec![egui::Event::PointerMoved(grab)],
            vec![egui::Event::PointerButton {
                pos: grab,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::default(),
            }],
            vec![egui::Event::PointerMoved(grab + Vec2::new(0.0, 20.0))],
            vec![egui::Event::PointerMoved(target)],
            vec![egui::Event::PointerButton {
                pos: target,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::default(),
            }],
        ] {
            canvas_frame(&ctx, &mut schematic, &mut board, events);
        }

        assert_eq!(board.wires()[0].bends.len(), 1, "a corner was pulled out");
        let bend = board.wires()[0].bends[0];
        let expected = schematic.to_board(target, origin);
        assert!(
            (bend - expected).length() < 6.0,
            "the corner should follow the pointer ({bend:?} vs {expected:?})"
        );
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

    /// Circles stroked with `color`.
    fn circle_strokes(output: &egui::FullOutput, color: Color32) -> usize {
        fn walk(shape: &egui::Shape, color: Color32, count: &mut usize) {
            match shape {
                egui::Shape::Circle(circle) if circle.stroke.color == color => *count += 1,
                egui::Shape::Vec(shapes) => {
                    for shape in shapes {
                        walk(shape, color, count);
                    }
                }
                _ => {}
            }
        }
        let mut count = 0;
        for clipped in &output.shapes {
            walk(&clipped.shape, color, &mut count);
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

        let frame = |events: Vec<egui::Event>, schematic: &mut Schematic, board: &mut Board| {
            let input = egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(900.0, 700.0))),
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
