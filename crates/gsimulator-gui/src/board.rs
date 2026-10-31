//! Schematic board: MCU pins, power rails and virtual peripherals wired
//! together the way an EDA tool would do it.
//!
//! The model is deliberately independent of the simulation: it only consumes a
//! [`PinLevels`] snapshot and produces a [`BoardState`] (which LED is lit, which
//! nets are shorted, ...). That keeps the electrical rules unit-testable
//! without opening a window, and keeps the simulator free of any external
//! device code - the board lives entirely in the front end.

use egui::{Pos2, Vec2};
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::rc::Rc;

/// Identifier of a device placed on the board.
pub type DeviceId = u32;

/// Which side of a device symbol a pin is drawn on.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Side {
    Left,
    Right,
}

/// Electrical role of a device pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PinRole {
    /// Wants a positive voltage (LED anode, `+`, ...).
    Positive,
    /// Wants the return path (LED cathode, `-`, ...).
    Negative,
    /// Data arriving into the device (UART `RX`).
    Rx,
    /// Data leaving the device (UART `TX`).
    Tx,
    /// One of the quadrature contacts of an encoder.
    Phase,
    /// Momentary switch contact (encoder shaft).
    Button,
    /// Supplies a logic high (the `VCC` power symbol).
    PowerHigh,
    /// Supplies a logic low (the `GND` power symbol).
    PowerLow,
    /// Clock of a synchronous bus (`SPI` `CLK`).
    Clock,
    /// Active low chip select of a synchronous bus (`SPI` `CS`).
    Select,
}

/// One pin of a device symbol.
#[derive(Clone, Copy, Debug)]
pub struct PinDef {
    pub name: &'static str,
    pub role: PinRole,
    pub side: Side,
}

/// A connectable point on the schematic.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Debug)]
pub enum PinRef {
    Mcu { port: u8, pin: u8 },
    Device { device: DeviceId, pin: u8 },
}

impl PinRef {
    /// Short human readable name, used in labels and messages.
    pub fn label(&self, board: &Board) -> String {
        match self {
            PinRef::Mcu { port, pin } => {
                format!(
                    "P{}{}",
                    gsim_stm32f1::PORT_NAMES
                        .get(*port as usize)
                        .copied()
                        .unwrap_or("?"),
                    pin
                )
            }
            PinRef::Device { device, pin } => match board.device(*device) {
                Some(device) => format!("{}.{}", device.name, device.pin_name(*pin)),
                None => format!("device{device}.{pin}"),
            },
        }
    }
}

/// Highlight colour of a device symbol (the LED uses it as its light colour).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Accent {
    pub name: &'static str,
    pub rgb: [u8; 3],
}

impl Accent {
    pub const ALL: [Accent; 4] = [
        Accent {
            name: "红",
            rgb: [230, 60, 50],
        },
        Accent {
            name: "绿",
            rgb: [60, 210, 90],
        },
        Accent {
            name: "蓝",
            rgb: [70, 140, 255],
        },
        Accent {
            name: "黄",
            rgb: [245, 200, 60],
        },
    ];

    pub fn color32(self) -> egui::Color32 {
        egui::Color32::from_rgb(self.rgb[0], self.rgb[1], self.rgb[2])
    }
}

/// The kinds of virtual peripherals the board can hold.
///
/// Adding a device means adding a variant here, a pin table below, a symbol in
/// `canvas.rs` and its electrical rule in [`Board::evaluate`].
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum DeviceKind {
    Led,
    Uart,
    Encoder,
    /// `VCC` power symbol: every net it touches is pulled high.
    Vcc,
    /// `GND` power symbol: every net it touches is pulled low.
    Gnd,
    /// CAN transceiver between the controller's `TXD`/`RXD` and the virtual bus.
    Can,
    /// MAX7219 8x8 LED matrix chain on an SPI port.
    Matrix,
}

/// Pin index of the terminal's receive contact (data from the MCU).
pub const UART_RX: u8 = 0;
/// Pin index of the terminal's transmit contact (data into the MCU).
pub const UART_TX: u8 = 1;
/// Pin index of the encoder contact `A`.
pub const ENCODER_A: u8 = 0;
/// Pin index of the encoder contact `B`.
pub const ENCODER_B: u8 = 1;
/// Pin index of the encoder push button.
pub const ENCODER_SW: u8 = 2;
/// Pin index of the only pin of a power symbol.
pub const POWER_PIN: u8 = 0;
/// Pin index of the transceiver's controller transmit input (`CANx_TX`).
pub const CAN_TXD: u8 = 0;
/// Pin index of the transceiver's controller receive output (`CANx_RX`).
pub const CAN_RXD: u8 = 1;
/// Pin index of the matrix data input (`MOSI`).
pub const MATRIX_DIN: u8 = 0;
/// Pin index of the matrix shift clock (`SCK`).
pub const MATRIX_CLK: u8 = 1;
/// Pin index of the matrix latch (`CS`/`LOAD`).
pub const MATRIX_CS: u8 = 2;
/// Pin index of the matrix data output (`MISO`).
pub const MATRIX_DOUT: u8 = 3;

pub const LED_PINS: [PinDef; 2] = [
    PinDef {
        name: "A",
        role: PinRole::Positive,
        side: Side::Left,
    },
    PinDef {
        name: "K",
        role: PinRole::Negative,
        side: Side::Right,
    },
];

pub const UART_PINS: [PinDef; 2] = [
    PinDef {
        name: "RX",
        role: PinRole::Rx,
        side: Side::Left,
    },
    PinDef {
        name: "TX",
        role: PinRole::Tx,
        side: Side::Right,
    },
];

pub const ENCODER_PINS: [PinDef; 3] = [
    PinDef {
        name: "A",
        role: PinRole::Phase,
        side: Side::Left,
    },
    PinDef {
        name: "B",
        role: PinRole::Phase,
        side: Side::Left,
    },
    PinDef {
        name: "SW",
        role: PinRole::Button,
        side: Side::Right,
    },
];

pub const VCC_PINS: [PinDef; 1] = [PinDef {
    name: "+",
    role: PinRole::PowerHigh,
    side: Side::Right,
}];

pub const GND_PINS: [PinDef; 1] = [PinDef {
    name: "-",
    role: PinRole::PowerLow,
    side: Side::Right,
}];

pub const CAN_PINS: [PinDef; 2] = [
    PinDef {
        name: "TXD",
        role: PinRole::Rx,
        side: Side::Left,
    },
    PinDef {
        name: "RXD",
        role: PinRole::Tx,
        side: Side::Right,
    },
];

pub const MATRIX_PINS: [PinDef; 4] = [
    PinDef {
        name: "DIN",
        role: PinRole::Rx,
        side: Side::Left,
    },
    PinDef {
        name: "CLK",
        role: PinRole::Clock,
        side: Side::Left,
    },
    PinDef {
        name: "CS",
        role: PinRole::Select,
        side: Side::Left,
    },
    PinDef {
        name: "DOUT",
        role: PinRole::Tx,
        side: Side::Right,
    },
];

impl DeviceKind {
    pub const ALL: [DeviceKind; 7] = [
        DeviceKind::Led,
        DeviceKind::Uart,
        DeviceKind::Encoder,
        DeviceKind::Vcc,
        DeviceKind::Gnd,
        DeviceKind::Can,
        DeviceKind::Matrix,
    ];

    pub fn pins(self) -> &'static [PinDef] {
        match self {
            DeviceKind::Led => &LED_PINS,
            DeviceKind::Uart => &UART_PINS,
            DeviceKind::Encoder => &ENCODER_PINS,
            DeviceKind::Vcc => &VCC_PINS,
            DeviceKind::Gnd => &GND_PINS,
            DeviceKind::Can => &CAN_PINS,
            DeviceKind::Matrix => &MATRIX_PINS,
        }
    }

    pub fn body_size(self) -> Vec2 {
        match self {
            DeviceKind::Led => Vec2::new(92.0, 58.0),
            DeviceKind::Uart => Vec2::new(120.0, 74.0),
            DeviceKind::Encoder => Vec2::new(78.0, 82.0),
            DeviceKind::Vcc | DeviceKind::Gnd => Vec2::new(56.0, 46.0),
            DeviceKind::Can => Vec2::new(112.0, 68.0),
            DeviceKind::Matrix => Vec2::new(96.0, 96.0),
        }
    }

    /// Name prefix used when a device of this kind is placed.
    pub fn prefix(self) -> &'static str {
        match self {
            DeviceKind::Led => "LED",
            DeviceKind::Uart => "UART",
            DeviceKind::Encoder => "ENC",
            DeviceKind::Vcc => "VCC",
            DeviceKind::Gnd => "GND",
            DeviceKind::Can => "CAN",
            DeviceKind::Matrix => "MX",
        }
    }

    /// Label shown in the toolbar, the device list and the library.
    pub fn label(self) -> &'static str {
        match self {
            DeviceKind::Led => "LED",
            DeviceKind::Uart => "UART 终端",
            DeviceKind::Encoder => "旋转编码器",
            DeviceKind::Vcc => "VCC 电源",
            DeviceKind::Gnd => "GND 地",
            DeviceKind::Can => "CAN 收发器",
            DeviceKind::Matrix => "SPI 灯阵",
        }
    }

    /// Group the part is filed under in the device library.
    pub fn category(self) -> &'static str {
        match self {
            DeviceKind::Led => "指示",
            DeviceKind::Uart => "通信",
            DeviceKind::Encoder => "交互",
            DeviceKind::Vcc | DeviceKind::Gnd => "电源",
            DeviceKind::Can => "通信",
            DeviceKind::Matrix => "显示",
        }
    }

    /// One line explanation shown on the library card.
    pub fn summary(self) -> &'static str {
        match self {
            DeviceKind::Led => "按网络电平点亮的指示灯",
            DeviceKind::Uart => "收发串口字节的文本终端",
            DeviceKind::Encoder => "带按键的正交旋转编码器",
            DeviceKind::Vcc => "高电平电源符号，可放多个",
            DeviceKind::Gnd => "地符号，可放多个",
            DeviceKind::Can => "把 CAN1 接到虚拟总线",
            DeviceKind::Matrix => "MAX7219 驱动的 8×8 点阵",
        }
    }

    /// Extra words the library search matches, so both `led` and `二极管`
    /// find the same part.
    pub fn keywords(self) -> &'static str {
        match self {
            DeviceKind::Led => "led 发光二极管 指示灯 灯",
            DeviceKind::Uart => "uart usart 串口 串行 终端 terminal 控制台",
            DeviceKind::Encoder => "encoder 编码器 旋钮 旋转 ec11",
            DeviceKind::Vcc => "vcc 电源 正极 高电平 供电",
            DeviceKind::Gnd => "gnd 地 负极 零伏 接地",
            DeviceKind::Can => "can canbus 总线 收发器 收发 tja1050",
            DeviceKind::Matrix => "max7219 灯阵 点阵 数码管 spi 显示 oled",
        }
    }

    /// Stable slug used in widget ids.
    pub fn slug(self) -> &'static str {
        match self {
            DeviceKind::Led => "led",
            DeviceKind::Uart => "uart",
            DeviceKind::Encoder => "encoder",
            DeviceKind::Vcc => "vcc",
            DeviceKind::Gnd => "gnd",
            DeviceKind::Can => "can",
            DeviceKind::Matrix => "matrix",
        }
    }

    /// True for the two power symbols, which are drawn without a body box.
    pub fn is_power(self) -> bool {
        matches!(self, DeviceKind::Vcc | DeviceKind::Gnd)
    }

    /// Pin names of the symbol, e.g. `A K`.
    pub fn pin_summary(self) -> String {
        self.pins()
            .iter()
            .map(|def| def.name)
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// True when `needle` (already lowercased) matches the library entry.
    pub fn matches(self, needle: &str) -> bool {
        [
            self.label(),
            self.category(),
            self.summary(),
            self.keywords(),
        ]
        .iter()
        .any(|text| text.to_lowercase().contains(needle))
    }

    /// Index of the pin with the given role, if the symbol has one.
    pub fn pin_with_role(self, role: PinRole) -> Option<u8> {
        self.pins()
            .iter()
            .position(|def| def.role == role)
            .map(|index| index as u8)
    }

    /// Connection point of `pin` for a symbol of this kind whose body occupies
    /// `rect`, at the tip of its stub.
    ///
    /// Pins are spread evenly along the side they belong to, so a symbol with
    /// two contacts on the left (`encoder A`/`B`) does not stack them.
    pub fn pin_pos(self, rect: egui::Rect, pin: u8) -> Pos2 {
        let Some(def) = self.pins().get(pin as usize) else {
            return rect.center();
        };
        let (index, count) = self.pins_on_side(def.side, pin);
        let fraction = (index as f32 + 1.0) / (count as f32 + 1.0);
        let y = rect.top() + rect.height() * fraction;
        match def.side {
            Side::Left => Pos2::new(rect.left() - STUB, y),
            Side::Right => Pos2::new(rect.right() + STUB, y),
        }
    }

    /// Position of `pin` within the pins of its side, and how many there are.
    fn pins_on_side(self, side: Side, pin: u8) -> (usize, usize) {
        let mut index = 0;
        let mut count = 0;
        for (position, def) in self.pins().iter().enumerate() {
            if def.side != side {
                continue;
            }
            if position as u8 == pin {
                index = count;
            }
            count += 1;
        }
        (index, count)
    }
}

/// Runtime state of a placed device.
///
/// The schematic itself stays a pure topology; the parts that need memory
/// (a terminal's log, an encoder's phase, a matrix's frame buffer) keep it
/// here. The two bus parts are shared with the simulation through `Rc`, which
/// is how the bridge lets them see the events of the slice being executed.
#[derive(Clone, Debug)]
pub enum Hardware {
    /// Stateless part (the LED and the power symbols are evaluated from the
    /// nets alone).
    None,
    Uart(gsim_devices::Uart),
    Encoder(gsim_devices::Encoder),
    Can(Rc<RefCell<gsim_devices::CanTransceiver>>),
    Matrix(Rc<RefCell<gsim_devices::Max7219>>),
}

impl Hardware {
    pub fn uart(&self) -> Option<&gsim_devices::Uart> {
        match self {
            Hardware::Uart(uart) => Some(uart),
            _ => None,
        }
    }

    pub fn uart_mut(&mut self) -> Option<&mut gsim_devices::Uart> {
        match self {
            Hardware::Uart(uart) => Some(uart),
            _ => None,
        }
    }

    pub fn encoder(&self) -> Option<&gsim_devices::Encoder> {
        match self {
            Hardware::Encoder(encoder) => Some(encoder),
            _ => None,
        }
    }

    pub fn encoder_mut(&mut self) -> Option<&mut gsim_devices::Encoder> {
        match self {
            Hardware::Encoder(encoder) => Some(encoder),
            _ => None,
        }
    }

    pub fn can(&self) -> Option<&Rc<RefCell<gsim_devices::CanTransceiver>>> {
        match self {
            Hardware::Can(transceiver) => Some(transceiver),
            _ => None,
        }
    }

    pub fn matrix(&self) -> Option<&Rc<RefCell<gsim_devices::Max7219>>> {
        match self {
            Hardware::Matrix(matrix) => Some(matrix),
            _ => None,
        }
    }
}

/// A device instance placed on the board.
#[derive(Clone, Debug)]
pub struct Device {
    pub id: DeviceId,
    pub kind: DeviceKind,
    pub name: String,
    /// Top-left corner in board space.
    pub pos: Pos2,
    /// Highlight colour of the symbol; the LED uses it as its light colour.
    pub accent: Accent,
    pub hardware: Hardware,
}

impl Device {
    pub fn pin_def(&self, pin: u8) -> Option<&'static PinDef> {
        self.kind.pins().get(pin as usize)
    }

    pub fn pin_name(&self, pin: u8) -> &'static str {
        self.pin_def(pin).map(|def| def.name).unwrap_or("?")
    }

    pub fn rect(&self) -> egui::Rect {
        egui::Rect::from_min_size(self.pos, self.kind.body_size())
    }

    /// Connection point of `pin` in board space, at the tip of its stub.
    pub fn pin_pos(&self, pin: u8) -> Pos2 {
        self.kind.pin_pos(self.rect(), pin)
    }
}

/// Length of the pin stub that sticks out of a symbol, in board units.
pub const STUB: f32 = 22.0;

/// A point to point wire. Every wire joins exactly two pins.
#[derive(Clone, Copy, Debug)]
pub struct Wire {
    pub a: PinRef,
    pub b: PinRef,
}

/// The schematic: devices and the wires between them.
#[derive(Clone, Debug)]
pub struct Board {
    devices: Vec<Device>,
    wires: Vec<Wire>,
    next_id: DeviceId,
    /// Virtual CAN bus every transceiver on the board is attached to.
    can_bus: Rc<RefCell<gsim_devices::CanBus>>,
}

impl Default for Board {
    fn default() -> Self {
        Board::new()
    }
}

impl Board {
    pub fn new() -> Self {
        Board {
            devices: Vec::new(),
            wires: Vec::new(),
            next_id: 1,
            can_bus: Rc::new(RefCell::new(gsim_devices::CanBus::new())),
        }
    }

    /// Board shipped on first start: `VCC`/`GND` power symbols, an LED wired to
    /// PA5 and GND (so the built-in stimulus makes it blink right away), a UART
    /// terminal on USART1's pins and a rotary encoder on PA0/PA1/PA2.
    pub fn demo() -> Self {
        let mut board = Board::new();

        board.add_vcc(Pos2::new(430.0, 110.0));
        let gnd = board.add_gnd(Pos2::new(430.0, 200.0));

        let led = board.add_led(Pos2::new(600.0, 120.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        board.connect(
            PinRef::Device {
                device: gnd,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: led,
                pin: 1,
            },
        );

        let uart = board.add_uart(Pos2::new(600.0, 300.0));
        // MCU TX (PA9) -> terminal RX, terminal TX -> MCU RX (PA10).
        board.connect(
            PinRef::Mcu { port: 0, pin: 9 },
            PinRef::Device {
                device: uart,
                pin: UART_RX,
            },
        );
        board.connect(
            PinRef::Device {
                device: uart,
                pin: UART_TX,
            },
            PinRef::Mcu { port: 0, pin: 10 },
        );

        let encoder = board.add_encoder(Pos2::new(600.0, 480.0));
        for (index, mcu_pin) in [(ENCODER_A, 0u8), (ENCODER_B, 1), (ENCODER_SW, 2)] {
            board.connect(
                PinRef::Device {
                    device: encoder,
                    pin: index,
                },
                PinRef::Mcu {
                    port: 0,
                    pin: mcu_pin,
                },
            );
        }

        board
    }

    pub fn devices(&self) -> &[Device] {
        &self.devices
    }

    pub fn wires(&self) -> &[Wire] {
        &self.wires
    }

    /// The virtual bus every CAN transceiver on the board talks to.
    pub fn can_bus(&self) -> Rc<RefCell<gsim_devices::CanBus>> {
        Rc::clone(&self.can_bus)
    }

    /// Handles of every CAN transceiver placed on the board, in placement order.
    pub fn transceivers(&self) -> Vec<Rc<RefCell<gsim_devices::CanTransceiver>>> {
        self.devices
            .iter()
            .filter_map(|device| device.hardware.can().cloned())
            .collect()
    }

    /// Handles of every LED matrix placed on the board, in placement order.
    pub fn matrices(&self) -> Vec<Rc<RefCell<gsim_devices::Max7219>>> {
        self.devices
            .iter()
            .filter_map(|device| device.hardware.matrix().cloned())
            .collect()
    }

    pub fn device(&self, id: DeviceId) -> Option<&Device> {
        self.devices.iter().find(|device| device.id == id)
    }

    pub fn device_mut(&mut self, id: DeviceId) -> Option<&mut Device> {
        self.devices.iter_mut().find(|device| device.id == id)
    }

    /// Placed device of `kind` with the lowest id, if any.
    pub fn first_of(&self, kind: DeviceKind) -> Option<&Device> {
        self.devices.iter().find(|device| device.kind == kind)
    }

    /// Places a new device of any kind and returns its id.
    pub fn add_device(&mut self, kind: DeviceKind, pos: Pos2) -> DeviceId {
        let id = self.next_id;
        self.next_id += 1;
        let index = self.devices.iter().filter(|d| d.kind == kind).count() + 1;
        let hardware = match kind {
            DeviceKind::Led | DeviceKind::Vcc | DeviceKind::Gnd => Hardware::None,
            DeviceKind::Uart => Hardware::Uart(gsim_devices::Uart::new(0)),
            DeviceKind::Encoder => Hardware::Encoder(gsim_devices::Encoder::new()),
            DeviceKind::Can => Hardware::Can(Rc::new(RefCell::new(
                gsim_devices::CanTransceiver::attach(
                    Rc::clone(&self.can_bus),
                    &format!("CAN{index} 节点"),
                    0,
                ),
            ))),
            DeviceKind::Matrix => Hardware::Matrix(Rc::new(RefCell::new(
                gsim_devices::Max7219::new(0, 0, 4, 1),
            ))),
        };
        self.devices.push(Device {
            id,
            kind,
            name: format!("{}{index}", kind.prefix()),
            pos,
            accent: Accent::ALL[(index - 1) % Accent::ALL.len()],
            hardware,
        });
        id
    }

    /// Places a new LED and returns its id.
    pub fn add_led(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Led, pos)
    }

    /// Places a new UART terminal and returns its id.
    pub fn add_uart(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Uart, pos)
    }

    /// Places a new rotary encoder and returns its id.
    pub fn add_encoder(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Encoder, pos)
    }

    /// Places a new `VCC` symbol and returns its id.
    pub fn add_vcc(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Vcc, pos)
    }

    /// Places a new `GND` symbol and returns its id.
    pub fn add_gnd(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Gnd, pos)
    }

    /// Places a new CAN transceiver and returns its id.
    pub fn add_can(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Can, pos)
    }

    /// Places a new LED matrix chain and returns its id.
    pub fn add_matrix(&mut self, pos: Pos2) -> DeviceId {
        self.add_device(DeviceKind::Matrix, pos)
    }

    /// Mutable access to the placed devices.
    pub fn devices_mut(&mut self) -> &mut [Device] {
        &mut self.devices
    }

    /// The MCU pin `pin` is wired to, if any (only a direct wire to an MCU pin
    /// is reported; a pin behind another part is not resolved).
    pub fn mcu_pin_for(&self, pin: PinRef) -> Option<(u8, u8)> {
        for wire in &self.wires {
            let other = if wire.a == pin {
                wire.b
            } else if wire.b == pin {
                wire.a
            } else {
                continue;
            };
            if let PinRef::Mcu { port, pin } = other {
                return Some((port, pin));
            }
        }
        None
    }

    /// Removes a device together with every wire that touched it.
    pub fn remove_device(&mut self, id: DeviceId) {
        self.devices.retain(|device| device.id != id);
        self.wires.retain(|wire| {
            !matches!(wire.a, PinRef::Device { device, .. } if device == id)
                && !matches!(wire.b, PinRef::Device { device, .. } if device == id)
        });
    }

    /// Connects two pins. Returns `false` when the wire would be a self loop or
    /// already exists.
    pub fn connect(&mut self, a: PinRef, b: PinRef) -> bool {
        if a == b || self.is_connected(a, b) {
            return false;
        }
        self.wires.push(Wire { a, b });
        true
    }

    pub fn is_connected(&self, a: PinRef, b: PinRef) -> bool {
        self.wires
            .iter()
            .any(|wire| (wire.a == a && wire.b == b) || (wire.a == b && wire.b == a))
    }

    pub fn remove_wire(&mut self, index: usize) -> Option<Wire> {
        (index < self.wires.len()).then(|| self.wires.remove(index))
    }

    pub fn clear_wires(&mut self) {
        self.wires.clear();
    }

    /// Pins grouped into electrically connected nets, including single pins.
    pub fn nets(&self) -> Vec<Vec<PinRef>> {
        let mut sets: Vec<Vec<PinRef>> = Vec::new();
        for wire in &self.wires {
            let a = sets.iter().position(|set| set.contains(&wire.a));
            let b = sets.iter().position(|set| set.contains(&wire.b));
            match (a, b) {
                (Some(a), Some(b)) if a == b => {}
                (Some(a), Some(b)) => {
                    let other = sets.remove(b.max(a));
                    let keep = sets.get_mut(b.min(a)).expect("index is valid");
                    keep.extend(other);
                }
                (Some(a), None) => sets[a].push(wire.b),
                (None, Some(b)) => sets[b].push(wire.a),
                (None, None) => sets.push(vec![wire.a, wire.b]),
            }
        }
        sets
    }

    /// Level of every net, plus the state of every device, for one snapshot of
    /// the simulated pin levels.
    pub fn evaluate(&self, levels: &PinLevels) -> BoardState {
        let nets = self.nets();
        let mut net_of: HashMap<PinRef, usize> = HashMap::new();
        for (index, net) in nets.iter().enumerate() {
            for node in net {
                net_of.insert(*node, index);
            }
        }

        let mut levels_of_nets = Vec::with_capacity(nets.len());
        let mut shorts = Vec::new();
        for net in &nets {
            let level = self.net_level(net, levels);
            if level == NetLevel::Shorted {
                shorts.push(net.clone());
            }
            levels_of_nets.push(level);
        }

        let mut leds = BTreeMap::new();
        for device in &self.devices {
            if device.kind == DeviceKind::Led {
                let positive = net_of.get(&PinRef::Device {
                    device: device.id,
                    pin: device.kind.pin_with_role(PinRole::Positive).unwrap_or(0),
                });
                let negative = net_of.get(&PinRef::Device {
                    device: device.id,
                    pin: device.kind.pin_with_role(PinRole::Negative).unwrap_or(1),
                });
                let lit = match (positive, negative) {
                    (Some(positive), Some(negative)) => {
                        positive != negative
                            && levels_of_nets[*positive] == NetLevel::High
                            && levels_of_nets[*negative] == NetLevel::Low
                    }
                    _ => false,
                };
                leds.insert(device.id, lit);
            }
        }

        BoardState {
            nets,
            levels_of_nets,
            shorts,
            leds,
        }
    }

    /// Effective level of one net.
    fn net_level(&self, net: &[PinRef], levels: &PinLevels) -> NetLevel {
        let mut high = false;
        let mut low = false;
        for node in net {
            match node {
                PinRef::Mcu { port, pin } => {
                    if levels.level[*port as usize][*pin as usize] {
                        high = true;
                    } else {
                        low = true;
                    }
                }
                PinRef::Device { device, .. } => match self.device(*device).map(|d| d.kind) {
                    Some(DeviceKind::Vcc) => high = true,
                    Some(DeviceKind::Gnd) => low = true,
                    // Signal parts do not drive anything in this phase.
                    _ => {}
                },
            }
        }
        match (high, low) {
            (true, true) => NetLevel::Shorted,
            (true, false) => NetLevel::High,
            (false, true) => NetLevel::Low,
            (false, false) => NetLevel::Floating,
        }
    }
}

/// Effective level of one net.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum NetLevel {
    High,
    Low,
    Floating,
    /// VCC tied to GND, or a push-pull output fighting the rail.
    Shorted,
}

/// Pin levels read out of the simulated device.
#[derive(Clone, Debug, Default)]
pub struct PinLevels {
    pub level: [[bool; 16]; 5],
    pub output: [[bool; 16]; 5],
}

impl PinLevels {
    pub fn level_of(&self, port: u8, pin: u8) -> bool {
        self.level
            .get(port as usize)
            .and_then(|port| port.get(pin as usize))
            .copied()
            .unwrap_or(false)
    }

    pub fn is_output(&self, port: u8, pin: u8) -> bool {
        self.output
            .get(port as usize)
            .and_then(|port| port.get(pin as usize))
            .copied()
            .unwrap_or(false)
    }
}

/// Result of evaluating the board.
#[derive(Clone, Debug, Default)]
pub struct BoardState {
    pub nets: Vec<Vec<PinRef>>,
    pub levels_of_nets: Vec<NetLevel>,
    pub shorts: Vec<Vec<PinRef>>,
    pub leds: BTreeMap<DeviceId, bool>,
}

impl BoardState {
    pub fn led_lit(&self, id: DeviceId) -> bool {
        self.leds.get(&id).copied().unwrap_or(false)
    }
}

/// Views of the off-chip parts for the status windows.
///
/// The parts only exist while they are placed on the drawing, so the frame's
/// snapshot is completed from the board instead of from the simulation.
impl Board {
    /// Copies the state of the placed transceiver and LED matrix into
    /// `snapshot`, and appends their rows to the peripheral browser.
    pub fn fill_snapshot(&self, snapshot: &mut crate::sim::Snapshot) {
        if let Some(matrix) = self.matrices().first() {
            snapshot.matrix = crate::sim::MatrixSnapshot::of(&matrix.borrow());
        }

        let transceivers = self.transceivers();
        match transceivers.first() {
            Some(handle) => {
                let transceiver = handle.borrow();
                snapshot
                    .can
                    .observe(&self.can_bus.borrow(), Some(&transceiver));
            }
            None => snapshot.can.observe(&self.can_bus.borrow(), None),
        }

        for device in &self.devices {
            match &device.hardware {
                Hardware::Matrix(matrix) => {
                    let matrix = matrix.borrow();
                    snapshot.peripherals.push(status(
                        &device.name,
                        "点阵显示（MAX7219）",
                        matrix_rows(&matrix),
                    ));
                }
                Hardware::Can(transceiver) => {
                    let transceiver = transceiver.borrow();
                    let bus = self.can_bus.borrow();
                    let (tx, rx) = transceiver.pins();
                    snapshot.peripherals.push(status(
                        &device.name,
                        "CAN 收发器（外部）",
                        vec![
                            ("节点", transceiver.name().to_string()),
                            ("模式", transceiver.mode().label().to_string()),
                            (
                                "引脚",
                                format!(
                                    "TXD = {}，RXD = {}",
                                    pin_label(tx),
                                    pin_label(rx)
                                ),
                            ),
                            ("已发送帧", format!("{}", transceiver.stats().tx_frames)),
                            ("已接收帧", format!("{}", transceiver.stats().rx_frames)),
                            ("回环帧", format!("{}", transceiver.stats().loopback_frames)),
                            ("总线帧数", format!("{}", bus.stats().frames)),
                            ("待投递", format!("{}", bus.pending(transceiver.node()))),
                        ],
                    ));
                }
                _ => {}
            }
        }
    }
}

fn status(
    name: &str,
    kind: &'static str,
    rows: Vec<(&'static str, String)>,
) -> gsim_stm32f1::status::PeripheralStatus {
    gsim_stm32f1::status::PeripheralStatus {
        name: name.to_string(),
        kind,
        base: 0,
        clock_hz: 0,
        rows,
    }
}

fn pin_label((port, pin): (u8, u8)) -> String {
    format!(
        "P{}{}",
        gsim_stm32f1::PORT_NAMES
            .get(usize::from(port))
            .copied()
            .unwrap_or("?"),
        pin
    )
}

/// Rows describing one LED matrix module in the peripheral browser.
fn matrix_rows(matrix: &gsim_devices::Max7219) -> Vec<(&'static str, String)> {
    let mut rows = vec![
        ("级联片数", format!("{}", matrix.chips().len())),
        ("LOAD 引脚", pin_label(matrix.cs_pin())),
        ("SPI 单元", format!("SPI{}", matrix.unit() + 1)),
        ("已接收字节", format!("{}", matrix.frames())),
        ("已锁存字", format!("{}", matrix.words())),
    ];
    if let Some(chip) = matrix.chips().first() {
        rows.push(("亮度", format!("{}/15", chip.intensity())));
        rows.push(("扫描行", format!("{}", chip.scanned_rows())));
        rows.push(("关断", if chip.enabled() { "否" } else { "是" }.to_string()));
        rows.push((
            "显示测试",
            if chip.display_test() { "是" } else { "否" }.to_string(),
        ));
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;

    fn levels(low: &[(u8, u8)], high: &[(u8, u8)]) -> PinLevels {
        let mut levels = PinLevels::default();
        for (port, pin) in low {
            levels.level[*port as usize][*pin as usize] = false;
            levels.output[*port as usize][*pin as usize] = true;
        }
        for (port, pin) in high {
            levels.level[*port as usize][*pin as usize] = true;
            levels.output[*port as usize][*pin as usize] = true;
        }
        levels
    }

    #[test]
    fn demo_board_lights_the_led_only_while_pa5_is_high() {
        let board = Board::demo();
        let led = board.first_of(DeviceKind::Led).expect("demo LED").id;

        let state = board.evaluate(&levels(&[], &[(0, 5)]));
        assert!(state.led_lit(led), "LED must light when PA5 is high");
        assert!(state.shorts.is_empty());

        let state = board.evaluate(&levels(&[(0, 5)], &[]));
        assert!(!state.led_lit(led), "LED must go dark when PA5 is low");
    }

    #[test]
    fn led_wired_to_vcc_and_a_low_pin_is_active_low() {
        let mut board = Board::new();
        let vcc = board.add_vcc(Pos2::new(10.0, 10.0));
        let led = board.add_led(Pos2::new(100.0, 10.0));
        assert!(board.connect(
            PinRef::Device {
                device: vcc,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: led,
                pin: 0
            }
        ));
        assert!(board.connect(
            PinRef::Mcu { port: 2, pin: 13 },
            PinRef::Device {
                device: led,
                pin: 1
            }
        ));

        assert!(board.evaluate(&levels(&[(2, 13)], &[])).led_lit(led));
        assert!(!board.evaluate(&levels(&[], &[(2, 13)])).led_lit(led));
    }

    #[test]
    fn led_with_floating_cathode_stays_dark() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(10.0, 10.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        assert!(!board.evaluate(&levels(&[], &[(0, 5)])).led_lit(led));
    }

    #[test]
    fn shorted_led_pins_never_light() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(10.0, 10.0));
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        board.connect(
            PinRef::Mcu { port: 0, pin: 5 },
            PinRef::Device {
                device: led,
                pin: 1,
            },
        );
        assert!(!board.evaluate(&levels(&[], &[(0, 5)])).led_lit(led));
    }

    #[test]
    fn rails_tied_together_report_a_short() {
        let mut board = Board::new();
        let vcc = board.add_vcc(Pos2::new(10.0, 10.0));
        let gnd = board.add_gnd(Pos2::new(10.0, 90.0));
        assert!(board.connect(
            PinRef::Device {
                device: vcc,
                pin: POWER_PIN,
            },
            PinRef::Device {
                device: gnd,
                pin: POWER_PIN,
            },
        ));
        let state = board.evaluate(&levels(&[], &[]));
        assert_eq!(state.shorts.len(), 1);
        assert_eq!(state.levels_of_nets[0], NetLevel::Shorted);
    }

    #[test]
    fn duplicate_and_self_wires_are_rejected() {
        let mut board = Board::new();
        let gnd = board.add_gnd(Pos2::new(10.0, 10.0));
        let a = PinRef::Mcu { port: 0, pin: 5 };
        let b = PinRef::Device {
            device: gnd,
            pin: POWER_PIN,
        };
        assert!(board.connect(a, b));
        assert!(!board.connect(a, b), "duplicate wire");
        assert!(!board.connect(b, a), "duplicate wire, reversed");
        assert!(!board.connect(a, a), "self loop");
        assert_eq!(board.wires().len(), 1);
    }

    #[test]
    fn nets_merge_transitively() {
        let mut board = Board::new();
        board.add_vcc(Pos2::new(0.0, 0.0));
        let vcc = PinRef::Device {
            device: board.first_of(DeviceKind::Vcc).expect("vcc").id,
            pin: POWER_PIN,
        };
        let led = board.add_led(Pos2::new(100.0, 0.0));
        board.connect(PinRef::Mcu { port: 0, pin: 5 }, vcc);
        board.connect(
            vcc,
            PinRef::Device {
                device: led,
                pin: 0,
            },
        );
        let nets = board.nets();
        assert_eq!(nets.len(), 1);
        assert_eq!(nets[0].len(), 3);
    }

    #[test]
    fn removing_a_device_drops_its_wires() {
        let mut board = Board::demo();
        let led = board.first_of(DeviceKind::Led).expect("demo LED").id;
        let before = board.wires().len();
        board.remove_device(led);
        assert!(board.device(led).is_none());
        assert_eq!(
            board.wires().len(),
            before - 2,
            "only the LED wires go away"
        );
        assert!(!board.wires().iter().any(
            |wire| matches!(wire.a, PinRef::Device { device, .. } if device == led)
                || matches!(wire.b, PinRef::Device { device, .. } if device == led)
        ));
    }

    #[test]
    fn the_demo_board_ships_the_power_symbols_and_three_parts() {
        let board = Board::demo();
        let kinds: Vec<DeviceKind> = board.devices().iter().map(|device| device.kind).collect();
        assert_eq!(
            kinds,
            vec![
                DeviceKind::Vcc,
                DeviceKind::Gnd,
                DeviceKind::Led,
                DeviceKind::Uart,
                DeviceKind::Encoder
            ]
        );
        assert_eq!(board.first_of(DeviceKind::Uart).unwrap().name, "UART1");
        assert_eq!(board.first_of(DeviceKind::Encoder).unwrap().name, "ENC1");
        // LED: 2 wires, UART: 2 wires, encoder: 3 wires.
        assert_eq!(board.wires().len(), 7);
    }

    #[test]
    fn a_uart_terminal_only_answers_on_the_pins_it_is_wired_to() {
        let mut board = Board::new();
        let uart = board.add_uart(Pos2::new(0.0, 0.0));
        let rx = PinRef::Device {
            device: uart,
            pin: UART_RX,
        };
        let tx = PinRef::Device {
            device: uart,
            pin: UART_TX,
        };
        assert_eq!(board.mcu_pin_for(rx), None);
        board.connect(PinRef::Mcu { port: 0, pin: 9 }, rx);
        board.connect(tx, PinRef::Mcu { port: 0, pin: 10 });
        assert_eq!(board.mcu_pin_for(rx), Some((0, 9)));
        assert_eq!(board.mcu_pin_for(tx), Some((0, 10)));
        // A power symbol is just another device pin.
        let gnd = board.add_gnd(Pos2::new(0.0, 100.0));
        let gnd_pin = PinRef::Device {
            device: gnd,
            pin: POWER_PIN,
        };
        board.connect(gnd_pin, PinRef::Mcu { port: 0, pin: 0 });
        assert_eq!(board.mcu_pin_for(gnd_pin), Some((0, 0)));
    }

    #[test]
    fn encoder_pins_are_spread_along_their_side() {
        let mut board = Board::new();
        let encoder = board.add_encoder(Pos2::new(100.0, 100.0));
        let device = board.device(encoder).unwrap();
        let a = device.pin_pos(ENCODER_A);
        let b = device.pin_pos(ENCODER_B);
        let sw = device.pin_pos(ENCODER_SW);
        assert_eq!(a.x, b.x, "A and B are on the same side");
        assert!(a.y < b.y, "the two contacts must not overlap");
        assert!(sw.x > device.rect().right());
        assert!(a.x < device.rect().left());
        assert!((a.y - b.y).abs() > 8.0);
    }

    #[test]
    fn placed_devices_carry_their_own_runtime_state() {
        let mut board = Board::new();
        let uart = board.add_uart(Pos2::new(0.0, 0.0));
        let encoder = board.add_encoder(Pos2::new(200.0, 0.0));
        let led = board.add_led(Pos2::new(400.0, 0.0));

        assert_eq!(
            board.device(uart).unwrap().hardware.uart().unwrap().unit(),
            0
        );
        assert_eq!(
            board
                .device(encoder)
                .unwrap()
                .hardware
                .encoder()
                .unwrap()
                .detents(),
            0
        );
        assert!(matches!(
            board.device(led).unwrap().hardware,
            Hardware::None
        ));
    }

    #[test]
    fn device_names_are_unique_and_numbered() {
        let mut board = Board::new();
        let first = board.add_led(Pos2::new(0.0, 0.0));
        let second = board.add_led(Pos2::new(0.0, 100.0));
        assert_eq!(board.device(first).unwrap().name, "LED1");
        assert_eq!(board.device(second).unwrap().name, "LED2");
        assert_ne!(first, second);
    }

    #[test]
    fn device_pin_positions_stick_out_of_the_body() {
        let mut board = Board::new();
        let led = board.add_led(Pos2::new(100.0, 100.0));
        let device = board.device(led).unwrap();
        let anode = device.pin_pos(0);
        let cathode = device.pin_pos(1);
        assert!(anode.x < device.rect().left());
        assert!(cathode.x > device.rect().right());
        assert_eq!(anode.y, cathode.y);
    }
}
