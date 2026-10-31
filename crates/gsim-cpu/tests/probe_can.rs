use std::cell::RefCell;
use std::rc::Rc;

use gsim_core::{AccessWidth, MemoryBus};
use gsim_cpu::{Cpu, ElfImage};
use gsim_stm32f1::Soc;

#[test]
fn probe_can_firmware_traffic() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../firmware/CAN/build/Debug/CAN.elf");
    let image = ElfImage::open(&path).expect("CAN.elf");
    let soc = Rc::new(RefCell::new(Soc::with_hse(8_000_000)));
    let mut cpu = Cpu::new(Rc::clone(&soc)).expect("cpu");
    cpu.load(&image).expect("load");
    let _ = cpu.run(200_000_000);

    let iser0 = cpu.read_core_register(0xE000_E100).unwrap_or(0xDEAD_BEEF);
    let iser1 = cpu.read_core_register(0xE000_E104).unwrap_or(0xDEAD_BEEF);
    let ipr5 = cpu.read_core_register(0xE000_E414).unwrap_or(0xDEAD_BEEF);
    let aircr = cpu.read_core_register(0xE000_ED0C).unwrap_or(0xDEAD_BEEF);
    let shpr3 = cpu.read_core_register(0xE000_ED20).unwrap_or(0xDEAD_BEEF);
    let (pending, enabled, irqs, pc) = (
        soc.borrow().nvic.is_pending(gsim_core::irq::lines::CAN1_RX0),
        soc.borrow().nvic.is_enabled(gsim_core::irq::lines::CAN1_RX0),
        cpu.interrupts(),
        cpu.pc(),
    );
    eprintln!(
        "ISER0=0x{iser0:08X} ISER1=0x{iser1:08X} IPR5=0x{ipr5:08X} AIRCR=0x{aircr:08X} SHPR3=0x{shpr3:08X}"
    );
    eprintln!("pending={pending} soc_enabled={enabled} irqs={irqs} pc=0x{pc:08X}");
    let _ = AccessWidth::Word;
}