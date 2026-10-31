//! Target description and memory map served through `qXfer`.

use gsim_stm32f1::map;

/// Number of registers in the `g`/`G` packets: `r0` to `r15` plus `xPSR`.
pub const REGISTER_COUNT: usize = 17;

/// Register number of `xPSR` in the packets below.
pub const XPSR_REGISTER: usize = 16;

/// Register numbers that are good enough for GDB to place a breakpoint.
pub const PC_REGISTER: usize = 15;

/// Page size of the medium density STM32F103 flash.
pub const FLASH_PAGE_SIZE: u32 = 1024;

/// Size of the peripheral window: APB1, APB2 and AHB next to each other.
const PERIPHERAL_MAP_SIZE: u32 = 0x0004_0000;

/// XML description of the Cortex-M3 register file.
///
/// The registers are declared in packet order, so `g` simply concatenates
/// `r0`..`r15` and `xPSR`.
pub const TARGET_XML: &str = "\
<?xml version=\"1.0\"?>
<!DOCTYPE target SYSTEM \"gdb-target.dtd\">
<target version=\"1.0\">
  <architecture>arm</architecture>
  <feature name=\"org.gnu.gdb.arm.m-profile\">
    <reg name=\"r0\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r1\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r2\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r3\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r4\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r5\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r6\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r7\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r8\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r9\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r10\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r11\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"r12\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>
    <reg name=\"lr\" bitsize=\"32\" type=\"uint32\"/>
    <reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>
    <reg name=\"xpsr\" bitsize=\"32\" type=\"uint32\"/>
  </feature>
</target>
";

/// Memory map of the simulated device, as `qXfer:memory-map:read` returns it.
///
/// Declaring the flash bank makes GDB erase and program it instead of issuing
/// plain writes, which is what a real ST-Link does as well.
pub fn memory_map() -> String {
    format!(
        "\
<?xml version=\"1.0\"?>
<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">
<memory-map>
  <memory type=\"flash\" start=\"0x{flash_base:08x}\" length=\"0x{flash_size:x}\">
    <property name=\"blocksize\">0x{page:x}</property>
  </memory>
  <memory type=\"ram\" start=\"0x{sram_base:08x}\" length=\"0x{sram_size:x}\"/>
  <memory type=\"ram\" start=\"0x{peripheral_base:08x}\" length=\"0x{peripheral_size:x}\"/>
</memory-map>
",
        flash_base = map::FLASH_BASE,
        flash_size = map::FLASH_SIZE,
        page = FLASH_PAGE_SIZE,
        sram_base = map::SRAM_BASE,
        sram_size = map::SRAM_SIZE,
        peripheral_base = map::APB1_BASE,
        peripheral_size = PERIPHERAL_MAP_SIZE,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_target_description_declares_every_packet_register() {
        assert_eq!(TARGET_XML.matches("<reg name=").count(), REGISTER_COUNT);
        assert!(TARGET_XML.contains("<reg name=\"xpsr\""));
        assert!(TARGET_XML.contains("org.gnu.gdb.arm.m-profile"));
    }

    #[test]
    fn the_memory_map_covers_flash_and_sram() {
        let map = memory_map();
        assert!(map.contains("type=\"flash\" start=\"0x08000000\""));
        assert!(map.contains("type=\"ram\" start=\"0x20000000\""));
        assert!(map.contains("<property name=\"blocksize\">0x400</property>"));
    }
}
