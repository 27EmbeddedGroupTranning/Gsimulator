/// Implements [`gsim_core::Peripheral`] for a block that exposes the usual
/// inherent `read_reg` / `write_reg` / `advance` / `reset` methods.
#[macro_export]
macro_rules! impl_peripheral {
    ($ty:ty, $name:literal) => {
        impl gsim_core::Peripheral for $ty {
            fn name(&self) -> &'static str {
                $name
            }

            fn read(
                &mut self,
                cx: &gsim_core::Ctx,
                offset: u32,
                width: gsim_core::AccessWidth,
            ) -> Result<u32, gsim_core::BusError> {
                <$ty>::read_reg(self, cx, offset, width)
            }

            fn write(
                &mut self,
                cx: &gsim_core::Ctx,
                offset: u32,
                width: gsim_core::AccessWidth,
                value: u32,
            ) -> Result<(), gsim_core::BusError> {
                <$ty>::write_reg(self, cx, offset, width, value)
            }

            fn advance(&mut self, cx: &gsim_core::Ctx, elapsed_ns: u64) {
                <$ty>::advance(self, cx, elapsed_ns)
            }

            fn reset(&mut self, _cx: &gsim_core::Ctx) {
                <$ty>::reset(self)
            }
        }
    };
}
