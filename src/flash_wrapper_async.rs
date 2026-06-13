//! Erase-size override for the RMK storage backend.

use embedded_storage_async::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

/// Wraps a NOR-flash driver and reports a fixed 16 KiB erase granularity.
///
/// The embassy flash driver reports the largest sector size of the
/// STM32F401's heterogeneous flash layout, but the two sectors reserved for
/// the RMK storage region (sectors 1 and 2, `0x4000..0xC000`) are both
/// 16 KiB. Overriding [`NorFlash::ERASE_SIZE`] to match lets the storage
/// layer address exactly those sectors. Every operation is forwarded to the
/// wrapped driver unchanged.
pub struct Flash16K<F>(pub F);

impl<F: ErrorType> ErrorType for Flash16K<F> {
    /// Error type propagated from the wrapped flash driver.
    type Error = F::Error;
}

impl<F: ReadNorFlash> ReadNorFlash for Flash16K<F> {
    /// Read granularity of the wrapped driver.
    const READ_SIZE: usize = F::READ_SIZE;

    /// Total capacity of the wrapped driver.
    fn capacity(&self) -> usize { self.0.capacity() }

    /// Read `bytes.len()` bytes starting at `offset`.
    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(offset, bytes).await
    }
}

impl<F: NorFlash> NorFlash for Flash16K<F> {
    /// Erase granularity reported to the storage layer: 16 KiB, matching
    /// STM32F401 sectors 1 and 2.
    const ERASE_SIZE: usize = 16_usize.saturating_mul(1024);
    /// Write granularity of the wrapped driver.
    const WRITE_SIZE: usize = F::WRITE_SIZE;

    /// Erase the range `from..to`.
    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> { self.0.erase(from, to).await }

    /// Write `bytes` starting at `offset`.
    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.write(offset, bytes).await
    }
}
