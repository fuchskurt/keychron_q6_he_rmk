// Based on the implementation from https://github.com/anpage/rmk-keychron-q6-max/blob/main/src/flash16k.rs
// Copyright (C) 2025 Alex Page
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

/// Wrapper to report 16KB erase size for STM32F4 sectors 1-2
pub struct Flash16K<F>(pub F);

impl<F: ErrorType> ErrorType for Flash16K<F> {
    type Error = F::Error;
}

impl<F: ReadNorFlash> ReadNorFlash for Flash16K<F> {
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> { self.0.read(offset, bytes) }

    fn capacity(&self) -> usize { self.0.capacity() }
}

impl<F: NorFlash> NorFlash for Flash16K<F> {
    const ERASE_SIZE: usize = 16 * 1024;
    const WRITE_SIZE: usize = F::WRITE_SIZE;

    // 16KB for sectors 1-2

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> { self.0.erase(from, to) }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> { self.0.write(offset, bytes) }
}
