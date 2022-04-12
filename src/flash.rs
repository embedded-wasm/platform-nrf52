
use core::cell::UnsafeCell;

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use nrf52840_hal::nvmc::Nvmc;
use nrf52840_pac::NVMC;

const APP_START: usize = 0x00000000;
const APP_LEN: usize = 512 * 1024;

const WASM_START: usize = 0x00080000;
const WASM_LEN: usize = 64 * 1024;

pub struct FlashControl {
    nvm: UnsafeCell<Nvmc<NVMC>>,
    len: usize,
}

unsafe impl Sync for FlashControl {}

impl FlashControl {
    /// Create a new flash controller
    pub fn new(nvmc: NVMC) -> Self {

        let section = unsafe {
            core::slice::from_raw_parts_mut(WASM_START as *const u8 as *mut u8, WASM_LEN)
        };

        Self{
            nvm: UnsafeCell::new(Nvmc::new(nvmc, section)),
            len: WASM_LEN,
        }
    }
}

/// Dynamic file implementation for flash controller
impl <const BLOCK_SIZE: usize> ghostfat::DynamicFile<BLOCK_SIZE> for FlashControl {
    fn len(&self) -> usize {
        self.len
    }
    #[cfg(nope)]
    fn read_chunk(&self, index: usize, buff: &mut [u8]) -> usize {
        defmt::info!("Read file index: 0x{:08x}", index);
        buff.len()
    }


    fn read_chunk(&self, index: usize, buff: &mut [u8]) -> usize {
        defmt::info!("Read file chunk: 0x{:02x} index: 0x{:08x} len: {}", index, index * BLOCK_SIZE, buff.len());

        let res = unsafe { (*self.nvm.get()).read((index * BLOCK_SIZE) as u32, buff) };

        // Read data
        if let Err(e) = res {
            defmt::error!("Failed to read index: 0x{:08x} len: {}: {:?}", index, Nvmc::<NVMC>::ERASE_SIZE, defmt::Debug2Format(&e));
            return 0;
        }

        return buff.len()
    }

    #[cfg(nope)]
    fn write_chunk(&mut self, index: usize, data: &[u8]) -> usize {
        defmt::info!("Write file index: 0x{:08x}", index);
        data.len()
    }

    fn write_chunk(&mut self, index: usize, data: &[u8]) -> usize {
        defmt::info!("Write file index: 0x{:08x}", index);

        // Erase only on the first block
        // TODO: this assumes chunk writes are always aligned... is this always correct?
        if index % Nvmc::<NVMC>::ERASE_SIZE == 0 {
            if let Err(e) = self.nvm.get_mut().erase((index * BLOCK_SIZE) as u32, Nvmc::<NVMC>::ERASE_SIZE as u32) {
                defmt::error!("Failed to erase index: 0x{:08x} len: {}: {:?}", index, Nvmc::<NVMC>::ERASE_SIZE, defmt::Debug2Format(&e));
                return 0;
            }
        }

        // Write data
        if let Err(e) = self.nvm.get_mut().write((index * BLOCK_SIZE) as u32, data) {
            defmt::error!("Failed to write index: 0x{:08x} len: {}: {:?}", index, Nvmc::<NVMC>::ERASE_SIZE, defmt::Debug2Format(&e));
            return 0;
        }

        return data.len();        
    }
}