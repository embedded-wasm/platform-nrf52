// https://github.com/cs2dsb/stm32-usb.rs/blob/master/firmware/usb_bootloader/src/ghost_fat.rs

use core::ptr::read_volatile;

use packing::{
    Packed,
    PackedSize,
};

use usbd_scsi::{
    BlockDevice,
    BlockDeviceError,
};

use defmt::{info, debug, warn};

use cortex_m::asm;

// 64kb left for bootloader since logging is huge
// TODO: Needs to be configurable, can this be passed in from a linker section maybe?
const UF2_FLASH_START: u32 = 0x08010000;

const BLOCK_SIZE: usize = 512;
const UF2_BLOCK_SIZE: usize = 256;
const UF2_BLOCKS_PER_FAT_BLOCK: u32 = (BLOCK_SIZE / UF2_BLOCK_SIZE) as u32;
const NUM_FAT_BLOCKS: u32 = 8000;
const RESERVED_SECTORS: u32 = 1;
const ROOT_DIR_SECTORS: u32 = 4;
const SECTORS_PER_FAT: u32 = (NUM_FAT_BLOCKS * 2 + 511) / 512;
const START_FAT0: u32 = RESERVED_SECTORS;
const START_FAT1: u32 = START_FAT0 + SECTORS_PER_FAT;
const START_ROOTDIR: u32 = START_FAT1 + SECTORS_PER_FAT;
const START_CLUSTERS: u32 = START_ROOTDIR + ROOT_DIR_SECTORS;
const UF2_SIZE: u32 = 0x10000 * 2;
const UF2_SECTORS: u32 = UF2_SIZE / (BLOCK_SIZE as u32);

const RESTART_DELAY_MS: u32 = 200;



const ASCII_SPACE: u8 = 0x20;



#[derive(Clone, Copy, Default, Packed)]
#[packed(little_endian, lsb0)]
pub struct DirectoryEntry {    
    #[pkd(7, 0, 0, 10)]
    pub name: [u8; 11],
    /*
        pub name: [u8; 8],
        pub ext: [u8; 3],
    */
    #[pkd(7, 0, 11, 11)]
    pub attrs: u8,

    #[pkd(7, 0, 12, 12)]
    _reserved: u8,

    #[pkd(7, 0, 13, 13)]
    pub create_time_fine: u8,

    #[pkd(7, 0, 14, 15)]
    pub create_time: u16,

    #[pkd(7, 0, 16, 17)]
    pub create_date: u16,
    
    #[pkd(7, 0, 18, 19)]
    pub last_access_date: u16,
    
    #[pkd(7, 0, 20, 21)]
    pub high_start_cluster: u16,
    
    #[pkd(7, 0, 22, 23)]
    pub update_time: u16,
    
    #[pkd(7, 0, 24, 25)]
    pub update_date: u16,
    
    #[pkd(7, 0, 26, 27)]
    pub start_cluster: u16,
    
    #[pkd(7, 0, 28, 31)]
    pub size: u32,
}


pub enum FatFileContent {
    Static([u8; 255]),
    Uf2,
}

pub struct FatFile {
    pub name: [u8; 11],
    pub content: FatFileContent,
}


impl FatFile {
    fn with_content<N: AsRef<[u8]>, T: AsRef<[u8]>>(name_: N, content_: T) -> Self {
        let mut name = [0; 11];
        let mut content = [0; 255];

        let bytes = name_.as_ref();
        let l = bytes.len().min(name.len());
        name[..l].copy_from_slice(&bytes[..l]);
        for b in name[l..].iter_mut() {
            *b = ASCII_SPACE
        }

        let bytes = content_.as_ref();
        let l = bytes.len().min(content.len());
        content[..l].copy_from_slice(&bytes[..l]);
        for b in content[l..].iter_mut() {
            *b = ASCII_SPACE
        }

        Self {
            name,
            content: FatFileContent::Static(content),
        }
    }
}

const UF2_INDEX: usize = 2;

pub fn fat_files() -> [FatFile; 3] {
    let info = FatFile::with_content("INFO_UF2TXT", "UF2 Bootloader 1.2.3\r\nModel: BluePill\r\nBoard-ID: xyz_123\r\n");
    let index = FatFile::with_content("INDEX   HTM", "<!doctype html>\n<html><body><script>\nlocation.replace(INDEX_URL);\n</script></body></html>\n");

    let mut name = [ASCII_SPACE; 11];
    name.copy_from_slice("CURRENT UF2".as_bytes());

    let current_uf2 = FatFile {
        name,
        content: FatFileContent::Uf2,
    };
    
    [info, index, current_uf2]
}

#[derive(Clone, Copy, Eq, PartialEq, Debug, Packed)]
#[packed(little_endian, lsb0)]
pub struct FatBootBlock {
    #[pkd(7, 0, 0, 2)]
    pub jump_instruction: [u8; 3],

    #[pkd(7, 0, 3, 10)]
    pub oem_info: [u8; 8],
    
    #[pkd(7, 0, 11, 12)]
    pub sector_size: u16,
    
    #[pkd(7, 0, 13, 13)]
    pub sectors_per_cluster: u8,
    
    #[pkd(7, 0, 14, 15)]
    pub reserved_sectors: u16,
    
    #[pkd(7, 0, 16, 16)]
    pub fat_copies: u8,
    
    #[pkd(7, 0, 17, 18)]
    pub root_directory_entries: u16,
    
    #[pkd(7, 0, 19, 20)]
    pub total_sectors16: u16,
    
    #[pkd(7, 0, 21, 21)]
    pub media_descriptor: u8,
    
    #[pkd(7, 0, 22, 23)]
    pub sectors_per_fat: u16,
    
    #[pkd(7, 0, 24, 25)]
    pub sectors_per_track: u16,
    
    #[pkd(7, 0, 26, 27)]
    pub heads: u16,
    
    #[pkd(7, 0, 28, 31)]
    pub hidden_sectors: u32,
    
    #[pkd(7, 0, 32, 35)]
    pub total_sectors32: u32,
    
    #[pkd(7, 0, 36, 36)]
    pub physical_drive_num: u8,
    
    #[pkd(7, 0, 37, 37)]
    _reserved: u8,
    
    #[pkd(7, 0, 38, 38)]
    pub extended_boot_sig: u8,
    
    #[pkd(7, 0, 39, 42)]
    pub volume_serial_number: u32,
    
    #[pkd(7, 0, 43, 53)]
    pub volume_label: [u8; 11],
    
    #[pkd(7, 0, 54, 61)]
    pub filesystem_identifier: [u8; 8],
}

pub fn fat_boot_block() -> FatBootBlock {
    const RESERVED_SECTORS: u16 = 1;
    const ROOT_DIR_SECTORS: u16 = 4;
    const NUM_FAT_BLOCKS: u16 = 8000;
    const SECTORS_PER_FAT: u16 = (NUM_FAT_BLOCKS * 2 + 511) / 512;
    let mut fat = FatBootBlock {
        jump_instruction: [0xEB, 0x3C, 0x90],
        oem_info: [0x20; 8],
        sector_size: 512,
        sectors_per_cluster: 1,
        reserved_sectors: RESERVED_SECTORS,
        fat_copies: 2,
        root_directory_entries: (ROOT_DIR_SECTORS * 512 / 32),
        total_sectors16: NUM_FAT_BLOCKS - 2,
        media_descriptor: 0xF8,
        sectors_per_fat: SECTORS_PER_FAT,
        sectors_per_track: 1,
        heads: 1,
        hidden_sectors: 0,
        total_sectors32: NUM_FAT_BLOCKS as u32 - 1,
        physical_drive_num: 0,
        _reserved: 0,
        extended_boot_sig: 0x29,
        volume_serial_number: 0x00420042,
        volume_label: [0x20; 11],
        filesystem_identifier: [0x20; 8],
    };
    fat.oem_info[..7].copy_from_slice("UF2 UF2".as_bytes());
    fat.volume_label[..8].copy_from_slice("BLUEPILL".as_bytes());
    fat.filesystem_identifier[..5].copy_from_slice("FAT16".as_bytes());

    fat
}


/// # Dummy fat implementation that provides a [UF2 bootloader](https://github.com/microsoft/uf2)
pub struct GhostFat {
    fat_boot_block: FatBootBlock,
    fat_files: [FatFile; 3],
    uf2_blocks_written: u32,
    tick_ms: u32,
    restart_ms: u32,
}

impl GhostFat {
    pub fn new() -> Self {
        Self{
            fat_boot_block: fat_boot_block(),
            fat_files: fat_files(),
            uf2_blocks_written: 0,
            tick_ms: 0,
            restart_ms: 0,
        }
    }
}

impl BlockDevice for GhostFat {
    const BLOCK_BYTES: usize = BLOCK_SIZE;

    fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), BlockDeviceError> {
        assert_eq!(block.len(), BLOCK_SIZE);

        info!("GhostFAT reading block: {}", lba);

        // Clear the buffer since we're sending all of it
        for b in block.iter_mut() { *b = 0 }
   
        if lba == 0 {
            // Block 0 is the fat boot block
            self.fat_boot_block.pack(&mut block[..FatBootBlock::BYTES]).unwrap();
            block[510] = 0x55;
            block[511] = 0xAA;

        } else if lba < START_ROOTDIR {
            let mut section_index = lba - START_FAT0;

            if section_index >= SECTORS_PER_FAT {
                section_index -= SECTORS_PER_FAT;
            }

            if section_index == 0 {
                block[0] = 0xF0;
                for i in 1..(self.fat_files.len() * 2 + 4) {
                    block[i] = 0xFF;
                }
            }
         
            let uf2_first_sector = self.fat_files.len() + 1;
            let uf2_last_sector = uf2_first_sector + UF2_SECTORS as usize - 1;
            
            for i in 0..256_usize {
                let v = section_index as usize * 256 + i;
                let j = 2 * i;
                if v >= uf2_first_sector && v < uf2_last_sector {
                    block[j+0] = (((v + 1) >> 0) & 0xFF) as u8;
                    block[j+1] = (((v + 1) >> 8) & 0xFF) as u8;
                } else if v == uf2_last_sector {
                    block[j+0] = 0xFF;
                    block[j+1] = 0xFF;
                }
            }         

        } else if lba < START_CLUSTERS {
            let section_index = lba - START_ROOTDIR;            
            if section_index == 0 {
                let mut dir = DirectoryEntry::default();
                dir.name.copy_from_slice(&self.fat_boot_block.volume_label);
                dir.attrs = 0x28;
                let len = DirectoryEntry::BYTES;
                dir.pack(&mut block[..len]).unwrap();
                dir.attrs = 0;
                for (i, info) in self.fat_files.iter().enumerate() {
                    dir.name.copy_from_slice(&info.name);
                    dir.start_cluster = i as u16 + 2;
                    dir.size = match info.content {
                        FatFileContent::Static(content) => content.len() as u32,
                        FatFileContent::Uf2 => {
                            // TODO: read data for this object
                            0
                        },
                    };
                    let start = (i+1) * len;
                    dir.pack(&mut block[start..(start+len)]).unwrap();
                }
            }
            
        } else {
            let section_index = (lba - START_CLUSTERS) as usize;

            if section_index < UF2_INDEX {//self.fat_files.len() {
                let info = &self.fat_files[section_index];
                if let FatFileContent::Static(content) = &info.content {
                    block[..content.len()].copy_from_slice(content);
                }

            } else {
                //UF2
                info!("UF2: {}", section_index);

                // TODO: read data and return
          
            }
        }
        Ok(())
    }

    fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), BlockDeviceError> {
        info!("GhostFAT writing block: {}", lba);

        //TODO: Should BDE have an error to represent this kind of protocol error?
        //      It likely doesn't matter as FAT/SCSI/USB doesn't have a nice way to report
        //      a user facing error. The best we can manage is something like a write error
        //      or phase error. Some DFU firmwares report back errors by creating a file 
        //      called error.txt in the root. Could be an option but it's not part of UF2.
        const PROTOCOL_ERROR: Result<(), BlockDeviceError> = Err(BlockDeviceError::WriteError);

        if lba < (START_CLUSTERS + self.fat_files.len() as u32) {
            info!("    GhostFAT skipping non-UF2 area");
            return Ok(());
        }

        // TODO: write block to flash

        Ok(())
    }
    fn max_lba(&self) -> u32 {
        NUM_FAT_BLOCKS - 1
    }
}
