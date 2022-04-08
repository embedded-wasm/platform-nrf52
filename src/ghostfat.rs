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


const UF2_SIZE: u32 = 0x10000 * 2;
const UF2_SECTORS: u32 = UF2_SIZE / (512 as u32);


const ASCII_SPACE: u8 = 0x20;


pub struct GhostFatConfig<const BLOCK_SIZE: u32 = 512> {
    pub num_blocks: u32,
    pub reserved_sectors: u32,
    pub root_dir_sectors: u32,

}

impl <const BLOCK_SIZE: u32> Default for GhostFatConfig<BLOCK_SIZE> {
    fn default() -> Self {
        Self { 
            num_blocks: 8000,
            reserved_sectors: 1,
            root_dir_sectors: 4,
        }
    }
}

impl <const BLOCK_SIZE: u32> GhostFatConfig<BLOCK_SIZE> {

    const fn sector_size(&self) -> u32 {
        BLOCK_SIZE
    }

    const fn sectors_per_fat(&self) -> u32 {
        (self.num_blocks * 2 + BLOCK_SIZE - 1) / BLOCK_SIZE
    }

    const fn start_fat0(&self) -> u32 {
        self.reserved_sectors
    }

    const fn start_fat1(&self) -> u32 {
        self.start_fat0() + self.sectors_per_fat()
    }

    const fn start_rootdir(&self) -> u32 {
        self.start_fat1() + self.sectors_per_fat()
    }

    const fn start_clusters(&self) -> u32 {
        self.start_rootdir() + self.root_dir_sectors
    }
}


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

pub fn fat_boot_block(config: &GhostFatConfig) -> FatBootBlock {
    let mut fat = FatBootBlock {
        jump_instruction: [0xEB, 0x3C, 0x90],
        oem_info: [0x20; 8],
        sector_size: config.sector_size() as u16,
        sectors_per_cluster: 1,
        reserved_sectors: config.reserved_sectors as u16,
        fat_copies: 2,
        root_directory_entries: (config.root_dir_sectors as u16 * 512 / 32),
        total_sectors16: config.num_blocks as u16 - 2,
        media_descriptor: 0xF8,
        sectors_per_fat: config.sectors_per_fat() as u16,
        sectors_per_track: 1,
        heads: 1,
        hidden_sectors: 0,
        total_sectors32: config.num_blocks - 1,
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
    config: GhostFatConfig,
    fat_boot_block: FatBootBlock,
    fat_files: [FatFile; 3],
    uf2_blocks_written: u32,
}

impl GhostFat {
    pub fn new(config: GhostFatConfig) -> Self {
        Self{
            fat_boot_block: fat_boot_block(&config),
            fat_files: fat_files(),
            uf2_blocks_written: 0,
            config,
        }
    }
}

impl BlockDevice for GhostFat {
    const BLOCK_BYTES: usize = 512;

    fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), BlockDeviceError> {
        assert_eq!(block.len(), Self::BLOCK_BYTES);

        info!("GhostFAT reading block: {}", lba);

        // Clear the buffer since we're sending all of it
        for b in block.iter_mut() { *b = 0 }
   
        // Block 0 is the fat boot block
        if lba == 0 {
            self.fat_boot_block.pack(&mut block[..FatBootBlock::BYTES]).unwrap();
            block[510] = 0x55;
            block[511] = 0xAA;

        // File allocation table(s) follow the boot block
        } else if lba < self.config.start_rootdir() {
            let mut section_index = lba - self.config.start_fat0();

            // TODO: why?
            // https://github.com/lupyuen/bluepill-bootloader/blob/master/src/ghostfat.c#L207
            if section_index >= self.config.sectors_per_fat() {
                section_index -= self.config.sectors_per_fat();
            }

            // Set allocations for static files
            if section_index == 0 {
                block[0] = 0xF0;
                for i in 1..(self.fat_files.len() * 2 + 4) {
                    block[i] = 0xFF;
                }
            }
         
            // Assuming each file is one block, uf2 is offset by this
            let uf2_first_sector = self.fat_files.len() + 1;
            let uf2_last_sector = uf2_first_sector + UF2_SECTORS as usize - 1;
            
            // TODO: is this setting allocations for the uf2 file?
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

        // Directory entries follow
        } else if lba < self.config.start_clusters() {

            let section_index = lba - self.config.start_rootdir();            
            if section_index == 0 {
                let mut dir = DirectoryEntry::default();
                dir.name.copy_from_slice(&self.fat_boot_block.volume_label);
                dir.attrs = 0x28;

                let len = DirectoryEntry::BYTES;
                dir.pack(&mut block[..len]).unwrap();
                dir.attrs = 0;
                
                // Generate directory entries for registered files
                for (i, info) in self.fat_files.iter().enumerate() {
                    dir.name.copy_from_slice(&info.name);
                    dir.start_cluster = i as u16 + 2;
                    dir.size = match info.content {
                        FatFileContent::Static(content) => content.len() as u32,
                        FatFileContent::Uf2 => {
                            // TODO: set data length for this object
                            0
                        },
                    };
                    let start = (i+1) * len;
                    dir.pack(&mut block[start..(start+len)]).unwrap();
                }
            }
        
        // Then finally clusters (containing actual data)
        } else {
            let section_index = (lba - self.config.start_clusters()) as usize;

            if section_index < self.fat_files.len() {
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
        info!("GhostFAT writing block {}: {:?}", lba, block);

        if lba == 0 {
            warn!("Attempted write to boot sector");
            return Ok(())

        // Write to FAT
        } else if lba < self.config.start_rootdir() {

        // Write directory entry
        } else if lba < self.config.start_clusters() {

        // Write cluster data
        } else {

        }

        // TODO: write block to flash

        Ok(())
    }

    fn max_lba(&self) -> u32 {
        self.config.num_blocks - 1
    }
}
