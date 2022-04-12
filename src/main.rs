#![no_main]
#![no_std]
#![feature(core_ffi_c)]

//extern crate alloc;

// External library links
// https://github.com/rust-embedded/book/issues/255


use nrf52840_hal::clocks::Clocks;
use nrf52840_hal::usbd::{UsbPeripheral, Usbd};

use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use usbd_scsi::{Scsi, BlockDevice};

use ghostfat::{GhostFat, File};

use fugit::ExtU32;

use defmt_rtt as _;
use panic_probe as _;

use wasm_embedded_spec::{spi, gpio, i2c};
use wasm_embedded_rt_wasm3::{Wasm3Runtime, Engine as _};

mod newlib;

mod monotonic;
use monotonic::MonoTimer;

mod flash;
use flash::FlashControl;


const BIN: &'static [u8] = &[ 0x00, 0x11, 0x22 ];


/// Combine USB classes to avoid mutex nightmares
pub struct UsbCtx {
    pub usb_serial: SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
    pub usb_store: Scsi<'static, Usbd<UsbPeripheral<'static>>, GhostFat<'static>>,
}


#[rtic::app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [PWM0, PWM1])]
mod app {
    use nrf52840_hal::clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped};
    use super::*;

    #[monotonic(binds = TIMER0, default = true)]
    type Monotonic = MonoTimer<nrf52840_hal::pac::TIMER0>;

    #[shared]
    struct Shared {
        usb: UsbCtx
    }

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
    }

    /// Wild clock-related dance to extend the lifetime of the USB allocator to `'static` required by RTIC
    #[init(local = [
        CLOCKS: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None,
        USB_ALLOCATOR: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None,
        BIN: [u8; 1024] = [0u8; 1024],
        FILES: Option<[File<'static>; 2]> = None,
        FLASH: Option<FlashControl> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let periph = cx.device;

        // Setup clocks
        let clocks = Clocks::new(periph.CLOCK);
        let clocks = clocks.enable_ext_hfosc();

        *cx.local.CLOCKS = Some(clocks);
        let clocks = cx.local.CLOCKS.as_ref().unwrap();

        defmt::info!("init");

        // Setup monotonic timer
        let mono = Monotonic::new(periph.TIMER0);

        // Setup USB allocator
        *cx.local.USB_ALLOCATOR = Some(Usbd::new(UsbPeripheral::new(periph.USBD, &clocks)));
        let usb_bus = cx.local.USB_ALLOCATOR.as_ref().unwrap();

        // Attach USB classes
        let usb_serial = SerialPort::new(&usb_bus);

        // Setup files
        *cx.local.FLASH = Some(FlashControl::new(periph.NVMC));
        
        *cx.local.FILES = Some([
            File::new_ro("a.txt", b"12345\r\n"),
            //File::new_rw("b.bin", cx.local.BIN.as_mut()),
            File::new_dyn("b.bin", cx.local.FLASH.as_mut().unwrap()),
        ]);

        let block_dev = GhostFat::new(cx.local.FILES.as_mut().unwrap(), Default::default());
        let usb_store = Scsi::new(&usb_bus, 64, block_dev, "V", "P", "0.1");

        // Setup USB device
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, 
                UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(0xE3) // Custom device class (combining CDC and MSD/SCSI)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

        // Schedule USB task
        poll_usb::spawn().ok();

        // Schedule WASM task
        poll_wasm::spawn().ok();

        defmt::info!("Init complete");

        (Shared { usb: UsbCtx{ usb_serial, usb_store } }, Local{ usb_dev }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            unsafe { core::arch::asm!("nop") };
        }
    }

    #[task(priority = 1, shared = [], local = [])]
    fn poll_wasm(mut cx: poll_wasm::Context) {
        // Setup WASM context
        let mut ctx = Context{};

        defmt::info!("Configuring runtime");

        // Setup runtime
        let mut rt = match Wasm3Runtime::new(&mut ctx, BIN) {
            Ok(rt) => rt,
            Err(e) =>{
                defmt::error!("Failed to initialise runtime: {:?}", defmt::Debug2Format(&e));
                loop {}
            }
        };

        defmt::info!("Executing task");

        // Execute task
        if let Err(e) = rt.run() {
            defmt::error!("Runtime error: {:?}", defmt::Debug2Format(&e));
        }
    }

    /// USB polling software task, periodic (todo: enable IRQs too?)
    #[task(priority = 3, shared = [usb], local = [usb_dev])]
    fn poll_usb(mut cx: poll_usb::Context) {
 
        let usb_dev = &mut cx.local.usb_dev;

        // Lock on shared USB devices
        cx.shared.usb.lock(|usb| {

            // Poll USB device
            usb_dev.poll(&mut [&mut usb.usb_store, &mut usb.usb_serial]);
            let mut buf = [0u8; 64];

            // Echo on serial port (placeholder)
            if let Ok(count) = usb.usb_serial.read(&mut buf) {
            defmt::info!("{:?}", &buf[..count]);

                for (i, c) in buf.iter().enumerate() {
                    if i >= count {
                        break;
                    }
                    
                    usb.usb_serial.write(&[c.clone()]).ok();
                }
            };

            // Update block device?
            // https://github.com/cs2dsb/stm32-usb.rs/blob/master/firmware/usb_bootloader/src/bin/msc.rs#L483
            //usb.usb_store.block_device_mut().tick(TICK_MS);
        });

        // Schedule next USB tick
        poll_usb::spawn_after(1u32.millis()).ok();
    }
}



pub struct Block<const N: usize = 16384> {
    data: [u8; N],
}

impl <const N: usize> Block<N> {
    pub fn new() -> Self {
        Self {
            data: [0u8; N],
        }
    }
}


impl <const N: usize> BlockDevice for Block<N> {
    const BLOCK_BYTES: usize = 512;

    fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), usbd_scsi::BlockDeviceError> {
        defmt::debug!("Read block {} ({} bytes)", lba, block.len());

        block.copy_from_slice(&self.data[lba as usize * Self::BLOCK_BYTES..][..block.len()]);

        defmt::trace!("Data: {:?}", block);

        Ok(())
    }

    fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), usbd_scsi::BlockDeviceError> {
        defmt::debug!("Write block {}", lba);
        defmt::trace!("Data: {:?}", block);

        self.data[lba as usize * Self::BLOCK_BYTES..][..block.len()].copy_from_slice(block);

        Ok(())
    }

    fn max_lba(&self) -> u32 {
        (N / Self::BLOCK_BYTES - 1) as u32
    }
}

struct Context {

}

impl wasm_embedded_spec::spi::Spi for Context {
    fn init(
        &mut self,
        dev: u32,
        baud: u32,
        mosi: i32,
        miso: i32,
        sck: i32,
        cs: i32,
    ) -> Result<i32, wasm_embedded_spec::Error> {
        todo!()
    }

    fn deinit(&mut self, handle: i32) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn write<'a>(&mut self, handle: i32, data: &[u8]) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn transfer<'a>(&mut self, handle: i32, data: &mut [u8]) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn exec<'a>(&mut self, handle: i32, ops: &[embedded_hal::spi::blocking::Operation<u8>]) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }
}

impl wasm_embedded_spec::i2c::I2c for Context {
    fn init(&mut self, dev: u32, _baud: u32, sda: i32, sck: i32) -> Result<i32, wasm_embedded_spec::Error> {
        todo!()
    }

    fn deinit(&mut self, handle: i32) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn write(&mut self, handle: i32, addr: u16, data: &[u8]) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn read(&mut self, handle: i32, addr: u16, buff: &mut [u8]) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn write_read(
        &mut self,
        handle: i32,
        addr: u16,
        data: &[u8],
        buff: &mut [u8],
    ) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }
}

impl wasm_embedded_spec::gpio::Gpio for Context {
    fn init(&mut self, port: u32, pin: u32, output: bool) -> Result<i32, wasm_embedded_spec::Error> {
        todo!()
    }

    fn deinit(&mut self, handle: i32) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn set(&mut self, handle: i32, state: embedded_hal::digital::PinState) -> Result<(), wasm_embedded_spec::Error> {
        todo!()
    }

    fn get(&mut self, handle: i32) -> Result<embedded_hal::digital::PinState, wasm_embedded_spec::Error> {
        todo!()
    }
}
