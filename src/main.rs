#![no_main]
#![no_std]


//extern crate alloc;

use nrf52840_hal::clocks::Clocks;
use nrf52840_hal::usbd::{UsbPeripheral, Usbd};

use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use usbd_scsi::{Scsi, BlockDevice};

use defmt_rtt as _;


use wasm_embedded_spec::{spi, gpio, i2c};

const BIN: &'static [u8] = &[ 0x00, 0x11, 0x22 ];


#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

/// Combine USB classes to avoid mutex nightmares
pub struct UsbCtx {
    pub usb_serial: SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
    pub usb_store: Scsi<'static, Usbd<UsbPeripheral<'static>>, Block>,
}

pub struct Block {

}

impl BlockDevice for Block {
    const BLOCK_BYTES: usize = 128;

    fn read_block(&self, lba: u32, block: &mut [u8]) -> Result<(), usbd_scsi::BlockDeviceError> {
        todo!()
    }

    fn write_block(&mut self, lba: u32, block: &[u8]) -> Result<(), usbd_scsi::BlockDeviceError> {
        todo!()
    }

    fn max_lba(&self) -> u32 {
        todo!()
    }
}

#[rtic::app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [PWM0])]
mod app {
    use nrf52840_hal::clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped};
    use wasm_embedded_rt_wasm3::{Wasm3Runtime, Engine as _};
    use super::*;

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
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let periph = nrf52840_hal::pac::Peripherals::take().unwrap();

        defmt::info!("init");

        // Setup clocks
        let clocks = Clocks::new(periph.CLOCK);
        *cx.local.CLOCKS = Some(clocks.enable_ext_hfosc());
        let clocks = cx.local.CLOCKS.as_ref().unwrap();

        // Setup USB allocator
        *cx.local.USB_ALLOCATOR = Some(Usbd::new(UsbPeripheral::new(periph.USBD, &clocks)));
        let usb_bus = cx.local.USB_ALLOCATOR.as_ref().unwrap();

        // Attach USB classes
        let usb_serial = SerialPort::new(&usb_bus);

        let block_dev = Block{};
        let usb_store = Scsi::new(&usb_bus, 128, block_dev, "Fake Vendor", "Fake Product", "Fake Rev");

        // Setup USB device
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

        // TODO: migrate WASM to tasks

        
        // Setup WASM context
        let mut ctx = Context{};

        // Setup runtime
        let mut rt = match Wasm3Runtime::new(&mut ctx, BIN) {
            Ok(rt) => rt,
            Err(e) =>{
                defmt::error!("Failed to initialise runtime: {:?}", defmt::Debug2Format(&e));
                loop {}
            }
        };

        // Execute task
        if let Err(e) = rt.run() {
            defmt::error!("Runtime error: {:?}", defmt::Debug2Format(&e));
        }

        (Shared { usb: UsbCtx{ usb_serial, usb_store } }, Local{usb_dev}, init::Monotonics())
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {}
    }

    /// USB polling software task, called by IRQs
    #[task(binds = USBD, priority = 3, shared = [usb], local = [usb_dev])]
    fn poll_usb(mut cx: poll_usb::Context) {
 
         let usb_dev = &mut cx.local.usb_dev;
 
         cx.shared.usb.lock(|usb| {
 
             // Poll USB device
             usb_dev.poll(&mut [&mut usb.usb_store, &mut usb.usb_serial]);
             let mut buf = [0u8; 64];
 
             // Echo on serial port (placeholder)
             if let Ok(count) = usb.usb_serial.read(&mut buf) {
                 for (i, c) in buf.iter().enumerate() {
                     if i >= count {
                         break;
                     }
                     usb.usb_serial.write(&[c.clone()]).ok();
                 }
             };
 
         });
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
