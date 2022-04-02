#![no_std]
#![no_main]


//extern crate alloc;

use wasm_embedded_spec::{spi, gpio, i2c};
use wasm_embedded_rt_wasm3::{Wasm3Runtime, Engine as _};

const BIN: &'static [u8] = &[ 0x00, 0x11, 0x22 ];


#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
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

#[cortex_m_rt::entry]
fn main() -> ! {

    // Setup WASM context
    let mut ctx = Context{};

    let mut rt = match Wasm3Runtime::new(&mut ctx, BIN) {
        Ok(rt) => rt,
        Err(e) =>{
            defmt::error!("Failed to initialise runtime: {:?}", defmt::Debug2Format(&e));
            loop {}
        }
    };

    rt.run();

    //println!("Hello, world!");
    //
    loop {}
}

