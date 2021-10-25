#![no_std]
#![no_main]

use wasm_embedded_rt::wasm3;

const BIN: &'static [u8] = &[ 0x00, 0x11, 0x22 ];


#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

struct Context {

}

#[cortex_m_rt::entry]
fn main() -> ! {

    let mut ctx = Context{};

    let _ = wasm3::run(&mut ctx, BIN);

    //println!("Hello, world!");
    //
    loop {}
}

