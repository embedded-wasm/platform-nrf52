//! Newlib stubs, to be filled

// Fake heap for newlib use, unclear how to mix this with a rust global_allocator
static mut NEWLIB_HEAP: &[u8] = &[0u8; 10 * 1024];
static mut HEAP_PTR: usize = 0;

#[no_mangle]
pub unsafe extern "C" fn _sbrk(incr: i32) -> i32 {
    defmt::warn!("SBRK called incr: {} rem: {}", incr, NEWLIB_HEAP.len() - HEAP_PTR);

    // Check heap bounds
    if HEAP_PTR + incr as usize > NEWLIB_HEAP.len() {
        return 0;
    }

    let ptr = (&NEWLIB_HEAP[HEAP_PTR..][..incr as usize]).as_ptr();

    defmt::warn!("Pointer: {}", ptr);

    // Increment heap ptr
    HEAP_PTR += incr as usize;

    return ptr as i32;
}

#[no_mangle]
pub extern "C" fn _write(_handle: i32 , data: *const core::ffi::c_uchar, len: i32) -> i32 {
    // TODO: worry about handles for non-stdout writes

    // Skip zero length writes
    if len == 0 {
        return 0;
    }

    // Convert into rust string type
    let msg = unsafe { 
        let s = core::slice::from_raw_parts(data, len as usize);
        core::str::from_utf8_unchecked(s)
    };

    // Log via defmt
    defmt::info!("{}", msg);

    return len;
}

#[no_mangle]
pub extern "C" fn _close() {}

#[no_mangle]
pub extern "C" fn _lseek() {}

#[no_mangle]
pub extern "C" fn _read() -> i32 { -1 }

#[no_mangle]
pub extern "C" fn _fstat() -> i32 { -1 }

#[no_mangle]
pub extern "C" fn _isatty() {}

#[no_mangle]
pub extern "C" fn _exit() {}

#[no_mangle]
pub extern "C" fn _open() -> i32 { -1 }

#[no_mangle]
pub extern "C" fn _kill() {}

#[no_mangle]
pub extern "C" fn _getpid() {}
