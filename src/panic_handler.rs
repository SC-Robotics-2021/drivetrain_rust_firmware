/// Non-debug panic handler that simply halts the processor
///
#[allow(unused)]
use core::{
    fmt::Write,
    panic::PanicInfo,
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

#[inline(never)]
#[panic_handler]
fn panic(info_: &PanicInfo) -> ! {
    use cortex_m::interrupt;
    interrupt::disable();
    loop {
        compiler_fence(SeqCst);
    }
}
