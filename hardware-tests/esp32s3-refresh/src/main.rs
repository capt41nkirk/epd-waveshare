#![no_std]
#![no_main]

use core::cell::Cell;
use embedded_hal::{
    digital::InputPin,
    spi::{ErrorKind, ErrorType, Operation, SpiDevice},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use epd_waveshare::{
    epd7in5_v2::{Epd7in5, RefreshMode},
    prelude::WaveshareDisplay,
};
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Level, NoPin, Output, OutputConfig},
    spi::master::{Config as SpiConfig, Spi},
    time::{Duration, Instant, Rate},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

// Static RAM avoids putting the 48 KB frame on the main task's stack.
static mut FRAME: [u8; 48_000] = [0xff; 48_000];

struct ObservedBusy<'a> {
    pin: Input<'static>,
    seen: &'a Cell<bool>,
}
impl embedded_hal::digital::ErrorType for ObservedBusy<'_> {
    type Error = core::convert::Infallible;
}
impl InputPin for ObservedBusy<'_> {
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        let low = self.pin.is_low();
        if low {
            self.seen.set(true);
        }
        Ok(low)
    }
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.is_low().map(|low| !low)
    }
}

// The driver's BUSY polling issues SPI commands. This wrapper bounds a stuck
// BUSY wait without changing the library under test.
struct BoundedSpi<S> {
    device: S,
    deadline: Instant,
}
impl<S> ErrorType for BoundedSpi<S> {
    type Error = ErrorKind;
}
impl<S: SpiDevice<u8>> SpiDevice<u8> for BoundedSpi<S> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        if Instant::now() >= self.deadline {
            return Err(ErrorKind::Other);
        }
        self.device
            .transaction(operations)
            .map_err(|_| ErrorKind::Other)
    }
}

fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("TEST ERROR: {}", info);
    halt()
}

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let mut delay = Delay::new();
    // Leave time for the USB monitor to attach after flashing.
    delay.delay_millis(3_000);
    println!("TEST START: TRMNL EE04 / ESP32-S3 / UC8179; one sequence only");
    let _panel_enable = Output::new(p.GPIO43, Level::High, OutputConfig::default());
    delay.delay_millis(100);
    let bus = Spi::new(
        p.SPI2,
        SpiConfig::default().with_frequency(Rate::from_mhz(4)),
    )
    .unwrap()
    .with_sck(p.GPIO7)
    .with_mosi(p.GPIO9)
    .with_miso(NoPin)
    .with_cs(NoPin);
    let cs = Output::new(p.GPIO44, Level::High, OutputConfig::default());
    let dc = Output::new(p.GPIO10, Level::Low, OutputConfig::default());
    let rst = Output::new(p.GPIO38, Level::High, OutputConfig::default());
    let seen = Cell::new(false);
    let busy = ObservedBusy {
        pin: Input::new(p.GPIO4, InputConfig::default()),
        seen: &seen,
    };
    let mut spi = BoundedSpi {
        device: ExclusiveDevice::new(bus, cs, Delay::new()).unwrap(),
        deadline: Instant::now() + Duration::from_secs(30),
    };
    let mut epd = Epd7in5::new(&mut spi, busy, dc, rst, &mut delay, Some(1_000)).unwrap();
    println!("INIT OK");
    // SAFETY: main executes once; no interrupt, other core, or other code uses FRAME.
    let frame = unsafe { &mut *core::ptr::addr_of_mut!(FRAME) };
    // Full: white screen, black border, solid black 160x80 upper-left box.
    for y in 0..480 {
        for byte_x in 0..100 {
            frame[y * 100 + byte_x] = if y < 8
                || y >= 472
                || byte_x == 0
                || byte_x == 99
                || ((40..120).contains(&y) && (5..25).contains(&byte_x))
            {
                0x00
            } else {
                0xff
            };
        }
    }

    macro_rules! refresh {
        ($label:expr) => {{
            seen.set(false);
            let start = Instant::now();
            epd.display_frame(&mut spi, &mut delay).unwrap();
            println!(
                "{}: {} ms, BUSY observed={}",
                $label,
                (Instant::now() - start).as_millis(),
                seen.get()
            );
            if !seen.get() {
                println!("TEST ERROR: refresh did not assert BUSY; stopping");
                halt();
            }
            delay.delay_millis(4_000);
        }};
    }

    epd.update_frame(&mut spi, frame, &mut delay).unwrap();
    refresh!("FULL");

    // Fast full: retain the border/box, add black vertical bars in upper right.
    spi.deadline = Instant::now() + Duration::from_secs(30);
    for y in 40..200 {
        for byte_x in 50..90 {
            frame[y * 100 + byte_x] = if byte_x % 4 < 2 { 0x00 } else { 0xff };
        }
    }
    epd.set_refresh_mode(&mut spi, &mut delay, RefreshMode::Fast)
        .unwrap();
    epd.update_frame(&mut spi, frame, &mut delay).unwrap();
    refresh!("FAST FULL");

    // Partial: add one black 160x80 rectangle with a white horizontal stripe.
    // All prior content should remain unchanged.
    spi.deadline = Instant::now() + Duration::from_secs(30);
    let mut region = [0x00; 160 * 80 / 8];
    region[35 * 20..45 * 20].fill(0xff);
    epd.set_refresh_mode(&mut spi, &mut delay, RefreshMode::Partial)
        .unwrap();
    epd.update_partial_frame(&mut spi, &mut delay, &region, 320, 320, 160, 80)
        .unwrap();
    refresh!("PARTIAL");

    spi.deadline = Instant::now() + Duration::from_secs(30);
    epd.sleep(&mut spi, &mut delay).unwrap();
    println!("TEST DONE: full, fast full, partial, panel sleep completed; visually inspect image");
    // Do not reset, loop the test, or enter MCU deep sleep. USB remains visible.
    halt()
}
