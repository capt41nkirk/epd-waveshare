//! A simple Driver for the Waveshare 7.5" E-Ink Display (V2) via SPI
//!
//! # References
//!
//! - [Datasheet](https://www.waveshare.com/wiki/7.5inch_e-Paper_HAT)
//! - [Waveshare C driver](https://github.com/waveshare/e-Paper/blob/702def0/RaspberryPi%26JetsonNano/c/lib/e-Paper/EPD_7in5_V2.c)
//! - [Waveshare Python driver](https://github.com/waveshare/e-Paper/blob/702def0/RaspberryPi%26JetsonNano/python/lib/waveshare_epd/epd7in5_V2.py)
//!
//! Important note for V2:
//! Revision V2 has been released on 2019.11, the resolution is upgraded to 800×480, from 640×384 of V1.
//! The hardware and interface of V2 are compatible with V1, however, the related software should be updated.

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};

use crate::color::Color;
use crate::interface::DisplayInterface;
use crate::traits::{InternalWiAdditions, RefreshLut, WaveshareDisplay};

pub(crate) mod command;
use self::command::Command;
use crate::buffer_len;

/// Full size buffer for use with the 7in5 v2 EPD
#[cfg(feature = "graphics")]
pub type Display7in5 = crate::graphics::Display<
    WIDTH,
    HEIGHT,
    false,
    { buffer_len(WIDTH as usize, HEIGHT as usize) },
    Color,
>;

/// Width of the display
pub const WIDTH: u32 = 800;
/// Height of the display
pub const HEIGHT: u32 = 480;
/// Default Background Color
pub const DEFAULT_BACKGROUND_COLOR: Color = Color::White;
const IS_BUSY_LOW: bool = true;
const SINGLE_BYTE_WRITE: bool = false;

/// Refresh waveform for UC8179 / GDEY075T7 panels.
///
/// Fast modes require a panel with the corresponding OTP waveforms (as in
/// GxEPD2's GDEY075T7 driver). Older V2 panels may not support these modes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RefreshMode {
    /// Full cleaning refresh using the internal temperature sensor.
    #[default]
    Full,
    /// Fast full refresh using the OTP waveform selected at 90 degrees.
    Fast,
    /// Differential refresh using the OTP waveform selected at 110 degrees.
    Partial,
}

/// Epd7in5 (V2) driver
///
pub struct Epd7in5<SPI, BUSY, DC, RST, DELAY> {
    /// Connection Interface
    interface: DisplayInterface<SPI, BUSY, DC, RST, DELAY, SINGLE_BYTE_WRITE>,
    /// Background Color
    color: Color,
    refresh_mode: RefreshMode,
    initial_refresh: bool,
    ram_initialized: bool,
}

impl<SPI, BUSY, DC, RST, DELAY> InternalWiAdditions<SPI, BUSY, DC, RST, DELAY>
    for Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    fn init(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.initial_refresh = true;
        self.ram_initialized = false;
        // Reset the device
        self.interface.reset(delay, 10_000, 2_000);

        // UC8179 settings from GxEPD2_750_GDEY075T7 (Jean-Marc Zingg),
        // retaining the existing Waveshare reset and power-on timing.

        self.cmd_with_data(spi, Command::PowerSetting, &[0x07, 0x07, 0x3f, 0x3f, 0x09])?;
        self.cmd_with_data(spi, Command::BoosterSoftStart, &[0x17, 0x17, 0x28, 0x17])?;
        self.command(spi, Command::PowerOn)?;
        delay.delay_ms(100);
        self.wait_until_idle(spi, delay)?;
        self.cmd_with_data(spi, Command::PanelSetting, &[0x1F])?;
        self.cmd_with_data(spi, Command::TconResolution, &[0x03, 0x20, 0x01, 0xE0])?;
        self.cmd_with_data(spi, Command::DualSpi, &[0x00])?;
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x29, 0x07])?;
        self.cmd_with_data(spi, Command::TconSetting, &[0x22])?;
        self.cmd_with_data(spi, Command::PowerSaving, &[0x22])?;
        self.configure_refresh(spi, RefreshMode::Full)?;
        Ok(())
    }
}

impl<SPI, BUSY, DC, RST, DELAY> WaveshareDisplay<SPI, BUSY, DC, RST, DELAY>
    for Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    type DisplayColor = Color;
    fn new(
        spi: &mut SPI,
        busy: BUSY,
        dc: DC,
        rst: RST,
        delay: &mut DELAY,
        delay_us: Option<u32>,
    ) -> Result<Self, SPI::Error> {
        let interface = DisplayInterface::new(busy, dc, rst, delay_us);
        let color = DEFAULT_BACKGROUND_COLOR;

        let mut epd = Epd7in5 {
            interface,
            color,
            refresh_mode: RefreshMode::Full,
            initial_refresh: true,
            ram_initialized: false,
        };

        epd.init(spi, delay)?;

        Ok(epd)
    }

    fn wake_up(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.init(spi, delay)
    }

    fn sleep(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.command(spi, Command::PowerOff)?;
        delay.delay_ms(1);
        self.wait_until_idle(spi, delay)?;
        self.initial_refresh = true;
        self.ram_initialized = false;
        self.cmd_with_data(spi, Command::DeepSleep, &[0xA5])?;
        Ok(())
    }

    fn update_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        assert_eq!(buffer.len(), buffer_len(WIDTH as usize, HEIGHT as usize));
        self.wait_until_idle(spi, delay)?;
        self.initialize_ram(spi)?;
        self.command(spi, Command::PartialOut)?;
        self.cmd_with_data(spi, Command::DataStartTransmission2, buffer)
    }

    /// Write a tightly packed region; call `display_frame` to refresh it.
    /// Select `RefreshMode::Partial` for a differential refresh.
    ///
    /// Panics if x/width are not multiples of eight, the nonempty rectangle is
    /// outside the screen, or the buffer length is not exactly width * height / 8.
    fn update_partial_frame(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        buffer: &[u8],
        x: u32,
        y: u32,
        width: u32,
        height: u32,
    ) -> Result<(), SPI::Error> {
        assert!(width > 0 && height > 0);
        assert!(x < WIDTH && y < HEIGHT);
        assert!(width <= WIDTH - x && height <= HEIGHT - y);
        assert!(x % 8 == 0 && width % 8 == 0);
        assert_eq!(buffer.len(), (width / 8 * height) as usize);
        self.wait_until_idle(spi, delay)?;
        self.initialize_ram(spi)?;
        self.command(spi, Command::PartialIn)?;
        self.set_partial_window(spi, x, y, width, height)?;
        self.cmd_with_data(spi, Command::DataStartTransmission2, buffer)?;
        self.command(spi, Command::PartialOut)
    }

    fn display_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.initialize_ram(spi)?;
        let mode = self.effective_refresh_mode();
        self.configure_refresh(spi, mode)?;
        // Match GxEPD2's usePartialUpdateWindow=false: differential waveforms
        // scan the whole panel. Unchanged pixels are preserved by old/new RAM.
        // This also allows multiple partial writes before one refresh.
        self.command(spi, Command::PartialOut)?;
        self.set_partial_window(spi, 0, 0, WIDTH, HEIGHT)?;
        self.command(spi, Command::DisplayRefresh)?;
        // Give BUSY time to assert before polling, as GxEPD2 does.
        delay.delay_ms(1);
        self.wait_until_idle(spi, delay)?;
        self.initial_refresh = false;
        Ok(())
    }

    fn update_and_display_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.update_frame(spi, buffer, delay)?;
        self.display_frame(spi, delay)
    }

    fn clear_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.initialize_ram(spi)?;
        self.command(spi, Command::PartialOut)?;
        self.command(spi, Command::DataStartTransmission2)?;
        self.interface
            .data_x_times(spi, self.color.get_byte_value(), WIDTH / 8 * HEIGHT)?;
        self.display_frame(spi, delay)
    }

    fn set_background_color(&mut self, color: Color) {
        self.color = color;
    }

    fn background_color(&self) -> &Color {
        &self.color
    }

    fn width(&self) -> u32 {
        WIDTH
    }

    fn height(&self) -> u32 {
        HEIGHT
    }

    fn set_lut(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        refresh_rate: Option<RefreshLut>,
    ) -> Result<(), SPI::Error> {
        let mode = match refresh_rate.unwrap_or_default() {
            RefreshLut::Full => RefreshMode::Full,
            RefreshLut::Quick => RefreshMode::Partial,
        };
        self.set_refresh_mode(spi, delay, mode)
    }

    fn wait_until_idle(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.interface
            .wait_until_idle_with_cmd(spi, delay, IS_BUSY_LOW, Command::GetStatus)
    }
}

impl<SPI, BUSY, DC, RST, DELAY> Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    fn command(&mut self, spi: &mut SPI, command: Command) -> Result<(), SPI::Error> {
        self.interface.cmd(spi, command)
    }

    /// Resume after `power_off_retaining_ram`, without resetting the controller.
    /// The caller must guarantee uninterrupted panel logic power, preserved RAM,
    /// and an unchanged RESET pin since a successful refresh. Otherwise use `new`.
    pub fn resume_retained(
        spi: &mut SPI,
        busy: BUSY,
        dc: DC,
        rst: RST,
        delay: &mut DELAY,
        delay_us: Option<u32>,
    ) -> Result<Self, SPI::Error> {
        let mut epd = Self {
            interface: DisplayInterface::new(busy, dc, rst, delay_us),
            color: DEFAULT_BACKGROUND_COLOR,
            refresh_mode: RefreshMode::Full,
            initial_refresh: false,
            ram_initialized: true,
        };
        epd.command(spi, Command::PowerOn)?;
        delay.delay_ms(100);
        epd.wait_until_idle(spi, delay)?;
        Ok(epd)
    }

    /// Disable panel driving voltages while preserving powered controller RAM.
    /// Unlike `sleep`, this does not issue DeepSleep or invalidate the baseline.
    pub fn power_off_retaining_ram(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.command(spi, Command::PowerOff)?;
        delay.delay_ms(100);
        self.wait_until_idle(spi, delay)
    }

    fn cmd_with_data(
        &mut self,
        spi: &mut SPI,
        command: Command,
        data: &[u8],
    ) -> Result<(), SPI::Error> {
        self.interface.cmd_with_data(spi, command, data)
    }

    /// Select the waveform used by subsequent `display_frame` calls.
    ///
    /// Fast is a full-screen waveform and is valid for the first full-frame
    /// update too (GxEPD2's useFastFullUpdate). Only differential Partial needs
    /// an established baseline and falls back to Full after construction/reset.
    /// Partial mode uses controller old/new RAM copying (N2OCP), so no
    /// host-side old framebuffer or second write is needed. Keep panel power
    /// and RAM intact between differential updates; periodically select Full
    /// to remove ghosting. Fast modes follow GxEPD2's GDEY075T7 OTP sequences.
    pub fn set_refresh_mode(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        mode: RefreshMode,
    ) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.refresh_mode = mode;
        Ok(())
    }

    /// Waveform the next `display_frame` will actually execute. Query before
    /// refreshing, since a successful refresh establishes the partial baseline.
    pub fn effective_refresh_mode(&self) -> RefreshMode {
        match (self.initial_refresh, self.refresh_mode) {
            (true, RefreshMode::Partial) => RefreshMode::Full,
            (_, mode) => mode,
        }
    }

    fn configure_refresh(&mut self, spi: &mut SPI, mode: RefreshMode) -> Result<(), SPI::Error> {
        self.cmd_with_data(spi, Command::PanelSetting, &[0x1f])?;
        // N2OCP copies the new image to previous RAM after each refresh.
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x29, 0x07])?;
        match mode {
            RefreshMode::Full => {
                self.cmd_with_data(spi, Command::CascadeSetting, &[0x00])?;
                self.cmd_with_data(spi, Command::TemperatureCalibration, &[0x00])
            }
            RefreshMode::Fast | RefreshMode::Partial => {
                self.cmd_with_data(spi, Command::CascadeSetting, &[0x02])?;
                let temperature = if mode == RefreshMode::Fast {
                    0x5a
                } else {
                    0x6e
                };
                self.cmd_with_data(spi, Command::ForceTemperature, &[temperature])
            }
        }
    }

    fn initialize_ram(&mut self, spi: &mut SPI) -> Result<(), SPI::Error> {
        if !self.ram_initialized {
            self.command(spi, Command::PartialOut)?;
            self.command(spi, Command::DataStartTransmission1)?;
            self.interface.data_x_times(spi, 0x00, WIDTH / 8 * HEIGHT)?;
            self.command(spi, Command::DataStartTransmission2)?;
            self.interface
                .data_x_times(spi, self.color.get_byte_value(), WIDTH / 8 * HEIGHT)?;
            self.ram_initialized = true;
        }
        Ok(())
    }

    fn set_partial_window(
        &mut self,
        spi: &mut SPI,
        x: u32,
        y: u32,
        width: u32,
        height: u32,
    ) -> Result<(), SPI::Error> {
        let xe = x + width - 1;
        let ye = y + height - 1;
        self.cmd_with_data(
            spi,
            Command::PartialWindow,
            &[
                (x >> 8) as u8,
                x as u8,
                (xe >> 8) as u8,
                xe as u8,
                (y >> 8) as u8,
                y as u8,
                (ye >> 8) as u8,
                ye as u8,
                0x01,
            ],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retained_resume_and_standby_never_reset_or_clear_ram() {
        use embedded_hal_mock::eh1::{delay::NoopDelay, digital, spi};
        let mut spi = spi::Mock::new(&[
            spi::Transaction::transaction_start(),
            spi::Transaction::write(0x04),
            spi::Transaction::transaction_end(),
            spi::Transaction::transaction_start(),
            spi::Transaction::write(0x71),
            spi::Transaction::transaction_end(),
            spi::Transaction::transaction_start(),
            spi::Transaction::write(0x71),
            spi::Transaction::transaction_end(),
            spi::Transaction::transaction_start(),
            spi::Transaction::write(0x02),
            spi::Transaction::transaction_end(),
            spi::Transaction::transaction_start(),
            spi::Transaction::write(0x71),
            spi::Transaction::transaction_end(),
        ]);
        let mut busy = digital::Mock::new(&[
            digital::Transaction::get(digital::State::High),
            digital::Transaction::get(digital::State::High),
            digital::Transaction::get(digital::State::High),
        ]);
        let mut dc = digital::Mock::new(&[
            digital::Transaction::set(digital::State::Low),
            digital::Transaction::set(digital::State::Low),
            digital::Transaction::set(digital::State::Low),
            digital::Transaction::set(digital::State::Low),
            digital::Transaction::set(digital::State::Low),
        ]);
        let mut rst = digital::Mock::new(&[]);
        let mut delay = NoopDelay::new();
        let mut epd = Epd7in5::resume_retained(
            &mut spi,
            busy.clone(),
            dc.clone(),
            rst.clone(),
            &mut delay,
            Some(0),
        )
        .unwrap();
        assert!(!epd.initial_refresh);
        assert!(epd.ram_initialized);
        epd.power_off_retaining_ram(&mut spi, &mut delay).unwrap();
        assert!(!epd.initial_refresh);
        assert!(epd.ram_initialized);
        spi.done();
        busy.done();
        dc.done();
        rst.done();
    }

    #[test]
    fn epd_size() {
        assert_eq!(WIDTH, 800);
        assert_eq!(HEIGHT, 480);
        assert_eq!(DEFAULT_BACKGROUND_COLOR, Color::White);
    }
}
