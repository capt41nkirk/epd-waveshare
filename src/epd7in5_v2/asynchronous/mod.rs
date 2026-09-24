//! Async UC8179 driver. SPI transfers, reset delays and BUSY polling yield.
//! Matches the synchronous driver waveform and retained-RAM semantics.
//! Callers should bound operations with an executor timeout. After cancellation
//! or an SPI error, discard the driver and reset with `new` before reusing it;
//! controller RAM and waveform state may no longer match the cached state.
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

use crate::color::Color;
use self::interface::DisplayInterface;
mod interface;
use crate::traits::RefreshLut;

use super::command::Command;
use super::{RefreshMode, WIDTH, HEIGHT, DEFAULT_BACKGROUND_COLOR};
use crate::buffer_len;

const IS_BUSY_LOW: bool = true;
const SINGLE_BYTE_WRITE: bool = false;

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

impl<SPI, BUSY, DC, RST, DELAY> Epd7in5<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    async fn init(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.initial_refresh = true;
        self.ram_initialized = false;
        // Reset the device
        self.interface.reset(delay, 10_000, 2_000).await;

        // UC8179 settings from GxEPD2_750_GDEY075T7 (Jean-Marc Zingg),
        // retaining the existing Waveshare reset and power-on timing.

        self.cmd_with_data(spi, Command::PowerSetting, &[0x07, 0x07, 0x3f, 0x3f, 0x09]).await?;
        self.cmd_with_data(spi, Command::BoosterSoftStart, &[0x17, 0x17, 0x28, 0x17]).await?;
        self.command(spi, Command::PowerOn).await?;
        delay.delay_ms(100).await;
        self.wait_until_idle(spi, delay).await?;
        self.cmd_with_data(spi, Command::PanelSetting, &[0x1F]).await?;
        self.cmd_with_data(spi, Command::TconResolution, &[0x03, 0x20, 0x01, 0xE0]).await?;
        self.cmd_with_data(spi, Command::DualSpi, &[0x00]).await?;
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x29, 0x07]).await?;
        self.cmd_with_data(spi, Command::TconSetting, &[0x22]).await?;
        self.cmd_with_data(spi, Command::PowerSaving, &[0x22]).await?;
        self.configure_refresh(spi, RefreshMode::Full).await?;
        Ok(())
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
    /// Initialize and reset the panel using asynchronous SPI and delays.
    pub async fn new(
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

        epd.init(spi, delay).await?;

        Ok(epd)
    }

    /// Reset and initialize the panel, invalidating its retained baseline.
    pub async fn wake_up(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.init(spi, delay).await
    }

    /// Enter deep sleep, invalidating the retained RAM baseline.
    pub async fn sleep(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay).await?;
        self.command(spi, Command::PowerOff).await?;
        delay.delay_ms(1).await;
        self.wait_until_idle(spi, delay).await?;
        self.initial_refresh = true;
        self.ram_initialized = false;
        self.cmd_with_data(spi, Command::DeepSleep, &[0xA5]).await?;
        Ok(())
    }

    /// Write a full framebuffer without starting a refresh.
    pub async fn update_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        assert_eq!(buffer.len(), buffer_len(WIDTH as usize, HEIGHT as usize));
        self.wait_until_idle(spi, delay).await?;
        self.initialize_ram(spi).await?;
        self.command(spi, Command::PartialOut).await?;
        self.cmd_with_data(spi, Command::DataStartTransmission2, buffer).await
    }

    /// Write a tightly packed region; call `display_frame` to refresh it.
    /// Select `RefreshMode::Partial` for a differential refresh.
    ///
    /// Panics if x/width are not multiples of eight, the nonempty rectangle is
    /// outside the screen, or the buffer length is not exactly width * height / 8.
    pub async fn update_partial_frame(
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
        self.wait_until_idle(spi, delay).await?;
        self.initialize_ram(spi).await?;
        self.command(spi, Command::PartialIn).await?;
        self.set_partial_window(spi, x, y, width, height).await?;
        self.cmd_with_data(spi, Command::DataStartTransmission2, buffer).await?;
        self.command(spi, Command::PartialOut).await
    }

    /// Refresh the panel and asynchronously wait for BUSY to clear.
    pub async fn display_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay).await?;
        self.initialize_ram(spi).await?;
        let mode = self.effective_refresh_mode();
        self.configure_refresh(spi, mode).await?;
        // Match GxEPD2's usePartialUpdateWindow=false: differential waveforms
        // scan the whole panel. Unchanged pixels are preserved by old/new RAM.
        // This also allows multiple partial writes before one refresh.
        self.command(spi, Command::PartialOut).await?;
        self.set_partial_window(spi, 0, 0, WIDTH, HEIGHT).await?;
        self.command(spi, Command::DisplayRefresh).await?;
        // Give BUSY time to assert before polling, as GxEPD2 does.
        delay.delay_ms(1).await;
        self.wait_until_idle(spi, delay).await?;
        self.initial_refresh = false;
        Ok(())
    }

    /// Write a full framebuffer and wait for its refresh to finish.
    pub async fn update_and_display_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.update_frame(spi, buffer, delay).await?;
        self.display_frame(spi, delay).await
    }

    /// Fill the panel with the background color and refresh it.
    pub async fn clear_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay).await?;
        self.initialize_ram(spi).await?;
        self.command(spi, Command::PartialOut).await?;
        self.command(spi, Command::DataStartTransmission2).await?;
        self.interface
            .data_x_times(spi, self.color.get_byte_value(), WIDTH / 8 * HEIGHT).await?;
        self.display_frame(spi, delay).await
    }

    /// Set the color used by clear_frame.
    pub fn set_background_color(&mut self, color: Color) {
        self.color = color;
    }

    /// Return the configured background color.
    pub fn background_color(&self) -> &Color {
        &self.color
    }

    /// Return the panel width in pixels.
    pub fn width(&self) -> u32 {
        WIDTH
    }

    /// Return the panel height in pixels.
    pub fn height(&self) -> u32 {
        HEIGHT
    }

    /// Select Full or differential Partial through the legacy LUT selector.
    pub async fn set_lut(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        refresh_rate: Option<RefreshLut>,
    ) -> Result<(), SPI::Error> {
        let mode = match refresh_rate.unwrap_or_default() {
            RefreshLut::Full => RefreshMode::Full,
            RefreshLut::Quick => RefreshMode::Partial,
        };
        self.set_refresh_mode(spi, delay, mode).await
    }

    /// Poll status with asynchronous delays until BUSY clears.
    pub async fn wait_until_idle(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.interface
            .wait_until_idle_with_cmd(spi, delay, IS_BUSY_LOW, Command::GetStatus).await
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
    async fn command(&mut self, spi: &mut SPI, command: Command) -> Result<(), SPI::Error> {
        self.interface.cmd(spi, command).await
    }

    /// Resume after `power_off_retaining_ram`, without resetting the controller.
    /// The caller must guarantee uninterrupted panel logic power, preserved RAM,
    /// and an unchanged RESET pin since a successful refresh. Otherwise use `new`.
    pub async fn resume_retained(
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
        epd.command(spi, Command::PowerOn).await?;
        delay.delay_ms(100).await;
        epd.wait_until_idle(spi, delay).await?;
        Ok(epd)
    }

    /// Disable panel driving voltages while preserving powered controller RAM.
    /// Unlike `sleep`, this does not issue DeepSleep or invalidate the baseline.
    pub async fn power_off_retaining_ram(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay).await?;
        self.command(spi, Command::PowerOff).await?;
        delay.delay_ms(100).await;
        self.wait_until_idle(spi, delay).await
    }

    async fn cmd_with_data(
        &mut self,
        spi: &mut SPI,
        command: Command,
        data: &[u8],
    ) -> Result<(), SPI::Error> {
        self.interface.cmd_with_data(spi, command, data).await
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
    pub async fn set_refresh_mode(
        &mut self,
        spi: &mut SPI,
        delay: &mut DELAY,
        mode: RefreshMode,
    ) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay).await?;
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

    async fn configure_refresh(&mut self, spi: &mut SPI, mode: RefreshMode) -> Result<(), SPI::Error> {
        self.cmd_with_data(spi, Command::PanelSetting, &[0x1f]).await?;
        // N2OCP copies the new image to previous RAM after each refresh.
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x29, 0x07]).await?;
        match mode {
            RefreshMode::Full => {
                self.cmd_with_data(spi, Command::CascadeSetting, &[0x00]).await?;
                self.cmd_with_data(spi, Command::TemperatureCalibration, &[0x00]).await
            }
            RefreshMode::Fast | RefreshMode::Partial => {
                self.cmd_with_data(spi, Command::CascadeSetting, &[0x02]).await?;
                let temperature = if mode == RefreshMode::Fast {
                    0x5a
                } else {
                    0x6e
                };
                self.cmd_with_data(spi, Command::ForceTemperature, &[temperature]).await
            }
        }
    }

    async fn initialize_ram(&mut self, spi: &mut SPI) -> Result<(), SPI::Error> {
        if !self.ram_initialized {
            self.command(spi, Command::PartialOut).await?;
            self.command(spi, Command::DataStartTransmission1).await?;
            self.interface.data_x_times(spi, 0x00, WIDTH / 8 * HEIGHT).await?;
            self.command(spi, Command::DataStartTransmission2).await?;
            self.interface
                .data_x_times(spi, self.color.get_byte_value(), WIDTH / 8 * HEIGHT).await?;
            self.ram_initialized = true;
        }
        Ok(())
    }

    async fn set_partial_window(
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
        ).await
    }
}

