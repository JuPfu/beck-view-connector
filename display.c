#include "display.h"

#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_RS 20
#define PIN_DC 21

static screen_control_t sScreen;

void display_init(ili9341_config_t *hw_config)
{
    /*
    DISPLAY
    =======
    Pico pin  Disp.pin   Description
    (pin 36)  VCC        3.3V power input.
    (pin 38)  GND        Ground.
    (pin 17)  CS         LCD chip select signal, low level enable.
    (pin 20)  RESET      LCD reset signal, low level reset.
    (pin 21)  DC/RS      LCD register / data selection signal; high level: register.
    (pin 19)  SDI(MOSI)  SPI bus write data signal.
    (pin 18)  SCK        SPI bus clock signal.
    (pin 36)  LED        Backlight control.
    (pin 16)  SDO(MISO)  SPI bus read data signal. Hasn't been used so far in here.

    TOUCH PANEL
    ===========
    Pico pin  Dev.pin    Description
    (pin 20)  T_IRQ      Touch event indicator (active low).
    (pin 15)  T_DIN      SPI MOSI signal.
    (pin 14)  T_CLK      SPI SCK signal.
    (pin 17)  T_CS       Device chip select (active low).
    (pin 16)  T_DO       SPI MISO signal.
    */
    sScreen.mpHWConfig = hw_config;
    ILI9341_Init(sScreen.mpHWConfig, spi0, 50 * MHz, PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI, PIN_RS, PIN_DC);
    sScreen.mCanvasPaper = kBlack;
    sScreen.mCanvasInk = kWhite;
}

void display_start_info(void)
{
    TftClearScreenBuffer(&sScreen, kBlack, kWhite);
    TftPutTextLabel(&sScreen, "Beck-View-Connector", 0, 0, true);
    TftPutTextLabel(&sScreen, "Frame:", 5, 16, true);
    TftPutTextLabel(&sScreen, "Fps:", 5, 32, true);
    TftPutTextLabel(&sScreen, "Duration:", 5, 48, true);
    TftFullScreenWrite(&sScreen);
}

void display_frame_info(uint frame_counter, float fps, float duration)
{
    char frame_buf[20], fps_buf[20], duration_buf[20];

    // Update Display
    snprintf(frame_buf, sizeof(frame_buf), "%9u", frame_counter);
    TftPutTextLabel(&sScreen, frame_buf, 120, 16, true);

    if (frame_counter > 0)
    {
        snprintf(fps_buf, sizeof(fps_buf), "%9.4f", fps);
        TftPutTextLabel(&sScreen, fps_buf, 120, 32, true);
    }
    else
    {
        snprintf(fps_buf, sizeof(fps_buf), "   0.0000");
        TftPutTextLabel(&sScreen, fps_buf, 120, 32, true);

        TftPutTextLabel(&sScreen, "                        ", 5, 80, true);
        TftPutTextLabel(&sScreen, "                        ", 5, 96, true);
        TftPutTextLabel(&sScreen, "                        ", 5, 112, true);
    }

    snprintf(duration_buf, sizeof(duration_buf), "%9.6f", duration);
    TftPutTextLabel(&sScreen, duration_buf, 120, 48, true);

    TftFullScreenSelectiveWrite(&sScreen, 120);
}

void display_eof_info(uint frame_counter, float avg_fps, float duration)
{
    char frame_buf[20], fps_buf[20], duration_buf[20];

    TftPutTextLabel(&sScreen, "Total Frames:", 5, 80, true);
    TftPutTextLabel(&sScreen, "Avg. Fps:", 5, 96, true);
    TftPutTextLabel(&sScreen, "Duration:", 5, 112, true);

    snprintf(frame_buf, sizeof(frame_buf), "%9u", frame_counter);
    TftPutTextLabel(&sScreen, frame_buf, 120, 80, true);

    snprintf(fps_buf, sizeof(fps_buf), "%9.3f", avg_fps);
    TftPutTextLabel(&sScreen, fps_buf, 120, 96, true);

    snprintf(duration_buf, sizeof(duration_buf), "%9.3f", duration);
    TftPutTextLabel(&sScreen, duration_buf, 120, 112, true);

    TftFullScreenSelectiveWrite(&sScreen, 200);
}
