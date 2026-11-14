#include "display.h"

#include "../include/st7789/ST7789_TFT.hpp"

namespace
{
    constexpr uint8_t PIN_CS = 17;
    constexpr uint8_t PIN_SCK = 18;
    constexpr uint8_t PIN_MOSI = 19;
    constexpr uint8_t PIN_RST = 20;
    constexpr uint8_t PIN_DC = 21;

    constexpr uint32_t TFT_SPI_FREQ_KHZ = 24000;

    constexpr uint16_t TFT_WIDTH = 135;  // Screen width in pixels
    constexpr uint16_t TFT_HEIGHT = 240; // Screen height in pixels

    constexpr uint16_t OFFSET_COL = 0;
    constexpr uint16_t OFFSET_ROW = 0;

    ST7789_TFT myTFT;

    void print_value(uint16_t x, uint16_t y, const char *format, float value)
    {
        char buffer[20];

        snprintf(buffer, sizeof(buffer), format, value);

        myTFT.TFTsetCursor(x, y);
        myTFT.print(buffer);
    };

    void draw_labels()
    {
        myTFT.TFTFontNum(myTFT.TFTFont_ArialRound);

        myTFT.TFTsetCursor(0, 46);
        myTFT.print("frame:");

        myTFT.TFTsetCursor(0, 76);
        myTFT.print("fps:");

        myTFT.TFTsetCursor(0, 104);
        myTFT.print("dur:");
    };
}

void display_init()
{
    //*************** SPI_SPEED + TYPE ***********
    myTFT.TFTInitSPIType(TFT_SPI_FREQ_KHZ, spi0);

    // ******** USER OPTION 1 GPIO *********
    // NOTE if using Hardware SPI clock and data pins will be tied to the chosen interface eg Spi0 CLK=18 DIN=19)

    myTFT.TFTSetupGPIO(PIN_RST, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);

    // ****** Screen Setup ******
    myTFT.TFTInitScreenSize(OFFSET_COL, OFFSET_ROW, TFT_WIDTH, TFT_HEIGHT);

    myTFT.TFTST7789Initialize();

    // Rotate display 270 degrees
    myTFT.TFTsetRotation(ST7789_TFT::TFT_rotate_e::TFT_Degrees_270);
}

void DisplayReset(void)
{
    myTFT.TFTfillScreen(ST7789_BLACK);
}

void display_start_info(void)
{
    DisplayReset();

    myTFT.TFTFontNum(myTFT.TFTFont_Mia);
    myTFT.TFTdrawText(0, 12, "Beck-View-Connector", ST7789_YELLOW, ST7789_RED);
    myTFT.TFTdrawText(192, 12, "run", ST7789_WHITE, ST7789_BLUE);

    draw_labels();
}

void display_frame_info(uint32_t frame_counter, float fps, float duration)
{
    char frame_buf[20];

    snprintf(frame_buf, sizeof(frame_buf), "%8lu", static_cast<unsigned long>(frame_counter));

    myTFT.TFTsetCursor(90, 46);
    myTFT.print(frame_buf);

    print_value(72, 76, "%9.4f", frame_counter > 0 ? fps : 0.0f);
    print_value(72, 104, "%9.4f", duration);
}

void display_eof_info(uint32_t frame_counter, float avg_fps, float duration)
{
    char frame_buf[20], fps_buf[20], duration_buf[20];

    myTFT.TFTFontNum(myTFT.TFTFont_Mia);
    myTFT.TFTdrawText(0, 12, "Beck-View-Connector", ST7789_YELLOW, ST7789_RED);
    myTFT.TFTdrawText(192, 12, "eof", ST7789_RED, ST7789_NAVY);

    myTFT.TFTsetCursor(5, 20);
    // myTFT.TFTFontNum(myTFT.TFTFont_ArialRound);

    draw_labels();
    // myTFT.TFTsetCursor(0, 46);
    // myTFT.print("frame:");
    // myTFT.TFTsetCursor(0, 76);
    // myTFT.print("fps:");
    // myTFT.TFTsetCursor(0, 104);
    // myTFT.print("dur:");

    myTFT.setTextColor(ST7789_RED, ST7789_BLACK);
    snprintf(frame_buf, sizeof(frame_buf), "%8u", frame_counter);
    myTFT.TFTsetCursor(90, 46);
    myTFT.print(frame_buf);

    snprintf(fps_buf, sizeof(fps_buf), "%9.3f", avg_fps);
    myTFT.TFTsetCursor(72, 76);
    myTFT.print(fps_buf);

    snprintf(duration_buf, sizeof(duration_buf), "%9.3f", duration);
    myTFT.TFTsetCursor(72, 104);
    myTFT.print(duration_buf);

    myTFT.setTextColor(ST7789_WHITE, ST7789_BLACK);
}
