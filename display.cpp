#include "display.h"

#include "../include/st7789/ST7789_TFT.hpp"

#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_RS 20
#define PIN_DC 21

// Section :: Globals
ST7789_TFT myTFT;

void display_init()
{
    //*************** USER OPTION 0 SPI_SPEED + TYPE ***********

    // hw spi
    uint32_t TFT_SCLK_FREQ = 24000; // Spi freq in KiloHertz , 1000 = 1Mhz
    myTFT.TFTInitSPIType(TFT_SCLK_FREQ, spi0);

    //**********************************************************

    // ******** USER OPTION 1 GPIO *********
    // NOTE if using Hardware SPI clock and data pins will be tied to
    // the chosen interface eg Spi0 CLK=18 DIN=19)
    int8_t SDIN_TFT = PIN_MOSI;
    int8_t SCLK_TFT = PIN_SCK;
    int8_t DC_TFT = PIN_DC;
    int8_t CS_TFT = PIN_CS;
    int8_t RST_TFT = PIN_RS;
    myTFT.TFTSetupGPIO(RST_TFT, DC_TFT, CS_TFT, SCLK_TFT, SDIN_TFT);
    //**********************************************************

    // ****** USER OPTION 2 Screen Setup ******
    uint16_t OFFSET_COL = 0;   // 2, These offsets can be adjusted for any issues->
    uint16_t OFFSET_ROW = 0;   // 3, with screen manufacture tolerance/defects
    uint16_t TFT_WIDTH = 135;  // Screen width in pixels
    uint16_t TFT_HEIGHT = 240; // Screen height in pixels
    myTFT.TFTInitScreenSize(OFFSET_COL, OFFSET_ROW, TFT_WIDTH, TFT_HEIGHT);
    // ******************************************

    myTFT.TFTST7789Initialize();

    // Rotate
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

    myTFT.TFTFontNum(myTFT.TFTFont_ArialRound);

    myTFT.TFTsetCursor(0, 46);
    myTFT.print("frame:");
    myTFT.TFTsetCursor(0, 76);
    myTFT.print("fps:");
    myTFT.TFTsetCursor(0, 104);
    myTFT.print("dur:");
}

void display_frame_info(uint frame_counter, float fps, float duration)
{
    char frame_buf[20], fps_buf[20], duration_buf[20];

    // Update Display
    snprintf(frame_buf, sizeof(frame_buf), "%8u", frame_counter);
    myTFT.TFTsetCursor(90, 46);
    myTFT.print(frame_buf);

    snprintf(fps_buf, sizeof(fps_buf), "%9.4f", frame_counter > 0 ? fps : 0.0f);
    myTFT.TFTsetCursor(72, 76);
    myTFT.print(fps_buf);

    snprintf(duration_buf, sizeof(duration_buf), "%9.4f", duration);
    myTFT.TFTsetCursor(72, 104);
    myTFT.print(duration_buf);
}

void display_eof_info(uint frame_counter, float avg_fps, float duration)
{
    char frame_buf[20], fps_buf[20], duration_buf[20];

    myTFT.TFTFontNum(myTFT.TFTFont_Mia);
    myTFT.TFTdrawText(0, 12, "Beck-View-Connector", ST7789_YELLOW, ST7789_RED);
    myTFT.TFTdrawText(192, 12, "eof", ST7789_RED, ST7789_NAVY);

    myTFT.TFTsetCursor(5, 20);
    myTFT.TFTFontNum(myTFT.TFTFont_ArialRound);

    myTFT.TFTsetCursor(0, 46);
    myTFT.print("frame:");
    myTFT.TFTsetCursor(0, 76);
    myTFT.print("fps:");
    myTFT.TFTsetCursor(0, 104);
    myTFT.print("dur:");

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
