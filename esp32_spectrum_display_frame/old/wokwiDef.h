//fj feb 21st 2022
//the display used in TTGO Tdisp board does not exists in wokwi simulator, so i need to define new display and display dimensions
//i also need to define same color names, since ILI9341 colors are called ILI9341_WHITE, instead of TFT_WHITE
//note: i don't need to install those libs on my laptop, since they are just compiled in wokwi


#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"                               //Graphics and font library for ILI9341 driver chip

#define TFT_CS 15                                           //connections to display
#define TFT_DC 2
#define TFT_MOSI 23
#define TFT_CLK 18
#define TFT_RST 4
#define TFT_MISO 19

#define SCREEN_WIDTH    320                                 //tft display width 
#define SCREEN_HEIGHT   240                                 //tft display height 

#define TFT_WHITE   ILI9341_WHITE
#define TFT_BLACK   ILI9341_BLACK


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
