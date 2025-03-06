//note: no need to install TFT_eSPI on wokwi, this is just compiled on laptop code

#include <TFT_eSPI.h>                                       //Graphics and font library for ST7735 driver chip search for tft_library, designer Bodmer
#include <SPI.h>

#define SCREEN_WIDTH    320                                 //tft display width 
#define SCREEN_HEIGHT   240                                 //tft display height 

TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH,SCREEN_HEIGHT);                                  //Invoke library, pins defined in User_Setup.h   
