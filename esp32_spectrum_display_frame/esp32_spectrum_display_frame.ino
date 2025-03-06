
/**@file*/

/* ESP8266/32 Audio Spectrum Analyser on an SSD1306/SH1106 Display
 * The MIT License (MIT) Copyright (c) 2017 by David Bird. 
 * The formulation and display of an AUdio Spectrum using an ESp8266 or ESP32 and SSD1306 or SH1106 OLED Display using a Fast Fourier Transform
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to 
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:  
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * See more at http://dsbird.org.uk 
*/

/*
TTGO Tdisp esp32 display support:
install of TFT_eSPI lib
go to C:\Utilisateurs\Windows\Documents\Arduino\libraries
Into User_Setup_Select.h
…
comment out //#include <User_Setup.h>           // Default setup is root library folder
uncomment #include <User_Setups/Setup25_TTGO_T_Display.h>
*/

/*
  ILI9341 support
  install of TFT_eSPI lib
  go to C:\Utilisateurs\Windows\Documents\Arduino\libraries
  Into User_Setup_Select.h
  #include <User_Setup.h> 
  and comment out any other include

  Into User_setup.h
  #define ILI9341_DRIVER 
  and comment out any other #define
  then down in same file, remove comment out on next lines decsribing connctions
  
  // ###### EDIT THE PIN NUMBERS IN THE LINES FOLLOWING TO SUIT YOUR ESP32 SETUP   ######

  // For ESP32 Dev board (only tested with ILI9341 display)
  // The hardware SPI can be mapped to any pins

  #define TFT_MISO 19
  #define TFT_MOSI 23
  #define TFT_SCLK 18
  #define TFT_CS   15  // Chip select control pin
  #define TFT_DC    2  // Data Command control pin
  #define TFT_RST   4  // Reset pin (could connect to RST pin)
  //#define TFT_RST  -1  // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST

  those defines give code connection of esp32 to display
*/
  
  


#include "arduinoFFT.h" // install https://github.com/kosme/arduinoFFT, in IDE, Sketch, Include Library, Manage Library, then search for FFT
//arduinoFFT FFT = arduinoFFT();

#include "ioDef.h"
#include "hardwareDef.h"

//#define WOKWI
#define HARDWARE

/////////////////////////////////////////////////////////////////////////
#define SAMPLES 512                           // Must be a power of 2, 1024 gives run time error on esp32 with double vReal[SAMPLES] memory allocation
#define AMPLITUDE_MAX 255
#define SAMPLING_FREQ 11520                   //serial: 11.52KHz sampling rate (10 cycles of 115.2KHz clock)
                                              //maximum frequency that can be analysed by the FFT Fmax=sampleF/2

#define SCREEN_WIDTH    320                                 //tft display width 
#define SCREEN_HEIGHT   240                                 //tft display height 

unsigned int sampling_period_us;  
unsigned long microseconds;
double vReal[SAMPLES];                          //double=float uses 4 bytes
double vImag[SAMPLES];
double adaptedNoise[SAMPLES];

//arduinoFFT FFT = arduinoFFT(); before 2.0.3 of arduinoFFT lib
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

//does not work for the FFT lib
//double* vReal = (double*)malloc(SAMPLES * sizeof(double));
//double* vImag = (double*)malloc(SAMPLES * sizeof(double));


//could be local to loop, but easier here to pass to printDuration
int startTimeLoop;
int durationSerialIn=0;
int durationInitBuffer=0;
int durationPlotTimeBuffer=0;
int durationTftTimeBuffer=0;
int durationPlotFreqBuffer=0;
int durationTftFreqBuffer=0;
int durationFftWindowing=0;
int durationFftCompute=0;
int durationFftComplexeToMagnitude=0;
int durationLoop=0;

unsigned long newTime, oldTime;
                                              

bool dbg=false;                                             

bool forceTimeBufferFlag=true;                //if forceBuffer mode, I substitute input for a computed waveform
bool tftTimeBufferFlag=true;                  //dispay input buffer on tft (before processing, so time domain)
bool plotTimeBufferFlag=false;                //plot input buffer on arduino IDE
bool tftFreqBufferFlag=true;                  //display output on tft, after processing, so frequency domain
bool plotFreqBufferFlag=true;                //plot output buffer on arduino IDE (frequency domain)
bool singleRunFlag=true;                      //stop after single process(true), or loop back for next buffer(false)
                                              //warning: time domain representation does not have a trigger function like an oscilloscope, 
                                              //so use singleRunFlag=true if willing to show time domain on tft
                                              //note:for the moment,this loop cyles trough 50/500/5000 hz fundamental + 3/5/7 harmonics
bool printDurationFlag=false;                  //prints to console execution time (slow: has an impact to loop execution)
                                              
                                             

/////////////////////////////////////////////////////////////////////////
void setup() {
  
  Serial.begin(9600);                                                               //i need slow speed to have plotter to work
  delay(100);                                                                       //for some reason, i need a delay after Serial.begin to see Serial.print to work 
  Serial.print("Serial dbg ready");Serial.println(" ");
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);                                    //serial input port
  
  tft.init(); 

  #if defined(WOKWI)
    //no cls, takes too long
  #elif defined HARDWARE 
    tft.fillScreen(TFT_BLACK);                                                      //cls only in hardware, wokwi too slow
  #endif
   

  tft.setRotation(0);                                                               //this is to get usb on down side
  tft.setTextSize(1); 
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

                                                                               //buttons used by students for user interface
  pinMode(BUTTON_PIN_RM,INPUT_PULLUP);    //rightmost
  pinMode(BUTTON_PIN_CR,INPUT_PULLUP);    //center right
  pinMode(BUTTON_PIN_CL,INPUT_PULLUP);    //center left
  pinMode(BUTTON_PIN_LM,INPUT_PULLUP);    //leftmost
  
  pinMode(LEDPIN, OUTPUT);
  
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));                 //analog: 25us  digital:86.6us not used anymore, left in case i add analog input

  drawSine();
  dottedLine();
  delay(2000);
  tft.fillScreen(TFT_BLACK); 

}

void loop() { 
  
  
  
  static int freqMult=3;
  startTimeLoop=0;
 
  //tft.drawString(".06 .07 .14 .25 .4  .8  1.6 4.5KHz",0,120,2);              //serial input bands 
   
  if (!forceTimeBufferFlag) {
    durationSerialIn=serialIn();                                           //serial input to vReal[i] from 0 to SAMPLES. vImag[i]=0
  }
  
  if (forceTimeBufferFlag) {
    float freq=50*freqMult;                                                 //use a float to get correct division in initBuffer
    durationInitBuffer=initBuffer(freq);                                                   
  };
  //if (!singleRunFlag) {tft.fillScreen(TFT_BLACK);}; 

  if (plotTimeBufferFlag) {
    durationPlotTimeBuffer=plotTimeBuffer();                                                        //plot input (time domain) to IDE plotter 
  }
  
  if (tftTimeBufferFlag) {
    durationTftTimeBuffer=tftTimeBuffer(0,0,SCREEN_WIDTH,SCREEN_HEIGHT/2);                        //display input (time domain) to tft display upper half starting from (0,0) top left corner
  }

//students:here insert 3 FFT instructions, and compute compute duration for each

  int startFftWindowing=millis();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  durationFftWindowing=millis()-startFftWindowing;

  int startFftCompute=millis();
  FFT.compute(FFTDirection::Forward);
  durationFftCompute=millis()-startFftCompute;

  int startFftComplexeToMagnitude=millis();

  FFT.complexToMagnitude();
  durationFftComplexeToMagnitude=millis()-startFftComplexeToMagnitude;


  
  if (plotFreqBufferFlag) {
    durationPlotFreqBuffer=plotFreqBuffer();                                                        //plot output (frequency domain) to IDE plotter
  }

 

  if (tftFreqBufferFlag) {
    durationTftFreqBuffer=tftFreqBuffer(0,SCREEN_HEIGHT/2,SCREEN_WIDTH,SCREEN_HEIGHT/2);                                                         //display output (frequency domain) to tft display
  } 

  while(true) {};                                                  //run/stop

//here compute loop duration
  durationLoop=millis()-startTimeLoop;

  if (printDurationFlag) {
    printDuration();  
  } 
  
  
  delay(2000);
  if (freqMult<100){freqMult=freqMult*10;} else {freqMult=1;};    //this is used when forcing time buffer
  tft.fillScreen(TFT_BLACK);

}

double maximum(double vReal[]){
  //On reitre les 2 prmière valeur du tableau car se sont les valeurs des composante continue du signal
  double max=vReal[2];
  for (int i=2; i< SAMPLES/2; i++){
    if( max< vReal[i]) { max=vReal[i];}
  }
  return max;
}

double minimum(double vReal[]){
  //On reitre les 2 prmière valeur du tableau car se sont les valeurs des composante continue du signal
  double min=vReal[2];
  for (int i=2; i< SAMPLES/2; i++){
    if( min> vReal[i]) { min=vReal[i];}
  }
  return min;
}


//students:enter serial data into vReal[i] and vImag[i]. Remember port used is Serial1
int serialIn() {
  int startTime;
  
  for (int i=0; i < SAMPLES; i++){
    while (Serial1.available() ==0){};
    vReal[i]=Serial1.read();
    vImag[i]=0;
  }
  return (millis()-startTime);
}

//students:print time buffer to arduino IDE plotter
int plotTimeBuffer() { 
  int startTime=millis();
  
  for (int i=0; i<SAMPLES; i++){
    Serial.println(vReal[i]);
  }

  return (millis()-startTime);
}

//students:init time buffer (vReal[i] and vImag[i]) with a signal of frequency freq. 
// Start with a sine wave that fills buffer with a single period (data 0 to 255)
// then fill buffer with a sine wave of frequency freq (parm given to routine)
// knowing that each sample (element of vReal[i]) represents 86.6us of time
// then add harmonics 3,5,7
// then add some digital noise
//caution: this has to be 0 to AMPLITUDE_MAX, so search max of fund+harmonics+noise, then map to 0,AMPLITUDE_MAX
int initBuffer(float freq) {
  int startTime=millis();
  int noiseMax=2;
  
  float y=0; // y point courant et y1 valeur suivante (où on veut aller)
  double maxi=maximum(vReal);
  double min=minimum(vReal);
  for (int i=0; i<SAMPLES; i++){
    float f1=sin((freq/22.7)*(TWO_PI/(SAMPLES-1))*i);
    float f3=sin(3*(freq/22.7)*(TWO_PI/(SAMPLES-1))*i);
    float f5=sin(5*(freq/22.7)*(TWO_PI/(SAMPLES-1))*i);
    float f7=sin(7*(freq/22.7)*(TWO_PI/(SAMPLES-1))*i);

    long noiseLevel=random(1,noiseMax);
    vReal[i]=(((f1) + (f3/3.0) + (f5/5.0) + (f7/7.0)+1)/2)*AMPLITUDE_MAX + (float) noiseLevel;
    vImag[i]=0;

    //vReal[i]=map(vReal[i], min, maxi, 0, AMPLITUDE_MAX);
  }
  return (millis()-startTime);
}


//students:print freq buffer to arduino IDE plotter
int plotFreqBuffer(void) {
  int startTime=millis();
  // Notre fréquence de coupure est de 5.5kHz
  float pasFreq= 5500.0/255.0;
  
  for (int i=0; i<SAMPLES/2; i++){
    Serial.print(i*pasFreq);Serial.print(" Hz : ");Serial.println(vReal[i]);
  }
  
  return (millis()-startTime);
}

//students:print time buffer to tft screen. top point is (top_left_x,top_left_y). width is time_width. height of time is time_height
// Graduation : Elles sont de chaque côté sur 20 pixels pour la graduation de l'axe x et y 
int tftTimeBuffer(int topLeftX, int topLeftY, int timeWidth,int timeHeight) {
  int startTime=millis();

  for (int i=0; i< SAMPLES; i++){
    float x=map(i,0,SAMPLES-1,topLeftX+20,topLeftX+timeWidth);
    float x1=map(i+1,0,SAMPLES-1,topLeftX+20,topLeftX+timeWidth);
    float y=map(vReal[i],0,AMPLITUDE_MAX,topLeftY+timeHeight-20,topLeftY);
    float y1=map(vReal[i+1],0,AMPLITUDE_MAX,topLeftY+timeHeight-20,topLeftY);
    //Serial.print("Affichage y :");Serial.println(y);
    //Serial.print("Affichage y1 : ");Serial.println(y1);
    tft.drawLine(x, y, x1, y1, TFT_WHITE);
  }
  // On trace, respectivement, l'axe x et y, en laissant un espace avec notre graphique (5 pixels)
  tft.drawLine(0,(SCREEN_HEIGHT/2)-15,SCREEN_WIDTH,(SCREEN_HEIGHT/2)-15,TFT_WHITE);
  tft.drawLine(15,0,15,SCREEN_HEIGHT/2,TFT_WHITE);

  // On trace les graduations 

  // On débute avec l'axe x
  // Avec 44 032us affiché sur 320 pixels, on choisit de graduer tous les 4000us.
  int facteur = 4000*320/44032; // Tous les facteur, on affiche une graduation
  int i=0;
  while(i*facteur<SCREEN_WIDTH){
    Serial.println(i);
    tft.drawLine(i*facteur+20,(SCREEN_HEIGHT/2)-10,i*facteur+20,(SCREEN_HEIGHT/2)-20,TFT_WHITE); // trace la graduation
    i++;
  }

  // On fait l'axe y
  // On va de 0 à 255
  // Avec 255 comme max de la fonction, on décide d'afficher tous les 50 en sachant que la hauteur de notre signal est de (SCREEN_HEIGHT/2)-20
  facteur = 50*((SCREEN_HEIGHT/2)-20)/255;
  int j=0;
  while(j*facteur<(SCREEN_HEIGHT/2)-20){
    Serial.println(j);
    tft.drawLine(10,(SCREEN_HEIGHT/2)-20-j*facteur,20,(SCREEN_HEIGHT/2)-20-j*facteur,TFT_WHITE); // trace la graduation
    j++;
    /*
    //Affichage de l'axe des abscisses
    // Buffer pour le résultat string
    char str[20];

    // Convertir un int en string grâce à sprintf
    sprintf(str, "%d", j*facteur);


    tft.setTextSize(1); 
    tft.drawString(str, j*facteur+20, SCREEN_HEIGHT-10);
    */
  }

  //Affichage de l'axe des abscisses
  tft.setTextSize(1); 
  tft.drawString(" Temps en us", (SCREEN_WIDTH/2) -30, SCREEN_HEIGHT/2);
  return (millis()-startTime);
}


//students:print freq buffer to tft screen. top point is (topLeftX,topLeftY). width is freqWidth. height of time is freqHeight
//         search for max freq value and use this as maximum of displayed values
// Graduation : Elles sont de chaque côté sur 20 pixels pour la graduation de l'axe x et y
int tftFreqBuffer(int topLeftX, int topLeftY, int freqWidth,int freqHeight) {
  int startTime=millis();
  
  double max= maximum(vReal);
  double min=minimum(vReal);
  for (int i=2; i< SAMPLES/2; i++){
    float x=map(i,0,SAMPLES/2,topLeftX+20,freqWidth + topLeftX);
    float x1=map(i+1,0,SAMPLES/2,topLeftX+20,freqWidth + topLeftX);
    float y=map(vReal[i],0,max,topLeftY + freqHeight - 1 -20, topLeftY); // vReal[0] car c'est la compososante continue, f=0 Hz, qui émet le plus de magnitidude
    float y1=map(vReal[i+1],0,max,topLeftY + freqHeight - 1 -20, topLeftY);
    tft.drawLine(x, y, x1, y1, TFT_WHITE);
  }
  // On trace, respectivement, l'axe x et y, en laissant un espace avec notre graphique (5 pixels)
  tft.drawLine(0,(SCREEN_HEIGHT-1)-15,SCREEN_WIDTH,(SCREEN_HEIGHT-1)-15,TFT_WHITE);
  tft.drawLine(15,SCREEN_HEIGHT/2,15,SCREEN_HEIGHT,TFT_WHITE);

  // On trace les graduations 

  // On débute avec l'axe x
  // Avec 5,5kHz affiché sur 320 pixels, on choisit de graduer tous les 1kHz.
  int facteur = 1*320/5.5; // Tous les facteur, on affiche une graduation
  int i=0;
  while(i*facteur<SCREEN_WIDTH){
    Serial.println(i);
    tft.drawLine(i*facteur+20,(SCREEN_HEIGHT)-10,i*facteur+20,(SCREEN_HEIGHT)-20,TFT_WHITE); // trace la graduation
    /*
    //Affichage de l'axe des abscisses
    tft.setTextSize(1); 
    //Affichage de l'axe des abscisses
    // Buffer pour le résultat string
    char str[20];

    // Convertir un int en string grâce à sprintf
    sprintf(str, "%d", i*facteur);


    tft.setTextSize(1); 
    tft.drawString(str, i*facteur, (SCREEN_HEIGHT)-5);
    */
    i++;
  }

  // On fait l'axe y
  // On va de 0 à 255
  // Avec max comme maximum de la fonction, on décide d'afficher tous les 3000 en sachant que la hauteur de notre signal est de (SCREEN_HEIGHT/2)-20
  facteur = 3000*((SCREEN_HEIGHT/2)-20)/max;
  int j=0;
  while(j*facteur<(SCREEN_HEIGHT/2)-20){
    Serial.println(j);
    tft.drawLine(10,(SCREEN_HEIGHT)-20-j*facteur,20,(SCREEN_HEIGHT)-20-j*facteur,TFT_WHITE); // trace la graduation
    /*
    //Affichage de l'axe des abscisses
    // Buffer pour le résultat string
    char str[20];

    // Convertir un int en string grâce à sprintf
    sprintf(str, "%d", j*facteur);


    tft.setTextSize(1); 
    tft.drawString(str, 5, (SCREEN_HEIGHT)-20-j*facteur);
    */
    j++;
  }

  //Affichage de l'axe des abscisses
  tft.setTextSize(1); 
  tft.drawString(" Fréquence en kHz", (SCREEN_WIDTH/2) -30, SCREEN_HEIGHT-5);

  return (millis()-startTime);
}

//students:print a horizontal dotted line middle of screen
int dottedLine() {
  int startTime=millis();
  
  // Ecrire une ligne de points
  for (int x = 0 ; x < SAMPLES ; x += 20) {  
    tft.drawLine(x,AMPLITUDE_MAX/2,x+10,AMPLITUDE_MAX/2,TFT_WHITE);                        //draw a line X0,Y0,X1,Y1,color                                                                        
  }

  return (millis()-startTime);
}

//student: draw a sine wave full screen
int drawSine() {
  int startTime=millis();
 
  float y=0,y1=0; // y point courant et y1 valeur suivante (où on veut aller)
  for (int i=0; i<SAMPLES; i++){
    y=-sin(i*(TWO_PI/(SAMPLES-1)))*((AMPLITUDE_MAX/2)-1) + AMPLITUDE_MAX/2;
    y1=-sin((i+1)*(TWO_PI/(SAMPLES-1)))*((AMPLITUDE_MAX/2)-1) + AMPLITUDE_MAX/2;
    tft.drawLine(i, (int) y,i+1, (int) y1, TFT_WHITE);
  }
  return (millis()-startTime);
}



void printDuration() {
  int startTime=millis();
  Serial.print("durationSerialIn = ");Serial.print(durationSerialIn);Serial.println(" ms");
  Serial.print("durationInitBuffer = ");Serial.print(durationInitBuffer);Serial.println(" ms");
  Serial.print("durationPlotTimeBuffer = ");Serial.print(durationPlotTimeBuffer);Serial.println(" ms");
  Serial.print("durationTftTimeBuffer = ");Serial.print(durationTftTimeBuffer);Serial.println(" ms");
  Serial.print("durationPlotFreqBuffer = ");Serial.print(durationPlotFreqBuffer);Serial.println(" ms");
  Serial.print("durationTftFreqBuffer = ");Serial.print(durationTftFreqBuffer);Serial.println(" ms");
  Serial.print("durationFftWindowing = ");Serial.print(durationFftWindowing);Serial.println(" ms");
  Serial.print("durationFftCompute = ");Serial.print(durationFftCompute);Serial.println(" ms");
  Serial.print("durationFftComplexeToMagnitude = ");Serial.print(durationFftComplexeToMagnitude);Serial.println(" ms");
  Serial.print("durationLoop = ");Serial.print(durationLoop);Serial.println(" ms");
  int durationPrintDuration=millis()-startTime;
  Serial.print("durationPrintDuration = ");Serial.print(durationPrintDuration);Serial.println(" ms");


  
}

