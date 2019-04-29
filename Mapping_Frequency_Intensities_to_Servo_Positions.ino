/*fix_fft.cpp and fix_fft.h Originally Written by:  Tom Roberts  11/8/89
Made portable:  Malcolm Slaney 12/15/94 malcolm@interval.com
Enhanced:  Dimitrios P. Bouras  14 Jun 2006 dbouras@ieee.org
Modified for 8bit values David Keller  10.10.2018
Main code adapted for Gizmo Project by Saym Hussain: 04/03/2019*/

/*This codes uses Fast Fourier Trasnforms to transform real-time audio into the frequency domain
where it is then classifed in to frequency bins between 0 - 4.5 kHz that correspind to known audio 
classificiations. The average intensity of each bin is taken and is then scales to a range between 
0 - 180, which controls the position of corresponding micro-servos.*/

#include <arduinoFFT.h> //import the library

#define SAMPLES 128 //Must be a power of 2
#define SAMPLING_FREQUENCY 9000 //Hz, must be less than 10000 due to ADC
#include <Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo
Servo myservo4;  // create servo object to control a servo
Servo myservo5;  // create servo object to control a servo
Servo myservo6;  // create servo object to control a servo

// twelve servo objects can be created on most boards

arduinoFFT FFT = arduinoFFT(); // create FFT object

unsigned int sampling_period_us; unsigned long microseconds;
double vReal[SAMPLES]; //Real part of FFT array
double vImag[SAMPLES]; //Imaginary part of FFT aray

// Initialises the Bins that hold the frequency intensity value
float Bin_1 = 0;
float Bin_2 = 0;
float Bin_3 = 0;
float Bin_4 = 0;
float Bin_5 = 0;
float Bin_6 = 0;

const int Servo1 = 2;    // Servo connected to digital pin 2
const int Servo2 = 3;    // Servo connected to digital pin 3
const int Servo3 = 4;    // Servo connected to digital pin 4
const int Servo4 = 5;    // Servo connected to digital pin 5
const int Servo5 = 6;    // Servo connected to digital pin 6
const int Servo6 = 7;    // Servo connected to digital pin 7

//Initialises the position variables that tell the servo amount of rotation required
int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;
int pos5 = 0;
int pos6 = 0;

void setup() {
  
  Serial.begin(115200);
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  myservo1.attach(2);  // attaches the servo on pin 2 to the servo object
  myservo2.attach(3);  // attaches the servo on pin 3 to the servo object
  myservo3.attach(4);  // attaches the servo on pin 4 to the servo object
  myservo4.attach(5);  // attaches the servo on pin 5 to the servo object
  myservo5.attach(6);  // attaches the servo on pin 6 to the servo object
  myservo6.attach(7);  // attaches the servo on pin 7 to the servo object
}

void loop() {

   /*SAMPLING*/
   for(int i=0; i<SAMPLES; i++)
   {
       microseconds = micros();    //Overflows after around 70 minutes!
    
       vReal[i] = analogRead(A0);  //Read the s analogue input from the microphone connected to A0
       vImag[i] = 0;               //Imaginary Domain set to 0 as this is no required for my use
    
       while(micros() < (microseconds + sampling_period_us)){
       }
   }

   /*FFT*/                                                                // builtin FFT library and methods
   FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
   FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
   FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
   double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

   /*ACCOUNT FOR DC OFFSET - vReal[0] ignored since it is the impulse value of all the frequencies*/
   //vReal[0] = vReal[0] - 17900;

   vReal[1] = vReal[1] - 7300;    //From testing in a silence space, this was the offset found
   vReal[1] = abs(vReal[1]);      //The offest may fluctuate, therefore the absolute value is taken in case the value of vReal[1] becomes negetive

   
   /*LED/SERVO MOTION*/

   //Sub-Bass Range
   Bin_1 = vReal[1];
   if (Bin_1 <= 2400) {             //if less than the max limit
      Bin_1 = (Bin_1/2400)*180;     //Scale to servo range
      pos1 = Bin_1;                 //Change servo position
    }
    else if (Bin_1 > 2400) {        //if more than the max limit
      pos1 = 180;                   //make the servo postion the maximumm
    }

   //Bass Range
   Bin_2 = vReal[2] + vReal[3];
   Bin_2 = Bin_2/2;                 //finds the avergae intensty for the associated frequency ranges
   if (Bin_2 <= 3000) {             //if less than the max limit
    Bin_2 = (Bin_2/3000)*180;       //Scale to servo range
    pos2 = Bin_2;                   //Change servo position
   }
   else if (Bin_2 > 3000) {         //if more than the max limit
    pos2 = 180;                     //make the servo postion the maximumm
   }

   //Low Midrange
   Bin_3 = vReal[4] + vReal[5] + vReal[6];
   Bin_3 = Bin_3/3;                 //finds the avergae intensty for the associated frequency ranges
   if (Bin_3 <= 2800) {             //if less than the max limit
    Bin_3 = (Bin_3/2800)*180;       //Scale to servo range
    pos3 = Bin_3;                   //Change servo position
   }
   else if (Bin_3 > 2800) {         //if more than the max limit
    pos2 = 180;                     //make the servo postion the maximumm
   }

   //Midrange
   Bin_4 = vReal[7] + vReal[8] + vReal[9] + vReal[10] + vReal[11] + vReal[12] + vReal[13] + vReal[14] + vReal[15] + vReal[16] + vReal[17] + vReal[18] + vReal[19] + vReal[20] + vReal[21] + vReal[22] + vReal[23] + vReal[24] + vReal[25] + vReal[26] + vReal[27] + vReal[28];
   Bin_4 = Bin_4/22;                //finds the avergae intensty for the associated frequency ranges
   if (Bin_4 <= 900) {              //if less than the max limit
    Bin_4 = (Bin_4/900)*180;        //Scale to servo range
    pos4 = Bin_4;                   //Change servo position
   }
   else if (Bin_4 > 900) {          //if more than the max limit
    pos4 = 180;                     //make the servo postion the maximumm
   }

   //Upper Midrange
   Bin_5 = vReal[29] + vReal[30] + vReal[31] + vReal[32] + vReal[33] + vReal[34] + vReal[35] + vReal[36] + vReal[37] + vReal[38] + vReal[39] + vReal[40] + vReal[41] + vReal[42] + vReal[43] + vReal[44] + vReal[45] + vReal[46] + vReal[47] + vReal[48] + vReal[49] + vReal[50] + vReal[51] + vReal[52] + vReal[53] + vReal[54] + vReal[55] + vReal[56];
   Bin_5 = Bin_5/28;                //finds the avergae intensty for the associated frequency ranges
   if (Bin_5 <= 650) {              //if less than the max limit
    Bin_5 = (Bin_5/650)*180;        //Scale to servo range
    pos5 = Bin_5;                   //Change servo position
   }
   else if (Bin_5 > 650) {          //if more than the max limit
    pos5 = 180;                     //make the servo postion the maximumm
   }
   
   //Prescence (cut at 4.5 kHz)
   Bin_6 = vReal[57] + vReal[58] + vReal[59] + vReal[60] + vReal[61] + vReal[62] + vReal[63];
   Bin_6 = Bin_6/7;                 //finds the avergae intensty for the associated frequency ranges
   if (Bin_6 <= 450) {              //if less than the max limit
    Bin_6 = (Bin_6/450)*180;        //Scale to servo range
    pos6 = Bin_6;                   //Change servo position
   }
   else if (Bin_6 > 450) {          //if more than the max limit
    pos6 = 180;                     //make the servo postion the maximumm
   }

/*Writes the position to the servo*/
   myservo1.write(pos1);
   myservo2.write(pos2);
   myservo3.write(pos3);
   myservo4.write(pos4);
   myservo5.write(pos5);
   myservo6.write(pos6);

   
   /*PRINT RESULTS*/
   //Serial.println(peak);                           //Print out what frequency is the most dominant.

   for(int i=1; i<(SAMPLES/2); i++)
   {
       /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/

       //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
       //Serial.print(" Hz ");
       //Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
   }
   //Serial.print('\n');
   
   //Serial.print((pos1));
   //Serial.print('\n');
   //Serial.print((pos2));
   //Serial.print('\n');
   //Serial.print((pos3));
   //Serial.print('\n');
   //Serial.print((pos4));
   //Serial.print('\n');
   //Serial.print((pos5));
   //Serial.print('\n');
   //Serial.print((pos6));
   //Serial.print('\n');
   //Serial.print(" ");
   //Serial.print('\n');
   
   delay(50);  //Repeat the process every 50 ms OR:
   //while(1);       //Run code once
}
