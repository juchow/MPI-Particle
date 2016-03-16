#if defined (SPARK)
#include "VisiGenieSpark.h"
#else
#include <genieArduino.h>
#endif
#include "SparkIntervalTimer.h"
#include "Adafruit_ADS1015.h"
// This Demo communicates with a 4D Systems Display, configured with ViSi-Genie, utilising the Genie Arduino Library - https://github.com/4dsystems/ViSi-Genie-Arduino-Library.
// The display has a slider, a cool gauge, an LED Digits, a string box and a User LED.
// The program receives messages from the Slider0 object using the Reported Events. This is triggered each time the Slider changes on the display, and an event
// is genereated and sent automatically. Reported Events originate from the On-Changed event from the slider itself, set in the Workshop4 software.
// Coolgauge is written to using Write Object, and the String is updated using the Write String command, showing the version of the library.
// The User LED is updated by the Arduino, by first doing a manual read of the User LED and then toggling it based on the state received back.

// As the slider changes, it sends its value to the Arduino (Arduino also polls its value using genie.ReadObject, as above), and the Arduino then
// tells the LED Digit to update its value using genie.WriteObject. So the Slider message goes via the Arduino to the LED Digit.
// Coolgauge is updated via simple timer in the Arduino code, and updates the display with its value.
// The User LED is read using genie.ReadObject, and then updated using genie.WriteObject. It is manually read, it does not use an Event.

// This demo illustrates how to use genie.ReadObject, genie.WriteObject, Reported Messages (Events), genie.WriteStr, genie.WriteContrast, plus supporting functions.

// Application Notes on the 4D Systems Website that are useful to understand this library are found: http://www.4dsystems.com.au/appnotes
// Good App Notes to read are:
// 4D-AN-P4017 - Connecting a 4D Display to an Arduino Host - http://www.4dsystems.com.au/downloads/Application-Notes/4D-AN-P4017_R_1_0.zip
// 4D-AN-P4018 - Writing to Genie Objects Using an Arduino Host - http://www.4dsystems.com.au/downloads/Application-Notes/4D-AN-P4018_R_1_0.zip
// 4D-AN-P4019 - A Simple Digital Voltmeter Application using an Arduino Host - http://www.4dsystems.com.au/downloads/Application-Notes/4D-AN-P4019_R_1_0.zip
// 4D-AN-P4025 - Arduino Danger Shield for Sparkfun Danger Shield - http://www.4dsystems.com.au/downloads/Application-Notes/4D-AN-P4025_R_1_0.zip

#define VERSION_NUM "MPI Firmware v.02"
#define attSetAddr0 0x38
#define attResetAddr0 0x3e
#define attSetAddr1 0x39
#define attResetAddr1 0x3f
#define adc1 0x48
#define RESETLINE D4            // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)
Genie genie;
//Adafruit_ADS1115 ads(adc1);
const int relayLatchTime = 3;   // time in milliseconds
int balSlider = 0;;                 // var to set volume balance
int volSlider = 0;
const uint8_t ledPin = D7;		  // LED for first Interval Timer
//const uint8_t ledPin2 = D3;		// LED for second Interval Timer
volatile unsigned long blinkCount = 0; // use volatile for variables shared with ISR
int leftVol;
int rightVol;

void setVolume(int leftVol1, int rightVol1);
// Create IntervalTimer objects
IntervalTimer myTimer;	     	  // 3 for the Core
//IntervalTimer myTimer2;

void setup()
{
    balSlider = 127;
    Wire.begin();
    Serial.begin(9600);
    while (!Serial);            // Leonardo: wait for serial monitor
  //  #if defined (SPARK)
    Serial1.begin(115200);      // Serial1 @ 200000 (200K) Baud
    genie.Begin(Serial1);       // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
    while (!Serial1);
  //  #else
    //Serial.begin(200000);     // Serial1 @ 200000 (200K) Baud
    //genie.Begin(Serial);      // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
    //#endif
    //ads.begin();
    transmit(attSetAddr0,0xff); // all outputs off
    delay(relayLatchTime);
    transmit(attResetAddr0,0xff);
    delay(relayLatchTime);
    transmit(attSetAddr1,0xff); // all outputs off
    delay(relayLatchTime);
    transmit(attResetAddr1,0xff);
    delay(relayLatchTime);

    genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events
    myTimer.begin(blinkLED, 1000, hmSec);
    pinMode(ledPin, OUTPUT);

    // Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
    // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
    // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
    // the 4D Arduino Adaptors invert this signal so must be Active High.
    pinMode(RESETLINE, OUTPUT); // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
    digitalWrite(RESETLINE, 1); // Reset the Display via D4
    delay(100);
    digitalWrite(RESETLINE, 0); // unReset the Display via D4

    delay (3500);               //let the display start up after the reset (This is important)

    //Turn the Display on (Contrast) - (Not needed but illustrates how)
    genie.WriteContrast(1);     // 1 = Display ON, 0 = Display OFF.
    //For uLCD43, uLCD-70DT, and uLCD-35DT, use 0-15 for Brightness Control, where 0 = Display OFF, though to 15 = Max Brightness ON.

    //Write a string to the Display to show the version of the library used
    //genie.WriteStr(0, GENIE_VERSION);
    genie.WriteStr(0, VERSION_NUM);
    genie.WriteObject(GENIE_OBJ_SLIDER, 0x01, balSlider);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,0x01, balSlider-127);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS,0x00, volSlider);
}


void loop()
{
  static long waitPeriod = millis();
  //static int gaugeAddVal = 1;
  //static int gaugeVal = 50;

  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display

  if (millis() >= waitPeriod)
  {
    // Write to CoolGauge0 with the value in the gaugeVal variable
    //genie.WriteObject(GENIE_OBJ_COOL_GAUGE, 0x00, gaugeVal);
    //gaugeVal += gaugeAddVal;
    //if (gaugeVal == 99) gaugeAddVal = -1;
    //if (gaugeVal == 0) gaugeAddVal = 1;

    // The results of this call will be available to myGenieEventHandler() after the display has responded
    // Do a manual read from the UserLEd0 object
    genie.ReadObject(GENIE_OBJ_USER_LED, 0x00);

    waitPeriod = millis() + 50; // rerun this code to update Cool Gauge and Slider in another 50ms time.
  }
}

void blinkLED(void)
{
    digitalWrite(ledPin,!digitalRead(ledPin));
    blinkCount++;	    	// increase when LED turns changes
}

void thermo(void)
{
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV (default), 188uV for ADS1115
// ads.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
// ads.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
// ads.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
// ads.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV
// ads.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV
/*
  int16_t adc_val0 = ads.readADC_SingleEnded(0);
  int16_t adc_val1 = ads.readADC_SingleEnded(1);
  int16_t adc_val2 = ads.readADC_SingleEnded(2);
  float Voltage0 = (adc_val0 * 0.1875)/1000; //scaled to 2/3x gain, do 1/2 value for 1x gain  6.144/32767
  int temp0 = (int)Voltage0*1000;
  temp0 = map(temp0,500,1750,0,125); //check if can have negative
  String tempStr0 = String(temp0);
  String tempStr1 = String(temp1);
  String tempStr2 = String(temp2);

  genie.WriteStr(1, tempStr0);
  genie.WriteStr(2, tempStr1);
  genie.WriteStr(3, tempStr2);

  genie.WriteObject(GENIE_OBJ_THERMOMETER, 0x00, Voltage);
  genie.WriteObject(GENIE_OBJ_SLIDER, 0x01, temp1);
  genie.WriteObject(GENIE_OBJ_SLIDER, 0x02, temp2);
  */
}

void transmit(byte Addr, byte trans)
{
    Wire.beginTransmission(Addr);
    Wire.write(trans);
    Wire.endTransmission();
}

void setVolume(int leftVol1, int rightVol1)
{
    int sendLeftVal;
    int sendRightVal;
    // now set
    //Wire.begin();
    sendLeftVal = (byte)leftVol1;
    Serial.println("leftVol1 is");
    Serial.println(leftVol);
    sendRightVal = (byte)rightVol1;
    Serial.println("rightVol1 is");
    Serial.println(rightVol);
    transmit(attSetAddr0,0);
    transmit(attSetAddr0,sendLeftVal);
    delay(relayLatchTime);
    transmit(attSetAddr1,0);
    transmit(attSetAddr1,sendRightVal);
    //Serial.println("transmitted attSetAddr1");
    delay(relayLatchTime);

    // now reset
    sendLeftVal = 0xFF ^ (byte)leftVol1;
    sendRightVal = 0xff ^ (byte)rightVol1;
    transmit(attResetAddr0,0);
    transmit(attResetAddr0,sendLeftVal);
    delay(relayLatchTime);
    transmit(attResetAddr1,0);
    transmit(attResetAddr1,sendRightVal);
    delay(relayLatchTime);

}


void setBalance(int newBal)
{

    int currBal = newBal;
    int newVol = volSlider;
    Serial.println("newVol is volSlider");
    if (currBal < 127)
    {
      leftVol = newVol + 127-currBal;
      Serial.println("leftVol is ");
      Serial.println(leftVol);
      rightVol = newVol;
      if (leftVol > 255)
      {
        leftVol = 255;
      }
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, 127-currBal);
      genie.WriteObject(GENIE_OBJ_LED,0x00, 1);
    }
    else if (currBal > 127)
    {
      leftVol = newVol;
      rightVol = newVol + currBal - 127;
      Serial.println("rightVol is ");
      Serial.println(rightVol);
      if (rightVol > 255)
      {
        rightVol = 255;
      }
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, currBal-127);
      genie.WriteObject(GENIE_OBJ_LED,0x00, 0);
    }
    else
    {
      leftVol = newVol;
      rightVol = newVol;
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, currBal-127);
      genie.WriteObject(GENIE_OBJ_LED,0x00, 0);
    }

}



/////////////////////////////////////////////////////////////////////
//
// This is the user's event handler. It is called by genieDoEvents()
// when the following conditions are true
//
//		The link is in an IDLE state, and
//		There is an event to handle
//
// The event can be either a REPORT_EVENT frame sent asynchronously
// from the display or a REPORT_OBJ frame sent by the display in
// response to a READ_OBJ request.
//

/* COMPACT VERSION HERE, LONGHAND VERSION BELOW WHICH MAY MAKE MORE SENSE
void myGenieEventHandler(void)
{
  genieFrame Event;
  int volSlider = 0;
  const int index = 0;  //HARD CODED TO READ FROM Index = 0, ie Slider0 as an example

  genieDequeueEvent(&Event);

  //Read from Slider0 for both a Reported Message from Display, and a Manual Read Object from loop code above
  if (genieEventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_SLIDER, index) ||
    genieEventIs(&Event, GENIE_REPORT_OBJ,   GENIE_OBJ_SLIDER, index))
  {
    volSlider = genieGetEventData(&Event);  // Receive the event data from the Slider0
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x00, volSlider);     // Write Slider0 value to to LED Digits 0
  }
} */

// LONG HAND VERSION, MAYBE MORE VISIBLE AND MORE LIKE VERSION 1 OF THE LIBRARY
void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event);

  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)                // If the Reported Message was from a 4D Button
    {
      if (Event.reportObject.index == 0)                              // If 4DButton0
      {
        volSlider --;

        if (volSlider < 0)
        {
          volSlider = 0;
        }

        genie.WriteObject(GENIE_OBJ_SLIDER, 0x00, volSlider);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, volSlider);    // Write Slider0 value to to LED Digits 0
        //leftVol = volSlider;
        setBalance(balSlider);
        //rightVol = volSlider;
        setVolume(255-leftVol,255-rightVol);
      }
      else if (Event.reportObject.index == 1)                     //If 4DButton1
      {
        volSlider ++;
        if (volSlider > 255)
        {
          volSlider = 255;
        }
        genie.WriteObject(GENIE_OBJ_SLIDER, 0x00, volSlider);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS,0x00, volSlider);
        //leftVol = volSlider;
        //rightVol = volSlider;
        setBalance(balSlider);
        setVolume(255-leftVol, 255-rightVol);
      }
      else if (Event.reportObject.index == 5)
      {
        balSlider --;
        if (balSlider < 0)
        {
          balSlider = 0;
        }
        setBalance(balSlider);
        setVolume(255-leftVol, 255-rightVol);
      }
      else if (Event.reportObject.index == 6)
      {
        balSlider ++;
        if (balSlider > 255)
        {
          balSlider = 255;
        }
        setBalance(balSlider);
        setVolume(255-leftVol, 255-rightVol);
      }
    }
  }


  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_SLIDER)                // If the Reported Message was from a Slider
    {
      if (Event.reportObject.index == 0)                              // If Slider0
      {
        volSlider = genie.GetEventData(&Event);                      // Receive the event data from the Slider0
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, volSlider);    // Write Slider0 value to to LED Digits 0
        //leftVol = volSlider;
        //rightVol = volSlider;
        setBalance(balSlider);
        setVolume(255-leftVol, 255-rightVol);
        //Serial.println("slider val is ");
        //Serial.println(255-volSlider);
      }
      if (Event.reportObject.index == 1)
      {
        balSlider = genie.GetEventData(&Event);
        setBalance(balSlider);
        setVolume(255-leftVol, 255-rightVol);
      }
    }
  }



  //If the cmd received is from a Reported Object, which occurs if a Read Object (genie.ReadOject) is requested in the main code, reply processed here.
  if (Event.reportObject.cmd == GENIE_REPORT_OBJ)
  {
    if (Event.reportObject.object == GENIE_OBJ_USER_LED)              // If the Reported Message was from a User LED
    {
      if (Event.reportObject.index == 0)                              // If UserLed0
      {
        bool UserLed0_val = genie.GetEventData(&Event);               // Receive the event data from the UserLed0
        //UserLed0_val = !UserLed0_val;                                 // Toggle the state of the User LED Variable
        genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, UserLed0_val);    // Write UserLed0_val value back to to UserLed0
      }
    }
  }

  //This can be expanded as more objects are added that need to be captured

  //Event.reportObject.cmd is used to determine the command of that event, such as an reported event
  //Event.reportObject.object is used to determine the object type, such as a Slider
  //Event.reportObject.index is used to determine the index of the object, such as Slider0
  //genie.GetEventData(&Event) us used to save the data from the Event, into a variable.
}

// scans for i2c devices on the bus

void i2cScan(){
    Serial.println("\nI2C Scanner");
    byte error, address;
    int nDevices;
    Serial.println("Scanning...");
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address<16)
          Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");

          nDevices++;
        }
        else if (error==4)
        {
          Serial.print("Unknown error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }
      }
      if (nDevices == 0)
        Serial.println("No I2C devices found\n");
      else
        Serial.println("done\n");

      delay(5000);           // wait 5 seconds for next scan

}
