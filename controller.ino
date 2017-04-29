#include <Adafruit_NeoPixel.h>

// Pattern types supported:
enum  pattern { NONE, RAINBOW_CYCLE, THEATER_CHASE, COLOR_WIPE, SCANNER, FADE };
// Patern directions supported:
enum  direction { FORWARD, REVERSE };

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
    public:

    // Member Variables:
    pattern  ActivePattern;  // which pattern is running
    direction Direction;     // direction to run the pattern

    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position

    uint32_t Color1, Color2;  // What colors are in use
    uint16_t TotalSteps;  // total number of steps in the pattern
    uint16_t Index;  // current step within the pattern

    void (*OnComplete)();  // Callback on completion of pattern

    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
    :Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }

    // Update the pattern
    void Update()
    {
        if((millis() - lastUpdate) > Interval) // time to update
        {
            lastUpdate = millis();
            switch(ActivePattern)
            {
                case RAINBOW_CYCLE:
                    RainbowCycleUpdate();
                    break;
                case THEATER_CHASE:
                    TheaterChaseUpdate();
                    break;
                case COLOR_WIPE:
                    ColorWipeUpdate();
                    break;
                case SCANNER:
                    ScannerUpdate();
                    break;
                case FADE:
                    FadeUpdate();
                    break;
                default:
                    break;
            }
        }
    }

    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == FORWARD)
        {
           Index++;
           if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
        else // Direction == REVERSE
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps-1;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
    }

    // Reverse pattern direction
    void Reverse()
    {
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps-1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
    }

    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = RAINBOW_CYCLE;
        Interval = interval;
        TotalSteps = 255;
        Index = 0;
        Direction = dir;
    }

    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
        }
        show();
        Increment();
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = THEATER_CHASE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
   }

    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
        for(int i=0; i< numPixels(); i++)
        {
            if ((i + Index) % 3 == 0)
            {
                setPixelColor(i, Color1);
            }
            else
            {
                setPixelColor(i, Color2);
            }
        }
        show();
        Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = COLOR_WIPE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Index = 0;
        Direction = dir;
    }

    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        setPixelColor(Index, Color1);
        show();
        Increment();
    }

    // Initialize for a SCANNNER
    void Scanner(uint32_t color1, uint8_t interval)
    {
        ActivePattern = SCANNER;
        Interval = interval;
        TotalSteps = (numPixels() - 1) * 2;
        Color1 = color1;
        Index = 0;
    }

    // Update the Scanner Pattern
    void ScannerUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            if (i == Index)  // Scan Pixel to the right
            {
                 setPixelColor(i, Color1);
            }
            else if (i == TotalSteps - Index) // Scan Pixel to the left
            {
                 setPixelColor(i, Color1);
            }
            else // Fading tail
            {
                 setPixelColor(i, DimColor(getPixelColor(i)));
            }
        }
        show();
        Increment();
    }

    // Initialize for a Fade
    void Fade(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = FADE;
        Interval = interval;
        TotalSteps = steps;
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
    }

    // Update the Fade Pattern
    void FadeUpdate()
    {
        // Calculate linear interpolation between Color1 and Color2
        // Optimise order of operations to minimize truncation error
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;

        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }

    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color)
    {
        // Shift R, G and B components one bit to the right
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }

    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if(WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }
};

void Ring1Complete();
void Ring2Complete();



//------------------------------------------------------------
//Completion Routines - get called on completion of a pattern
//------------------------------------------------------------






/* Beginning of original code without NeoPattern Class and definitions */




/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"


// added libraries for using the DC motors
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "BluefruitConfig.h"


// Adafruit NeoPixel Library

#include <Adafruit_NeoPixel.h>







/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0

#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/


#define LEFT_NEO 4
#define RIGHT_NEO 5



// Create the object for the motor shield and the objects for the DC motors
Adafruit_MotorShield motorShield = Adafruit_MotorShield();
// the left motor corresponds to port M1
Adafruit_DCMotor *leftMotor = motorShield.getMotor(1);
// the right motor corresponds to the port M2
Adafruit_DCMotor *rightMotor = motorShield.getMotor(2);

// setting the default speed of the motors


// Define the NeoPixel objects
Adafruit_NeoPixel leftNeo = Adafruit_NeoPixel(12, LEFT_NEO);
Adafruit_NeoPixel rightNeo = Adafruit_NeoPixel(12, RIGHT_NEO);


//// Define the NeoPattern objects
//// ring 1 is the left ring
NeoPatterns Ring1(12, 4, NEO_GRB + NEO_KHZ800, &Ring1Complete);
//// ring 2 is the right ring
NeoPatterns Ring2(12, 5, NEO_GRB + NEO_KHZ800, &Ring2Complete);




// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {


  // calling the begin function on the motor shield object
  motorShield.begin();

  Ring1.begin();
  Ring2.begin();


  Ring1.TheaterChase(Ring1.Color(0,255,0), Ring1.Color(0,0,255), 20);
  Ring2.TheaterChase(Ring1.Color(0,255,0), Ring1.Color(0,0,255), 20);


//  leftNeo.begin();
//  rightNeo.begin();
//
//  leftNeo.show();
//  rightNeo.show();
//
//  leftNeo.setBrightness(50);
//  rightNeo.setBrightness(50);
//
//  rightNeo.show();
//  leftNeo.show();
//
//
//  rightNeo.setPixelColor(0, 0, 0, 255);
//  rightNeo.setPixelColor(1, 0, 0, 255);
//  rightNeo.setPixelColor(2, 0, 0, 255);
//  rightNeo.setPixelColor(3, 0, 0, 255);
//  rightNeo.setPixelColor(4, 0, 0, 255);
//  rightNeo.setPixelColor(5, 0, 0, 255);
//  rightNeo.setPixelColor(6, 0, 0, 255);
//  rightNeo.setPixelColor(7, 0, 0, 255);
//  rightNeo.setPixelColor(8, 0, 0, 255);
//  rightNeo.setPixelColor(9, 0, 0, 255);
//  rightNeo.setPixelColor(10, 0, 0, 255);
//  rightNeo.setPixelColor(11, 0, 0, 255);
//
//  leftNeo.setPixelColor(0, 0, 0, 255);
//  leftNeo.setPixelColor(1, 0, 0, 255);
//  leftNeo.setPixelColor(2, 0, 0, 255);
//  leftNeo.setPixelColor(3, 0, 0, 255);
//  leftNeo.setPixelColor(4, 0, 0, 255);
//  leftNeo.setPixelColor(5, 0, 0, 255);
//  leftNeo.setPixelColor(6, 0, 0, 255);
//  leftNeo.setPixelColor(7, 0, 0, 255);
//  leftNeo.setPixelColor(8, 0, 0, 255);
//  leftNeo.setPixelColor(9, 0, 0, 255);
//  leftNeo.setPixelColor(10, 0, 0, 255);
//  leftNeo.setPixelColor(11, 0, 0, 255);
//
//    rightNeo.show();
//  leftNeo.show();




  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

int leftSpeed = 50;
int rightSpeed = 50;

void loop(void)

{

//  Ring1.Update();
//  Ring2.Update();

//  leftNeo.setPixelColor(2, 0, 0, 255);
//
//  rightNeo.setPixelColor(0, 0, 0, 255);
//  rightNeo.setPixelColor(1, 0, 0, 255);
//  rightNeo.setPixelColor(2, 0, 0, 255);
//  rightNeo.setPixelColor(3, 0, 0, 255);
//  rightNeo.setPixelColor(4, 0, 0, 255);
//  rightNeo.setPixelColor(5, 0, 0, 255);
//  rightNeo.setPixelColor(6, 0, 0, 255);
//  rightNeo.setPixelColor(7, 0, 0, 255);
//  rightNeo.setPixelColor(8, 0, 0, 255);
//  rightNeo.setPixelColor(9, 0, 0, 255);
//  rightNeo.setPixelColor(10, 0, 0, 255);
//  rightNeo.setPixelColor(11, 0, 0, 255);
//
//  leftNeo.setPixelColor(0, 0, 0, 255);
//  leftNeo.setPixelColor(1, 0, 0, 255);
//  leftNeo.setPixelColor(2, 0, 0, 255);
//  leftNeo.setPixelColor(3, 0, 0, 255);
//  leftNeo.setPixelColor(4, 0, 0, 255);
//  leftNeo.setPixelColor(5, 0, 0, 255);
//  leftNeo.setPixelColor(6, 0, 0, 255);
//  leftNeo.setPixelColor(7, 0, 0, 255);
//  leftNeo.setPixelColor(8, 0, 0, 255);
//  leftNeo.setPixelColor(9, 0, 0, 255);
//  leftNeo.setPixelColor(10, 0, 0, 255);
//  leftNeo.setPixelColor(11, 0, 0, 255);
//
//
//  leftNeo.show();
//  rightNeo.show();


  pinMode(3, OUTPUT);
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }


  // notes about electronics:
  // minus goes with black
  // long wire foes with red
  // put both of the neopixels red and black into the red and black of the converter


  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    if (buttnum == 2) {
      leftSpeed += 50;
      rightSpeed += 50;
      leftMotor->setSpeed(leftSpeed);
      rightMotor->setSpeed(rightSpeed);
    }
    if (buttnum == 4) {
      leftSpeed -= 50;
      rightSpeed -= 50;
      leftMotor->setSpeed(leftSpeed);
      rightMotor->setSpeed(rightSpeed);
    }


    if (pressed == false and (buttnum != 1) and (buttnum != 2) and (buttnum != 3) and (buttnum != 4)) {

      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);

    } else {

      // put all of the other if statements in here, will only enter this loop if a button is being pressed

      // pressing button 5 makes both tracks rotate forward
      if (buttnum == 5) {
        leftMotor->run(FORWARD);
        rightMotor->run(BACKWARD);
        //Ring1.ActivePattern = FADE;
        //Ring2.ActivePattern = FADE;
      }
      // pressing button 6 makes both tracks rotate backward
      if (buttnum == 6) {
        leftMotor->run(BACKWARD);
        rightMotor->run(FORWARD);
      }
      // pressing button 7 makes the right motor rotate forward
      if (buttnum == 7) {
        rightMotor->run(BACKWARD);
      }
      // pressing button 8 makes the right motor stop rotating
      if (buttnum == 8) {
        leftMotor->run(FORWARD);
      }

    }


    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }
  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer + 2);
    lon = parsefloat(packetbuffer + 6);
    alt = parsefloat(packetbuffer + 10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  //if (packetbuffer[1] == 'A') {
  //float x, y, z;
  //x = parsefloat(packetbuffer+2);
  //y = parsefloat(packetbuffer+6);
  //z = parsefloat(packetbuffer+10);
  //Serial.print("Accel\t");
  //Serial.print(x); Serial.print('\t');
  //Serial.print(y); Serial.print('\t');
  //Serial.print(z); Serial.println();
  //}

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer + 2);
    y = parsefloat(packetbuffer + 6);
    z = parsefloat(packetbuffer + 10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer + 2);
    y = parsefloat(packetbuffer + 6);
    z = parsefloat(packetbuffer + 10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer + 2);
    y = parsefloat(packetbuffer + 6);
    z = parsefloat(packetbuffer + 10);
    w = parsefloat(packetbuffer + 14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
}



// Ring1 Completion Callback
void Ring1Complete()
{

        Ring2.Interval = 40;
        Ring1.Color1 = Ring1.Wheel(random(255));
        Ring1.Interval = 20000;

}

// Ring 2 Completion Callback
void Ring2Complete()
{
        Ring1.Interval = 20;
        Ring2.Color1 = Ring2.Wheel(random(255));
        Ring2.Interval = 20000;
}
