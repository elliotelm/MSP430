
// Test Code by Elliot Marcus

// include the SPI library
// include I/O for GPIOs
#include <SPI.h>
#include <io.h>

#define RED_LED P1_0        // output dev board red LED
#define GREEN_LED P4_7      // output dev board green LED
#define BB_GP_ISR P8_1      // output dev board pin 8.1
#define BB_ON_OFF P8_2      // output dev board pin 8.2
#define BB_Shutdown P4_0    // input dev board pin 4.0
#define CC_POL P3_7         // input dev board pin 3.7
#define CC_INT P2_4         // input dev board pin 2.4
volatile uint8_t val = 0x00;// variable for SPI test

void setup()
{
  // initialize pins and default pin states
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BB_GP_ISR, OUTPUT);
  pinMode(BB_ON_OFF, OUTPUT);
  pinMode(BB_Shutdown, INPUT_PULLUP);
  pinMode(CC_POL, INPUT_PULLUP);
  pinMode(CC_INT, INPUT_PULLUP);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  // start the SPI library
  SPI.begin();                
}
void GREEN_LED_blink(int LEDON)
{
 digitalWrite(GREEN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
 delay(LEDON);                    // wait for LEDON time in ms
 digitalWrite(GREEN_LED, LOW);    // turn the LED off by making the voltage LOW
 delay(LEDON);                    // wait for LEDON time in ms
}
void RED_LED_blink()
{
 digitalWrite(RED_LED, HIGH);     // turn the LED on (HIGH is the voltage level)
 delay(200);                      // wait for 200ms
 digitalWrite(RED_LED, LOW);      // turn the LED off by making the voltage LOW
 delay(200);                      // wait for 200ms
}
void Beagle_GP_ISR()
{
 digitalWrite(BB_GP_ISR, HIGH);   // turn the pin on (HIGH is the voltage level)
 delay(20);                       // wait for 20ms
 digitalWrite(BB_GP_ISR, LOW);    // turn the pin off by making the voltage LOW
 delay(20);                       // wait for 20ms
 digitalWrite(BB_GP_ISR, HIGH);   // turn the pin on (HIGH is the voltage level)
 delay(20);                       // wait for 20ms
 digitalWrite(BB_GP_ISR, LOW);    // turn the pin off by making the voltage LOW
 delay(20);                       // wait for 20ms
}
void Beagle_ON_OFF(int ontime)
{
 digitalWrite(BB_ON_OFF, HIGH);   // turn the pin on (HIGH is the voltage level)
 delay(ontime);                   // wait for passed in time in ms
 digitalWrite(BB_ON_OFF, LOW);    // turn the pin off by making the voltage LOW
 delay(20);                       // wait for 20ms
 digitalWrite(BB_ON_OFF, HIGH);   // turn the pin on (HIGH is the voltage level)
 delay(ontime);                   // wait for passed in time in ms
 digitalWrite(BB_ON_OFF, LOW);    // turn the pin off by making the voltage LOW
 delay(20);                       // wait for 20ms
}

// MAIN
void loop()
{
  UCA0TXBUF = val;                // give the SPI transfer buffer a register value
  SPI.transfer(UCA0TXBUF);        // transfer data from register number
  val += 1;                       // increment the register value by 1
  if(val > 8)                     // just cycle between the first 9 registers
  {
    val = 0;
  }
  
  int BBShutdown = (P4IN&0x01);   // sets variable to pin register value p4.0
  int CCPOL = (P3IN&0x80);        // sets variable to pin register value p3.7
  int CCINT = (P2IN&0x10);        // sets variable to pin register value p2.4
  
  Beagle_GP_ISR();                // output BeagleBone GP ISR pin
  
  if(BBShutdown && CCPOL && CCINT)
  {
    Beagle_ON_OFF(400);           // output normal output signal on pin 8.2
    GREEN_LED_blink(400);         // output normal Green LED blink speed
  }
  if(!(BBShutdown))               // if BBShutdown input pin pulled low 
  {
    Beagle_ON_OFF(200);           // change output signal on pin 8.2
    GREEN_LED_blink(200);         // change Green LED blink speed
  }
  if(!(CCPOL))                    // if CCPOL pin pulled low 
  {
    Beagle_ON_OFF(100);           // change output signal on pin 8.2
    GREEN_LED_blink(100);         // change Green LED blink speed
  }
  if(!(CCINT))                    // if CCINT pin pulled low 
  {
    Beagle_ON_OFF(50);            // change output signal on pin 8.2
    GREEN_LED_blink(50);          // change Green LED blink speed
  }
  
  RED_LED_blink(); 
}

