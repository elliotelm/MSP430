
// Test Code by Elliot Marcus
// MSP430g2553

// include the SPI library
// include I/O for GPIOs
#include <SPI.h>
#include <io.h>
#include <msp430g2553.h>

#define RED_LED P1_0        // output dev board red LED
// P1_1 SPI MISO
// P1_2 SPI MOSI 
// P1_3 
// P1_4 SPI CLK
// P1_5 SPI test 

#define BB_GP_ISR P2_3      // output dev board pin 2.3
#define BB_ON_OFF P2_4      // output dev board pin 2.4
#define BB_Shutdown P2_5    // input dev board pin 2.5

#define CC_SHTDWN P2_0      // output dev board pin 2.0
#define CC_POL P2_1         // input dev board pin 2.1
#define CC_INT P2_2         // input dev board pin 2.2

volatile uint8_t val = 0xAA;// variable for SPI test
volatile char received_data = 0;


void setup()
{
  // initialize pins and default pin states
  pinMode(RED_LED, OUTPUT);
  pinMode(BB_GP_ISR, OUTPUT);
  pinMode(BB_ON_OFF, OUTPUT);
  pinMode(CC_SHTDWN, OUTPUT);
  pinMode(BB_Shutdown, INPUT_PULLUP);
  pinMode(CC_POL, INPUT_PULLUP);
  pinMode(CC_INT, INPUT_PULLUP);
  digitalWrite(RED_LED, LOW);
  // start the SPI library
  SPI.begin();                
}

void RED_LED_blink(int ledon)
{
 digitalWrite(RED_LED, HIGH);     // turn the LED on (HIGH is the voltage level)
 delay(ledon);                    // wait for ledon
 digitalWrite(RED_LED, LOW);      // turn the LED off by making the voltage LOW
 delay(ledon);                    // wait for ledon
}
void Beagle_GP_ISR()              // uC output
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

void Beagle_ON_OFF(int ontime)    // uC output
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

void Coulomb_SHTDWN()             // uC output
{
 digitalWrite(CC_SHTDWN, HIGH);   // turn the pin on (HIGH is the voltage level)
 delay(20);                       // wait for 20ms
 digitalWrite(CC_SHTDWN, LOW);    // turn the pin off by making the voltage LOW
 delay(20);                       // wait for 20ms
 digitalWrite(CC_SHTDWN, HIGH);   // turn the pin on (HIGH is the voltage level)
 delay(20);                       // wait for 20ms
 digitalWrite(CC_SHTDWN, LOW);    // turn the pin off by making the voltage LOW
 delay(20);                       // wait for 20ms
}


// MAIN
void loop()
{
// SPI init
// WDTCTL = WDTPW + WDTHOLD;                        // Disable watch dog timer

P1OUT |= BIT5;                                      // make p1.5 an output
P1DIR |= BIT5;                                      // set p1.5 high
P1SEL = BIT1 | BIT2 | BIT4;                         // Port 1 select UCA0SOMI, UCA0SIMO, UCA0CLK // to add UCA0STE need to change p1.5 from a GPIO and add to P1SEL & P1SEL2 BIT5
P1SEL2 = BIT1 | BIT2 | BIT4;                        // takes 2 select bits to set UCA0SOMI, UCA0SIMO, UCA0CLK

UCA0CTL1 = UCSWRST;                                 // hold in reset
UCA0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;        // 3-pin,  8-bit   SPI master
// UCA0CTL0 |= UCCKPH + UCMSB + UCMODE_1 + UCSYNC;  // 4-pin, 8-bit SPI slave, STE (slave enable) active high
UCA0CTL1 |= UCSSEL_2;                               // SMCLK
// UCxCLK is always used in slave mode
UCA0BR0 |= 0x02;                                    // baud rate
UCA0BR1 = 0;                                        // baud rate
UCA0MCTL = 0;                                       // No modulation
UCA0CTL1 &= ~UCSWRST;                               // release reset

P1OUT &= (~BIT5);                                   // Select device
/*
while(!(IFG2 & UCA0TXIFG));    // USCI_A0 TX  buffer  ready (no interrupt flag) and input test not in progress
UCA0TXBUF = 0xAA;              // Send 0xAA over SPI to Slave
while(!(IFG2 & UCA0RXIFG));    // USCI_A0 RX  Received (no interrupt flag)
received_data  =  UCA0RXBUF;   // Store received data
*/
  
  UCA0TXBUF = val;                // Send 0xAA over SPI
  SPI.transfer(UCA0TXBUF);        // transfer data 
  val += 1;                       // increment the value by 1
  if(val > 0xAF)                  // just cycle a few values
  {
   val = 0xAA;
  }
 P1OUT |= (BIT5);                                   // Unselect device
  

  
 Beagle_GP_ISR();                                   // output BeagleBone GP ISR pin 2.3
 Coulomb_SHTDWN();                                  // output CC_SHTDWN pin 2.0
 Beagle_ON_OFF(20);                                 // output BB_ON_OFF pin 2.4
 
 volatile int BBShutdown = (P2IN & BIT5);           // sets variable to pin register value p2.5
 volatile int CCPOL = (P2IN & BIT1);                // sets variable to pin register value p2.1
 volatile int CCINT = (P2IN & BIT2);                // sets variable to pin register value p2.2

  if(BBShutdown && CCPOL && CCINT)// none of the inputs are pulled low
  {
    Beagle_ON_OFF(400);           // output normal output signal on pin 2.4
    RED_LED_blink(400);           // output normal RED LED blink speed
  }
  if(!(BBShutdown))               // if BBShutdown input pin pulled low p2.5
  {
    Beagle_ON_OFF(200);           // change output signal on pin 2.4
    RED_LED_blink(200);           // change RED LED blink speed
  }
  if(!(CCPOL))                    // if CCPOL pin pulled low p2.1
  {
    Beagle_ON_OFF(100);           // change output signal on pin 2.4
    RED_LED_blink(100);           // change RED LED blink speed
  }
  if(!(CCINT))                    // if CCINT pin pulled low p2.2
  {
    Beagle_ON_OFF(50);            // change output signal on pin 2.4
    RED_LED_blink(50);            // change RED LED blink speed
  }
 
 }
