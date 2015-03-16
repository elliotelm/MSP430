
// Firmware by Elliot Marcus
// MSP430g2553

// include the SPI library
// include I/O for GPIOs
#include <SPI.h>
#include <io.h>
#include <msp430g2553.h>

// P1_1 SPI MISO output
// P1_2 SPI MOSI input
// P1_3 
// P1_4 SPI CLK input
// P1_5 SPI enable input 

#define TRUE 1
#define FALSE 0

#define BB_GP_ISR P2_3      // output pin 2.3 TO BB
#define BB_ON_OFF P2_4      // output pin 2.4 TO 5V boost/BB
#define BB_Shutdown P2_5    // input pin 2.5 FROM BB

#define CC_SHTDWN P2_0      // output pin 2.0 TO CC
#define CC_POL P2_1         // input pin 2.1 FROM CC
#define CC_INT P2_2         // input pin 2.2 FROM CC

#define Battery_Max_Charge 64000     // 
#define Battery_Min_Charge 0         // 
#define Battery_BB_Min 10            // lowest battery percentage that BeagleBone can be turned on at
#define Battery_Low_Threshold 5      // absolute lowest battery percentage allowed 
#define Charge_Step 1                // increment for when CC shows battery charged
#define Discharge_Step 1             // increment for when CC shows battery discharged

volatile double CC_Fire = 500;                      // battery charge 
volatile uint8_t Battery_Level = 100;               // battery charge percentage
volatile uint8_t BB_Power = FALSE;                  // treat as boolean, TRUE = BeagleBone is ON, FALSE = BeagleBone is OFF
volatile uint8_t CC_Shutdown = FALSE;               // treat as boolean, TRUE = Shutdown coulomb counter, FALSE = Turn coulomb counter back on

// initializations run here
void setup()
{
  WDTCTL = WDTPW + WDTHOLD;                         // Disable watch dog timer
  clock_init();                                     // initialize clocks/timers
  
  // initialize pins and default pin states
  pinMode(BB_GP_ISR, OUTPUT);                       // output pin 2.3 TO BB
  pinMode(BB_ON_OFF, OUTPUT);                       // output pin 2.4 TO 5V boost/BB
  pinMode(CC_SHTDWN, OUTPUT);                       // output pin 2.0 TO CC
  
  pinMode(BB_Shutdown, INPUT);                      // input pin 2.5 FROM BB
  pinMode(CC_POL, INPUT);                           // input pin 2.1 FROM CC
  pinMode(CC_INT, INPUT);                           // input pin 2.2 FROM CC
  
  digitalWrite(BB_GP_ISR, LOW);                     // avoid damage to BeagleBone keep output low during powerup
  digitalWrite(BB_ON_OFF, LOW);                     // avoid damage to BeagleBone keep output low during powerup
  digitalWrite(CC_SHTDWN, HIGH);                    // turn on coulomb counter
  
  spi_init();                                        // initialize spi
 
  P2IE |= BIT2;                                     // P2.2 interrupt enabled, CC_INT
  P1IES |= BIT2;                                    // P2.2 Hi/lo edge (fires on hi/lo transition)
  P1IFG &= ~BIT2;                                   // P2.2 IFG cleared, clear the flag
}

//*****************************************************************************************
// spi_init
// SPI to run in slave mode, BeagleBone is master
//*****************************************************************************************
void spi_init(void)
{
// TODO fix the SPI init to setup as a slave to the BB
// start the SPI library
SPI.begin(); 

UCA0CTL1 = UCSWRST;                                 // hold in reset
                                                    // 4-pin, 8-bit SPI slave, STE (slave enable) active high
UCA0CTL0 |= UCCKPH + UCMSB + UCMODE_1 + UCSYNC;     // Sync. Mode: Clock Phase, Async. Mode: MSB first  0:LSB / 1:MSB, Sync. Mode: USCI Mode: 1, Sync-Mode  0:UART-Mode / 1:SPI-Mode
// UCxCLK is always used in slave mode
IE2 |= UCA0RXIE;                                    // USCI_A0 receive interrupt enable
//IE2 |= UCA0TXIE;                                  // USCI_A0 transmit interrupt enable
UCA0CTL1 &= ~UCSWRST;                               // release reset
}// spi_init

//*****************************************************************************************
// clock_init
// sets clock and timer counter A compare interrupt times
// The external 32768 Hz crystal oscillator is divided by 8 twice, once by DIVA_3 and again
// by ID_3. New clock is 512 Hz
//*****************************************************************************************
void clock_init(void)
{
BCSCTL1 |= DIVA_3;                                  // Basic Clock System Control 1,  set  ACLK Divider 3: /8 
BCSCTL3 |= XCAP_3;                                  // Basic Clock System Control 3,  set  XIN/XOUT Cap : 12.5 pF 
CCTL0 = CCIE;                                       // Timer A Capture/Compare Control 0  set  Capture/compare interrupt enable 
CCR0 = 511;                                         // Timer A Capture/Compare 0,  (512-1)==> 1 second, (30720-1)==> 1 minute
TACTL = TASSEL_1 + ID_3 + MC_1;                     // Timer A Control,  set  Timer A clock source select: 1 - ACLK, Timer A input divider: 3 - /8 ,Timer A mode control: 2 - Continous up 
_BIS_SR(GIE);			                    // Active mode w/ interrupt
}// clock_init

//*****************************************************************************************
// Batt_Low_Check
// checks if the battery level(percentage) is below allowable limit (battery protection use only)
//*****************************************************************************************
void Batt_Low_Check()
{
 if (Battery_Level <= Battery_Low_Threshold)
   {
     CC_Shutdown = TRUE;                           // CC needs to be shutdown
     CC_Pow_Check();                               // turn off coulomb counter
     digitalWrite(BB_ON_OFF, LOW);                 // turn off 5V boost/BeagleBone  
   }
}// Batt_Low_Check

//*****************************************************************************************
// BB_Pow_Check
// checks if the BeagleBone is currently powered ON or OFF (Did the BeagleBone shutdown?)
//*****************************************************************************************
void BB_Pow_Check()              
{
 if (BB_Shutdown)                                  // BeagleBone holds pin high to indicate it is on
   {
     BB_Power = TRUE;                              // Beaglebone is ON
   }
 if (!(BB_Shutdown))                               // BeagleBone holds pin high to indicate it is on
   {
     BB_Power = FALSE;                             // Beaglebone is OFF
   }
}// BB_Pow_Check

//*****************************************************************************************
// CC_Pow_Check
// checks if the coulomb counter should be shutdown 
//*****************************************************************************************
void CC_Pow_Check()              
{
 if (CC_Shutdown)                                 // the battery has gone dangerously low
   {
     digitalWrite(CC_SHTDWN, LOW);                // turn off coulomb counter
   }
}// CC_Pow_Check

//*****************************************************************************************
// timer/counter A0 ISR
// This ISR is fired everytime the value of CCRO is reached, currently setup as 
// an RTC (Real Time Clock), fires once a second
//*****************************************************************************************
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
  static uint16_t ISRA0count = 0;                                   // counts number of times ISRA0 fires
  ISRA0count++;                                                     // count timerA0 overflows
  
  //if((ISRA0count % 3600) == 0)                                     // if 1 hour has passed
  if((ISRA0count % 60) == 0)                                        // if 1 min. has passed (remove for final firmware)
  {
    if ((!(BB_Power)) && (Battery_Level >= Battery_BB_Min))         // BB is OFF and Battery has enough power for measurements
    {
      digitalWrite(BB_ON_OFF, HIGH);                                // turn on the 5V boost/BeagleBone
      ISRA0count = 0;                                               // reset ISR count 
    }
  }
  
  if((ISRA0count % 60) == 0)                                          // if 1 min. has passed 
  {
    if ((Battery_Level > Battery_Low_Threshold) && (CC_Shutdown))     // CC is OFF and Battery is above absolute minimum
      {
        CC_Shutdown = FALSE;                                          // coulomb counter doesn't need to be turned off
        digitalWrite(CC_SHTDWN, HIGH);                                // turn on coulomb counter
      }
    if (!(BB_Power))                                                  // BB shutdown on its own and is off 
      {
        digitalWrite(BB_ON_OFF, LOW);                                 // turn off 5V boost
      }
    if (BB_Power)                                                     // if BB is on
      {
        digitalWrite(BB_GP_ISR, HIGH);                                // tell BB to turn itself off
      }
    Batt_Low_Check;                                                   // check to see if the battery is dangerously low
  }
  digitalWrite(BB_GP_ISR, LOW);                                       // turn off outputs to BB for safety
  BB_Pow_Check();                                                     // check to see if the BB is On or Off
     
 // TODO calculation for the situation in which battery went low, coulomb counter is off (need to manually adjust battery level for ?? hour(s) of charge)
}//timer/counter A0 ISR

//*****************************************************************************************
// PORT2 ISR
// This ISR is fired everytime the coulomb counter has a high to low transition indicating
// that 0.0001707 amp hours (.1707 mAh) or .614 coulombs passed through the system in either 
// direction.
// 3600 mAh battery 
//*****************************************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{    
  // TODO update the battery charge variables here
  if (CC_POL)                                                      // coulomb counter indicates charging
    {
      CC_Fire += Charge_Step;                                      // increment the battery charge
      // Battery_Level = ??                                        // calculate the battery charge percentage
    }
  else
    {
      CC_Fire -= Discharge_Step;                                   // decrement the battery charge
      // Battery_Level = ??                                        // calculate the battery charge percentage
    }
   UCA0TXBUF = Battery_Level;                                      // keep the battery level updated in the SPI data register
   P2IFG &= ~BIT2;                                                 // P2.2 IFG (interrupt flag) cleared 
}// PORT2 ISR

//*****************************************************************************************
// SPI ISR
// receive 
//*****************************************************************************************
// TODO handle SPI with an ISR
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR (void)
{
// UCA0TXBUF is read by the BeagleBone, the interrupt is cleared 
}
// transfer should not be necessary for slave
/*
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
while(!(IFG2 & UCA0TXIFG));    // USCI_A0 TX  buffer  ready (no interrupt flag) and input test not in progress
UCA0TXBUF = 0xAA;              // Send 0xAA over SPI to Slave
while(!(IFG2 & UCA0RXIFG));    // USCI_A0 RX  Received (no interrupt flag)
received_data  =  UCA0RXBUF;   // Store received data
}
*/

// MAIN
// spend as much time as possible doing nothing otherwise everything else is handled by ISRs
void loop()
{
}
