/******************************************************************************
 *                          MSP430G2553 ACLK Test
 *
 * Description: This code can be used to test if you have installed the
 *              32.768kHz crystal on your LaunchPad correctly. Using this code
 *              LED1 (on P1.0) will turn on for 1 second, and off for one
 *              second. You can verify this with either an oscilloscope, logic
 *              analyzer, or by inspection. Simple as that!
 *
 * 				This code was originally created for "NJC's MSP430
 * 				LaunchPad Blog".
 *
 * Author:	Nicholas J. Conn - http://msp430launchpad.com
 * Modified by Elliot Marcus to use Energia.exe
 * Email:	webmaster at msp430launchpad.com
 * Date:	03-15-12
 ******************************************************************************/
 
#include  <msp430g2553.h>
void setup()
{
}
unsigned int currentMinutes, currentSeconds;
void loop()
{
	WDTCTL = WDTPW + WDTHOLD;			// Stop WDT
 
	BCSCTL1 |= DIVA_3;				// ACLK/8
	BCSCTL3 |= XCAP_3;				//12.5pF cap- setting for 32768Hz crystal
 
	P1DIR |= BIT0;					// set P1.0 (LED1) as output
	P1OUT |= BIT0;					// P1.0 low
 
	currentMinutes = 0;
	currentSeconds = 0;
 
	CCTL0 = CCIE;					// CCR0 interrupt enabled
	CCR0 = 511;					// 512 -> 1 sec, 30720 -> 1 min
	TACTL = TASSEL_1 + ID_3 + MC_1;			// ACLK, /8, upmode
 
	_BIS_SR(LPM3_bits + GIE);			// Enter LPM3 w/ interrupt
}
 
// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	P1OUT ^= BIT0;					// Toggle LED
}
