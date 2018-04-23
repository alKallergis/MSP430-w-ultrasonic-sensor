//Copyright (c) 2017 Alex Kallergis

//LCD code based on:
//Copyright (c)	http://karuppuswamy.com/wordpress/2015/03/12/msp430-launchpad-interface-with-16x2-lcd-display/

//ultrasonic sensor code based on:
//Copyright (c) http://www.instructables.com/id/Ultrasonic-Sensor-with-MSP430-and-IARCCS/

//based on examples from:
// Copyright (c) 2012, Texas Instruments Incorporated
// All rights reserved.

/*This is a test project incorporating an msp430g2553,a 16x2 LCD, a servo and an ultrasonic sensor. Distance measurements
 * are taken from the ultrasonic sensor by triggering it in the Trigger pin and then waiting for a high-edge triggered
 * interrupt from the Echo pin. Timer 1 is used for time measurements in this part.
 * Also timer 0 is used to create the appropriate PWM signal for the servo on P1.2. TA0CCR1 is changed in a loop
 * with values from the distance sensor,thus the servo moves as the distance changes.
 *The distance is also shown on  the LCD.


 * servo connection:
 * P1.2
 *
 * untrasonic sersor connections:
 * Echo<---->p1.6
 * Trigger<---->p1.1

*uC and LCD Connections
*	TP1 - Vcc (+5v)
*	TP3 - Vss (Gnd)
*	P1.0 - D4
*	P1.1 - D5
*	P1.2 - D6
*	P1.3 - D7
*	P1.4 - EN
*	P1.5 - RS
*	Gnd  - RW
*	Gnd  - Vee through 1K Resistor	- this value determines contrast - i.e. direct connection to Gnd means all dots displayed
*	Gnd  - K (LED-)
*	Vcc  - A (LED+) +5V
*	Clock: 1MHz             */

#include <msp430g2553.h>

// uC Port definitions
#define lcd_port    	P2OUT
#define lcd_port_dir 	P2DIR

// LCD Registers masks based on pin to which it is connected
#define LCD_EN     	BIT4
#define LCD_RS      BIT5

void lcd_reset()
{
	lcd_port_dir = 0xff;	// output mode
	lcd_port = 0xff;
	__delay_cycles(20000);
	lcd_port = 0x03+LCD_EN;
	lcd_port = 0x03;
	__delay_cycles(10000);
	lcd_port = 0x03+LCD_EN;
	lcd_port = 0x03;
	__delay_cycles(1000);
	lcd_port = 0x03+LCD_EN;
	lcd_port = 0x03;
	__delay_cycles(1000);
	lcd_port = 0x02+LCD_EN;
	lcd_port = 0x02;
	__delay_cycles(1000);
}

void lcd_cmd (char cmd)
{
	// Send upper nibble
	lcd_port = ((cmd >> 4) & 0x0F)|LCD_EN;
	lcd_port = ((cmd >> 4) & 0x0F);

	// Send lower nibble
	lcd_port = (cmd & 0x0F)|LCD_EN;
	lcd_port = (cmd & 0x0F);

	__delay_cycles(2000);
}

void lcd_cmd_fast (char cmd)		//routine for fast commands,smaller delay used
{
	// Send upper nibble
	lcd_port = ((cmd >> 4) & 0x0F)|LCD_EN;
	lcd_port = ((cmd >> 4) & 0x0F);

	// Send lower nibble
	lcd_port = (cmd & 0x0F)|LCD_EN;
	lcd_port = (cmd & 0x0F);

	__delay_cycles(100);
}


void lcd_init ()
{
	lcd_reset();         // Call LCD reset
	lcd_cmd(0x28);       // 4-bit mode - 2 line - 5x7 font.
	lcd_cmd(0x0C);       // Display no cursor - no blink.
	lcd_cmd(0x06);       // Automatic Increment - No Display shift.
	lcd_cmd(0x80);       // Address DDRAM with 0 offset 80h.
	lcd_cmd(0x01);		 // Clear screen
}


void lcd_data (unsigned char dat)
{
	// Send upper nibble
	lcd_port = (((dat >> 4) & 0x0F)|LCD_EN|LCD_RS);
	lcd_port = (((dat >> 4) & 0x0F)|LCD_RS);
	
	// Send lower nibble
	lcd_port = ((dat & 0x0F)|LCD_EN|LCD_RS);
	lcd_port = ((dat & 0x0F)|LCD_RS);

	__delay_cycles(100); // a small delay may result in missing char display
}

void display_line(char *line)
{
	while (*line)
		lcd_data(*line++);
}


char *itoa (int value, char *result, int base)
{
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

void bubble_sort(long list[], int n)
{
  long t;
  int c,d;

  for (c = 0 ; c < ( n - 1 ); c++)
  {
    for (d = 0 ; d < n - c - 1; d++)
    {
      if (list[d] > list[d+1])
      {
        /* Swapping */

        t         = list[d];
        list[d]   = list[d+1];
        list[d+1] = t;
      }
    }
  }
}


#include <stdio.h>
#include <math.h>
volatile unsigned int k1=0,c0=0,c1=0,ADCPREV=0XFFFF;
int c=0,k=0;
float f;
double d;
char value[16];
int miliseconds;
int distance;
long sensor,sensor1=0,senslist[5];
int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
  BCSCTL1 = CALBC1_1MHZ;                    // Set range
  DCOCTL = CALDCO_1MHZ;                     // Set DCO step + modulation */
  BCSCTL2 &=~0x08;    					//smclk from dco
  P1DIR |= 0x5;                            // P1.0,P1.2 output
  P1SEL |= 0x04;                            // P1.2 option select
  P1OUT =  0x8;                            // P1.3 set, else reset
  P1REN |= 0x8;                            // P1.3 pullup
  P1IE |= 0x8;                             // P1.3 interrupt enabled
  P1IES |= 0x8;                            // P1.3 Hi/lo edge
  P1IFG &= ~0x8;                           // P1.3 IFG cleared
  CCTL0 = CCIE;
  CCTL1 = OUTMOD_7|CCIE;                         //
  CCR0 = 19999;
  CCR1 = 1900;
  TACTL = (TASSEL_2 + MC_1)|ID_0;                  // SMCLK, upmode,HALT
  //ADC10CTL0 = ADC10SHT_2 + ADC10ON + MSC; // ADC10ON, interrupt enabled,MULT CONVERS
 // ADC10CTL1 = INCH_4|ADC10DIV_0|CONSEQ_0;                       // input A3,S CHANNEL
 // ADC10AE0 |= 0x10;                         // PA.1 ADC option select
  TA1CCTL0 = CCIE;                             // CCR0 interrupt enabled
  TA1CCR0 = 1000;				    // 1ms at 1mhz
  TA1CTL = TASSEL_2 + MC_1;                  // SMCLK, upmode

  P1OUT&=~0x5;		//start with 0
 // ADC10CTL0 |= ENC|ADC10SC;             // Sampling and conversion start

  // Initialize LCD
  	lcd_init();
  	display_line("                 ");
  	__delay_cycles(1000000);
  	lcd_cmd(0x81);			//cursor @0,1
  	display_line("servo control");
  	lcd_cmd(0xC6);			//cursor @2,6
  	display_line("(mm)");
  __bis_SR_register(GIE);                // enable interrupts
  while(1)
  {
	    P1IE &= ~0x01;			// disable interupt
	  	P1DIR |= 0x02;			// trigger pin as output
	  	P1OUT |= 0x02;			// generate pulse
	  	__delay_cycles(10);             // for 10us
	  	P1OUT &= ~0x02;                 // stop pulse
	    P1DIR &= ~0x40;			// make pin P1.6 input (ECHO)
	    P1IFG = 0x00;                   // clear flag just in case anything happened before
	  	P1IE |= 0x40;			// enable interupt on ECHO pin
	  	P1IES &= ~0x40;			// rising edge on ECHO pin
	    __delay_cycles(3000);          //CHANGED FROM 30ms!!    delay for 30ms (after this time echo times out if there is no object detected)


	    senslist[c]=sensor;
	    c+=1;

	    if (c>=4) {			//list is full,proceed w/ calculations

		    c0=TA0R;		//timing counter. c1-c0=tdifference
	    	c=0;
	    	bubble_sort(senslist,4);
	    	sensor1=senslist[2];
		    distance = sensor1/58;           // converting ECHO lenght into cm
		    if(distance < 20 && distance != 0) P1OUT |= 0x01;  //turning LED on if distance is less than 20cm and if distance isn't 0.
		    else P1OUT &= ~0x01;
		    d=(sensor1*100.0/58.0); 		//?
		    if (d<1999){
		    CCR1=d;}
		    k=d/10;
		    c1=TA0R;			//timing counter
	    }
	    if (k1>=10)
	    {
	    k1=0;
	    lcd_cmd(0xc0);
	    sprintf(value,"%d",k);
	    //itoa(k,value,10);		//alternative
	    display_line("      ");
	    lcd_cmd_fast(0xc0);
	    //c0=TA0R;		//timing counter
	    display_line(value);
	    //c1=TA0R;			//timing counter
	    }
	    k1+=1;
  }
}

// Timer_A3 Interrupt Vector (TA0IV) handler
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A(void)

{
	if (TA0IV==TA0IV_TACCR1){
		//P1OUT&=~(0x01);
	}

}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{	//CCTL0&=~(CCIFG);    //not needed,flag is cleared automatically
	//P1OUT|=0x01;
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A1 (void)
{
  miliseconds++;
}


// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{if (P1IFG&0X8){

	TACTL ^=MC_1;			//timer A0 toggled(stopped-started)
	P1IFG &= ~0x8;                           // P1.3 IFG cleared
}
if(P1IFG&0x40)  //is there interrupt pending?
        {
          if(!(P1IES&0x40)) // is this the rising edge?
          {
            TA1CTL|=TACLR;   // clears timer A1
            miliseconds = 0;
            P1IES |= 0x40;  //falling edge
          }
          else
          {
            sensor = (long)miliseconds*1000 + (long)TA1R;	//calculating ECHO lenght

          }
	P1IFG &= ~0x40;				//clear flag
	}
}
