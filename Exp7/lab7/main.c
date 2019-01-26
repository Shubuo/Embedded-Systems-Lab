/*
    Embedded Systems Lab-7
    @author         :   Burak YORUK
    @Date           :   21.12.2018
    @Application    :   Interrupts and Data Flow
    @version        :   v1.0
    @MCU            :   Atmega328p
    @Crystal        :   8MHz
    @Instruction    :   Send LCD commands in an ISR invoked by a timer interrupt generated every 50 us.
    The LCD commands are queued in a circular buffer in memory and the ISR sends the commands to LCD one at a time.
 */

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include<avr/interrupt.h>
#include "LCDmodule.h"

#define LCDbufferSize    32

void PrintByte(char *,char *,char); // PrintByte function prototype
void DeQueueLCDbuffer(void);
void EnQueueLCDbuffer(unsigned char ByteIn);

// Global variables for data transfer to Timer-2 ISR:
extern unsigned char  LCDbuffer[LCDbufferSize];
extern unsigned char  Nbyte;
unsigned char Atten = 1;    //Checking the switch inputs
unsigned char Abyte = 0,   prv = 0;
unsigned char LCDon = 0;


int main(void)
    {
    unsigned char current = 0,previous = 0;
    unsigned char *pText;
    char LCDtext[16];

    DDRB |= 0xFF;                   //Port B OUT
    DDRD  = 0xFC;                   //0-1 input | 4-5 output 6(OC0A) da out
    DDRC  = 0b0 << DDC0 | 0b0 <<DDC1 ;  //input
    PRR &= ~_BV(PRADC) | ~_BV(PRTIM0)| ~_BV(PRTIM1);

	// #Timer 0 Init;  Psc = 100 | clk/256 = 31,250 times, COM0A =10, FastPWM WGM = 011
    TCCR0A |= 0b10000011 ;
    TCCR0B |= 0b00000001 ;  //_BV(CS22) | _BV(CS21); TCCR0B &= ~_BV(CS20);
    // #Timer 1 Init;  Psc = 010 | clk/8 = 1Mhz, COM1A =00,  ClearTimeronCompareMatch(CTC) GM = 0100
    TCCR1A |= 0;
    TCCR1B |= 0b00001010 ;  //_BV(CS21); TCCR1B &= ~_BV(CS20) | ~_BV(CS22);
    TIMSK1 |=  _BV(OCIE1A); //Enable Interrupt-1
    OCR1A  = 0x03E8;    //= 1000, For 1ms

    // ## set timer 2
    TCCR2A |= (1<<WGM21);   //Clear Timer on Compare Match
    TCCR2B |= (1<<CS21);    //Pre-scalar ->8,
    TIMSK2 |= (1<<OCIE2A);  //Tim2 Out Compare A match Interrupt Enable
    OCR2A   = 50;           //50 cycle 1us
    // ## ADC init
    ADMUX  |= (1<<REFS0) | (1<<ADLAR);
    ADMUX  |= (1<<MUX0);    // select ADC1 as source
    ADCSRA |= (1<<ADEN);
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1);

    LCD_Init();     //4ms
    LCD_Clear();    //4ms

    sei();

    while(1)     //CHECK SWITCH
        {
        current = PIND;

        if((current&_BV(PIND0))==0x00&&(previous&_BV(PIND0))==0x01)  //Active Low oldugu icin SW basildiginda deger 0 oluyor.if button pressed SW0 previous basili degil ve current basili
          if(Atten > 1) Atten--;
        if((current&_BV(PIND1))==0x00&&(previous&_BV(PIND1))==0x02) //SW1 bir kere basili
          if(Abyte < 11) Atten++;

//        if(TIFR0 &= 0x2)
//            PORTD |= _BV(PORTD5); // Set second ISR timing marker.
//  Part 3
//        PrintByte(LCDtext, "", Atten);
//        LCDon = 1;
//        LCD_MoveCursor(2,1);
//        LCD_WriteString(LCDtext);
//        LCDon = 0;

        if(prv != Atten)
            {
            LCDon = 1;
            PrintByte(LCDtext, "", Atten);
            EnQueueLCDbuffer(0xC0); //192 - 12 karaktere denk  //replaced(line-2,colmun-1)
            // LCD_WriteString(LCDtext); // replaced by the following
            pText = (unsigned char *)LCDtext; // initialize pointer to output text
            // Copy text output to LCDbuffer:
            while (*pText != 0x00)
                {
                EnQueueLCDbuffer(*pText);
                pText ++;
                };
            LCDon = 0;
            }
        previous = current;
        prv = Atten;
        };
    return 0;
    }


ISR(TIMER1_COMPA_vect){ //_VECTOR(11)
    PORTD |= _BV(PORTD4);   //PD4 output set after interrupt

    char LCDtext[16];
    static unsigned char Abyte = 0;
    unsigned char *pText;

    ADCSRA |= 1 << ADSC ; //Start Conversion 13 cycle ((ADCSRA) &(1<<ADSC)) == (1 << ADSC)
    while((ADCSRA & _BV(ADSC)) != 0);
    Abyte = ADCH;

    //Adjust PWM  Output with Atten
    if(Atten != 0)
        Abyte = (Abyte>>(Atten-1));
    else{
        Abyte = Abyte;
        }

//    if(LCDon==0){
//    PrintByte(LCDtext, "", Abyte);  // 220
//    LCD_MoveCursor(1,1);  // Place LCD cursor at column-1 of line-1:
//    LCD_WriteString(LCDtext);

  if(LCDon == 0){
    PrintByte(LCDtext, "", Abyte);
    EnQueueLCDbuffer(0x80);     //128   -> 8bit | using for move cursor to first-line of LCD
    pText = (unsigned char *)LCDtext;
    while (*pText != 0x00){
        EnQueueLCDbuffer(*pText);
        pText ++;
        };
    OCR0A = Abyte;           // set timer0 pwm as ADC output
    PORTD &= ~_BV(PORTD4);   // PD4 clear before recall used as trigger
    }
}

ISR(TIMER2_COMPA_vect){ //_VECTOR(7)
    PORTD |= _BV(PORTD5); // Set second ISR timing marker.
    if(Nbyte != LCDbufferSize)
        DeQueueLCDbuffer();
    PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
    TCNT2 = 0;  //Timer-2 Counter
}

//PORTx X e bagli tüm pinlerden oluþuyor | PINx ilgili porta bagli bitleri ayarlýyor
// #define _BV(bit) (1 << (bit))    kutuphanesinde yapılmıs

