/*
    Embedded Systems Lab-5
    @author         :   Burak YORUK
    @Date           :    13.12.2018
    @Application    :   Timers and Interrupts
    @version        :   v1.0
    @MCU            :   Atmega328p
    @Crystal        :   8MHz
    @Instruction    :   Use timers to generate a Pulse Width Modulation (PWM) waveform and to
perform certain tasks in an Interrupt Service Routine (ISR) with precise timing.

 */

#include <avr/io.h>//#define second
#include <stdio.h>
#include <util/delay.h>
#include<avr/interrupt.h>
#include "LCDmodule.h"

void PrintByte(char *,char *,char); // PrintByte function prototype

ISR(TIMER1_COMPA_vect){
    PORTD |= _BV(PORTD4);   //PD4 output set after interrupt

    char LCDtext[16];
    static unsigned char current = 0,previous = 0;
    static unsigned char Abyte = 0, Atten = 0;
    current = PIND;

    PORTD |= _BV(PORTD5);
    ADCSRA |=   1 << ADSC ; //Start Conversion 13 cycle ((ADCSRA) &(1<<ADSC)) == (1 << ADSC)
    while((ADCSRA & _BV(ADSC)) != 0);
    PORTD &= ~_BV(PORTD5);

    Abyte = ADCH;
    if (( current & _BV(PIND0) ) == 0x00 && ( previous & _BV(PIND0)  ) == 0x01 )  //Active Low oldugu icin SW basildiginda deger 0 oluyor.if button pressed SW0 previous basili degil ve current basili
    {
        if(Atten > 1) Atten--;
    }
    if (( current & _BV(PIND1) ) == 0x00 && ( previous & _BV(PIND1)  ) == 0x02 ) //SW1 bir kere basili
    {
        if(Abyte < 6) Atten++;
    }
    //second part
    if(Atten != 0)
        Abyte = (Abyte>>(Atten-1));
    else{
        Abyte = Abyte;
    }

    PrintByte(LCDtext, "", Abyte);  // 220
    LCD_MoveCursor(1,1);  // Place LCD cursor at column-1 of line-1:
    LCD_WriteString(LCDtext);
    previous = current;

    OCR0A = Abyte;           // set timer0 pwm as ADC output
    PORTD &= ~_BV(PORTD4);   // PD4 clear before recall used as trigger
    }


int main(void)
{
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
    TIMSK1 |=  _BV(OCIE1A); //Enable Interrupt
    OCR1A  = 0x03E8;    //For 1ms
//    OCR0A = 0x
    // ## ADC init
    ADMUX  |= (1<<REFS0) | (1<<ADLAR);
    ADMUX  |= (1<<MUX0);    // select ADC1 as source
    ADCSRA |= (1<<ADEN);
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1);

    LCD_Init();     //4ms
    LCD_Clear();    //4ms

    sei();

    while(1);

    return 0;
}

//PORTx X e bagli tüm pinlerden oluþuyor | PINx ilgili porta bagli bitleri ayarlýyor
// #define _BV(bit) (1 << (bit))    kutuphanesinde yapılmıs
