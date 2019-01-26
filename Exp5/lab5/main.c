/*
    Embedded Systems Lab-5
    @author         :   Burak YORUK
    @Date           :    22.11.2018
    @Application    :   Analog Input Output
    @version        :   v1.0
    @MCU            :   Atmega328p
    @Crystal        :   8MHz
    @Instruction    :   Sample an analog signal using the analog-to-digital converter (ADC)
available on the MCU.  Display the ADC results on a LCD.  Use an external digital-to-
analog converter (DAC) to obtain analog output.

 */
//#define first
#include <avr/io.h>//#define second

#include <stdio.h>
#include <util/delay.h>
#include "LCDmodule.h"

//// #define _BV(bit) (1 << (bit))    kütüphanede yapılmış

void PrintByte(char *,char *,char); // PrintByte function prototype

int main(void)
{
    DDRB |= 0xFF;                   //Port B OUT
    DDRD  = 0xFC;                   //0-1 input | 4-5 output
    DDRC  = 0b0 << DDC0 | 0 <<DDC1 ;  //input

	// ADC Init
    PRR = 0 << PRADC;
    ADCSRB = 0x00;
    DIDR0 = 1 << ADC0D | 1 << ADC1D ; //disables the selected digital input buffer to reduce power
    ADCSRA |=   1 << ADEN ; //Start ADC
    //Select PReScalar 110 for division 64, 8MHz / 64 = 125KHz
    ADCSRA |=   1 << ADPS2  | 1 << ADPS1 ;
    ADCSRA |=   0 << ADIE | 0 << ADATE;
//{ Bit Manipulation icin 2.yontem
//   ADCSRA |= 0x86; //0b1??? ?11? ADC Enable - APDS=0b110
//    ADCSRA &= 0xD6; //0b??0? 0??0 =>ADATE and ADIE (trigger and interrupt) is turned off, ADPS = 0b110 to obtain a clock frequency less than 200KHz with a 8MHz CPU clock
//}

    ADMUX &= 0b00111111;  //Selecting External AREF input
    ADMUX |= 1 << ADLAR ; //Left Adjust
	ADMUX |= 1 << MUX0  | 0 << MUX1 | 0 << MUX2 | 0 << MUX3 ;  //  ADC1   selected

	char LCDtext[16];
    unsigned char current,previous;
    unsigned char Abyte = 0, Atten = 0;
    previous = PIND;    //D portu herhangi pin
    //PORTx X e bagli tüm pinlerden oluþuyor | PINx ilgili porta bagli bitleri ayarlýyor
    LCD_Init();     //4ms
    LCD_Clear();    //4ms

    LCD_MoveCursor(1,1);
    LCD_WriteString("ADC Example");
    LCD_MoveCursor(2,1);
    LCD_WriteString("Atten= 0");

    while(1){
    PORTD |= _BV(PORTD4);
	current = PIND;
	ADCSRA |=   1 << ADSC ; //Start Conversion 13 cycle ((ADCSRA) &(1<<ADSC)) == (1 << ADSC)
	while((ADCSRA & _BV(ADSC)) != 0);
    Abyte = ADCH;

    if (( current & _BV(PIND0) ) == 0x00 && ( previous & _BV(PIND0)  ) == 0x01 )  //Active Low oldugu icin SW basildiginda deger 0 oluyor.if button pressed SW0 previous basili degil ve current basili
        {
            if(Atten > 1) Atten--;
        }
    if (( current & _BV(PIND1) ) == 0x00 && ( previous & _BV(PIND1)  ) == 0x02 ) //SW1 bir kere basili
        {
            if(Abyte < 6) Atten++;
        }
      second part
        if(Atten != 0)
            Abyte = (Abyte>>(Atten-1));
        else
            Abyte = Abyte;

          PORTB = Abyte;

	PrintByte(LCDtext, "", Abyte);  // 220
    LCD_MoveCursor(2,7);  // Place LCD cursor at column-1 of line-1:
    LCD_WriteString(LCDtext);

    previous = current;
	PORTD &= ~(_BV(PORTD4));
	_delay_us(670);

    };
    return 0;
}

