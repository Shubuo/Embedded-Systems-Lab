/*
    Embedded Systems Lab-8
    @author         :   Burak YORUK
    @Date           :   23.01.2019
    @Application    :   Serial Data Transmission
    @version        :   v1.0
    @MCU            :   Atmega328p
    @Crystal        :   8MHz
    @Instruction    :   Establish serial communication between the MCU and a personal computer.
    Utilize the USART module and the related interrupt functions to perform serial I/O operations.
    Optimize timing of serial I/O functions by using a circular queue buffer to store the data to be transmitted.
 */

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "LCDmodule.h"
#include <avr/interrupt.h>

#define LCDbufferSize    32
#define USARTbufferSize    32
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void PrintByte(char *,char *,char); // PrintByte function prototype

// Global variables for data transfer to Timer-2 ISR:
extern unsigned char  LCDbuffer[LCDbufferSize];
extern unsigned char  Nbyte;
unsigned char Atten = 1;    //Checking the switch inputs
unsigned char Abyte = 0,   prv = 0;
unsigned char LCDon = 0;

int main(void)
{
    DDRB |= 0xFF;                   //Port B OUT
    DDRD  = 0xF0;                   //0-1 input | 4-5 output 6(OC0A) da out
    DDRC  = 0xF0;
    PRR &= ~_BV(PRADC) | ~_BV(PRTIM0)| ~_BV(PRTIM1);

    // ## ADC init
    ADMUX  |= (1<<REFS0) | (1<<ADLAR);
    ADMUX  |= (1<<MUX0);    // select ADC1 as source
    ADCSRA |= (1<<ADEN);
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1);

	// #Timer 0 Init;  Psc = 100 | clk/256 = 31,250 times, COM0A =10, FastPWM WGM = 011
    TCCR0A |= 0b10000011 ;
    TCCR0B |= 0b00000001 ;  //_BV(CS22) | _BV(CS21); TCCR0B &= ~_BV(CS20);

   // ## set timer 1
    TCCR1B |= (1<<WGM12)  | (1<<CS11); //-> prescalar -> 8 CS11=1, WGM1[3:0] = 0100 ,
    TIMSK1 |= (1<<OCIE1A);  //enable interrupt for OCIE1A
    OCR1A  = 0x03E8;    //1000, For 1 ms

    // ## set timer 2
    TCCR2A |= (1<<WGM21);   //Clear Timer on Compare Match
    TCCR2B |= (1<<CS21);    //Pre-scalar ->8,
//    TIMSK2 |= (1<<OCIE2A);  //Tim2 Out Compare A match Interrupt Enable
    OCR2A   = 50;           //50 cycle 1us

    // ## set USART registers
    UBRR0   = BAUD_PRESCALE; // baud rate 9600 = 51
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);    //Rx-Tx Enable
    UCSR0B |= (1<<RXCIE0) | (1<<UDRIE0); // Receiver complete and Data register empty Interrupt Enable
    UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01); //= 011 for standard 8-bit data transmission

    LCD_Init();
    LCD_Clear();

    sei();

    while(1){
    PORTD |= _BV(PORTD5);   // Set timing marker.
    _delay_ms(50);
    PORTD &= ~_BV(PORTD5);  // Clear timing marker.
    _delay_ms(50);
    };

    return 0;
}

// ###          BUFFERA GONDER      ####

ISR(TIMER1_COMPA_vect){ //_VECTOR(11)
    PORTD |= _BV(PORTD4);   //PD4 output set after interrupt

    char LCDtext[16];
    static unsigned char Abyte = 0, counter = 0;
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

    PrintByte(LCDtext, "", Abyte);
    EnQueueLCDbuffer(0x80); // Go first Line of LCD
    pText = (unsigned char *)LCDtext;
    while (*pText != 0x00)
    {
        EnQueueLCDbuffer(*pText);
        pText ++;
    };

    // Her 10 ms de USART a GÖNDER
    if(counter == 10)
    {
//        PORTD |= _BV(PORTD4);
        pText = (unsigned char *)LCDtext;
        pText = (unsigned char *)LCDtext;
        EnQueueUSARTbuffer(0x20);   // delimiter space
        while (*pText != 0x00)
        {
            EnQueueUSARTbuffer(*pText); //Send ADC sample
            pText ++;
        };

        counter = 0;
//        PORTD &= ~_BV(PORTD4);
    }

    counter++;
    OCR0A = Abyte;   // set timer0 pwm as ADC output

    PORTD &= ~_BV(PORTD4);   // PD4 clear before recall used as trigger
}



// ###          LCD ye YAZDIR      ####


ISR(TIMER2_COMPA_vect){ //_VECTOR(7)
//    PORTD |= _BV(PORTD5); // Set second ISR timing marker.
    TCNT2 = 0;  //Timer-2 Counter
    DeQueueLCDbuffer(); //Bufferı LCD'ye yazdırıyoruz.
//    PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
}



// ###          Receiver Interrupt      ######



ISR(USART_RX_vect)
{

    static unsigned char prvAtten = 0, data;
    char LCDtext[16];
    unsigned char *pText;

    data = UDR0;    // UART Data Register
    // determine value of Atten
    if( ( data & 0xDF) == 0x55) // if received U ya da 117(u) de olur
        Atten = (Atten>1) ? Atten-1 : Atten;
    else if ( ( data & 0xDF) == 0x44)   //D ya da 100(d) de olur
        Atten = (Atten<11) ? Atten+1 : Atten;

    // if previous different from current Atten
    if(prvAtten != Atten)
    {
        PrintByte(LCDtext, "Atten= ", Atten);
        EnQueueLCDbuffer(0xC0); // LCD line2
        EnQueueUSARTbuffer(0x0D);   //Start New line on Serial Terminal
        pText = (unsigned char *)LCDtext;   //text out
        while (*pText != 0x00)
        {
            EnQueueLCDbuffer(*pText);   //LCD bufferına aldık yazdıracaklarımızı
            EnQueueUSARTbuffer(*pText);     //Usart bufferına aldık yazırdacaklarımızı
            pText ++;
        };
    }
    prvAtten = Atten;

}



// ###          Data Empty Interrupt     ######



ISR(USART_UDRE_vect)
{
//    PORTD |= _BV(PORTD5); // Set second ISR timing marker.
    DeQueueUSARTbuffer();   //Usart EnQ da interrupt enable yaparak bufferımızı porta yazırıyoruz.
//    PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
}



//PORTx X e bagli tüm pinlerden oluþuyor | PINx ilgili porta bagli bitleri ayarlýyor
// #define _BV(bit) (1 << (bit))    kutuphanesinde yapılmıs
