/*
    Embedded Systems Lab-4
    Burak YORUK
 */

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "LCDmodule.h"

void PrintByte(char *,char *,char); // PrintByte function prototype

int main(void)
{
    DDRB |= 0xFF;   //Port B OUT
    DDRD = 0xFC;   //0-1 input | 4-5 output

    //Declare the character array for one LCD line:
    char LCDtext[16];
    unsigned char current,previous;
    unsigned char Abyte = 0;
    unsigned char flag =0;

    LCD_Init();     //4ms
    LCD_Clear();    //4ms
    previous = PIND;    //D portu herhangi pin
    //PORTx X e bağlı tüm pinlerden oluşuyor | PINx ilgili porta bağlı bitleri ayarlıyor

    LCD_MoveCursor(1,1);        // Place LCD cursor at column-1 of line-1:
    LCD_WriteString("Atten = 0");        // Send LCDtext to the LCD module:

    while(1){
        PORTD |= _BV(PORTD4);
        current = PIND;
        //Active Low oldugu icin SW basildiginda deger 0 oluyor.
        // if button pressed SW0 previous basili degil ve current basili
        if (( current & _BV(PIND0) ) == 0x00 && ( previous & _BV(PIND0)  ) == 0x01 )
        {
            if(Abyte > 1){
                Abyte--;
                flag = 1;
            }
        }
            //SW1 bir kere basili
        if (( current & _BV(PIND1) ) == 0x00 && ( previous & _BV(PIND1)  ) == 0x02 )
        {
            if(Abyte < 11){
                Abyte++;
                flag = 1;
            }
        }

//
//        PORTD |= _BV(PORTD4);
//        PrintByte(LCDtext, "Atten=", Abyte);
//        PORTD &= ~(_BV(PORTD4));
//
//        PORTD |= _BV(PORTD5);
//        LCD_MoveCursor(1,1);
//        LCD_WriteString(LCDtext);
//        PORTD &= ~(_BV(PORTD5));

        if(flag == 1){

//            PORTD |= _BV(PORTD4);   // _BV(.) mask 0b00010000
            PrintByte(LCDtext, "", Abyte);  // 6us
//            PORTD &= ~(_BV(PORTD4));
            PORTD |= _BV(PORTD5);
            LCD_MoveCursor(1,7);
            LCD_WriteString(LCDtext);

            flag = 0;
            PORTD &= ~(_BV(PORTD5));
        }

        previous = current;
        _delay_us(500);
        PORTD &= ~(_BV(PORTD4));

    };

    return 0;
}


// Change the value of a one-byte variable (i.e. Atten) between 1 and 11 with two
//push-button switches:
// Decrement Atten (if Atten > 1) every time SW0 is pressed.
// Increment Atten (if Atten < 11) every time SW1 is pressed.
