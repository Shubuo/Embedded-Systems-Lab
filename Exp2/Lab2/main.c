/*
    Experiment 2 - Burak YÖRÜK
 */

#include <avr/io.h>

int main(void)
{
    DDRB |= 0xFF;
    DDRD &= 0xFC;
    PORTB = 0x01;

    while(1)
    {
        //Active Low oldugu icin SW basildiginda deger 0 oluyor.
        if((PIND & _BV(PIND0)) == 0x00) {
            if((PIND & _BV(PIND1)) == 0x00){
                if (PORTB == 0x01){ //Bit position check
                    PORTB = 0x01;
                    PORTB = 0x01;
                    PORTB = 0x01;
                    PORTB = 0x01;
                    PORTB = 0x01;
                    PORTB = 0x80;   //Reload rotation
                     }
                else
                    PORTB = PORTB >> 1 ;    //Right shift
                }
            else{
                if (PORTB == 0x80){ //Bit position check
                    PORTB = 0x80;
                    PORTB = 0x80;
                    PORTB = 0x80;
                    PORTB = 0x01;
                }
                else
                    PORTB = PORTB << 1 ;    //Left Shift
            }
        }
    }
}
