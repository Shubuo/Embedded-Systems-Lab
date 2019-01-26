/*
    Experiment 3 - Burak YÖRÜK
 */

#include <avr/io.h>
#include <util/delay.h>

void lab_delay()
{
    int i;
    for(i=0;i<999;i++)
        __asm__ volatile ( "NOP" );
}

int main(void)
{
    unsigned char current,previous;
    unsigned char counter = 0;

    DDRB |= 0xFF;
    DDRD &= 0xFC;   //input ise and li taným daha doðru
    PORTB = 0x01;

    previous = PIND;

    while(1)
    {
         current = PIND;
//         _delay_ms(1);
        // current ve previous da basılı gözüküyorsa
        if(( current & _BV(PIND0) ) == 0x00 ) //&& ( previous & _BV(PIND0) ) == 0x00
        {
            if(counter ==250){
                current = 0x00;
                previous = 0x01;    //önceki basılı değil
                counter = 0;
            }else{
                 counter++;
            }
        }

        //Active Low oldugu icin SW basildiginda deger 0 oluyor.
        // if button pressed SW0 previous açık ve current kapalı
        if( ( current & _BV(PIND0) ) == 0x00 && ( previous & _BV(PIND0)  ) == 0x01)
            {
            if((PIND & _BV(PIND1)) == 0x00){    //SW1 basili
                if (PORTB == 0x01){ //Bit position 0 check
//                    PORTB = 0x01;
//                    PORTB = 0x01;
                    PORTB = 0x80;   //Reload rotation
                     }
                else //eðer led yaniyorsa (gerekli pin 1 ise )bir bit kaydýracak
                    PORTB = PORTB >> 1 ;    //Right shift
                }
            else{                               //SW1 basili degil
                 if(PORTB == 0x80){             // Bit position 7
//                    PORTB = 0x80;
//                    PORTB = 0x80;
                     PORTB = 0x01;              //Reload rotation
                }
                else
                    PORTB = PORTB << 1 ;    //Left Shift
            }
        }

        previous = current;
        lab_delay();
    }
    return 0;
}

/*  12.5 cyc    (6*0,125)= 0,75us + (6.5*0,125) = 0,8125us * (x = 999,07) = 1000us
00000080 <lab_delay>:
  80:	80 e0       	ldi	r24, 0x00	; 0         1
  82:	90 e0       	ldi	r25, 0x00	; 0         1   for
  84:	00 00       	nop                         1
  86:	01 96       	adiw	r24, 0x01	; 1     2   add immidiate to word
  88:	23 e0       	ldi	r18, 0x03	; 3         1
  8a:	8f 31       	cpi	r24, 0x1F	; 31        1
  8c:	92 07       	cpc	r25, r18                1
  8e:	d1 f7       	brne	.-12     	; 0x84 <lab_delay+0x4>  0.5 branch if not equal
  90:	08 95       	ret                         4
*/
