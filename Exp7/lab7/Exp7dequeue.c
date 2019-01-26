#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "LCDmodule.h"

#define LCDbufferSize    32

// Global variables for data transfer to Timer-2 ISR:
unsigned char  LCDbuffer[LCDbufferSize];
unsigned char  Nbyte = 0;
unsigned char   *EnQptr; // enqueue pointer
unsigned char   *DeQptr; // dequeue pointer
unsigned char   *EOBptr; // pointer to end of the buffer
unsigned short int  NBavail; // number of bytes available

//First member of all data structures involved in the queue operations
//specify the data length in bytes.
typedef struct
{ unsigned char         DTsize;     // data length in bytes(max. 255)
  unsigned short int    Member1;    // first member of data item
  signed long int       Member2;    // second member of data item
} MyDataStruct;

void InitQueue(void)
// Initializes enqueue and dequeue pointers.
{ EnQptr = LCDbuffer;
  DeQptr = LCDbuffer;
  EOBptr = LCDbuffer;
  EOBptr += LCDbufferSize;
  NBavail = LCDbufferSize;
}

void EnQueueLCDbuffer(unsigned char ByteIn)
{
    static unsigned char *pEnq = LCDbuffer;
    static unsigned char *pEOB = LCDbuffer+LCDbufferSize;

    if(Nbyte != LCDbufferSize)
    {
        *pEnq = ByteIn;
        pEnq++;
        Nbyte++;
        if(pEnq == pEOB)
            pEnq = LCDbuffer;
    }
    if(Nbyte == 1)
        TIMSK2 |= (1<<OCIE2A);  //Nbyte'ım 1 olduğu anda Timer-2 interrupt üretiyorum
}

void DeQueueLCDbuffer(void)
/* If there are queued data in LCDbuffer (Nbyte > 0), then sends a
   single byte to the LCD module. If the buffer is empty (Nbyte = 0),
   then returns without doing anything. */
   // Nbyte bize sırada kaç byte olduğunu söylüyor
    {
      static unsigned char  *pLCDout = LCDbuffer;   //byte pointer
      static unsigned char  Bcount = LCDbufferSize; //number of bytes
      unsigned char  ByteOut;                       //byte index

      if (Nbyte > 0)
    // First send the 4 most significant bits of the LCD data:
      { ByteOut = *pLCDout >> 4;
    // Set the register select bit, RS=1, when sending display data to LCD:
        if ( (*pLCDout & 0x80) == 0 )  ByteOut |= 0x10; //LCD cursor'ı first-line a gidiyor.
    // Write 4 MSBs:
        _LCDport = ByteOut; //PORTB'ye byteOut Yaz
        _LCDport = 0x40 | ByteOut;  // Set Enable
        _LCDport = ByteOut;         // Clear Enable
        ByteOut = (ByteOut & 0xF0) | (*pLCDout & 0x0F);
        _LCDport = ByteOut;
        _LCDport = 0x40 | ByteOut;  // Set Enable
        _LCDport = ByteOut;         // Clear Enable
        _LCDport = 0x00;  // clear port output
    // Update the LCDbuffer status:
        Bcount --;        // decrement buffer position counter
        if (Bcount == 0)  // check if reached end of buffer array
        { pLCDout = LCDbuffer;     // go back to the first element
          Bcount = LCDbufferSize;  // reload buffer position counter
        }
        else
          pLCDout ++;  // increment dequeue buffer pointer
        Nbyte --;      // decrement number of stored bytes
      }
        if(Nbyte == 0)  //Part 7
        TIMSK2 &= ~(1<<OCIE2A); //Timer-2 interrupt'ımı kapadım.
        //TIM Interrupt Mask Reg.Enable Interrupt Source
        //CTC modda OCR2A sayısına ulaşıldığında OCIE2A set olur
      return;
    }

//    // Pointer to void is compatible with pointer to any data type.
//unsigned char EnQueue(void *pDataIn)
///* Stores an item into the queue buffer.pDataIn points to the data to be queued.
//    Returned values are:
//    0 => enqueue operation is successful
//    1 => buffer overrun‐ no space available in the buffer
//*/
//    {
//    unsigned char NByte; // number of bytes in data item
//    unsigned char i; // byte index
//    unsigned char *Bptr; // byte pointer
//
//     NByte = *(unsigned char *)pDataIn; // get the number of bytes
//     if (NByte > NBavail) // check for buffer overrun
//     return (unsigned char)1; // buffer overrun error
//     else
//     { Bptr = (unsigned char *)pDataIn; //copy input pointer
//     for (i = Nbyte; i > 0; i--) //copy data from input location
//     { *EnQptr = *Bptr;
//     EnQptr ++; // increment enqueue pointer
//     if (EnQptr == EOBptr) //check if reached end of buffer
//          EnQptr =LCDbuffer; //go back to the first location
//          Bptr ++; //increment input pointer
//     }
//     NBavail -= Nbyte; //less bytes are available now
//     return (unsigned char)0; //enqueue operation is successful
//     }
//    } //end of function EnQueue
