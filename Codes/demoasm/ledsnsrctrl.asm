#define __SFR_OFFSET 0
 
#include "avr/io.h"

.global start
.global forever

start:
    SBI DDRB, 0x02
    SBI DDRB, 0x03
    CBI DDRB, 0x04
    SBI PORTB, 0x04
    RET

forever:
L2: SBIS PINB, 0x04
    RJMP L1
    SBI PORTB, 0x02
    CBI PORTB, 0x03
    SBIC PINB, 0x04
    RJMP L2
L1: SBI PORTB, 0x03
    CBI PORTB, 0x02
    RET
