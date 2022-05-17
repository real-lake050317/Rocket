#define __SFR_OFFSET 0
 
#include "avr/io.h"

.global start
.global forever

start:
    SBI DDRB, 2
    SBI DDRB, 3
    CBI DDRB, 4
    SBI PORTB, 4
    RET

forever:
L2: SBIS PINB, 4
    RJMP L1
    SBI PORTB, 2
    CBI PORTB, 3
    SBIC PINB, 4
    RJMP L2
L1: SBI PORTB, 3
    CBI PORTB, 2 
    RET