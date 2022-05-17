.device ATmega328P
.equ SREG = 0x3f
.equ RAMEND = 0x08ff
.equ SPL = 0x3d
.equ SPH = 0x3e
.equ EICRA = 0x69
.equ EIMSK = 0x1d
.equ DDRB  = 0x04
.equ DDRD  = 0x0a
.equ PORTB = 0x05
.equ PORTD = 0x0b

.org 0x0000
    jmp reset

.org 0x0002
    jmp pushed_button

pushed_button:
    tst r20
    brne off_pins
    sbis PORTB, 4
    rjmp set_pin_12
    sbis PORTB, 2
    rjmp set_pin_10
    sbis PORTD, 5
    rjmp set_pin_5

    all_on:
        ldi r20, 1
        reti

    off_pins:
        sbic PORTD, 5
        rjmp off_pin_5
        sbic PORTB, 2
        rjmp off_pin_10
        sbic PORTB, 4
        rjmp off_pin_12

    all_off:
        ldi r20, 0
        reti

    set_pin_12:
        sbi PORTB, 4
        reti

    off_pin_12:
        cbi PORTB, 4
        rjmp all_off

    set_pin_10:
        sbi PORTB, 2
        reti

    off_pin_10:
        cbi PORTB, 2
        reti

    set_pin_5:
        sbi PORTD, 5
        rjmp all_on
        
    off_pin_5:
        cbi PORTD, 5
        reti

reset:
    sbi DDRB, 2
    sbi DDRB, 4
    sbi DDRD, 5

    ldi r16, LOW(RAMEND)
    out SPL, r16
    ldi r16, HIGH(RAMEND)
    out SPH, r16

    lds r17, EICRA
    ori r17, 0b00000010
    sts EICRA, r17

    in r18, EIMSK
    ori r18, 0b00000001
    out EIMSK, r18

    sei

main: rjmp main