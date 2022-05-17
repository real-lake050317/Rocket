.device ATmega328P

.equ PORTB = 0x05
.equ PORTD = 0x0b
.equ PIND  = 0x09
.equ DDRB  = 0x04
.equ DDRD  = 0x0a

.org 0x0000
    jmp main

main:
    sbi DDRB, 2
    sbi DDRB, 4
    sbi DDRD, 5
    cbi DDRD, 2
    clr r20

check_press_loop:
    sbis PIND, 2
    rjmp check_press_loop
    rjmp toggle_leds

check_release_loop:
    sbic PIND, 2
    rjmp check_release_loop
    rjmp check_press_loop

toggle_leds:
    tst r20
    brne off_pins
    sbis PORTB, 4
    rjmp set_pin_12 ; if pin 12 is not on, set it
    sbis PORTB, 2
    rjmp set_pin_10 ; if pin 10 is not on, set it
    sbis PORTD, 5
    rjmp set_pin_5 ; if pin 5 is not on, set it
    
all_on:
    ldi r20, 1
    rjmp check_release_loop

off_pins:
    sbic PORTD, 5
    rjmp off_pin_5
    sbic PORTB, 2
    rjmp off_pin_10
    sbic PORTB, 4
    rjmp off_pin_12

all_off:
    ldi r20, 0
    rjmp check_release_loop

set_pin_12:
    sbi PORTB, 4
    rjmp check_release_loop

off_pin_12:
    cbi PORTB, 4
    rjmp all_off

set_pin_10:
    sbi PORTB, 2
    rjmp check_release_loop

off_pin_10:
    cbi PORTB, 2
    rjmp check_release_loop

set_pin_5:
    sbi PORTD, 5
    rjmp all_on

off_pin_5:
    cbi PORTD, 5
    rjmp check_release_loop