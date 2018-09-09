;
; tinyblink
; Blinks an LED on pin 4 (PB2)
;

.DEVICE ATtiny10

; variables
.EQU delayMult1 = 0xff ; the delay is delay3*delaymult2*delaymult1 
.EQU delayMult2 = 0xff
.EQU delayMult3 = 0x0f

.CSEG ; code section
.ORG $0000 ; the starting address
main:
	; set up the stack
	ldi r16, high(RAMEND)
	out SPH, r16
	ldi r16, low(RAMEND)
	out SPL, r16
	
	; set clock divider
	ldi r16, 0x00 ; clock divided by 1
	ldi r17, 0xD8 ; the key for CCP
	out CCP, r17 ; Configuration Change Protection, allows protected changes
	out CLKPSR, r16 ; sets the clock divider

	; initialize port
	ldi r16, 1<<PB2 ; sets pin 4 (PB2) to putput
	out DDRB, r16 ; data direction
	ldi r16, 0x00 ; sets all pins low
	out PORTB, r16
	
	; nop for sync
	nop

loop:
	; turn on and delay
	ldi r16, (1<<PB2)
	out PORTB, r16
	rcall delay
	
	; turn off and delay
	ldi r16, 0x00
	out PORTB, r16
	rcall delay
	rjmp loop

delay:
	; not really needed, but keep r16-r18
	push r16
	push r17
	push r18
	
	ldi r16, delayMult1
	ldi r17, delayMult2
	ldi r18, delayMult3

	; start delay loop
delayLoop:
	subi r16, 1 ; subtract 1
	sbci r17, 0 ; if r16 was 0, subtract 1
	sbci r18, 0 ; if r17 was 0, subtract 1
	brne delayLoop ; while r18 is not 0, loop
	; end delay loop

	pop r17
	pop r16
	pop r18
	ret



