;
; PWM
; uses PWM to fade an LED on pin 1 (PB0)
; the fade follows a (sin(t)+1)/2 pattern
; that is stored in a table in memory
; 

.DEVICE ATtiny10

; variables
.EQU delayMult1 = 0xff ; the delay is delaymult2*delaymult1 
.EQU delayMult2 = 0x3f

.DEF sinhalf = r22 ; holds the current half of the wave
.DEF currentindex = r21 ; holds the current index in the table

.CSEG ; code section
.ORG $0000 ; the starting address
	rjmp main ; for the reset vector
	; no other interrupts should be enabled

; a table of 100 16-bit values representing an offset sin wave -pi/2 to pi/2
; they vary from 0 to 1023
sintable:
	.DW 0,0,1,2,4,6,9,12,16,20
	.DW 25,30,36,42,49,56,63,71,80,88
	.DW 98,107,117,128,139,150,161,173,185,198
	.DW 211,224,237,251,265,279,294,308,323,338
	.DW 353,369,384,400,416,431,447,463,479,495
	.DW 512,528,544,560,576,592,607,623,639,654
	.DW 670,685,700,715,729,744,758,772,786,799
	.DW 812,825,838,850,862,873,884,895,906,916
	.DW 925,935,943,952,960,967,974,981,987,993
	.DW 998,1003,1007,1011,1014,1017,1019,1021,1022,1023


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
	
	; set up timer and compare values
	; TCCR0A contains [COM0A1, COM0A0, COM0B1, COM0B0,-,-, WGM01, WGM00]
	; means: compare output mode(what pins do), wave mode
	;
	; TCCR0B contains [ICNC0, ICES0, -, WGM03, WGM02, CS02, CS01, CS00]
	; means: input capture noise cancel and edge select, wave mode, clock select
	;
	; TCCR0C contains [FOC0A, FOC0B, -,-,-,-,-,-]
	; means: force compare
	;
	; TIMSK0 contains [-,-, ICIE0,-,-, ICIE0B, ICIE0A, TOIE0]
	; means: interrupt enable
	;
	; TIFR0 contains [-,-, ICF0,-,-, OCF0B, OCF0A, TOV0]
	; means: interrupt flags
	;
	; GTCCR contains [TSM,-,-,-,-,-,-, PSR]
	; means: holds prescaler reset(halts count) , prescaler reset
	;
	; TCNT0 H:L is counter register
	; OCR0A H:L is compare register A
	; OCR0B H:L is compare register B
	; ICR0 H:L is input capture register
	;
	
	; set up PWM type and clock
	; this is 10-bit fast pwm with prescaler=1
	ldi r16, (1<<COM0A1)|(1<<WGM01)|(1<<WGM00)
	out TCCR0A, r16
	ldi r16, (1<<WGM02)|(1<<CS00)
	out TCCR0B, r16
	
	; get a value from the sin table
	ldi sinhalf, 0x00 ; start in the first half (increasing)
	ldi currentindex, 0x00 ; the first value
	rcall getsin ; puts value in 25:24
	
	; set the pwm duty cycle
	out OCR0AH, r25
	out OCR0AL, r24
	
	; set pin as output
	ldi r16, 1<<PB0 ; sets pin 1 (PB0) to putput
	out DDRB, r16 ; data direction
	
	; nop for sync
	nop

loop:
	; delay
	rcall delay
	
	; change the pwm duty cycle
	rcall getsin ; puts the value in 25:24
	out OCR0AH, r25
	out OCR0AL, r24
	
	nop
	rjmp loop

delay:
	push r16
	push r17
	
	ldi r16, delayMult1
	ldi r17, delayMult2

	; start delay loop
delayLoop:
	subi r16, 1
	sbci r17, 0
	brne delayLoop
	; end delay loop
	
	pop r17
	pop r16
	ret

; load the next value from the sin table into 25:24
getsin:
	; load value
	ldi ZH, HIGH(sintable+0x4000+1) ; the address of the first entry in the table
	ldi ZL, LOW(sintable+0x4000+1) ; the extra 1 is
	mov r17, currentindex ; add 2*currentindex
	lsl r17
	add ZL, r17
	ldi r17, 0x00
	adc ZH, r17
	ld r24, Z+
	ld r25, Z
	
	; check the half of the wave and increment or decrement
	sbrs sinhalf, 0
	rjmp getsin_up ; is sinhalf is 0, use increasing values
	rjmp getsin_down ; otherwise use deceasing

; increasing
getsin_up:
	clz
	ldi r17, 99
	cp currentindex, r17
	breq switchdown
	inc currentindex
	ret
switchdown:
	ldi sinhalf, 0x01
	dec currentindex
	ret
	
; decreasing
getsin_down:
	clz
	ldi r17, 0
	cp currentindex, r17
	breq switchup
	dec currentindex
	ret
switchup:
	ldi sinhalf, 0x00
	inc currentindex
	ret

