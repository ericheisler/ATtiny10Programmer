;
; interrupts
; makes use of several kinds of interrupts
; to control an LED on pin 3 (PB1)
; connect a button from pin 4 (PB2) to ground for interrupt 0 (toggles)
; connect a button from pin 1 (PB0) to ground for pin change interrupt (turns off)
; timer compare match A interrupt is also used (blinks)
;

; variables
.DEF rsreg = r23 ; to save status register
.DEF rintflag = r22 ; interrupt flag register
.EQU bint0 = 0 ; int0 flag bit
.EQU bpchange = 1 ; pin change bit
.EQU btcma = 2 ; timer compare match A bit

.EQU delayMult1 = 0xff ; the delay is delaymult2*delaymult1 
.EQU delayMult2 = 0xff

.CSEG ; code section
.ORG $0000 ; the start address
	; interrupt vectors
	rjmp main ; reset vector
	rjmp isr_int0 ; external interrupt 0
	rjmp isr_pchange ; pin change
	reti ; timer input capture
	reti ; timer overflow
	rjmp isr_tcma ; timer compare match A
	reti ; timer compare match B
	reti ; analog comparator
	reti ; watchdog timer
	reti ; Vcc voltage level monitor
	reti ; ADC conversion complete

; interrupt service routines
isr_int0:
	;push rsreg ; if rsreg is used elsewhere
	in rsreg, SREG ; save the status register
	ldi r16, 0x00 ; disable this interrupt temporarily for debounce
	out EIMSK, r16 ;
	sbr rintflag, 1<<bint0 ; set int0 flag
	; toggle pin 3
	in r16, PORTB
	ldi r17, 1<<PB1
	eor r16, r17
	out PORTB, r16
	out SREG, rsreg ; restore the status register
	;pop rsreg ; 
	reti ; return and enable int

isr_pchange:
	;push rsreg ; if rsreg is used elsewhere
	in rsreg, SREG ; save the status register
	ldi r16, 0x00 ; disable this interrupt temporarily for debounce
	out PCMSK, r16
	sbr rintflag, 1<<bpchange ; set flag
	; set pin 3 low
	ldi r16, 0x00
	out PORTB, r16
	out SREG, rsreg ; restore the status register
	;pop rsreg ; 
	reti ; return and enable int
endisr_pchange:
	out SREG, rsreg ; restore the status register
	pop rsreg ; 
	reti ; return and enable int

isr_tcma:
	;push rsreg ; if rsreg is used elsewhere
	in rsreg, SREG ; save the status register
	; toggle pin 3
	in r16, PORTB
	ldi r17, 1<<PB1
	eor r16, r17
	out PORTB, r16
	out SREG, rsreg ; restore the status register
	;pop rsreg ; 
	reti ; return and enable int

main:
	; set up the stack
	ldi r16, HIGH(RAMEND)
	out SPH, r16
	ldi r16, LOW(RAMEND)
	out SPL, r16
	
	; set up interrupts
	; external int0 type and mask
	ldi r16, 0x02 ; 0x00=low level, 0x01=change, 0x02=falling, 0x03=rising
	out EICRA, r16
	ldi r16, 0x01
	out EIMSK, r16 ; 

	; pin change enable and mask
	ldi r16, 0x01
	out PCICR, r16
	ldi r16, (1<<PCINT0) ; pin change on pin 1
	out PCMSK, r16

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
	ldi r16, (1<<WGM02)|(1<<CS02)|(1<<CS00) ; CTC mode, prescaler=1024
	out TCCR0B, r16
	ldi r16, (1<<OCIE0A) ; enable the interrupt
	out TIMSK0, r16
	; compare value A
	ldi r24, 0xff ; the low byte
	ldi r25, 0x07 ; the high byte
	out OCR0AH, r25
	out OCR0AL, r24

	; setup pins
	ldi r16, 1<<PB1 ; PB1 is output
	out DDRB, r16
	ldi r16, (1<<PB0)|(1<<PB2) ; enable pullup resistor on PB0 and PB2
	out PUEB, r16
	
	; enable sleep mode, idle
	ldi r16, 1<<SE
	out SMCR, r16

	sei ; interrupts enabled
	
; main loop
loop:
	sleep ; go to sleep
	nop ; dummy for waking up
	sbrc rintflag, bint0 ; skip if flag not set
	rcall postint0 ; to do if int0 flag is set
	sbrc rintflag, bpchange ; skip if flag not set
	rcall postpchange ; to do if int0 flag is set
	rjmp loop

; to do after int0
postint0:
	rcall delay ; wait a bit for debounce
	cbr rintflag, 1<<bint0 ; clear int0 flag
	ldi r16, 0x01
	out EIMSK, r16 ; reenable interrupt
	ret

; to do after pin change
postpchange:
	rcall delay ; wait a bit for debounce
	cbr rintflag, 1<<bpchange ; clear flag
	ldi r16, (1<<PCINT0) 
	out PCMSK, r16 ; reenable interrupt
	ret

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