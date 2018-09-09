;
; for using the ADC
; Set up a variable voltage divider and connect
; it to pin 3 (PB1).
; connect an LED to pin 1 (PB0)
; If voltage is more than 2.5V, LED is on
; less than 2.5V, LED is off
;

; variables
.DEF rsreg = r23 ; to save status register

.CSEG ; code section
.ORG $0000 ; the address
	; interrupt vectors
	rjmp main ; reset vector
	reti ; external interrupt 0
	reti ; pin change
	reti ; timer input capture
	reti ; timer overflow
	reti ; timer compare match A
	reti ; timer compare match B
	reti ; analog comparator
	reti ; watchdog timer
	reti ; Vcc voltage level monitor
	rjmp isr_adcconv ; ADC conversion complete

; interrupt service routines
isr_adcconv:
	in rsreg, SREG ; save the status register
	ldi r16, 0x00
	sbic ADCL, 7 ; if the value is less than 1/2*Vcc, set pin 1 low, else high
	ldi r16, 1<<PB0
	out PORTB, r16
	out SREG, rsreg ; restore the status register
	reti ; return and enable int

main:
	; set up the stack
	ldi r16, HIGH(RAMEND)
	out SPH, r16
	ldi r16, LOW(RAMEND)
	out SPL, r16

	; setup pins
	ldi r16, 1<<PB0 ; PB0 is output
	out DDRB, r16

	; set up the ADC
	; ADCSRA contains [ADEN, ADSC, ADATE, ADIF, ADIE, ADPS2, ADPS1, ADPS0]
	; which means: enable, start, trigger enable, int flag, int enable, clock prescaler
	;
	; ADCSRB contains [-,-,-,-,-,ADTS2, ADTS1, ADTS0]
	; means: auto trigger soure 0=free running, 1=analog comparator, 2=int0, 3,5=timer comp. A,B,
	;        4=tmer overflow, 6=pinchange int, 7=timer capture
	;
	; ADMUX contains [-,-,-,-,-,-,MUX1,MUX0]
	; sets the channel(pin) for conversion
	;
	; DIDR0 contains [-,-,-,-,ADC3D to ADC0D]
	; disables the digital input, not necessary, but uses less power
	;
	ldi r17, (1<<ADC1D) ; disable digital on pin 3 PB1
	out DIDR0, r17
	ldi r18, (1<<MUX0) ; pin 3 PB1
	out ADMUX, r18
	ldi r19, 0x00 ; free running
	out ADCSRB, r19
	ldi r20, (1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADIE) ; enable, start, trigger, int enable, prescaler=2(min)
	out ADCSRA, r20
	
	; enable sleep mode, ADC noise reduction mode
	ldi r16, (1<<SM0)|(1<<SE)
	out SMCR, r16

	sei ; interrupts enabled
	
; main loop
loop:
	sleep ; go to idle mode
	nop ; just wait for ADC conversion to finish, everything is done in the interrupt routine
	nop
	rjmp loop
