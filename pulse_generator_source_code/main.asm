;***************************************************************************
;*
;* Title: ppg_IV_fsm.asm
;* Author: Zachary Wong (111737587), Emmanuel Benard (111441237)
;* ESE 280 Lab Section 01
;* Version: 1.0
;* Last updated: 12/3/2019
;*
;* DESCRIPTION: This program uses interrupts and a table driven finite state
;* machine in order to implement a programmable pulse generator. The interrupts
;* respond to a keypad press or a push button press.
;*
;* VERSION HISTORY
;* 1.0 Original version
;***************************************************************************

.nolist
.include "m324adef.inc"
.list 

;r23: register used for constant for no press ($FE for keypad, $00 for button)

.dseg
key_press_flag:					.byte 1		;saves key value
fsm_input:						.byte 1		;holds pseudo key value for a digit ($DD)	
pb_press_flag:					.byte 1		;logic '1' if button pressed, logic '0' if not	
burst_flag:						.byte 1		;set when system generates burst ($BB)
continuous_flag:				.byte 1		;set when continuous burst ($CC)
burst_count_setting_bcd:		.byte 3		;setting unpacked BCD	
burst_count_setting_bin:		.byte 1		;setting in binary
burst_count_bin:				.byte 1		;pulses left to generate in burst

.cseg
reset:
.org RESET
	rjmp start
.org INT0addr				;INT0 interrupt vector
	rjmp keypress_ISR
.org INT1addr				;INT1 interrupt vector
	rjmp pb_press_ISR

start:
	;Port B is an ouput port
	ldi r16, $FF        ;load r16 with all 1s
    out DDRB, r16       ;port B - all bits configured as outputs

	;Configure port D as an input port
    ldi r16, $00        ;load r16 with all 0s
    out DDRD, r16       ;port D - all bits configured as inputs
    ldi r16, $FF        ;enable pull-up resistors by outputting
    out PORTD, r16      ;all 1s to PORTD	

	;Configure port A as an output port
	ldi r16, $FF		;load r16 with all 1s
	out DDRA, r16		;port A - all bits configured as outputs

	;Initialize stack pointer to allow subroutine calls
    ldi r16, LOW(RAMEND)    ;load low byte of stack pointer    
    out SPL, r16
    ldi r16, HIGH(RAMEND)   ;load high byte of stack pointer
    out SPH, r16

	;put FSM in initial state
	ldi pstatel, LOW(idle)
	ldi pstateh, HIGH(idle)

	rcall init_lcd_dog		;initialize display
	rcall clr_dsp_buffs		;clear LED displays
	rcall update_lcd_dog

	ldi r17, $FE			;storing no key value
	sts key_press_flag, r17
	ldi r17, 0
	sts pb_press_flag, r17	;storing logic '0' for button

	;interrupt sense control bits
	ldi r16, (1 << ISC01) | (1 << ISC00) | (1 << ISC10) | (1 << ISC11)
    sts EICRA, r16							;rising edge requests interrupt (EICRA in ext. I/O)
    ldi r16, (1<<INT0) | (1<<INT1)			;enable interrupt request at INT0 and INT1  
    out EIMSK, r16
	sei										;set global interrupt enable

main_loop:
	lds r22, key_press_flag			;load key value obtained from interrupt routine into r22
	ldi r23, $FE
	sts key_press_flag, r23
	cpi r22, 10						;check if key code is greater or less than 10
	brlo digit_pressed				;input psuedo code for digit if < 10
	breq clear_pressed				;input key code of symbol pressed if > 10
	cpi r22, $0C
	breq enter_pressed
	lds r17, pb_press_flag			;check if a push button has been pressed
	cpi r17, $01
	breq pb_set
	mov r16, r22
	rcall fsm
	rjmp main_loop

digit_pressed: 
	ldi r22, $DD					;load in fsm input value for digit
	sts fsm_input, r22
	lds r16, fsm_input
	rcall fsm						;call fsm
	rjmp main_loop

clear_pressed:
	sts fsm_input, r22				;store fsm input value for CLEAR key ($0A)
	lds r16, fsm_input
	rcall fsm						;call fsm
	rjmp main_loop

enter_pressed:
	sts fsm_input, r22				;store fsm input value for ENTER key ($0C)
	lds r16, fsm_input
	rcall fsm
	rjmp main_loop

pb_set:
	sts fsm_input, r16		
	lds r16, fsm_input
	rcall fsm	
	lds r16, continuous_flag		;load in continuous flag	
	rcall fsm
	lds r16, burst_flag				;load in burst flag
	rcall fsm
	rjmp main_loop

prompt_1: .db 1, "n = ", 0		;prompt on line 1



.nolist
;====================================
.include "lcd_dog_asm_driver_m324a.inc"  ; LCD DOG init/update procedures.
;====================================
.list
;************************
;NAME:      clr_dsp_buffs
;FUNCTION:  Initializes dsp_buffers 1, 2, and 3 with blanks (0x20)
;ASSUMES:   Three CONTIGUOUS 16-byte dram based buffers named
;           dsp_buff_1, dsp_buff_2, dsp_buff_3.
;RETURNS:   nothing.
;MODIFIES:  r25,r26, Z-ptr
;CALLS:     none
;CALLED BY: main application and diagnostics
;********************************************************************
clr_dsp_buffs:
     ldi R25, 48               ; load total length of both buffer.
     ldi R26, ' '              ; load blank/space into R26.
     ldi ZH, high (dsp_buff_1) ; Load ZH and ZL as a pointer to 1st
     ldi ZL, low (dsp_buff_1)  ; byte of buffer for line 1.
   
    ;set DDRAM address to 1st position of first line.
store_bytes:
     st  Z+, R26       ; store ' ' into 1st/next buffer byte and
                       ; auto inc ptr to next location.
     dec  R25          ; 
     brne store_bytes  ; cont until r25=0, all bytes written.
     ret

;*******************
;NAME:      load_msg
;FUNCTION:  Loads a predefined string msg into a specified diplay
;           buffer.
;ASSUMES:   Z = offset of message to be loaded. Msg format is 
;           defined below.
;RETURNS:   nothing.
;MODIFIES:  r16, Y, Z
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
; Message structure:
;   label:  .db <buff num>, <text string/message>, <end of string>
;
; Message examples (also see Messages at the end of this file/module):
;   msg_1: .db 1,"First Message ", 0   ; loads msg into buff 1, eom=0
;   msg_2: .db 1,"Another message ", 0 ; loads msg into buff 1, eom=0
;
; Notes: 
;   a) The 1st number indicates which buffer to load (either 1, 2, or 3).
;   b) The last number (zero) is an 'end of string' indicator.
;   c) Y = ptr to disp_buffer
;      Z = ptr to message (passed to subroutine)
;********************************************************************
load_msg:
     ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
     ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming 
                               ; (dsp_buff_1 for now).
     lpm R16, Z+               ; get dsply buff number (1st byte of msg).
     cpi r16, 1                ; if equal to '1', ptr already setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
     cpi r16, 2                ; if equal to '2', ptr now setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
        
get_msg_byte:
     lpm R16, Z+               ; get next byte of msg and see if '0'.        
     cpi R16, 0                ; if equal to '0', end of message reached.
     breq msg_loaded           ; jump and stop message loading operation.
     st Y+, R16                ; else, store next byte of msg in buffer.
     rjmp get_msg_byte         ; jump back and continue...
msg_loaded:
     ret

;***************************************************************************
;* 
;* "keypress_ISR" - Count Interrupts at INT0
;*
;* Description: Counts rising edge at INT0 (PD2)
;*
;* Author:	Zachary Wong (111737587), Emmanuel Benard (111441237)
;* Version:
;* Last updated:            11/16/19
;* Target:                  ATmega324A
;* Number of words:			15
;* Number of cycles:        16
;* Low registers modified:  none
;* High registers modified: r16, r18
;*
;* Parameters: none
;*
;* Notes: 
;*
;***************************************************************************
keypress_ISR:
	in r18, SREG		;save SREG
	push r18
	lds r18, key_press_flag	;load in flag from variable
	in r16, PIND			;read keycode
	andi r16, $F0			;reading keycode from key press (higher nibble)
	swap r16				;swap higher and lower nibbles to right justify
	mov r18, r16
	call keycode2keyvalue
	sts key_press_flag, r18	;store result in variable
	pop r18					;restore SREG
	out SREG, r18
	reti					;return from interrupt


;***************************************************************************
;* 
;* "pb_press_ISR" - Count Interrupts at INT1
;*
;* Description: Counts rising edge at INT1 (PD3)
;*
;* Author:	Zachary Wong (111737587), Emmanuel Benard (111441237)
;* Version:
;* Last updated:            11/16/19
;* Target:                  ATmega324A
;* Number of words:			16
;* Number of cycles:        16
;* Low registers modified:  none
;* High registers modified: r18
;*
;* Parameters: none
;*
;* Notes: 
;*
;***************************************************************************
pb_press_ISR:
	in r18, SREG			;save SREG
	push r18
	lds r18, pb_press_flag	;load in flag value
	ldi r18, 1				;logic '1'
	sts pb_press_flag, r18		;store in flag
	pop r18					;restore SREG
	out SREG, r18			
delay:
	ldi r16, 150		;load constant for debounce delay
	rcall var_delay
	sbic PIND, 3
	rjmp delay
	ldi r16, (1 << INTF1)
	out EIFR, r16			;clear interrupt flag
	reti					;return from interrupt

;***************************************************************************
;* 
;* NAME: var_delay
;*
;* Description: A simple nested loop to simulate delay.
;*
;* Author: Zachary Wong (111737587), Emmanuel Benard (111441237)
;* Version: 1.0
;* Last updated: 11/3/19
;* Target: ATmega324a @ 1MHz
;* Number of words: 5
;* Number of cycles: 99(n-1) + 102
;* Low registers modified: none
;* High registers modified: r16, r17
;*
;* Parameters: r16 - constant for delay
;*
;* Returns: nothing
;*
;* Notes: n is the value of the constant in r16
;*
;***************************************************************************
var_delay:
outer_loop:
	ldi r17, 32
inner_loop:
	dec r17
	brne inner_loop
	dec r16
	brne outer_loop
	ret

;***************************************************************************
;* 
;* NAME: get_key_value
;*
;* Description: This program processes a keypad input if a key has been pressed
;*
;* Author: Ken Short
;* Version: 1.0
;* Last updated: 11/4/19
;* Target: ATmega324a @ 1MHz
;* Number of words: 12
;* Number of cycles: N/A
;* Low registers modified: none
;* High registers modified: r16, r17, r18, ZL, ZH
;*
;* Parameters: none
;*
;* Returns: Carry = 0 if not data. Carry = 1 if there is data. Key value right
;* justified in r18.
;*
;* Calls: keycode2keyvalue
;* Notes: 
;*
;***************************************************************************
get_key_value:
	clc						;clear carry indicates no key press
	sbis PIND, 2			;wait for keypad press
	ret						;return if no key pressed
	in r16, PIND			;input data from PortD pins
	andi r16, $F0			;reading keycode from key press (higher nibble)
	swap r16				;swap higher and lower nibbles to right justify
	mov r18, r16
	call keycode2keyvalue
	sec						;set carry to indicate key press
	cbi PORTC, 7			;negative pulse to clear 2nd flip flop
	sbi PORTC, 7
	ret


;***************************************************************************
;* 
;* NAME: keycode2keyvalue
;*
;* Description: This program performs a table lookup of the appropiate key 
;* values when a key has been pressed.
;*
;* Author: Ken Short
;* Version: 1.0
;* Last updated: 11/4/19
;* Target: ATmega324a @ 1MHz
;* Number of words: 14
;* Number of cycles: N/A
;* Low registers modified: none
;* High registers modified: r16, r17, r18, ZL, ZH
;*
;* Parameters: none
;*
;* Returns: nothing 
;*
;* Notes: 
;*
;***************************************************************************
keycode2keyvalue:
	;Perform table lookup 
	;Nibble from scan code is offset into table
key_map:
	ldi ZH, high (keytable * 2)	;Z pointer starts at beginning of table
	ldi ZL, low (keytable * 2)		
	ldi r16, $00				;add offset to Z pointer
	add ZL, r18
	adc ZH, r16
	lpm r18, Z					;load byte from table pointed by Z
	ret

	;Table of desired key values associated with key scan codes
keytable:	.db $01, $02, $03, $0F
			.db $04, $05, $06, $0E
			.db $07, $08, $09, $0D
			.db $0A, $00, $0B, $0C

;***************************************************************************
;*
;* "BCD2bin16" - BCD to 16-Bit Binary Conversion
;*
;* This subroutine converts a 5-digit packed BCD number represented by
;* 3 bytes (fBCD2:fBCD1:fBCD0) to a 16-bit number (tbinH:tbinL).
;* MSD of the 5-digit number must be placed in the lowermost nibble of fBCD2.
;*
;* Let "abcde" denote the 5-digit number. The conversion is done by
;* computing the formula: 10(10(10(10a+b)+c)+d)+e.
;* The subroutine "mul10a"/"mul10b" does the multiply-and-add operation
;* which is repeated four times during the computation.
;*
;* Number of words	:30
;* Number of cycles	:108
;* Low registers used	:4 (copyL,copyH,mp10L/tbinL,mp10H/tbinH)
;* High registers used  :4 (fBCD0,fBCD1,fBCD2,adder)	
;*
;***************************************************************************

;***** "mul10a"/"mul10b" Subroutine Register Variables

.def	copyL	=r12		;temporary register
.def	copyH	=r13		;temporary register
.def	mp10L	=r14		;Low byte of number to be multiplied by 10
.def	mp10H	=r15		;High byte of number to be multiplied by 10
.def	adder	=r19		;value to add after multiplication	

;***** Code

mul10a:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" high nibble
	swap	adder
mul10b:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" low nibble
	mov	copyL,mp10L	;make copy
	mov	copyH,mp10H
	lsl	mp10L		;multiply original by 2
	rol	mp10H
	lsl	copyL		;multiply copy by 2
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (4)
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (8)
	rol	copyH		
	add	mp10L,copyL	;add copy to original
	adc	mp10H,copyH	
	andi	adder,0x0f	;mask away upper nibble of adder
	add	mp10L,adder	;add lower nibble of adder
	brcc	m10_1		;if carry not cleared
	inc	mp10H		;	inc high byte
m10_1:	ret	

;***** Main Routine Register Variables

.def	tbinL	=r14		;Low byte of binary result (same as mp10L)
.def	tbinH	=r15		;High byte of binary result (same as mp10H)
.def	fBCD0	=r16		;BCD value digits 1 and 0
.def	fBCD1	=r17		;BCD value digits 2 and 3
.def	fBCD2	=r18		;BCD value digit 5

;***** Code

BCD2bin16:
	andi	fBCD2,0x0f	;mask away upper nibble of fBCD2
	clr	mp10H		
	mov	mp10L,fBCD2	;mp10H:mp10L = a
	mov	adder,fBCD1
	rcall	mul10a		;mp10H:mp10L = 10a+b
	mov	adder,fBCD1
	rcall	mul10b		;mp10H:mp10L = 10(10a+b)+c
	mov	adder,fBCD0		
	rcall	mul10a		;mp10H:mp10L = 10(10(10a+b)+c)+d
	mov	adder,fBCD0
	rcall	mul10b		;mp10H:mp10L = 10(10(10(10a+b)+c)+d)+e
	ret

;***************************************************************************
;* 
;* "fsm" - Simplified Table Driven Finite State Machine
;*
;* Description:
;* This table driven FSM can handle 255 or fewer input symbols.
;*
;* Author:              Ken Short
;* Version:             2.0
;* Last updated:        11/30/19
;* Target:              ATmega324a
;* Number of words:
;* Number of cycles:
;* Low regs modified:   r16, r18, r20, r21, r31, and r31
;* High registers used:
;*
;* Parameters:          present state in r25:r24 prior to call
;*                      input symbol in r16 prior to call
;*
;* Notes: 
;*
;***************************************************************************

.def pstatel = r24 ;low byte of present state address
.def pstateh = r25 ;high byte of present state address

;input symbols for finite state machine
.equ digit = $DD   
.equ clear = $0A             
.equ enter = $0C
.equ no_key = $FE
.equ pb_press = $01
.equ burst_input = $BB
.equ cont_input = $CC
.equ eol = $FF  ;end of list (subtable) do not change

;state table for finite state machine
;each row consists of input symbol, next state address, task
;subroutine address

state_table:

idle:	.dw clear,		prompt,		set_display
		.dw no_key,		idle,		do_nothing
		.dw eol,		idle,		wrong_input

prompt: .dw digit,		prompt,		get_input
		.dw no_key,		prompt,		do_nothing
		.dw enter,		convert,	process
		.dw eol,		prompt,		wrong_input

convert: .dw pb_press,	count,		get_count_bin
		 .dw eol,		convert,	do_nothing

count:	.dw cont_input,		cont,		infinite_burst	
		.dw burst_input,	burst,		generate_burst
		.dw eol,			count,		do_nothing

cont:	.dw clear,		prompt,		set_display
		.dw	eol,		cont,		do_nothing

burst:	.dw pb_press,	count,		get_count_bin
		.dw clear,		prompt,     set_display
		.dw eol,		burst,		do_nothing

fsm:
;load Z with a byte pointer to the subtable corresponding to the
;present state
    mov ZL, pstatel ;load Z pointer with pstate address * 2
    add ZL, ZL ;since Z will be used as a byte pointer with the lpm instr.
    mov ZH, pstateh
    adc ZH, ZH

;search subtable rows for input symbol match
search:
    lpm r18, Z ;get symbol from state table
    cp r18, r16 ;compare table entry with input symbol
    breq match

;check input symbol against eol
check_eol:
    cpi r18, eol ;compare low byte of table entry with eol
    breq match

nomatch:
    adiw ZL, $06 ;adjust Z to point to next row of state table
    rjmp search ;continue searching

;a match on input value to row input value has been found
;the next word in this row is the next state address
;the word following that is the task subroutine's address
match:
	;make preseent state equal to next state value in row
	;this accomplishes the stat transition
    adiw ZL, $02 ;point to low byte of state address
    lpm pstatel, Z+; ;copy next state addr. from table to preseent stat
    lpm pstateh, Z+

	;execute the subroutine that accomplihes the task associated
	;with the transition
    lpm r20, Z+ ;get subroutine address from state table
    lpm r21, Z ;and put it in Z pointer
    mov ZL, r20
    mov ZH, r21
    icall ;Z pointer is now used as a word pointer
    ret



;***************************************************************************
;* 
;* "taskn" - Stub subroutines for testing
;*
;* Description:
;* These subroutines are the tasks for the table driven FSM.
;* When a program is being developed, you should start with each of these
;* subroutines consisting of just a nop and a return. You can then simulate
;* the program and verify that the transitions defined by you transition
;* table and original state diagram take place in response to input
;* sequences.
;*
;* Author:              Ken Short
;* Version:
;* Last updated:		12/3/19
;* Target:			ATmega324a
;* Number of words:	75
;* Number of cycles: N/A
;* Low registers used: r14
;* High registers used: r16, r17, r18, r20, r21, r22, r23
;*
;* Parameters: none
;*
;* Notes: 
;*
;***************************************************************************


;subroutine stubs for tasks to be implemented

;if nothing is pressed, simply do nothing
do_nothing:
	nop
	ret
;************************************************************************
;turn on buzzer if input is wrong
wrong_input:
    sbi PORTA, 6    
	ldi r16, 255
	rcall var_delay		;call delay with constant of 255
	cbi PORTA, 6
    ret
;************************************************************************
;Initialize buffers for LCD display
set_display:
	ldi  ZH, high(prompt_1<<1)  
	ldi  ZL, low(prompt_1<<1)   
	rcall load_msg						;load prompt message into buffer.
	rcall update_lcd_dog				;update display
	ldi R23, 3
	ldi XH, high (dsp_buff_1 + 4)			;Load X pointer to 4th byte of buffer for line 1
	ldi XL, low (dsp_buff_1 + 4)
	ldi YL, LOW(burst_count_setting_bcd)	;initialize Y pointer to variable byte
	ldi YH, HIGH(burst_count_setting_bcd)
    ret
;*************************************************************************
;Converting keycode to ASCII character
get_input:
	lds r22, key_press_flag		;get keycode
	ldi r23, $FE
	sts key_press_flag, r23

store_digit:
	mov r20, r22			;copy key value
	st Y+, r22

display_digit:
	ldi r16, $30				;add the hex digit by $30 to get ASCII char (number)
	add r20, r16
	st X+, r20					;Store key value in X-pointer 
								;and increment pointer
	rcall update_lcd_dog		;update display
	dec R25
	cpi R25, 0
	breq set_display
    ret
;*************************************************************************
;Converting unpacked bcd to binary
process:
	ldi YL, LOW(burst_count_setting_bcd)
	ldi YH, HIGH(burst_count_setting_bcd)
	ld r18, Y					;store most significant digit in r18
	adiw YH:YL, $0001			;increment pointer
	ld r17, Y
	adiw YH:YL, $0001
	ld r16, Y					;store least significant digit in r16
	;convert to packed BCD
	swap r17					
	or r16, r17
	clr r17
	mov r17, r18
	clr r18
	rcall BCD2bin16
	mov r21, r14						;copy binary value
	sts burst_count_setting_bin, r14	;store binary value in variable(s)
	sts burst_count_bin, r21
	ret
;**************************************************************************
;Retrieving count variable
get_count_bin:
	lds r21, burst_count_bin	;check if burst count is 0
	cpi r21, 0					;if yes, set continuous burst flag
	breq set_cont				
	ldi r21, $BB				;if not, set burst flag
	sts burst_flag, r21
	ret

set_cont:
	ldi r20, $CC
	sts continuous_flag, r20	;setting continuous flag
	ret
;**************************************************************************
;Generating continuous burst
infinite_burst:
	ldi r16, 10				;load constant for var_delay
	sbi PORTA, 7				;generate positive pulse
	rcall var_delay				;call delay
	cbi PORTA, 7
	ldi r16, 20
	rcall var_delay
	ret
;**************************************************************************
;Generating burst based on burst setting
generate_burst:
	lds r21, burst_count_bin
	ldi r16, 10
	sbi PORTA, 7
	rcall var_delay
	cbi PORTA, 7
	ldi r16, 20
	rcall var_delay
	dec r21
	sts burst_count_bin, r21
	brne generate_burst
	ret
;**************************************************************************