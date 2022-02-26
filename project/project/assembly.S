#include <avr/io.h>

#define __SFR_OFFSET 0

.global sudoku_solver
.global INT0_SOLVER_HOOK

.extern solver_input
.extern solver_guard
.extern solver_image
.extern hook_action
.extern sudoku_cells


; --------------------------------
;		Hook definitions
; --------------------------------
.set BREAK_hook_flag,	0
.set UPDATE_hook_flag,	1


; Memory organization parameters
.equ RS, 15		; Row size (bytes stored per row)
.equ BO, 9		; Block masks offset within memory "row"

; --------------------------------
;		Register aliases
; --------------------------------
; Temporary "scratchpad" register
#define temp	R25
; Current-row and current-blocks masks registers
#define CRM		R10
#define CBM_1	R11
#define CBM_2	R12
#define CBM_3	R13
; Nines column mask register
#define NCM		R28
; Control registers
#define CRCE	R14
#define CRVD	R27
; Misc
#define MDR		R26
#define FCC		R29

; Register stack frames
#define mask_1		R1
#define digit_1		R16
#define mask_2		R2
#define digit_2		R17
#define mask_3		R3
#define digit_3		R18
#define mask_4		R4
#define digit_4		R19
#define mask_5		R5
#define digit_5		R20
#define mask_6		R6
#define digit_6		R21
#define mask_7		R7
#define digit_7		R22
#define mask_8		R8
#define digit_8		R23
#define mask_9		R9
#define digit_9		R24

; --------------------------------
;		MDR Flags
; --------------------------------
#define CRVD_9		6
#define CRM_9		5
#define CRCE_9		4
#define FROB		0		; First Row Of Band

; --------------------------------
;		FCC Flags
; --------------------------------
#define NCM_9		0



/*------------------------------------------------------------------------
 *
 *	COMPUTATIONAL ENGINE MACRO
 *
 *-----------------------------------------------------------------------*/

.macro solver_block		_CBID, _mask, _digit, _BBID, _CBM, _prec_CBID, _prec_mask, _prec_digit, _prec_BBID, _SUCC_CODE_BLOCK_START, _PREC_LEGAL_DIGIT_LOOP, _PREC_BACKTRACKING_TRANSITION
	
	#define CBID		\_CBID
	#define mask		\_mask
	#define digit		\_digit
	#define BBID		\_BBID
	#define CBM			\_CBM
	#define prec_CBID	\_prec_CBID
	#define prec_mask	\_prec_mask
	#define prec_digit	\_prec_digit
	#define prec_BBID	\_prec_BBID

	#define SUCC_CODE_BLOCK_START			\_SUCC_CODE_BLOCK_START
	#define PREC_LEGAL_DIGIT_LOOP			\_PREC_LEGAL_DIGIT_LOOP
	#define PREC_BACKTRACKING_TRANSITION	\_PREC_BACKTRACKING_TRANSITION

	; BSEL is the one-hot encoding of CBID (used to select the desired bit in aggregate-data registers)
	.set BSEL,		(1 << (CBID-1))
	.set prec_BSEL,	(1 << (prec_CBID-1))

	CODE_BLOCK_START_\@:
	
	; Code block 1 "starts" a new row, so special "housekeeping" actions are needed
	.if CBID == 1
		; Check the algorithm's termination condition (terminate when FCC's top 7 bits == 81)
		cpi FCC, 81 << 1
		brlo end_termination_condition\@
			; Fill progress bar (0x00 value since STK500 LEDs are in common anode configuration)
			clr temp
			out PORTC, temp
			
			; Jump to termination point
			rjmp ALGORITHM_END
		end_termination_condition\@:
		
		; Advance memory address to point to current row
		adiw Z, RS

		; Flip all current-row valid data bits (CRVD and MDR's 6th bit)
		com CRVD
		ldi temp, 1 << CRVD_9
		eor MDR, temp

		; Load current-row data from memory to registers
		ldd CRCE, Z+RS
		ldd temp, Z+RS+1		; MDR's data is temporarily placed in temp for further processing
		ldd CRM,  Z+RS+2

		; Load block masks if this is the first row of a Sudoku band (MDR's 0 bit is set)
		bst temp, FROB
		brtc end_first_row_in_band_load\@
			; Load block masks
			ldd CBM_1, Z+RS+3+BO
			ldd CBM_2, Z+RS+3+BO+1
			ldd CBM_3, Z+RS+3+BO+2
			; Overwrite MDR's current-row relevant bits and also update the block masks for digit 9
			andi temp, 0x3F
			andi MDR, ~(0x3F)
			or MDR, temp
		end_first_row_in_band_load\@:
		brts end_not_first_row_in_band\@
			; Overwrite MDR's current-row relevant bits (not the block mask bits)
			andi temp, 0x31
			andi MDR, ~(0x31)
			or MDR, temp
		end_not_first_row_in_band\@:
	.endif
	
	; Check if the block contains a frame from the previous row (CRVD flag 0 => frame must be pushed to the stack and digit must be removed from the column mask if its not a clue)
	.if CBID == 9
		bst MDR, CRVD_9
	.else
		bst CRVD, (CBID-1)
	.endif
	brts end_frame_push\@
		; Push previous row's cell frame to stack
		push mask
		push digit

		; Get column mask from previous row
		ldd temp, Z+3+(CBID-1)

		; Act based on the represented digit. 
		; - If the represented digit is 9 (digit = 0x00), the corresponding NCM bit must be cleared
		; - If the cell was a clue (digit = 0xFF), nothing should happen
		; - If the represented digit was any other (1-8), the corresponding bit from the column mask in memory must be cleared
		tst digit
		brne end_digit_is_zero\@
			.if CBID == 9
				cbr FCC, NCM_9
			.else
				cbr NCM, BSEL
			.endif
			rjmp end_digit_is_other\@
		end_digit_is_zero\@:
		sbrc digit, 6					; Here
		brmi end_digit_is_other\@		; we are actually checking if digit has the two MSBs set (not if the entire byte is 0xFF), but this is sufficient given the possible values of digit throughout the code
		end_digit_is_FF\@:
			eor temp, digit
		end_digit_is_other\@:

		; Store column mask to current row in memory
		std Z+RS+3+(CBID-1), temp

		; Mark frame as row-valid (set relevant bit to 1)
		.if CBID == 9
			ori	MDR, (1 << CRVD_9)
		.else
			ori CRVD, BSEL
		.endif
	end_frame_push\@:
	
	; Initialize digit register to 0xFF (denotes a clue cell)
	ser digit

	; Check if cell contains a clue (the cell would have the empty flag cleared in the original grid) and jump to the next code block
	.if CBID == 9
		sbrs MDR, CRCE_9
	.else
		sbrs CRCE, (CBID-1)
	.endif
	rjmp SUCC_CODE_BLOCK_START

	; Increment filled cells counter, avoiding clue cells (increment by 2, since FCC's bit 0 is an NCM flag). There is no addi in AVR, so we subtract the negative step (therefore, we add the absolute step)
	subi FCC, -2

	; Construct cell's legal digit mask (Calculated as the union of the individual sets, i.e. the conjunction of the individual masks)
	ldd mask, Z+RS+3+(CBID-1)
	and mask, CRM
	and mask, CBM

	; Initialize register to 0x00
	clr digit

	; Check if 9 is legal and jump to next code block
	bst MDR, CRM_9					; check if it is legal in row
	brtc end_digit_9_legal\@
	bst MDR, BBID					; check if it is legal in block
	brtc end_digit_9_legal\@
	.if CBID == 9					; check if it is legal in column
		bst FCC, NCM_9
	.else
		bst NCM, (CBID-1)
	.endif
	brtc end_digit_9_legal\@
		; Remove 9 from row and block masks, as well as from NCM
		cbr MDR, (1 << CRM_9) | (1 << BBID)
		.if CBID == 9
			cbr FCC, NCM_9
		.else
			cbr NCM, (1 << (CBID-1))
		.endif
		; Jump to the next code block
		rjmp SUCC_CODE_BLOCK_START
	end_digit_9_legal\@:
	

	LEGAL_DIGIT_LOOP_\@:

	; Remove digit from mask (similar to marking it as checked)
	eor mask, digit
	
	; Check if mask is 0 (empty, does not contain any legal digits) and jump to previous block (backtrack) 
	brne end_mask_empty_check\@
		
		; Restore row and block masks (add last digit)
		eor CRM, digit
		eor CBM, digit

		; Decrement filled cells count (avoiding clue cells). Decrement by 2, since FCC's bit 0 is an NCM flag
		subi FCC, 2

		BACKTRACKING_TRANSITION_\@:
		
		; Backtracking to previous row (from Code Block 1 -> to Code Block 9) requires "housekeeping"
		.if CBID == 1
			; Flip all current-row valid data bits (CRVD and MDR's 6th bit)
			com CRVD
			ldi temp, 1 << CRVD_9
			eor MDR, temp

			; Initialize row mask to empty (do NOT fetch it from memory, since program flow goes to the last column, where the row mask has to be empty, since all digits have been used)
			clr CRM

			; Set T flag to is-current-row-first-of-current-block value
			bst MDR, FROB
			
			; Load previous-row data from memory to registers (for MDR, overwrite only bits 4:0 and set bit 5 to 0)
			ldd CRCE, Z+0
			ldd temp, Z+1
			andi temp, 0x1F
			andi MDR, 0xC0
			or MDR, temp

			; Initialize band's block masks to empty if the current row is the first row of a band (do NOT fetch the masks from memory, all of them have to be empty, since all their digits have been used)
			brtc end_band_initialization\@
				clr CBM_1
				clr CBM_2
				clr CBM_3
				andi MDR, ~(0x0E)
			end_band_initialization\@:
			
			; Decrement memory pointer to point to previous row
			sbiw Z, RS
		.endif
		
		.if CBID == 1
			; Check if the preceding block does not contain current-row valid data (CRVD flag 0 => frame must be popped from stack)
			bst MDR, CRVD_9
			brts end_preceding_block_clue_check_\@
				; Pop frame from stack in reverse order
				pop prec_digit
				pop prec_mask

				; Mark frame as row-valid
				ori MDR, 1 << CRVD_9
			end_preceding_block_clue_check_\@:

			; Check if the preceding code block contains a clue and jump into its backtrack transition code
			sbrs MDR, CRCE_9
			rjmp PREC_BACKTRACKING_TRANSITION
		.else
			; Check if the preceding block does not contain current-row valid data (CRVD flag 0 => frame must be popped from stack)
			bst CRVD, (prec_CBID-1)
			brts end_preceding_block_clue_check_\@
				; Pop frame from stack in reverse order
				pop prec_digit
				pop prec_mask

				; Mark frame as row-valid
				ori CRVD, prec_BSEL
			end_preceding_block_clue_check_\@:

			; Check if the preceding code block contains a clue and jump into its backtrack transition code
			sbrs CRCE, (prec_CBID-1)
			rjmp PREC_BACKTRACKING_TRANSITION
		.endif
		
		; If the digit in the preceding code block is 9, add it back to the masks (the flags for 9 reside in MDR and FCC)
		tst prec_digit
		brne end_prec_digit_9_check\@
			; Add 9 back to row and block masks, as well as to NCM
			sbr MDR, (1 << CRM_9) | (1 << prec_BBID)
			.if prec_CBID == 9
				sbr FCC, NCM_9
			.else
				sbr NCM, (1 << (prec_CBID-1))
			.endif
		end_prec_digit_9_check\@:
		
		; Backtrack to preceding block
		rjmp PREC_LEGAL_DIGIT_LOOP
	end_mask_empty_check\@:
	
	; Copy digit to temp
	mov temp, digit

	; Get next legal digit (first "1" from LSB to MSB)
	mov digit, mask
	neg digit
	and digit, mask

	; Add new digit to temp (now temp will contain both the previous digit and the current one)
	or temp, digit

	; Toggle row & block masks, i.e. add previous digit and remove the current digit
	eor CRM, temp
	eor CBM, temp

	; Jump to the start of the succeeding code block (SUCC_CODE_BLOCK_START)
	.if CBID == 9
		; Code block 9's succeeding code block is 1, which is not direcly below so an actual jump is necessary
		rjmp SUCC_CODE_BLOCK_START
	.endif
	; For all other code blocks, their successors will be expanded directly below them, therefore no rjmp instruction is needed
.endm



/*------------------------------------------------------------------------
 *
 *	SUDOKU SOLVER - COMPUTATIONAL ENGINE
 *
 *-----------------------------------------------------------------------*/

sudoku_solver:

	; Mixing Assembly and C conventions (save registers)
	push R0
	push R2
	push R3
	push R4
	push R5
	push R6
	push R7
	push R8
	push R9
	push R10
	push R11
	push R12
	push R13
	push R14
	push R15
	push R16
	push R17
	push R28
	push R29
	
	; Restore Register File (+PC, +SREG) from image or initialize
	; -----------------------------------------------------------
	lds temp, solver_guard
	sbrs temp, 1
	rjmp initialize
		
		; Clear the image flag
		cbr temp, 1 << 1
		sts solver_guard, temp

		; Restore from image
		; Restore PC to stack
		lds R16, solver_image+34
		push R16
		lds R16, solver_image+33
		push R16
		; Restore SREG
		lds R16, solver_image+32
		out SREG, R16
		; Restore Register File
		lds R0, solver_image+0
		lds R1, solver_image+1
		lds R2, solver_image+2
		lds R3, solver_image+3
		lds R4, solver_image+4
		lds R5, solver_image+5
		lds R6, solver_image+6
		lds R7, solver_image+7
		lds R8, solver_image+8
		lds R9, solver_image+9
		lds R10, solver_image+10
		lds R11, solver_image+11
		lds R12, solver_image+12
		lds R13, solver_image+13
		lds R14, solver_image+14
		lds R15, solver_image+15
		lds R16, solver_image+16
		lds R17, solver_image+17
		lds R18, solver_image+18
		lds R19, solver_image+19
		lds R20, solver_image+20
		lds R21, solver_image+21
		lds R22, solver_image+22
		lds R23, solver_image+23
		lds R24, solver_image+24
		lds R25, solver_image+25
		lds R26, solver_image+26
		lds R27, solver_image+27
		lds R28, solver_image+28
		lds R29, solver_image+29
		lds R30, solver_image+30
		lds R31, solver_image+31
		; Restore PC to the pushed value (by "returning")
		ret

	; Initialize
	initialize:
		lds temp, solver_input+9*RS
		mov NCM, temp

		lds temp, solver_input+9*RS+1
		mov FCC, temp
	
		clr temp
		clr CRM
		clr CBM_1
		clr CBM_2
		clr CBM_3
		clr CRCE
		clr CRVD
		clr MDR
		
		clr mask_1
		clr digit_1
		clr mask_2
		clr digit_2
		clr mask_3
		clr digit_3
		clr mask_4
		clr digit_4
		clr mask_5
		clr digit_5
		clr mask_6
		clr digit_6
		clr mask_7
		clr digit_7
		clr mask_8
		clr digit_8
		clr mask_9
		clr digit_9

		ldi ZH, hi8(solver_input - 2*RS)
		ldi ZL, lo8(solver_input - 2*RS)

	end_restore_initialize:

	; Set the guard
	ldi temp, 1
	sts solver_guard, temp
	clr temp

	; ====== SOLVER ALGORITHM ======
	solver_block	1, mask_1, digit_1, 1, CBM_1, 9, mask_9, digit_9, 3, CODE_BLOCK_START_1, LEGAL_DIGIT_LOOP_8, BACKTRACKING_TRANSITION_8
	solver_block	2, mask_2, digit_2, 1, CBM_1, 1, mask_1, digit_1, 1, CODE_BLOCK_START_2, LEGAL_DIGIT_LOOP_0, BACKTRACKING_TRANSITION_0
	solver_block	3, mask_3, digit_3, 1, CBM_1, 2, mask_2, digit_2, 1, CODE_BLOCK_START_3, LEGAL_DIGIT_LOOP_1, BACKTRACKING_TRANSITION_1

	solver_block	4, mask_4, digit_4, 2, CBM_2, 3, mask_3, digit_3, 1, CODE_BLOCK_START_4, LEGAL_DIGIT_LOOP_2, BACKTRACKING_TRANSITION_2
	solver_block	5, mask_5, digit_5, 2, CBM_2, 4, mask_4, digit_4, 2, CODE_BLOCK_START_5, LEGAL_DIGIT_LOOP_3, BACKTRACKING_TRANSITION_3
	solver_block	6, mask_6, digit_6, 2, CBM_2, 5, mask_5, digit_5, 2, CODE_BLOCK_START_6, LEGAL_DIGIT_LOOP_4, BACKTRACKING_TRANSITION_4

	solver_block	7, mask_7, digit_7, 3, CBM_3, 6, mask_6, digit_6, 2, CODE_BLOCK_START_7, LEGAL_DIGIT_LOOP_5, BACKTRACKING_TRANSITION_5
	solver_block	8, mask_8, digit_8, 3, CBM_3, 7, mask_7, digit_7, 3, CODE_BLOCK_START_8, LEGAL_DIGIT_LOOP_6, BACKTRACKING_TRANSITION_6
	solver_block	9, mask_9, digit_9, 3, CBM_3, 8, mask_8, digit_8, 3, CODE_BLOCK_START_0, LEGAL_DIGIT_LOOP_7, BACKTRACKING_TRANSITION_7

	ALGORITHM_END:

	; Clear the guard
	clr temp
	sts solver_guard, temp

	; Extract Sudoku cell data from the register file and the stack and store them to the sudoku_cells array (for the non-clue cells)
	sbrc MDR, CRCE_9
	sts sudoku_cells+80, digit_9
	sbrc CRCE, (8-1)
	sts sudoku_cells+79, digit_8
	sbrc CRCE, (7-1)
	sts sudoku_cells+78, digit_7
	sbrc CRCE, (6-1)
	sts sudoku_cells+77, digit_6
	sbrc CRCE, (5-1)
	sts sudoku_cells+76, digit_5
	sbrc CRCE, (4-1)
	sts sudoku_cells+75, digit_4
	sbrc CRCE, (3-1)
	sts sudoku_cells+74, digit_3
	sbrc CRCE, (2-1)
	sts sudoku_cells+73, digit_2
	sbrc CRCE, (1-1)
	sts sudoku_cells+72, digit_1

	#define outer R17
	#define inner R18
	#define digit R19
	#define bsel  R20
	#define mask  R1

	; Extract the remaining Sudoku cell data from the stack in reverse order and store only those whose cells do not contain clues
	ldi ZH, hi8(sudoku_cells+81-9)
	ldi ZL, lo8(sudoku_cells+81-9)
	ldi YH, hi8(solver_input+7*RS)
	ldi YL, lo8(solver_input+7*RS)

	ldi outer, 8
	outer_loop_start:
		; Fetch CRCE data from solver_input
		ld CRCE, Y
		ldd MDR, Y+1

		; Pop CBID 9
		sbiw Z, 1
		pop digit
		sbrc MDR, CRCE_9
		st Z, digit
		pop mask

		; Pop CBID 8..1
		ldi bsel, 0x80
		inner_loop_start:
			sbiw Z, 1
			pop digit
			mov temp, CRCE
			and temp, bsel
			breq finished_with_store
			st Z, digit
			finished_with_store:
			pop mask
		lsr bsel
		brne inner_loop_start
	sbiw Y, RS
	dec outer
	brne outer_loop_start

	; Mixing Assembly and C conventions (restore registers)
	pop R29
	pop R28
	pop R17
	pop R16
	pop R15
	pop R14
	pop R13
	pop R12
	pop R11
	pop R10
	pop R9
	pop R8
	pop R7
	pop R6
	pop R5
	pop R4
	pop R3
	pop R2
	pop R0

	clr R1

ret



/*------------------------------------------------------------------------
 *
 *	INT0 ISR - SOFTWARE INTERRUPT - SOLVER HOOK
 *	- No GPWR register is modified after it returns
 *	- Executes actions only if the interrupted code was the solver
 *	- Supported actions are: break the algorithm and update the progress bar
 *	- Break:  All GPWR, the SREG and PC are stored in an "image" array in SRAM
 *	          to be recovered at a later time. The pushed PC on the stack is
 *	          hijacked, so the program returns to the algorithm's exit.
 *	- Update: The filled cells value is read from the FCC register and used
 *	          to update the progress bar.
 *
 *-----------------------------------------------------------------------*/

INT0_SOLVER_HOOK:
	; Store R16 and SREG directly to solver_image[16] and solver_image[32] respectively
	sts solver_image+16, R16
	in R16, SREG
	sts solver_image+32, R16

	; In the ISR in C, a function call is performed to here, thus the stack contains the address of the ISR in C.
	; We pop this address from the stack and discard it. The reti statement at the end will return to the return address of the C ISR.
	; The top the stack will now contain the PC of the interrupted code.
	pop R16
	pop R16

	; Check, through the solver_guard, that it was the solver's code that was interrupted (and not any other piece of code)
	; Otherwise, clear the hook_action and return.
	lds R16, solver_guard
	sbrs R16, 0
	rjmp int0_solver_hook_end

	lds R16, hook_action

	; === Select appropriate actions ===
	; Actions for BREAK flag
	sbrs R16, BREAK_hook_flag
	rjmp end_break_flag_actions
		; Set guard's PAUSED flag 
		lds R16, solver_guard
		ori R16, 1 << 1
		sts solver_guard, R16

		; Save the interrupted PC to solver_image
		pop R16
		sts solver_image+33, R16
		pop R16
		sts solver_image+34, R16

		; Replace the interrupted PC with the exit label
		ldi R16, pm_lo8(ALGORITHM_END)
		push R16
		ldi R16, pm_hi8(ALGORITHM_END)
		push R16
		; Save entire register file to solver_image (except for R16, which has already been saved)
		sts solver_image+0, R0
		sts solver_image+1, R1
		sts solver_image+2, R2
		sts solver_image+3, R3
		sts solver_image+4, R4
		sts solver_image+5, R5
		sts solver_image+6, R6
		sts solver_image+7, R7
		sts solver_image+8, R8
		sts solver_image+9, R9
		sts solver_image+10, R10
		sts solver_image+11, R11
		sts solver_image+12, R12
		sts solver_image+13, R13
		sts solver_image+14, R14
		sts solver_image+15, R15
		sts solver_image+17, R17
		sts solver_image+18, R18
		sts solver_image+19, R19
		sts solver_image+20, R20
		sts solver_image+21, R21
		sts solver_image+22, R22
		sts solver_image+23, R23
		sts solver_image+24, R24
		sts solver_image+25, R25
		sts solver_image+26, R26
		sts solver_image+27, R27
		sts solver_image+28, R28
		sts solver_image+29, R29
		sts solver_image+30, R30
		sts solver_image+31, R31
	end_break_flag_actions:

	; Actions for UPDATE flag
	lds R16, hook_action
	bst R16, UPDATE_hook_flag
	brtc end_update_flag_actions
		push R17

		; Get the filled cells count (FCC bits7:1)
		mov R16, FCC
		lsr R16
		; Initialize progress bar output
		ldi R17, 0

		; Construct the progress bar data iteratively
		bar_loop_start:
		cpi R16, 10
		brlt end_bar_loop
			lsl R17
			ori R17, 1
		subi R16, 10
		rjmp bar_loop_start
		end_bar_loop:

		; Output to PORT C
		com R17
		out PORTC, R17
		
		pop R17
	end_update_flag_actions:
	

	int0_solver_hook_end:

	; Clear interrupt trigger so the hook does not keep firing constantly
	sbi INT0_PORT, INT0_BIT

	; Clear hook_action variable
	clr R16
	sts hook_action, R16

	; Restore SREG and R16
	lds R16, solver_image+32
	out SREG, R16

	; Restore R16
	lds R16, solver_image+16
reti