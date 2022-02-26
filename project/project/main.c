/*
 * ------------------ Project details ----------------------
 * File name: sudoku.c
 * 
 * Student 1 details: Alexandros Poupakis 2016030061
 * Student 2 details: Georgios Agoritsis 2017030081 
 * Deadline: 06/12/2021 23:59:59
 *
 * Project name: A SUDOKU game - Milestone 1
 * Development platform: Microchip Studio
 * Microcontroler device: ATmega16 
 * Development system: STK500
 * Course: HRY411 - Embedded Microprocessor Systems
 * Department - University: ECE - TUC
 *
 * Description: For this project, ATmega16 microcontroller
 *	was used. This .c code was tested on STK500 development
 *	system and it includes:
 *		1. The USART communication protocol
 *		2. The LED driver
 *		3. Setting the memory address space for the SUDOKU data
 *	
 * ---------------------------------------------------------
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

// Uncomment the following line to compile the project for simulation. If the symbol is left undefined, the project is compiled for execution on an AVR
// #define SIMULATION

/*------------------------------------------------------------------------------------------
 *
 *	Notes:
 *		1. Clock frequency for this project is 10MHz.
 *		2. Commands valid for program verification:
 *			- AT<CR><LF>
 *			- C<CR><LF>
 *			- N<X><Y><VALUE><CR><LF> (where X, Y, VALUE can be a 0-9 decimal digit)
 *			- P<CR><LF>
 *			- S<CR><LF>
 *			- T<CR><LF>
 *			- OK<CR><LF>
 *			- B<CR><LF>
 *			- D<X><Y><CR><LF> (where X, Y can be a 0-9 decimal digit)
 *		3. Simulation - breakpoints at:
 *			- ISR(TIMER1_COMPA_vect){[...]}
 *			- ISR(USART_RXC_vect){[...]}
 *
 *-----------------------------------------------------------------------------------------*/

/*------------------------------------------------------
 *
 * Function declarations
 *
 *-----------------------------------------------------*/
void initialize(void);
void INT0_init(void);
void TIMER1_init(uint16_t);
void USART_init(uint16_t);
void USART_command_actions();
void sudoku_insert_value(void);
void sudoku_clear(void);
void transmit_ok_response(void);
void transmit_done_response(void);
void transmit_cell_digit(uint8_t, uint8_t);
void send_char(unsigned char);
uint8_t convert_decimal_digit_to_onehot(uint8_t);
uint8_t convert_onehot_digit_to_decimal(uint8_t);

// Assembly routine
extern void sudoku_solver(void);
extern void INT0_SOLVER_HOOK(void);

/*------------------------------------------------------
 * 
 * Define the timer 1 (sudoku progress bar leds) target value for the CTC mode.
 * 40 Hz firing rate, for a 10 MHz clock.
 * Equation: value = f_clk / (prescaler * f_timer) - 1
 *
 *-----------------------------------------------------*/
#define TIMER1_TARGET_VALUE (31250 - 1)


/*------------------------------------------------------
 * 
 * Define USART baudrate and the respective contents of its URRB register.
 * - 10 MHz clock
 * - asynchronous normal mode
 * - Reference: ATmega16 manual (pdf) p.147
 *
 *-----------------------------------------------------*/
#define F_OSC 10000000
#define BAUDRATE 9600
#define UBRR_value (F_OSC / 16 / BAUDRATE - 1)

/*------------------------------------------------------
 *
 * State and command definitions
 *
 *-----------------------------------------------------*/
// FSM states
#define STATE_IDLE		0
#define STATE_SOLVING	1

// 8-bit Command Codes
#define CC_NONE		0x00
#define CC_INVALID	0x0F
#define CC_AT	0x01
#define CC_C	0x02
#define CC_N	0x03
#define CC_P	0x04
#define CC_S	0x05
#define CC_T	0x06
#define CC_OK	0x07
#define CC_B	0x08
#define CC_D	0x09

// macro to construct final command byte and special command byte (without CR, LF flags)
#define CR_flag		(1 << 6)
#define LF_flag		(1 << 7)
#define CRLF_flags	(CR_flag | LF_flag)
#define FCB( command_code, command_base_length_minus_1 )	(CRLF_flags | command_code << 2 | command_base_length_minus_1)
#define SCB( command_code )									(command_code << 2)

// 8-bit final command state (format: CRLF (2 bits) : command code (4 bits) : n remaining command characters (2 bits))
#define NONE			SCB( CC_NONE )
#define INVALID			SCB( CC_INVALID )
#define AT_CR_LF		FCB( CC_AT,	1 )
#define C_CR_LF			FCB( CC_C,	0 )
#define N_X_Y_VAL_CR_LF	FCB( CC_N,	3 )
#define P_CR_LF			FCB( CC_P,	0 )
#define S_CR_LF			FCB( CC_S,	0 )
#define T_CR_LF			FCB( CC_T,	0 )
#define OK_CR_LF		FCB( CC_OK,	1 )
#define B_CR_LF			FCB( CC_B,	0 )
#define D_X_Y_CR_LF		FCB( CC_D,	2 )

// macro to initialize command with a given command code
#define INIT_CMD(command_code)	(command_code << 2 | 0)

// macro to retrieve a command's current length
#define CMD_BASE_LEN(cmd)	((cmd & 0x3) + 1)

// macro to check the command code
#define IS_CMD(cmd, command_code)	(((cmd >> 2) & 0x0F) == command_code)

// Hook flags (break out of the solver engine, update the progress bar)
#define BREAK_hook_flag		0
#define UPDATE_hook_flag	1

/*------------------------------------------------------
 *
 * Global variables
 *	- state variable		Holds current FSM state
 *	- cmd variable			Holds the command based on the USART input
 *	- cmd_digits			Variables hold the <X><Y><VALUE> or <X><Y> values for N and D USART commands respectively
 *	- sudoku_clear_flag		Used in the USART communication protocol (1 when the cells are cleared, 0 otherwise)
 * 
 *	- sudoku_cells			Array containing digit data for each cell
 *
 *	- last_sent_x			Cell indices (1-based) for storing the last transmitted cell digit (for S and T USART commands)
 *	- last_sent_y
 *
 *	- solver_input			Contains all necessary data in the appropriate format for the solver to function. This array is modified during the solver's execution
 *	- solver_image			Contains all necessary information to perform a context switch back to the solver after its preemption
 *	- solver_guard			Variable to interface between the software interrupt and the solver (bit 0 denotes whether the solver is currently executed, bit 1 denotes the existence of a valid solver image)
 *
 *	- hook_action			Denotes which actions the software interrupt/solver hook must execute
 *
 *-----------------------------------------------------*/
uint8_t state, cmd;
uint8_t cmd_digits[3];
uint8_t sudoku_clear_flag;

uint8_t sudoku_cells[9 * 9];

uint8_t last_sent_x;
uint8_t last_sent_y;

#define SOLVER_INPUT_ROW_SIZE	15

uint8_t solver_input[9 * SOLVER_INPUT_ROW_SIZE + 2];
volatile uint8_t solver_image[32 + 1 + 2];
volatile uint8_t solver_guard = 0;

volatile uint8_t hook_action = 0;

// Macros for named access to solver_input's fields (indices are 1-based)
#define si_CRCE(row)		solver_input[(row-1)*SOLVER_INPUT_ROW_SIZE]
#define si_MDR(row)			solver_input[(row-1)*SOLVER_INPUT_ROW_SIZE + 1]
#define si_CRM(row)			solver_input[(row-1)*SOLVER_INPUT_ROW_SIZE + 2]
#define si_CMi(col)			solver_input[3 + col - 1]
#define si_CBM_i(row, col)	solver_input[(row-1)*SOLVER_INPUT_ROW_SIZE + 3 + 9 + ((col-1) / 3)]
#define si_NCM				solver_input[9*SOLVER_INPUT_ROW_SIZE]
#define si_FCC				solver_input[9*SOLVER_INPUT_ROW_SIZE + 1]

// ------- MDR Flags -------
#define CRVD_9		6
#define CRM_9		5
#define CRCE_9		4
#define FROB		0		// First Row Of Band

// ------- FCC flags -------
#define NCM_9		0

/*------------------------------------------------------
 *
 *	Main program
 *	-includes: 
 *		1. Program Initializations
 *			- Stack initialization
 *			- Timer 1 initialization
 *			- USART initialization
 *			- sudoku_clear_flag initialization
 *			- cmd and state variables initialization
 *			- We do not need to initialize port D as:
 *				When the Transmitter is enabled, the normal port operation of the TxD pin is overridden
 *				by the USART and given the function as the transmitter's serial output. Same goes for
 *				the Receiver and the RxD pin.
 *		2. An infinite loop
 *
 *-----------------------------------------------------*/
int main(void){
	//Program initializations
	initialize();
	
	// Infinite loop 
    while (1){
		// When a USART command is complete, execute the corresponding actions
		if ((cmd & CRLF_flags) == CRLF_flags){
			USART_command_actions();
		}
		
		if (state == STATE_SOLVING){
			sudoku_solver();
			if ((solver_guard & (1 << 1)) == 0){
				state = STATE_IDLE;
				transmit_done_response();
			}
		}
	}
}

void initialize(){
	// Enable global interrupts for when timer/counter overflows and for USART
	sei();	
		
	// When the program starts, the LED display should not display any number
	// (we will initialize each digit with the value 0xFF) and the sudoku data 
	// should be cleared.
	sudoku_clear();
	
	// If sudoku_clear_flag is equal to 0x00 then data have not been cleared (flag is not set).
	sudoku_clear_flag = 1;
	
	// Set Port C to output for the progress bar leds
	DDRC = 0xFF;
	
	// INT0 Initialization
	INT0_init();
	
	// Timer 1 initialization
	TIMER1_init(TIMER1_TARGET_VALUE);
	
	// USART Initialization
	USART_init(UBRR_value);
	
	// initialize global variables
	state = STATE_IDLE;
	cmd = NONE;
	last_sent_x = 0;
	last_sent_y = 0;
	
	return;
}

/*------------------------------------------------------------------------
 *
 *	External interrupt INT 0 Initialization function
 *	- Triggered on the low level of the INT0 pin
 *	- Used as a software interrupt by setting the pin direction to output
 *	- Reference: ATmega16 manual (pdf) p.68
 *
 *-----------------------------------------------------------------------*/
void INT0_init(){
	// Configure INT0 pin as output
	INT0_DDR |= 1 << INT0_BIT;
	
	// Set INT0 pin high (to avoid firing the interrupt while setting it up)
	INT0_PORT |= 1 << INT0_BIT;
	
	// Configure interrupt 0 sense control to fire on the pin's low level (ISC01, ISC00 set to 00)
	MCUCR &= ~((1 << ISC01) | (1 << ISC00));
	
	// Enable external interrupt request 0
	GICR |= 1 << INT0;
}

/*------------------------------------------------------------------------
 *
 *	Timer 1 Initialization function
 *	- CTC mode, channel A
 *	- Prescaler set to 8
 *	- Reference: ATmega16 manual (pdf) p.100
 *			 
 *-----------------------------------------------------------------------*/
void TIMER1_init(uint16_t timer1_target_value){
	// Set the timer 1 CTC mode (clear timer on compare) and prescaler to 8
	TCCR1A = (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);

	// Set Output Compare Register 1 to timer 1's target value (as per the datasheet, we write the "HI8" byte first)
	OCR1AH = (uint8_t)(timer1_target_value >> 8);
	OCR1AL = (uint8_t) timer1_target_value;

	// Intitialize timer 1's counter to 0
	TCNT1H = 0;
	TCNT1L = 0;

	// Enable timer 1's Output Compare A Match Interrupt
	TIMSK = (1 << OCIE1A);
}

/*------------------------------------------------------------------------
 *
 *	USART Initialization function
 *	- Asynchronous operation (UMSEL='0'), 8 bit frame (UCSZ2:0="011")
 *	- parity mode disabled (UPM1:0="00"), 1 stop bit (USBS='0')
 *	- asynchronous mode (UCPOL='0')
 *	- Baud rate settings: f_osc=10MHz, BAUD=9600
 *		-> UBRR = (f_osc/16BAUD) - 1 = 64 (UBRRL=0x64)
 *	- RxD and TxD connected at PD0 and PD1
 *	- Receiver and Transmitter Enabled ((1<<RXEN) and (1<<TXEN) in UCSRB)
 *	- RXC flag interrupt enabled (1<<RXCIE) in UCSRB
 *		(Writing this bit to one enables interrupt on the RXC Flag.)
 *	- Reference: ATmega16 manual (pdf) p.150
 *			 
 *-----------------------------------------------------------------------*/
void USART_init(uint16_t ubrr_value){
	/* Set baud rate */
	UBRRH = (uint8_t)(ubrr_value >> 8);
	UBRRL = (uint8_t) ubrr_value;
	
	/* Enable receiver and transmitter */
	/* Writing RXCIE bit to one enables interrupt on the RXC Flag. */
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
	
	/* Set frame format: 8data, 1stop bit */
	/* The URSEL must be one when writing the UCSRC. */
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
	
	return;
}


/*------------------------------------------------------------------------
 *
 *	EXTERNAL INTERRUPT 0 - SOFTWARE INTERRUPT - SOLVER HOOK
 *	Is created with the attribute IS_NAKED, so the compiler does not include a prologue and epilogue
 *			 
 *-----------------------------------------------------------------------*/
ISR(INT0_vect, ISR_NAKED){
	INT0_SOLVER_HOOK();
}

/*------------------------------------------------------------------------
 *
 *	TIMER 1 ISR
 *	Fires with a rate of 40 Hz.
 *	Interrupt for triggering solver's hook to get the number of filled cells and output it to the progress bar
 *			 
 *-----------------------------------------------------------------------*/
ISR(TIMER1_COMPA_vect){
	// Enable update hook action
	hook_action |= 1 << UPDATE_hook_flag;
	
	// Set INT0 pin low to trigger the software interrupt/hook (once this one finishes)
	INT0_PORT &= ~( 1 << INT0_BIT );
}


/*----------------------------------------------------------------
 *
 *	USART Interrupt Handling C function 
 * 
 *	-Note:
 *	 The Receive Complete (RXC) Flag indicates if there 
 *	 are unread data present in the receive buffer.
 *	 This flag is one when unread data exist in the 
 *	 receive buffer, and zero when the receive buffer is empty.
 *
 *----------------------------------------------------------------*/
ISR(USART_RXC_vect){
	// Process received data from buffer.
	// USART Receive Complete (RXC) flag bit is set when there are unread data 
	// in the receive buffer and cleared when the receive buffer is empty.
	
	#ifndef SIMULATION
		uint8_t received =  UDR;
	#else
		uint8_t received =  UDR;
		// Read UDR twice in order to unset the RXC flag bit during simulation
		received =  UDR;
		// Stimuli file uses TCNT2 for USART data input
		received = TCNT2;
	#endif
	
	
	// Finalize commands (handle <CR> and <LF>)
	if (received == 0x0D || received == 0x0A){
		// accept <CR> if it's the first control character
		if (received == 0x0D && (cmd & CRLF_flags) == 0 && cmd != NONE && cmd != INVALID){
			cmd = cmd | CR_flag;
		}
		// accept <LF> if it immediately follows <CR>
		else if (received == 0x0A && (cmd & CRLF_flags) == CR_flag && cmd != NONE && cmd != INVALID){
			cmd = cmd | LF_flag;
		}
		// when an erroneous <LF> is received, reset the command
		else if (received == 0x0A){
			cmd = NONE;
		}
		// anything else (combination or character) is invalid (need to consume characters until a <LF> is received)
		else {
			cmd = INVALID;
		}
	}
	// Initialize commands based on their first character
	else if (cmd == NONE){
		if (received == 'A'){
			cmd = INIT_CMD(CC_AT);
		}
		else if (received == 'C'){
			cmd = INIT_CMD(CC_C);
		}
		else if (received == 'N'){
			cmd = INIT_CMD(CC_N);
		}
		else if (received == 'P'){
			cmd = INIT_CMD(CC_P);
		}
		else if (received == 'S'){
			cmd = INIT_CMD(CC_S);
		}
		else if (received == 'T'){
			cmd = INIT_CMD(CC_T);
		}
		else if (received == 'O'){
			cmd = INIT_CMD(CC_OK);
		}
		else if (received == 'B'){
			cmd = INIT_CMD(CC_B);
		}
		else if (received == 'D'){
			cmd = INIT_CMD(CC_D);
		}
		else {
			cmd = INVALID;
		}
	}
	// Continue commands with more than one base characters, if no control character (i.e. CR or LF) has been received
	else if (cmd != INVALID && (cmd & CRLF_flags) == 0){
		// Check that the command length is within bounds
		if (CMD_BASE_LEN(cmd) < 4){
			// Continue AT command
			if (received == 'T' && IS_CMD(cmd, CC_AT)){
				// increment command's base length (guaranteed to increment only lower 2 bits)
				cmd++;
			}
			// Continue OK command
			else if (received == 'K' && IS_CMD(cmd, CC_OK)){
				// increment command's base length (guaranteed to increment only lower 2 bits)
				cmd++;
			}
			// Continue N<X><Y><VAL> or D<X><Y> command
			else if (received >= '1' && received <= '9' && (IS_CMD(cmd, CC_N) || IS_CMD(cmd, CC_D))){
				// convert ASCII digit to integer and "append" it to the command
				cmd_digits[CMD_BASE_LEN(cmd) - 1] = received - '0';
				
				// increment command's base length (guaranteed to increment only lower 2 bits)
				cmd++;
			}
			// Any other combination yields an invalid command
			else {
				cmd = INVALID;
			}
		}
		// Adding another character to the command would exceed it's length bounds, so the command is invalid
		else {
			cmd = INVALID;
		}
	}
	
	// If a command has finished, trigger the software interrupt to preempt the solver
	if ((cmd & CRLF_flags) == CRLF_flags){
		// If a debugging command has been received, set the break flag in hook_action
		if (cmd == B_CR_LF || cmd == D_X_Y_CR_LF){
			hook_action |= 1 << BREAK_hook_flag;
		}
		
		// Set INT0 pin low to trigger the software interrupt/hook (once this one finishes)
		INT0_PORT &= ~( 1 << INT0_BIT );
	}
	
	return;
}

/*---------------------------------------------------------
 *
 *	Function to select appropriate actions based on the finished USART command
 *
 *----------------------------------------------------------*/
void USART_command_actions(){
	// Check received command against the valid USART commands.
	// A valid command may not be executed if it is not allowed in the FSM's current state.
	// STATE_IDLE accepts AT<CR><LF>, C<CR><LF>, N<X><Y><VALUE><CR><LF>, P<CR><LF>, S<CR><LF>, T<CR><LF> commands
	// STATE_SOLVING accepts AT<CR><LF>, B<CR><LF>, D<X><Y><CR><LF> commands.
	uint8_t command = cmd;
	
	// clear a finished (but not necessarily valid) command
	if ((cmd & CRLF_flags) == CRLF_flags){
		cmd = NONE;
	}
	
	switch (command){
		case AT_CR_LF:
			transmit_ok_response();
			break;
		
		case C_CR_LF:
			if (state == STATE_SOLVING)	break;
			// set sudoku_clear_flag
			sudoku_clear_flag = 1;
			sudoku_clear();
			transmit_ok_response();
			break;
		
		case N_X_Y_VAL_CR_LF:
			if (state == STATE_SOLVING)	break;
			// check sudoku_clear_flag and call clear() function if sudoku_clear_flag is not set
			if (sudoku_clear_flag == 0){
				sudoku_clear_flag = 1;
				sudoku_clear();
			}
			sudoku_insert_value();
			transmit_ok_response();
			break;
		
		case P_CR_LF:
			if (state == STATE_SOLVING)	break;
			state = STATE_SOLVING;
			// clear sudoku_clear_flag
			sudoku_clear_flag = 0;
			transmit_ok_response();
			break;
		
		case S_CR_LF:
			if (state == STATE_SOLVING)	break;
			// initialize indices
			last_sent_x = 1;
			last_sent_y = 1;
			transmit_cell_digit(last_sent_x, last_sent_y);
			break;
		
		case T_CR_LF:
			if (state == STATE_SOLVING)	break;
			// check whether all cell digits have been sent (in which case, transmit D<CR><LF>)
			if (last_sent_x >= 9 && last_sent_y >= 9){
				transmit_done_response();
				break;
			}
			// increment indices (such that data is sent row-wise)
			last_sent_y++;
			if (last_sent_y > 9){
				last_sent_y = 1;
				last_sent_x++;
			}
			transmit_cell_digit(last_sent_x, last_sent_y);
			break;
		
		case OK_CR_LF:
			// Do nothing
			break;
		
		case B_CR_LF:
			if (state == STATE_IDLE)	break;
			state = STATE_IDLE;
			transmit_ok_response();
			break;
		
		case D_X_Y_CR_LF:
			if (state == STATE_IDLE)	break;
			transmit_cell_digit(cmd_digits[0], cmd_digits[1]);
			break;
	}
}

/*---------------------------------------------------------
 *
 *	Function to insert USART received data to the Sudoku 
 *	allocated memory space.
 *	-This function stores <VALUE> field in the memory, based 
 *	 on the value of <X> and <Y> variables. 
 *
 *----------------------------------------------------------*/
void sudoku_insert_value(){	
	// Received X & Y are 1-based.
	uint8_t x = cmd_digits[0];
	uint8_t y = cmd_digits[1];
	uint8_t decimal_digit = cmd_digits[2];
	uint8_t digit = convert_decimal_digit_to_onehot(decimal_digit);
	
	sudoku_cells[9*(x-1)+(y-1)] = digit;
	
	// Update solver_input data flags & masks
	// --------------------------------------
	
	// Handle 9 differently
	if (decimal_digit == 9){
		// Remove 9 from CRM
		si_MDR(x) &= ~( 1 << CRM_9 );
		
		// Handle a 9 on the 9-th column even more differently
		if (y == 9){
			// Remove 9 from NCM and CBM_3 and mark cell as clue
			si_FCC &= ~( 1 << NCM_9 );
			si_MDR(x) &= ~( (1 << CRCE_9) | 1 << 3 );
		}
		else {
			// Remove 9 from NCM and CBM_3 and mark cell as clue
			si_NCM &= ~( 1 << (y-1) );
			si_MDR(x) &= ~( 1 <<  ((y-1) / 3 + 1));
			si_CRCE(x) &= ~( 1 << (y-1) );
		}
	}
	else {
		// Mark cell as clue
		if (y == 9)		si_MDR(x) &= ~(1 << CRCE_9);
		else			si_CRCE(x) &= ~( 1 << (y-1) );
		
		// Remove digit from CRM, CMi, and CBMs in all 3 rows
		si_CRM(x) &= ~digit;
		si_CMi(y) &= ~digit;
		si_CBM_i(x, y) &= ~digit;
		if (x % 3 == 0){
			si_CBM_i(x-2, y) &= ~digit;
			si_CBM_i(x-1, y) &= ~digit;
		}
		if (x % 3 == 1){
			si_CBM_i(x+1, y) &= ~digit;
			si_CBM_i(x+2, y) &= ~digit;
		}
		if (x % 3 == 2){
			si_CBM_i(x-1, y) &= ~digit;
			si_CBM_i(x+1, y) &= ~digit;
		}
	}
		
	// Increment FCC (bits 7:1)
	si_FCC += 2;
}

/*----------------------------------------------------
 *
 *	Clear Sudoku data. 
 *	Initializes all cells with 0 and sets the filled cells number to 0.
 *
 *----------------------------------------------------*/
void sudoku_clear(){
	for (uint8_t i=0; i<81; i++){
		sudoku_cells[i] = 0x0F;
	}
	
	// Initialize solver_input data
	for (uint8_t i=0; i<9*SOLVER_INPUT_ROW_SIZE + 2; i++){
		solver_input[i] = 0;
	}
	
	// Set all legal digit masks to 1s and all cell empty flags to 1s
	for (uint8_t row=1; row<=9; row++){
		si_CRCE(row) = 0xFF;
		si_MDR(row) = 0x3E;
		si_CRM(row) = 0xFF;
		si_CBM_i(row, 1) = 0xFF;
		si_CBM_i(row, 4) = 0xFF;
		si_CBM_i(row, 7) = 0xFF;
	}
	
	for (uint8_t col=1; col<=9; col++){
		si_CMi(col) = 0xFF;
	}
	
	// Set MDR's "first row of band" bit to 1 for rows 1, 4, 7
	si_MDR(1) |= 1;
	si_MDR(4) |= 1;
	si_MDR(7) |= 1;
	
	si_NCM = 0xFF;
	si_FCC = 1;
}

/*-----------------------------------------------------------
 *
 *	Transmit OK Response function for USART input data
 *	- Part of this code is from the ATmega16 manual (p. 151)
 *	- Response will be OK<CR><LF> (0x4F 0x4B 0x0D 0x0A)
 *
 *-----------------------------------------------------------*/
void transmit_ok_response(){
	send_char('O');
	send_char('K');
	send_char(0x0D);	// '<CR>'
	send_char(0x0A);	// '<LF>'
}

/*-----------------------------------------------------------
 *
 *	Transmit DONE Response function for USART input data and set USART FSM state to IDLE
 *	- Part of this code is from the ATmega16 manual (p. 151)
 *	- Response will be D<CR><LF> (0x44 0x0D 0x0A)
 *
 *-----------------------------------------------------------*/
void transmit_done_response(){
	send_char('D');
	send_char(0x0D);	// '<CR>'
	send_char(0x0A);	// '<LF>'
}

/*-----------------------------------------------------------
 *
 *	Transmit digit response function for USART
 *	- Accepts X and Y grid coordinates (1-based)
 *	- Retrieves the corresponding cell digit
 *	- Part of this code is from the ATmega16 manual (p. 151)
 *	- Response will be N<X><Y><CR><LF>
 *
 *-----------------------------------------------------------*/
void transmit_cell_digit(uint8_t x, uint8_t y){
	uint8_t decimal_digit = convert_onehot_digit_to_decimal(sudoku_cells[9*(x-1) + y-1]);
	
	send_char('N');
	send_char('0' + x);
	send_char('0' + y);
	send_char('0' + decimal_digit);
	send_char(0x0D);	// '<CR>'
	send_char(0x0A);	// '<LF>'
};

/*-----------------------------------------------------------
 *
 *	This function is used to transmit AVR responses 
 *	- The UDRE Flag indicates if the transmit buffer (UDR) is ready to receive new data.
 *	- If UDRE is one, the buffer is empty, and therefore ready to be written.
 *
 *-----------------------------------------------------------*/
void send_char(uint8_t data){
	// Wait for empty transmit buffer
	while(!(UCSRA & (1<<UDRE)));
	
	#ifndef SIMULATION
		UDR = data;
	#else
		TCNT2 = data;
	#endif
}


uint8_t convert_decimal_digit_to_onehot(uint8_t decimal_digit){
	if (decimal_digit == 0)	return 0x0F;
	
	return 1 << (decimal_digit-1);
}

uint8_t convert_onehot_digit_to_decimal(uint8_t digit){
	if (digit == 0x0F)	return 0;
	
	uint8_t decimal_digit = (digit == 0)? 9 : 0;
	while (digit != 0){
		digit = digit >> 1;
		decimal_digit++;
	}
	return decimal_digit;
}
