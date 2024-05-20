/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 16 maggio 2024, 12.09
 */


#include "xc.h"
#include "parser.h"
#include "timer.h"

#define WAIT_FOR_START (0)
#define EXECUTION (1)


int CONTROL_STATE = 0;




#define OC_1 1
#define OC_2 2
#define OC_3 3
#define OC_4 4


void set_pwm (int oc, int duty_cycle) {
    //set how long the output from the selected output compare's pwm is set on high as a percentage of one period
    switch (oc) {
        case OC_1:
            OC1R = duty_cycle * (OC1RS / 100);
            break;
        case OC_2:
            OC2R = duty_cycle * (OC2RS / 100);
            break;
        case OC_3:
            OC3R = duty_cycle * (OC3RS / 100);
            break;
        case OC_4:
            OC4R = duty_cycle * (OC4RS / 100);
            break;
    }
}

void move_forward (int duty_cycle) {
    if (duty_cycle > 100)
        duty_cycle = 100;
    set_pwm (OC_2, duty_cycle);
    set_pwm (OC_4, duty_cycle);
    set_pwm (OC_1, 0);
    set_pwm (OC_3, 0);
}

void stop () {
    set_pwm (OC_2, 0);
    set_pwm (OC_4, 0);
    set_pwm (OC_1, 0);
    set_pwm (OC_3, 0);
}

void steer_right () {
    set_pwm (OC_2, 90);
    set_pwm (OC_4, 0);
    set_pwm (OC_1, 0);
    set_pwm (OC_3, 0); 
}

void steer_left () {
    set_pwm (OC_2, 0);
    set_pwm (OC_4, 90);
    set_pwm (OC_1, 0);
    set_pwm (OC_3, 0);
}
void move_backward (int duty_cycle) {
    set_pwm (OC_2, 0);
    set_pwm (OC_4, 0); 
    set_pwm (OC_3, duty_cycle);
    set_pwm (OC_1, duty_cycle);
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt () { //interrupt for button debouncing
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 0;
    if (PORTEbits.RE8 == 1) {   
        if (CONTROL_STATE == WAIT_FOR_START) 
            CONTROL_STATE = EXECUTION;
        else
            CONTROL_STATE = WAIT_FOR_START;
    }
}
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt () {
    IFS1bits.INT1IF = 0;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    //10 ms "wait" for debouncing
    tmr_setup_period(TIMER2, 10);    
} 




typedef struct {
    int command;
    int duration;
} instruction;
/*#define STATE_DOLLAR (1)
#define STATE_TYPE (2)
#define STATE_PAYLOAD (3)

typedef struct {
    int state;
    char msg_type [6];
    char msg_payload [16];
    int type_idx = 0;
    int payload_idx;
    
} parser_state;
*/
//FIFO (circular buffer) of instructions
instruction commands_list [10];
int inst_write_idx = 0;
int inst_read_idx = 0;
int instruction_counter = 0;


int instruction_time = 0;
void execute_commands () { 
    //control wheels based on current command in FIFO queue: 1 for forward, 2 for left steering, 3 for right steering and 4 for backward movement 
    instruction* curr_instruction = &(commands_list [inst_read_idx]);
    switch (curr_instruction->command) {
        case 1:
            move_forward (90);
            break;
        case 2:
            steer_left ();
            break;
        case 3:
            steer_right ();
            break;
        case 4:
            move_backward (90);
            break;
        default: //fifo is clear, no action
            stop ();
            return;
            break;
    }
    //update time taken for this instruction and move to next instruction in FIFO if command's duration has elapsed
    instruction_time ++;
    if (instruction_time >= curr_instruction->duration) {
        instruction_time = 0;
        curr_instruction->command = 0;
        instruction_counter --;
        inst_read_idx++;
        inst_read_idx = inst_read_idx % 10;
    } 
}
#define rx_buf_size 20
char in_buf [rx_buf_size];
int in_read_idx = 0;
int in_write_idx = 0;
int chars_to_parse = 0;

void __attribute ((__interrupt__, __auto_psv__)) _U1RXInterrupt () {
    //LATAbits.LATA0 = 1;
    /*while (U1STAbits.URXDA == 1) {
        in_buf [in_write_idx] = U1RXREG;
        in_write_idx ++;
        in_write_idx %= rx_buf_size;
        chars_to_parse ++;
    }*/
    char value = U1RXREG;
    in_buf [in_write_idx] = value;
    in_write_idx ++;
    in_write_idx %= rx_buf_size;
    chars_to_parse ++;
    //if (U1STAbits.URXDA == 1)
    IFS0bits.U1RXIF = 0;
}
char type_string [] = "PCCMD"; 
int get_instruction (parser_state* ps, int * x, int * t) {
    //check if type from parser state is compatible with expected types, 
    //check needed for acknowledgement message 
    for (int i = 0; type_string [i] != '\0'; i++) {
        if (type_string [i] != ps->msg_type [i])
            return 0;
    }
    //check if at least two numbers are in the payload: 
    //for a valid instruction 2 values are necessary: the type of command (forward/backward/left/right) and the duration in ms
    //two numbers are present in the payload if neither the first character of the payload string nor the character after the first next_value call are '\0'
    int i = 0;
    if (ps->msg_payload [i] == '\0') return 0;
    *x = extract_integer(ps->msg_payload);
    i = next_value(ps->msg_payload, i);
    if (ps->msg_payload [i] == '\0') return 0;
    *t = extract_integer(ps->msg_payload + i);
    return 1;
}
/*int parser (parser_state * ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->type_idx = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type [ps->type_idx] = '\0';
                ps->payload_idx = 0;
            } else if (ps->type_idx == 6) {
                ps-> state = STATE_DOLLAR;
                ps->type_idx = 0;
            } else {
                ps->msg_type [ps->type_idx] = byte;
                ps->type_idx++;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR;
                ps->msg_payload[ps->payload_idx] = '\0';
            }
            break;
        default:
            break;
    }
    
}
*/
#define tx_buf_size 24
char uart_buf [tx_buf_size];
int tx_write_idx = 0;
int tx_read_idx = 0;

void upload_to_uart_buf (char* out_msg, char* uart_out) {
    for (int i = 0; out_msg [i] != '\0'; i++) {
        uart_out [tx_write_idx % tx_buf_size] = out_msg [i];
        tx_write_idx ++;
        tx_write_idx %= tx_buf_size;
    }
}
void upload_char_to_uart_buf (char out_char, char* uart_out) {
    uart_out [tx_write_idx % tx_buf_size] = out_char;
    tx_write_idx ++;
    tx_write_idx %= tx_buf_size;
}

void __attribute__ ((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    //LATAbits.LATA0 = 1;
    if (tx_read_idx == tx_write_idx) {
        IEC0bits.U1TXIE = 0;
    } else {
        U1TXREG = uart_buf [tx_read_idx % tx_buf_size];
        tx_read_idx ++;
        tx_read_idx %= tx_buf_size;
        IFS0bits.U1TXIF = 0;
    }
    //LATAbits.LATA0 = 0;
}
#define baud_rate 9600
int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    TRISAbits.TRISA0 = 0;
    
    TRISBbits.TRISB8 = TRISFbits.TRISF1 = 0;//pins for left and right indicators set to output
    
    
    /*configure UART1 to mikrobus 2*/
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    //RPINR18bits.U1RXR = 64
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    //setup for 9600 bds
    
    int ubrg_value = ((72000000 / 16) / baud_rate) - 1;
    U1BRG = ubrg_value;
    
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    //set the flag for reception
    U1STAbits.URXISEL = 0b00;
    
    //UART transmission interrupt flag raise
    U1STAbits.UTXISEL0 = 1;
    U1STAbits.UTXISEL1 = 0;
    
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1TXIE = 1;
    
    
    
    //OC setup
    TRISDbits.TRISD1 = TRISDbits.TRISD2 = TRISDbits.TRISD3 = TRISDbits.TRISD4 = 0;
    //peripheral clock, sync with ocxrs
    OC1CON2bits.SYNCSEL = OC2CON2bits.SYNCSEL = OC3CON2bits.SYNCSEL = OC4CON2bits.SYNCSEL = 0b11111;
    OC1CON1bits.OCTSEL = OC2CON1bits.OCTSEL = OC3CON1bits.OCTSEL = OC4CON1bits.OCTSEL = 0b111;
    //Fp = 72 Mhz, for 10 khz ocxrs = 7200
    OC1RS = OC2RS = OC3RS = OC4RS = 7200;
    /*motor control:
     * oc3: right wheels backward
     * oc1: left wheels backward
     * oc2: left wheels forward
     * oc4: right wheels forward
    */
    //pin remap for OC
    RPOR0bits.RP65R = 0b010000;
    RPOR1bits.RP66R = 0b010001;
    RPOR1bits.RP67R = 0b010010;
    RPOR2bits.RP68R = 0b010011;
    
    OC1CON1bits.OCM = OC2CON1bits.OCM = OC3CON1bits.OCM = OC4CON1bits.OCM = 6;
    
    
    //button press control
    //map INT1 to pin RE8
    RPINR0bits.INT1R = 0x58; //map interrupt INT1 to RPI88 (pin of button T2)
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;
    
    parser_state ps;
    ps.state = STATE_DOLLAR;
    ps.index_type = 0;
    ps.index_payload = 0;
    int msg_state = 0;
    char out_msg [16];
    char response_prefix [7] = "$MACK,"; //start of acknowledgement message both for correct and wrong message
    int x, t;
    tmr_setup_period (TIMER1, 1);
    int res;
    IFS0bits.U1TXIF = 1;
    int count_1_Hz = 0;
    while (1) {
        //check for valid or erroneous message from parser
        switch (msg_state) {
            case ERR_MESSAGE:
                //LATAbits.LATA0 = 1;
                //"$MACK,0*" TO BE SENT THROUGH THE UART 
                for (int i = 0; i < 6; i++) {
                    out_msg [i] = response_prefix [i];
                }
                out_msg [6] = '0';
                out_msg [7] = '*';
                out_msg [8] = '\0';
                upload_to_uart_buf(out_msg, uart_buf);
                IEC0bits.U1TXIE = 1;
                msg_state = NO_MESSAGE;
                break;
            case NEW_MESSAGE:
                //LATAbits.LATA0 = 1;
                for (int i = 0; i < 7; i++) {
                    out_msg [i] = response_prefix [i];
                }
                if (instruction_counter >= 10) {//fifo full, error message and no loading on fifo
                    out_msg [6] = '0';
                } else if (get_instruction (&ps, &x, &t)){
                    //correct amount of data in payload
                    //"$MACK,1*" TO BE SENT THROUGH THE UART
                    //values to be uploaded in instruction FIFO queue
                    out_msg [6] = '1';
                    instruction_counter ++;
                    
                    commands_list [inst_write_idx].command = x;
                    commands_list [inst_write_idx].duration = t;
                    inst_write_idx ++;
                    inst_write_idx %= 10; 
                } else { 
                    //not enough data in payload string of parser state (either one or zero values)
                    //"$MACK,0*" TO BE SENT THROUGH THE UART
                    out_msg [6] = '0';
                }
                
                out_msg [7] = '*';
                out_msg [8] = '\0';
                msg_state = NO_MESSAGE;
                upload_to_uart_buf(out_msg, uart_buf);
                IEC0bits.U1TXIE = 1;
                break;
            default:
                //no action if no message completed from the parser
                break;
        }
        //data parsing 
        //LATAbits.LATA0 = chars_to_parse;
        //LATAbits.LATA0 = res;
        //process bytes received through UART and check for messages using protocol
        if (chars_to_parse > 0) {
            IEC0bits.U1RXIE = 0;
            chars_to_parse --;
            IEC0bits.U1RXIE = 1;
            //upload_char_to_uart_buf(in_buf [in_read_idx], uart_buf);
            //IFS0bits.U1TXIF = 1;
            IEC0bits.U1TXIE = 1;
            msg_state = parse_byte(&ps, in_buf [in_read_idx]);
            //LATAbits.LATA0 = parse_byte(&ps, in_buf [in_read_idx]);
            /*if (msg_state != NO_MESSAGE)
                LATAbits.LATA0 = 1;
            */
            in_read_idx ++;
            in_read_idx %= rx_buf_size;
        }
        //blinking and execution control
        switch (CONTROL_STATE) {
            case WAIT_FOR_START:
                stop ();
                count_1_Hz ++;
                if (count_1_Hz >= 1000) { //frequency of 1 Hz, 1000 times the main loop frequency (1 kHz)
                    //LATAbits.LATA0 = !LATAbits.LATA0;
                    //blink left and right indicators and A0 LED
                    //indicators on pins RB8 and RF1
                    LATBbits.LATB8 = !LATBbits.LATB8;
                    LATFbits.LATF1 = LATAbits.LATA0 = LATBbits.LATB8;
                    count_1_Hz = 0;
                }
                break;
            case EXECUTION:
                //blinking of A0 LED at 1 Hz
                //execution of commands in FIFO queue (if any are present)
                execute_commands();
                LATBbits.LATB8 = LATFbits.LATF1 = 0;
                count_1_Hz ++;
                if (count_1_Hz >= 1000) {
                    LATAbits.LATA0 = !LATAbits.LATA0;
                    count_1_Hz = 0;
                }
                break;
        }
        tmr_wait_period (TIMER1);
        //LATAbits.LATA0 = U1STAbits.OERR;
    }
    return 0;
}
