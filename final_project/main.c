/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 16 maggio 2024, 12.09
 */


#include "xc.h"
#include "parser.h"
#include "timer.h"
#include"string.h"
#include "stdio.h"


/*to do from assignment 1 evaluation:
 * buffers full check
 * more than one character written in uart tx buffer
*/


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
#define min_distance 20
void execute_commands (int distance) { 
    
    //control wheels based on current command in FIFO queue: 1 for forward, 2 for left steering, 3 for right steering and 4 for backward movement 
    instruction* curr_instruction = &(commands_list [inst_read_idx]);
    switch (curr_instruction->command) {
        case 1:
            if (distance >= min_distance) move_forward (90);
            else stop ();
            break;
        case 2:
            if (distance >= min_distance) steer_left ();
            else stop ();
            break;
        case 3:
            if (distance >= min_distance) steer_right ();
            else stop ();
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
#define tx_buf_size 64
char uart_buf [tx_buf_size];
int tx_write_idx = 0;
int tx_read_idx = 0;
int char_to_send = 0;

void upload_to_uart_buf (char* out_msg, char* uart_out) {
    for (int i = 0; out_msg [i] != '\0'; i++) {
        while (char_to_send >= tx_buf_size) /*LATGbits.LATG9 = 1*/;
        //IEC0bits.U1TXIE = 0;
        char_to_send ++;
        //LATGbits.LATG9 = 0;
        uart_out [tx_write_idx % tx_buf_size] = out_msg [i];
        tx_write_idx ++;
        tx_write_idx %= tx_buf_size;
        //IEC0bits.U1TXIE = 1;
    }
}

void upload_char_to_uart_buf (char out_char, char* uart_out) {
    uart_out [tx_write_idx % tx_buf_size] = out_char;
    tx_write_idx ++;
    tx_write_idx %= tx_buf_size;
}

void __attribute__ ((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    //LATAbits.LATA0 = 1;
    IFS0bits.U1TXIF = 0;
    while (U1STAbits.UTXBF == 0) {
        if (char_to_send <= 0) {
            //IEC0bits.U1TXIE = 0;
            break;
        } else {
            U1TXREG = uart_buf [tx_read_idx % tx_buf_size];
            tx_read_idx ++;
            tx_read_idx %= tx_buf_size;
            char_to_send --;
        }
    //LATAbits.LATA0 = 0;
    }
}

float compute_dist_from_volt (float v) {
    return 2.34 - 4.74 * v + 4.06 * v * v - 1.60 * v * v * v + 0.24 * v * v * v * v;
}

#define baud_rate 9600
int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    ANSELBbits.ANSB11 = 1;
    ANSELBbits.ANSB15 = 1;
    
    TRISBbits.TRISB11 = 1;
    TRISBbits.TRISB14 = 1;
    TRISAbits.TRISA0 = 0;
    TRISBbits.TRISB9 = 0;
    TRISGbits.TRISG9 = 0;
    
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
    U1STAbits.UTXISEL0 = 0;
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
    
    
    //adc setup
    //enable IR sensor
    LATBbits.LATB9 = 1;
    //to add logic for adcs and samc
    AD1CON3bits.ADCS = 128; 
    AD1CON1bits.ASAM = 1;
    AD1CON1bits.SSRC = 7;
    AD1CON3bits.SAMC = 23;
    //using scan mode on 2 pins on channel 0
    AD1CON2bits.CHPS = 0; //only channel 0 used
    //AD1CHS0bits.CH0SA = 0b00101; //positive input is AN 5
    //enable scan mode for AN 11 and AN 14/15
    AD1CSSL = 0;
    AD1CSSLbits.CSS11 = 1;
    AD1CSSLbits.CSS14 = 1;    
    AD1CON2bits.CSCNA = 1;
    AD1CON1bits.ADON = 1;
    
    
    
    parser_state ps;
    ps.state = STATE_DOLLAR;
    ps.index_type = 0;
    ps.index_payload = 0;
    int msg_state = 0;
    char out_msg [16];
    char response_prefix [7] = "$MACK,"; //start of acknowledgement message both for correct and wrong message
    int x, t;
    tmr_setup_period (TIMER1, 1);
    int IR_volt;
    float IR_analog_volt;
    int IR_distance = 150;
    //IFS0bits.U1TXIF = 1;
    int count_1_Hz = 0, count_10_Hz = 0;
    while (1) {
        //measure values from IR sensor
        IR_volt = ADC1BUF1;
        IR_analog_volt = 3.3 / 1024 * IR_volt;
        IR_distance = compute_dist_from_volt(IR_analog_volt);
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
                IEC0bits.U1TXIE = 0;
                if (U1STAbits.TRMT == 0) //if transmisssion still ongoing no need to put characeter in uart buffer to start transmission of new message
                    upload_to_uart_buf(out_msg, uart_buf);
                else {
                    //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                    //and the rest of the message in the circular buffer
                    U1TXREG = out_msg [0];
                    upload_to_uart_buf (out_msg + 1, uart_buf);
                }
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
                IEC0bits.U1TXIE = 0;
                if (U1STAbits.TRMT == 0) //if transmisssion still ongoing no need to put characeter in uart buffer to start transmission of new message
                    upload_to_uart_buf(out_msg, uart_buf);
                else {
                    //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                    //and the rest of the message in the circular buffer
                    U1TXREG = out_msg [0];
                    upload_to_uart_buf (out_msg + 1, uart_buf);
                }
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
            //IEC0bits.U1TXIE = 1;
            msg_state = parse_byte(&ps, in_buf [in_read_idx]);
            //LATAbits.LATA0 = parse_byte(&ps, in_buf [in_read_idx]);
            /*if (msg_state != NO_MESSAGE)
                LATAbits.LATA0 = 1;
            */
            in_read_idx ++;
            in_read_idx %= rx_buf_size;
        }
        count_1_Hz ++;
        count_10_Hz ++;
        //blinking and execution control
        switch (CONTROL_STATE) {
            case WAIT_FOR_START:
                stop ();
                if (count_1_Hz >= 1000) { //frequency of 1 Hz, 1000 times the main loop frequency (1 kHz)
                    //LATAbits.LATA0 = !LATAbits.LATA0;
                    //blink left and right indicators and A0 LED
                    //indicators on pins RB8 and RF1
                    LATBbits.LATB8 = !LATBbits.LATB8;
                    LATFbits.LATF1 = LATAbits.LATA0 = LATBbits.LATB8;
                    //count_1_Hz = 0;
                }
                break;
            case EXECUTION:
                //blinking of A0 LED at 1 Hz
                //execution of commands in FIFO queue (if any are present)
                execute_commands(IR_distance);
                LATBbits.LATB8 = LATFbits.LATF1 = 0;
                if (count_1_Hz >= 1000) {
                    LATAbits.LATA0 = !LATAbits.LATA0;
                    //count_1_Hz = 0;
                }
                break;
        }
        if (count_1_Hz >= 1000) {
            //read values from adc for battery level and send them through the uart
            count_1_Hz = 0;
            
            int batt_lev = ADC1BUF0;           
            float batt_meas = 3.3 / 1024 * batt_lev * 3; 
            sprintf (out_msg, "$MBATT,%f*", batt_meas);
            IEC0bits.U1TXIE = 0;
            if (U1STAbits.TRMT == 0) //if transmisssion still ongoing no need to put characeter in uart buffer to start transmission of new message
                upload_to_uart_buf(out_msg, uart_buf);
            else {
                //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                //and the rest of the message in the circular buffer
                U1TXREG = out_msg [0];
                upload_to_uart_buf (out_msg + 1, uart_buf);
            }
            IEC0bits.U1TXIE = 1;
        }
        if (count_10_Hz >= 100) {
            count_10_Hz = 0;
            sprintf (out_msg, "$MDIST,%f*", IR_analog_volt);
            IEC0bits.U1TXIE = 0;
            if (U1STAbits.TRMT == 0) //if transmisssion still ongoing no need to put characeter in uart buffer to start transmission of new message
                upload_to_uart_buf(out_msg, uart_buf);
            else {
                //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                //and the rest of the message in the circular buffer
                U1TXREG = out_msg [0];
                upload_to_uart_buf (out_msg + 1, uart_buf);
            }
            IEC0bits.U1TXIE = 1;
        }
        LATGbits.LATG9 = U1STAbits.TRMT;
        tmr_wait_period (TIMER1);
        //LATAbits.LATA0 = U1STAbits.OERR;
    }
    return 0;
}
