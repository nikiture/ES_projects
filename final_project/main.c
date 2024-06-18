/*
 * File:   main.c
 * Authors: 
 * Torre Nicolò, S4938354;
 * Andrea Scorrano, S6463777;
 * Girum Molla Desalegn, S6020433;
 * 
 * Created on 16 maggio 2024, 12.09
 */


#include "xc.h"
#include "parser.h"
#include "timer.h"
#include"string.h"
#include "stdio.h"


#define WAIT_FOR_START (0)
#define EXECUTION (1)


int CONTROL_STATE = 0;




#define OC_1 (1)
#define OC_2 (2)
#define OC_3 (3)
#define OC_4 (4)


/*motor control in current pin mapping:
    * oc3: right wheels backward
    * oc1: left wheels backward
    * oc2: left wheels forward
    * oc4: right wheels forward
*/

void set_pwm (int oc, int duty_cycle) {
    //set how long the output from the selected output compare's pwm is set on high
    //by setting the primary register as a percentage of the value in the secondary register
    //while the OC is set on edge-aligned pwm mode with Periferic clock as source and syncronisation with secondary register 
    switch (oc) {
        case OC_1:
            OC1R = duty_cycle * (OC1RS / 100.0);
            break;
        case OC_2:
            OC2R = duty_cycle * (OC2RS / 100.0);
            break;
        case OC_3:
            OC3R = duty_cycle * (OC3RS / 100.0);
            break;
        case OC_4:
            OC4R = duty_cycle * (OC4RS / 100.0);
            break;
    }
}

void move_forward (int duty_cycle) {
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

int instruction_time = 0; //counter of time taken on current command in milliseconds
int command_to_pop = 0; //flag to trigger command popping in switching from execution state to wait_for_start state;
//triggered inside of button press interrupts and used to pop command in main loop

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt () { //interrupt for button debouncing
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 0;
    if (PORTEbits.RE8 == 1) {  
        if (CONTROL_STATE == WAIT_FOR_START) {
            CONTROL_STATE = EXECUTION;
        }
        else {//change to WAIT_FOR_START state and removal of current instruction
            CONTROL_STATE = WAIT_FOR_START;
            if (instruction_time > 0) 
                command_to_pop = 1;
        }
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

//FIFO (circular buffer) of instructions
instruction commands_list [10];
int inst_write_idx = 0; //index used by parsing to add instructions to the FIFO 
int inst_read_idx = 0; //index used by control function to extract instructions from the FIFO
int instruction_counter = 0; //counter of instructions in FIFO; used to check if FIFO full






void pop_command (instruction* curr_command) {
    //reset the instruction time to 0, set the current command to 0 (no action), move to the next item in the command FIFO    
    instruction_time = 0;
    curr_command->command = 0;
    instruction_counter --;
    inst_read_idx++;
    inst_read_idx = inst_read_idx % 10;
}

//minimum distance between vehicle and an obstacle before stopping
#define min_distance 20 

void execute_commands (int distance) { 
    //control wheels based on current command in FIFO queue: 1 for forward movement, 2 for counterclockwise steering, 3 for clockwise steering and 4 for backward movement 
    //if command value of current instruction is 0 the FIFO is empty
    instruction* curr_instruction = &(commands_list [inst_read_idx]);
    //if moving forward/ steering  check for obstacle distance and don't move if closer than 20 cm
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
            return; //if FIFO empty no update on time taken on current task
            break;
    }
    //update time taken for current task and move to next instruction in FIFO if command's duration has elapsed;
    //current setup is to avoid risk of race condition 
    instruction_time ++;
    if (instruction_time >= curr_instruction->duration) {
        pop_command (curr_instruction);
    } 
}
//circular buffer for reception
#define rx_buf_size 20
char rx_buf [rx_buf_size];
int rx_read_idx = 0; //index to add characters on buffer (UART reception ISR)
int rx_write_idx = 0; //index to remove characters from buffer (character parser function))
int chars_to_parse = 0; //counter of elements in circular buffer to check for buffer full/empty

void __attribute ((__interrupt__, __auto_psv__)) _U1RXInterrupt () {
    IFS0bits.U1RXIF = 0;
    //move characters from UART reception register to reception buffer until either buffer full or register are empty
    while (U1STAbits.URXDA == 1) {
        if (chars_to_parse >= rx_buf_size) {
            break;
        }
        //put character on buffer and update index and counter
        rx_buf [rx_write_idx] = U1RXREG;
        rx_write_idx ++;
        rx_write_idx %= rx_buf_size;
        chars_to_parse ++;
    }
}

char type_string [] = "PCCMD"; //string used to check for valid type in the parsing process

int get_instruction (parser_state* ps, int * x, int * t) {
    //check if type from parser state is compatible with expected types, 
    //check needed for acknowledgment message 
    for (int i = 0; type_string [i] != '\0'; i++) {
        if (type_string [i] != ps->msg_type [i])
            return 0;
    }
    //check if at least two numbers are in the payload: 
    //for a valid instruction 2 values are necessary: the type of command (forward/backward/clockwise/counterclockwise) and the duration in ms
    //two numbers are present in the payload if neither the first character of the payload string nor the character after the first next_value call are '\0'
    int i = 0;
    if (ps->msg_payload [i] == '\0') return 0;
    *x = extract_integer(ps->msg_payload);
    i = next_value(ps->msg_payload, i);
    if (ps->msg_payload [i] == '\0') return 0;
    *t = extract_integer(ps->msg_payload + i);
    return 1;
}
//transmission buffer
#define tx_buf_size 16
char uart_buf [tx_buf_size];
int tx_write_idx = 0; //index to add characters on the buffer
int tx_read_idx = 0; //index to read characters from the buffer
int char_to_send = 0; //counter of characters in the buffer still to be sent through the uart

void upload_to_uart_buf (char* out_msg, char* uart_out) {
    //add characters of the input string out_msg to the transmission buffer; if the buffer is full re-enable the transmission ISR and wait for the buffer to empty
    //Transmission ISR interrupted to avoid race conditions on the counter of characters to send
    for (int i = 0; out_msg [i] != '\0'; i++) {
        while (char_to_send >= tx_buf_size) {
            IEC0bits.U1TXIE = 1;
        }
        IEC0bits.U1TXIE = 0;
        char_to_send ++;
        uart_out [tx_write_idx % tx_buf_size] = out_msg [i];
        tx_write_idx ++;
        tx_write_idx %= tx_buf_size;
    }
}

void __attribute__ ((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    //move characters from the transmission buffer to the transmission register until the buffer is empty or the uart register is full
    IFS0bits.U1TXIF = 0;
    while (U1STAbits.UTXBF == 0) {
        if (char_to_send <= 0) {
            break;
        } else {
            U1TXREG = uart_buf [tx_read_idx % tx_buf_size];
            tx_read_idx ++;
            tx_read_idx %= tx_buf_size;
            char_to_send --;
        }
    }
}

float compute_dist_from_volt (float v) {
    //compute distance in meters d_m from analog voltage returned from IR sensor (d_v) 
    //using function "d_m = 2.34 - 4.74 * d_v + 4.06 * d_v^2 - 1.6 * d_v^3 + 0.24 * d_v^4
    return 2.34 - 4.74 * v + 4.06 * v * v - 1.60 * v * v * v + 0.24 * v * v * v * v;
}

#define baud_rate 9600
int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    //IR sensor mounted on buggy mikrobus 2: ADC measure on ANS 11 and 14    
    ANSELBbits.ANSB11 = 1;
    ANSELBbits.ANSB14 = 1;
    
    TRISBbits.TRISB11 = 1;
    TRISBbits.TRISB14 = 1;

    
    TRISAbits.TRISA0 = 0;
    //IR enable pin on b9 
    TRISBbits.TRISB9 = 0;
    TRISGbits.TRISG9 = 0;
    
    TRISBbits.TRISB8 = TRISFbits.TRISF1 = 0;//pins for left and right indicators on buggy
    
    
    /*configure UART1 to mikrobus 2*/
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
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
    //set the flag for reception to trigger on a new character put on the uart register
    U1STAbits.URXISEL = 0b00;
    
    //UART transmission interrupt flag trigger on character transfer 
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;
    
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1TXIE = 1;
    
    
    
    //OC setup
    TRISDbits.TRISD1 = TRISDbits.TRISD2 = TRISDbits.TRISD3 = TRISDbits.TRISD4 = 0;
    //peripheral clock, sync with ocxrs
    OC1CON2bits.SYNCSEL = OC2CON2bits.SYNCSEL = OC3CON2bits.SYNCSEL = OC4CON2bits.SYNCSEL = 0b11111;
    OC1CON1bits.OCTSEL = OC2CON1bits.OCTSEL = OC3CON1bits.OCTSEL = OC4CON1bits.OCTSEL = 0b111;
    //Fp = 72 Mhz, for 10 khz ocxrs = 72 MHz/ 10 kHz = 7200
    OC1RS = OC2RS = OC3RS = OC4RS = 7200;
    /*motor control:
     * oc3: right wheels backward
     * oc1: left wheels backward
     * oc2: left wheels forward
     * oc4: right wheels forward
    */
    //pin remap for OC
    RPOR0bits.RP65R = 0b010000; //OC1 (0b10000) to pin D1
    RPOR1bits.RP66R = 0b010001; //OC2 (0b10001) to pin D2
    RPOR1bits.RP67R = 0b010010; //OC3 (0b10010) to pin D3
    RPOR2bits.RP68R = 0b010011; //OC4 (0b10011) to pin D4
    
    //set all OCs in edge-aligned PWM mode
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
    //automatic sampling and conversion
    AD1CON3bits.ADCS = 8; 
    AD1CON1bits.ASAM = 1;
    AD1CON1bits.SSRC = 7;
    AD1CON3bits.SAMC = 10;
    
    //using scan mode on 2 pins on channel 0
    AD1CON2bits.SMPI = 1;
    AD1CON2bits.CSCNA = 1; // Enable Channel Scanning

    AD1CSSLbits.CSS11=1; // Enable AN11 for scan
    AD1CSSLbits.CSS14=1; // Enable A14 for scan
    //in current setup battery measurements from ADCBUF0, IR distance measurements from ADCBUF1
    
    AD1CON1bits.ADON = 1;
    
    
    
    parser_state ps;
    ps.state = STATE_DOLLAR;
    ps.index_type = 0;
    ps.index_payload = 0;
    
    int msg_state = NO_MESSAGE;
    
    char out_msg [16];
    char response_prefix [7] = "$MACK,"; //start of acknowledgment message both for correct and wrong message
    int x, t;
    
    tmr_setup_period (TIMER1, 1); //control loop of 1kHz frequency
    
    int IR_volt;
    float IR_analog_volt;
    int IR_distance = 150;
    
    int count_1_Hz = 0, count_10_Hz = 0; //counters (in ms) for the 10 Hz task (distance transmission) and the 1 Hz tasks (LED blinking, battery measurement transmission)
    
    while (1) {
        count_1_Hz ++;

        count_10_Hz ++;
        
        //measure values from IR sensor
        IR_volt = ADC1BUF1;
        //conversion from digital to analog value (voltage from pin)
        IR_analog_volt = 3.3 / 1024 * IR_volt;
        IR_distance = compute_dist_from_volt(IR_analog_volt) * 100; // conversion  from pin voltage to estimated distance in cm
        //check for valid or invalid message from parser
        switch (msg_state) {
            case ERR_MESSAGE:
                //"$MACK,0*" TO BE SENT THROUGH THE UART 
                for (int i = 0; i < 6; i++) {
                    out_msg [i] = response_prefix [i];
                }
                out_msg [6] = '0';
                out_msg [7] = '*';
                out_msg [8] = '\n';
                out_msg [9] = '\0';
                IEC0bits.U1TXIE = 0;
                if (U1STAbits.TRMT == 0) //if transmission still ongoing no need to put character in uart buffer to start transmission of new message
                    upload_to_uart_buf(out_msg, uart_buf);
                else {
                    //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                    //to ensure uart tx flag raise from uart module
                    //and the rest of the message in the circular buffer
                    U1TXREG = out_msg [0];
                    upload_to_uart_buf (out_msg + 1, uart_buf);
                }
                IEC0bits.U1TXIE = 1;
                msg_state = NO_MESSAGE;
                break;
            case NEW_MESSAGE:
                for (int i = 0; i < 6; i++) {
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
                out_msg [8] = '\n';
                out_msg [9] = '\0';
                msg_state = NO_MESSAGE;
                IEC0bits.U1TXIE = 0;
                if (U1STAbits.TRMT == 0) //if transmission still ongoing no need to put character in uart buffer to start transmission of new message
                    upload_to_uart_buf(out_msg, uart_buf);
                else {
                    //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                    //to ensure uart tx flag raise from uart module
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
        
        //a continues flow of numbers sent through the uart does not cause overflow in the current implementation
        //in the edge-case a buffer overflow occurs we flush all the elements in the reception buffer and in the uart reception register,
        //send an error message ("$MACK0*") through the uart and reset the parser state to STATE_DOLLAR
        if (U1STAbits.OERR == 1) {
            chars_to_parse = 0;
            ps.state = STATE_DOLLAR;
            for (int i = 0; i < 7; i++) {
                out_msg [i] = response_prefix [i];
            }
            out_msg [6] = '0';
            out_msg [7] = '*';
            out_msg [8] = '\n';
            out_msg [9] = '\0';
            if (U1STAbits.TRMT == 0) //if transmission still ongoing no need to put character in uart buffer to start transmission of new message
                upload_to_uart_buf(out_msg, uart_buf);
            else {
                //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                //to ensure uart tx flag raise from uart module
                //and the rest of the message in the circular buffer
                U1TXREG = out_msg [0];
                upload_to_uart_buf (out_msg + 1, uart_buf);
            }
            IEC0bits.U1TXIE = 1;            
        }
        else {//process bytes received through UART and check for messages using protocol
            while (chars_to_parse > 0) {
                msg_state = parse_byte(&ps, rx_buf [rx_read_idx]);
                IEC0bits.U1RXIE = 0;
                chars_to_parse --;
                IEC0bits.U1RXIE = 1;
                rx_read_idx ++;
                rx_read_idx %= rx_buf_size;
            }
        }
        //if switch from execution state to waiting state pop of current command in FIFO (if started)
        if (command_to_pop == 1) {
            pop_command(&(commands_list [rx_read_idx]));
            command_to_pop = 0;
        }
        //blinking and execution control
        switch (CONTROL_STATE) {
            case WAIT_FOR_START:
                //no movement, LEDs blinking at 1 Hz frequency
                stop ();
                if (count_1_Hz >= 1000) { //frequency of 1 Hz, 1000 times the main loop frequency (1 kHz) (counter reset in the battery measurement transmission task)
                    //blink left and right indicators and A0 LED synchronously
                    //indicators on pins RB8 and RF1
                    LATBbits.LATB8 = !LATBbits.LATB8;
                    LATFbits.LATF1 = LATAbits.LATA0 = LATBbits.LATB8;
                }
                break;
            case EXECUTION:
                //blinking of A0 LED at 1 Hz and indicators turned off
                //execution of commands in FIFO queue (if any are present)
                execute_commands(IR_distance);
                LATBbits.LATB8 = LATFbits.LATF1 = 0;
                if (count_1_Hz >= 1000) {//frequency of 1 Hz, 1000 times the main loop frequency (1 kHz) (counter reset in the battery measurement transmission task)
                    LATAbits.LATA0 = !LATAbits.LATA0;
                }
                break;
        }
        if (count_1_Hz >= 1000) {
            //read values from adc for battery level and send them through the uart
            count_1_Hz = 0;
            
            int batt_lev = ADC1BUF0;           
            float batt_meas = 3.3 / 1024 * batt_lev * 3; 
            sprintf (out_msg, "$MBATT,%.2f*\n", batt_meas);
            IEC0bits.U1TXIE = 0;
            if (U1STAbits.TRMT == 0) //if transmission still ongoing no need to put character in uart buffer to start transmission of new message
                upload_to_uart_buf(out_msg, uart_buf);
            else {
                //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                //and the rest of the message in the circular buffer
                U1TXREG = out_msg [0];
                upload_to_uart_buf (out_msg + 1, uart_buf);
            }
            IEC0bits.U1TXIE = 1;
        }
        if (count_10_Hz >= 100) {//10 Hz frequency is 100 ms period (and 100 loops at 1kHz frequency)
            count_10_Hz = 0;
            sprintf (out_msg, "$MDIST,%d*\n", IR_distance);
            IEC0bits.U1TXIE = 0;
            if (U1STAbits.TRMT == 0) //if transmission still ongoing no need to put character in uart buffer to start transmission of new message
                upload_to_uart_buf(out_msg, uart_buf);
            else {
                //if no ongoing transmission needed to start a new one by putting the first character directly in the uart fifo 
                //to ensure uart tx flag raise from uart module
                //and the rest of the message in the circular buffer
                U1TXREG = out_msg [0];
                upload_to_uart_buf (out_msg + 1, uart_buf);
            }
            IEC0bits.U1TXIE = 1;
        }
        tmr_wait_period(TIMER1);
    }
    return 0;
}
