/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 21 marzo 2024, 13.02
 */


#include <p33EP512MU810.h>

#include "xc.h"
#include "timer.h"
#define baud_rate 9600
#define ubrg_value ((72000000 / 16) / baud_rate) - 1
//char rec;

/*part 1*/
#if 0
void __attribute__ ((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
        //IFS0bits.U1RXIF = 0;/**/
        
        //if (U1STAbits.URXDA == 1) {
        //LATAbits.LATA0 = !LATAbits.LATA0;
    char rec;
    rec = U1RXREG;
    //U1STAbits.UTXEN = 1;
    U1TXREG = rec;
    //U1STAbits.UTXEN = 0;
    //tmr_setup_peiriod (TIMER2, 10);
    //U1TXREG = U1RXREG;
    IFS0bits.U1RXIF = 0;
}


int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;
    /*configure UART1 to mikrobus 2*/
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    //RPINR18bits.U1RXR = 64
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    //setup for 9600 bds
    
    //long ubrg_value = ((72000000 / 16) / baud_rate) - 1;
    U1BRG = ubrg_value;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    //set the flag for reception
    U1STAbits.URXISEL = 0b00;
    //char msg = "test";
    //U1TXREG = 'T';
    /*for (int i = 0; i < 5; i++) {
        U1TXREG = msg [i];
    }*/
    /*RPINR0bits.INT1R = ; //map interrupt INT1 to RPI75 (pin of rd11)
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;*/
    
    
    //U1TXREG = 'T';
    //U1STAbits.UTXEN = 0;
    while (1) {
    /*basic*/
        /**/
        //LATGbits.LATG9 = !U1STAbits.TRMT;
        /*if (IFS0bits.U1RXIF == 1) {
            IFS0bits.U1RXIF = 0;/**/
        //if (U1STAbits.URXDA == 1) {
        /*    LATAbits.LATA0 = !LATAbits.LATA0;
            /*rec = U1RXREG;
            U1TXREG = rec;*/
            //U1TXREG = U1RXREG;
        //}
        
        
    }
    return 0;
}
#elif 1
/*part 2*/
char msg [3];
int value_rec = 0; // "flag" for Uart char reception
int LD_toggle = 1;//variable for enabling of LED2 blinking
int button, button_press = 0; //"flags" for buttons pressed
int char_read = 0;//counter for characters received through the UART

void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt () {
    //tmr_wait_ms(TIMER2, 50);
    IFS1bits.T4IF = 0;
    IEC1bits.T4IE = 0;
    //check if button that triggered interrupt is still triggered
    if (button == 2) { 
        if (PORTEbits.RE8 == 1) {  
            button_press = 2;
        }
    }
    if (button == 3) {
        if (PORTEbits.RE9 == 1) {
            button_press = 3;
        }
    }
        /**/
}
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt () {
    IFS1bits.INT1IF = 0;
    IFS1bits.T4IF = 0;
    IEC1bits.T4IE = 1;
    button = 3;
    tmr_setup_period(TIMER4, 10);   
}

void __attribute__((__interrupt__, __auto_psv__)) _INT2Interrupt () {
    IFS1bits.INT2IF = 0;
    IFS1bits.T4IF = 0;
    IEC1bits.T4IE = 1;
    button = 2;
    tmr_setup_period(TIMER4, 10);    
}


void __attribute__ ((__interrupt__, __auto_psv__)) _T3Interrupt () {
    if (LD_toggle == 1) { //if enabled (based on UART inputs) toggle LED2
        LATGbits.LATG9 = !LATGbits.LATG9;
    }
    IFS0bits.T3IF = 0;
}
int idx = 0;
char val_msg [2] = "LD";
void __attribute__ ((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    /*for (int i = 0; i < 3; i++) {
        msg [i] = U1RXREG; //store each of the 3 chars on the buffer in a string 
    }
    char_read += 3;
    value_rec = 1;*/
    msg [idx] = U1RXREG;
    if (idx <= 1 && msg [idx] == val_msg [idx]) {//check if value currently received corresponds to
        //the character of a valid string, if so continue forming the string
        idx ++;      
    } else { //3 characters stored or invalid string formed
        if (idx > 1) {//3 characters read, the first two are valid
            value_rec = 1; //"flag" the main function that input from the Uart has been received
        }
        idx = 0; //restart string formation from the beginning
    }
    char_read ++;
    
    IFS0bits.U1RXIF = 0;
}
void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}
int main() {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;
    /*test of the "D=.."*/
    /*
    tmr_setup_period(TIMER1, 7);
    /**/
    tmr_setup_period (TIMER1, 10); //timer of the main function
    int missed_deadlines = 0;
    tmr_setup_period(TIMER3, 200); //timer for the LED2 blinking
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    
    
    
    /*configure UART1 to mikrobus 2*/
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    //RPINR18bits.U1RXR = 64
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    U1BRG = ubrg_value;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    //set the flag for reception
    U1STAbits.URXISEL = 0b00; //flag raised when 3 characters are read
    
    RPINR1bits.INT2R = 0x58; //map interrupt INT2 to RPI88 (pin of button T2)
    IFS1bits.INT2IF = 0;
    IEC1bits.INT2IE = 1;
    
    RPINR0bits.INT1R = 0x59; //map interrupt INT1 to RPI 89 (pin corrensonding to button T3)
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;
    
    while(1) {
        algorithm();
        // code to handle the assignment
        if (value_rec == 1) { //process received string if new message from UART has been received
            if (msg [0] == 'L' && msg [1] == 'D') {
                if (msg [2] == '1') { // if "LD1" toggle LED1
                    LATAbits.LATA0 = !LATAbits.LATA0;
                }
                else if (msg [2] == '2') { //if "LD2" toggle if the blinking is enabled or not
                    LD_toggle = !LD_toggle;
                }
            }
            value_rec = 0;
        }
        switch (button_press) {
            case 2://button T2: number of chars received
                U1TXREG = 'C';
                U1TXREG = '=';
                U1TXREG = '0' + ((char_read / 10) % 10);
                U1TXREG = '0' + (char_read % 10);
                button_press = 0;
                break;
            case 3://button T3: number of deadlines missed
                U1TXREG = 'D';
                U1TXREG = '=';
                U1TXREG = '0' + ((missed_deadlines / 10) % 10);
                U1TXREG = '0' + (missed_deadlines % 10);
                button_press = 0;
                break;
            default:
                break;
        }
        missed_deadlines += tmr_wait_period(TIMER1); //if the deadline is missed the counter of missed deadlines is increasd by 1, otherwise by 0
    }
}
#endif
