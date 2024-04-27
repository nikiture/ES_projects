/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 27 aprile 2024, 9.27
 */


#include "xc.h"
#include "timer.h"
#include<string.h>
#include<stdio.h>
#define CS1 LATBbits.LATB3
#define CS2 LATBbits.LATB4
#define CS3 LATDbits.LATD6


int mask_3_lsb = 0xFF - 7;
int meas_buf_x [5];
int meas_buf_y [5];
int meas_buf_z [5];
int meas_idx = 0;
#define tx_buf_size 60
#define msg_size 30
char uart_out [tx_buf_size] = {0};
int write_idx = 0, read_idx = 0;

void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

int spi_write(unsigned int addr) {
    int value_read = 0;
    //int data;
    //send throgh SPI address
    //CS1 = 0;
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = addr;
    //empty the buffer
    while (SPI1STATbits.SPIRBF == 0);
    value_read = SPI1BUF;
    //CS1 = 1;    
    /*//send 0x00 address
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;
    //read value in buffer
    while (SPI1STATbits.SPIRBF == 0);
    data = SPI1BUF;
    //store value from buffer in value to return with proper operations
    value_read +=  data / 8; //3 least significant bits of first register are not relevant to the data
    //send 0x00 address
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = 0x00;
    //read value in buffer
    while (SPI1STATbits.SPIRBF == 0);*/
    return value_read;
}

void __attribute__ ((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    //LATAbits.LATA0 = 1;
    if (read_idx == write_idx) {
        //LATAbits.LATA0 = 1;
        IEC0bits.U1TXIE = 0;
    } else {
        U1TXREG = uart_out [read_idx % tx_buf_size];
        //read_idx = (read_idx + 1)%tx_buf_size;
        read_idx ++;
        //LATGbits.LATG9 = 1;
        IFS0bits.U1TXIF = 0;
    }
    //LATAbits.LATA0 = 0;
}

/*
void __attribute__ ((__interrupt__, __auto_psv__)) _T4Interrupt () {
    IFS1bits.T4IF = 0;
    if (write_idx != 0)
        for (int i = 0; i < tx_buf_size; i++) {
            LATAbits.LATA0 = 1;
            while (U1STAbits.UTXBF == 1);
            U1TXREG = uart_out [i];
            LATAbits.LATA0 = 0;
        }
}
*/

void __attribute__ ((__interrupt__, __auto_psv__)) _T3Interrupt () {
    //LATAbits.LATA0 = 1;
    CS3 = 0;
    spi_write (0x42|0x80);
    meas_buf_x [meas_idx] = spi_write (0x00) & mask_3_lsb;
    meas_buf_x [meas_idx] = meas_buf_x [meas_idx] | (spi_write (0x00) << 8);
    meas_buf_y [meas_idx] = spi_write (0x00) & mask_3_lsb;
    meas_buf_y [meas_idx] = meas_buf_y [meas_idx] | (spi_write (0x00) << 8);
    meas_buf_z [meas_idx] = spi_write (0x00) & mask_3_lsb;
    meas_buf_z [meas_idx] = meas_buf_z [meas_idx] | (spi_write (0x00) << 8);
    meas_idx ++;
    IFS0bits.T3IF = 0;
    CS3 = 1;
    //LATAbits.LATA0 = 0;
}
int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISDbits.TRISD6 = 0;
    
    //setup spi
    TRISAbits.TRISA1 = 1;
    TRISFbits.TRISF12 = 0;
    TRISFbits.TRISF13 = 0;

    RPINR20bits.SDI1R = 0b0010001;
    RPOR12bits.RP109R = 0b000101;
    //RPOR12bits.RP109R
    RPOR11bits.RP108R = 0b000110;

    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.MODE16 = 0;
    SPI1CON1bits.CKP = 1;
    SPI1CON1bits.PPRE = 0; //64:1 primary prescaler
    SPI1CON1bits.SPRE = 5; //1:1 secondary prescaler

    SPI1STATbits.SPIEN = 1;
    CS1 = CS2 = CS3 = 1;
    //turn on and setup spi
    CS3 = 0;
    spi_write(0x4B);
    spi_write(0x01);
    CS3 = 1;
    //transition to active mode
    //write 0x00 on register 0x4C
    CS3 = 0;
    spi_write(0x4C);
    spi_write(0x00);
    CS3 = 1;
    //transition to 25 khz
    //write 0b110 on register 0x4c
    CS3 = 0;
    spi_write (0x4C);
    spi_write (0b110);
    CS3 = 1;
    
    //RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    const int baud_rate = 9600;
    U1BRG = ((72000000 / 16) / baud_rate) - 1;
    //set uart flag to trigger when buffer has at least one space free 
    U1STAbits.UTXISEL0 = 1;
    U1STAbits.UTXISEL1 = 0;
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1TXIE = 1;
    


    float mag_average [3];
    char out_msg [msg_size];
    
    tmr_setup_period (TIMER1, 10);
    tmr_setup_period (TIMER3, 40);
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    while (1) {
        algorithm ();
        IEC0bits.T3IE = 0;
        if (meas_idx > 4) {
            //LATAbits.LATA0 = 1;
            meas_idx = 0;
            mag_average [0] = mag_average [1] = mag_average [2] = 0;
            for (int i = 0; i < 5; i++) {
                mag_average [0] += meas_buf_x [i] /8;
                mag_average [1] += meas_buf_y [i] /8;
                mag_average [2] += meas_buf_z [i] /8;
            }
            for (int i = i; i < 3; i++) {
                mag_average [i] /= 5.0;
            }
            IEC0bits.T3IE = 1;
            
            sprintf (out_msg, "MAG,%.3f,%.3f,%.3f;\n", mag_average [0], mag_average [1], mag_average[2]);
                //LATGbits.LATG9 = 1;
            //sprintf (out_msg, "testT");
            //sprintf (out_msg, "%f\n", (double) mag_average [0]);
            //turn off uart interrupt enabler
            IEC0bits.U1TXIE = 0;
            //IEC1bits.T4IE = 0;
            //sprintf (out_msg, "W:%d; R:%d;\n", write_idx, read_idx);
            
            for (int i = 0; i < strlen (out_msg); i++) {
                uart_out [write_idx % tx_buf_size] = out_msg [i];
                //write_idx = (write_idx + 1) % tx_buf_size;
                write_idx ++;
                //write_idx = write_idx % tx_buf_size;
            }
            
            /*for (int i = 0; i < strlen (out_msg); i++) {
                while (U1STAbits.UTXBF == 1);
                U1TXREG = out_msg [i];
            }*/
            //reenable uart interrupt 
            /*U1TXREG = uart_out [read_idx % tx_buf_size];
            read_idx ++;
            */
            IEC0bits.U1TXIE = 1;
            //IEC1bits.T4IE = 1;
            //LATAbits.LATA0 = 0;
        }
        //LATAbits.LATA0 = IFS0bits.U1TXIF;
        /*if (IEC0bits.U1TXIE != 1)
            IEC0bits.U1TXIE = 1;
        */
        //if (IEC0bits.T3IE != 1)
        IEC0bits.T3IE = 1;
        
        
        
        
        tmr_wait_period (TIMER1);
        //LATAbits.LATA0 = (meas_idx > 5);
        //LATAbits.LATA0 = !LATAbits.LATA0;
    }
    return 0;
}
