/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 10 aprile 2024, 15.36
 */


#include <p33EP512MU810.h>

#include "xc.h"
#include <string.h>
#include "timer.h"
#define baud_rate 9600
#define ubrg_value ((72000000 / 16) / baud_rate) - 1
#define CS1 LATBbits.LATB3
#define CS2 LATBbits.LATB4
#define CS3 LATDbits.LATD6

int spi_write (unsigned int addr) {
    int value_read = 0;
    int data;
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
    while (SPI1STATbits.SPIRBF == 0);
    data = 
    //store value from buffer in value to return with proper operations
    /*...*/
    return value_read;
}
int main(void) {
    int chip_ID = 0;
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISDbits.TRISD6 = 0;
    TRISAbits.TRISA0 = 0;
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    
    tmr_setup_period(TIMER1, 100);
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    U1BRG = ubrg_value;
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.UTXISEL0 = 0;
    
    
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
    SPI1CON1bits.SPRE = 7; //1:1 secondary prescaler
    
    
    
    SPI1STATbits.SPIEN = 1;
    
    /*part 1
    while (1) {
        //transition to sleep mode: power mode bit to '1'
        //write 0x01 on register 0x4B
        CS3 = 0;
        
        spi_write(0x4B);
        spi_write (0x01);
        CS3 = 1;
        //transition to active mode
        //write 0x00 on register 0x4C
        CS3 = 0;
        spi_write (0x4C);
        spi_write (0x00);
        CS3 = 1;
        //receive chip ID
        CS3 = 0;
        spi_write (0x40|0x80);
        chip_ID = spi_write (0x00);
        CS3 = 1;
        //send chip ID through UART
        char out_msg [50];
        char num_str [47];
        sprintf (out_msg, "ID=");
        sprintf (num_str, "%d", chip_ID);
        strcat (out_msg, num_str);
        for (int i = 0; i < strlen (out_msg); i++){
            while (IFS0bits.U1TXIF == 0 && U1STAbits.TRMT == 0);
            IFS0bits.U1TXIF = 0;
            U1TXREG = out_msg [i];
        }
        tmr_wait_period(TIMER1);
    }
    /**/
    /*part 2*/
    //transition to sleep mode: power mode bit to '1'
    //write 0x01 on register 0x4B
    
    int mask_3_lsb;
    //mask_3_lsb = 0b1111111111111000;
    mask_3_lsb = 0xFF - 7;
    int meas_Lsb;
    int meas_Msb;
    int total_meas;
    char out_msg [20];
    while (1) {
        CS3 = 0;
        spi_write(0x4B);
        spi_write (0x01);
        CS3 = 1;
        //transition to active mode
        //write 0x00 on register 0x4C
        CS3 = 0;
        spi_write (0x4C);
        spi_write (0x00);
        CS3 = 1;
        CS3 = 0;
        meas_Lsb = spi_write (0x42|0x80);
        meas_Lsb = spi_write (0x43|0x80);
        //spi_write (0x43|0x80);
        meas_Msb = spi_write (0x00);
        CS3 = 1;
        //meas_Lsb = meas_Lsb & mask_3_lsb;
        /*sprintf (out_msg, "MAGXLSB=%d", meas_Lsb);
        for (int i = 0; i < strlen (out_msg); i++){
            while (IFS0bits.U1TXIF == 0 && U1STAbits.TRMT == 0);
            IFS0bits.U1TXIF = 0;
            U1TXREG = out_msg [i];
        }*/
        //meas_Msb = meas_Msb << 8;
        //meas_Msb = meas_Msb * 256;
        /*sprintf (out_msg, "MAGXMSB=%d", meas_Msb);
        for (int i = 0; i < strlen (out_msg); i++){
            while (IFS0bits.U1TXIF == 0 && U1STAbits.TRMT == 0);
            IFS0bits.U1TXIF = 0;
            U1TXREG = out_msg [i];
        }*/
        //total_meas = (meas_Lsb | meas_Msb) / 8;
        total_meas = ((meas_Lsb & mask_3_lsb) | (meas_Msb << 8)) / 8;
        sprintf (out_msg, "MAGX=%d", total_meas);
        for (int i = 0; i < strlen (out_msg); i++){
            while (U1STAbits.UTXBF == 1);
            U1TXREG = out_msg [i];
        }
        LATAbits.LATA0 = SPI1STATbits.SPIROV;
        tmr_wait_period(TIMER1);
    }/**/
    
    
    
    return 0;
}
