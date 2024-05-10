/*
 * File:   main.c
 * Author: Torre Nicolò, Andrea Scorrano, Girum Desalegn (group 7)
 *
 * Created on 27 aprile 2024, 9.27
 */


#include "xc.h"
#include "timer.h"
#include"string.h"
#include "stdio.h"
#include "math.h"

#define CS1 LATBbits.LATB3
#define CS2 LATBbits.LATB4
#define CS3 LATDbits.LATD6

#define sample_counter 4
#define transmit_counter 20

#define tx_buf_size 60
#define msg_size 30

int16_t mask_3_lsb = 0xF8;
int16_t mask_1_lsb = 0xFE;

int16_t meas_buf_x [5];
int16_t meas_buf_y [5];
int16_t meas_buf_z [5];
int meas_idx = 0;

//circular buffer and indexes used to access it
char uart_out [tx_buf_size];
int write_idx = 0, read_idx = 0;


void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

int spi_write(unsigned int addr) {
    int value_read = 0;
    
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = addr;
    
    while (SPI1STATbits.SPIRBF == 0);
    value_read = SPI1BUF;
    
    return value_read;
}

void __attribute__ ((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    if (read_idx == write_idx) {
        IEC0bits.U1TXIE = 0;
    } else {
        U1TXREG = uart_out [read_idx % tx_buf_size];
        read_idx ++;
        read_idx =  read_idx % tx_buf_size;
        IFS0bits.U1TXIF = 0;
    }
    
}

void upload_to_uart_buf (char* out_msg, char* uart_buf) {
    for (int i = 0; i < strlen (out_msg); i++) {
        uart_buf [write_idx % tx_buf_size] = out_msg [i];
        write_idx ++;
        write_idx = write_idx % tx_buf_size;
    }
}

void register_xyz_axes () {
    //reading values from the xyz axis, including the masking of the lsb on the 3 registers not containing measurement values
    //3 bits for x and y axes, 1 bit for z axis
    CS3 = 0;
    spi_write (0x42|0x80);
    meas_buf_x [meas_idx] = spi_write (0x00) & mask_3_lsb;
    meas_buf_x [meas_idx] = (meas_buf_x [meas_idx] | (spi_write (0x00) << 8))/8;
    meas_buf_y [meas_idx] = spi_write (0x00) & mask_3_lsb;
    meas_buf_y [meas_idx] = (meas_buf_y [meas_idx] | (spi_write (0x00) << 8))/8;
    meas_buf_z [meas_idx] = spi_write (0x00) & mask_1_lsb;
    meas_buf_z [meas_idx] = (meas_buf_z [meas_idx] | (spi_write (0x00) << 8))/2;
    meas_idx ++;
    CS3 = 1;
}

void spi_setup () {
    TRISAbits.TRISA1 = 1;
    TRISFbits.TRISF12 = 0;
    TRISFbits.TRISF13 = 0;

    RPINR20bits.SDI1R = 0b0010001;
    RPOR12bits.RP109R = 0b000101;
    RPOR11bits.RP108R = 0b000110;

    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.MODE16 = 0;
    SPI1CON1bits.CKP = 1;
    SPI1CON1bits.PPRE = 0; //64:1 primary prescaler
    SPI1CON1bits.SPRE = 5; //1:1 secondary prescaler
}
int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISDbits.TRISD6 = 0;
    
    //setup spi
    spi_setup ();
    
    SPI1STATbits.SPIEN = 1;
    
    CS1 = CS2 = CS3 = 1;
    //turn on and setup spi
    CS3 = 0;
    spi_write(0x4B);
    spi_write(0x01);
    CS3 = 1;
    tmr_wait_ms (TIMER2, 2);
    //transition to active mode and setting 25 Hz sampling rate
    //write 0b110 on register 0x4C's data rate bits (bits 3, 4 and 5)
    //combined with the transtition to active mode, writing 0b110000 on register 0x4C
    CS3 = 0;
    spi_write (0x4C);
    spi_write (0b110000);
    CS3 = 1;
    tmr_wait_ms (TIMER2, 2);
    
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    const int baud_rate = 9600;
    U1BRG = ((72000000 / 16) / baud_rate) - 1;
    //set uart flag to trigger when all transmit operations are completed 
    U1STAbits.UTXISEL0 = 1;
    U1STAbits.UTXISEL1 = 0;
    //enable UART transmission on uart
    U1MODEbits.UARTEN = 1;

    U1STAbits.UTXEN = 1;
    
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1TXIE = 1;
    


    int16_t mag_average [3];
    char out_msg [msg_size];
    int loop_count = 0;
    float yaw = 0;
    
    tmr_setup_period (TIMER1, 10);

    while (1) {
        algorithm ();
        
        if (loop_count % sample_counter == 0) { //main loop is 100 Hz, one read from magnetometer every four loops for 25 Hz sampling
            register_xyz_axes();
        } if (loop_count > transmit_counter) { //transmitting values at 5 Hz is once every 20 main loops
            loop_count = 0;
            meas_idx = 0;
            
            mag_average [0] = mag_average [1] = mag_average [2] = 0;
            for (int i = 0; i < 5; i++) {
                mag_average [0] += meas_buf_x [i];
                mag_average [1] += meas_buf_y [i];
                mag_average [2] += meas_buf_z [i];
            }
            for (int i = i; i < 3; i++) {
                mag_average [i] /= 5;
            }
            sprintf (out_msg, "$MAG, %d, %d, %d*\n", mag_average [0], mag_average [1], mag_average [2]);

            IEC0bits.U1TXIE = 0;

            upload_to_uart_buf (out_msg, uart_out);

            IEC0bits.U1TXIE = 1;
            
            //yaw computation adjusted for the buggy's orientation
            yaw = atan2f (-(float) mag_average [0], (float) mag_average [1]);
            //conversion from radians to degree of yaw
            yaw = (yaw / 3.14) * 180;
            
            sprintf (out_msg, "$YAW, %.3f*\n", (double) yaw);
            
            IEC0bits.U1TXIE = 0;
            upload_to_uart_buf(out_msg, uart_out);
            IEC0bits.U1TXIE = 1;
        }
        loop_count ++;
        tmr_wait_period (TIMER1);
    }
    return 0;
}
