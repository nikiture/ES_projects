/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 18 aprile 2024, 12.16
 */


#include "xc.h"
#define baud_rate 9600
#define ubrg_value ((72000000 / 16) / baud_rate) - 1
#include "../../Timers/Timers_first.X/timer.h"
#include <stdio.h>
#include <string.h>
#include <p33EP512MU810.h>


float compute_dist_from_volt (float v) {
    return 2.34 - 4.74 * v + 4.06 * v * v - 1.60 * v * v * v + 0.24 * v * v * v * v;
}
int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;// ANSELF = 0;
    ANSELBbits.ANSB5 = 1;
    ANSELBbits.ANSB11 = 1;
    float battery_value;
    char out_msg [30];
    tmr_setup_period (TIMER1, 200);
    TRISBbits.TRISB11 = 1;
    

    /*configure UART1 to mikrobus 2*/
    TRISDbits.TRISD11 = 1;
    TRISDbits.TRISD0 = 0;
    RPINR18bits.U1RXR = 75; //connect UART1 rx to pin RD11
    //RPINR18bits.U1RXR = 64
    RPOR0bits.RP64R = 1; //CONNECT UART 1 tx to pin RD0
    U1BRG = ubrg_value;
   /* IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;*/
    //enable UART
    U1MODEbits.UARTEN = 1;
    //enable tx on uart1
    U1STAbits.UTXEN = 1;
    /*part 1*/
    //configure adc
    AD1CON3bits.ADCS = 8;
    AD1CON1bits.ASAM = 0;
    AD1CON1bits.SSRC = 0;
    AD1CON2bits.CHPS = 0; //only channel 0 used
    AD1CHS0bits.CH0SA = 0b01011; //positive input is AN 11
    AD1CON2bits.CSCNA = 1;
    AD1CSSLbits.CSS11 = 1;
    AD1CON1bits.ADON = 1;
    
    while (1) {
        //read value
        //turn off DONE flag
        AD1CON1bits.DONE = 0;
        //activate sampling
        AD1CON1bits.SAMP = 1;
        tmr_wait_ms (TIMER2, 10);
        //turn off sampling
        AD1CON1bits.SAMP = 0;
        //wait for conversion completed
        tmr_wait_ms (TIMER2, 10);
        while (AD1CON1bits.DONE == 0);
        //read value from conversion buffer
        //battery_value = ADC1BUFA;
        battery_value = ADC1BUF0;
        //10 bits: 1024 levels
        //12 bits: 4096 levels
        //0-5v
        //1 : 1024 = x : 3.3
        battery_value =  3 * battery_value / 1024 * 3.3;
        
        //send value through the uart
        sprintf(out_msg, "BATT=%f", battery_value);
        for (int i = 0; i < strlen(out_msg); i++) {
            while (U1STAbits.UTXBF == 1);
            U1TXREG = out_msg [i];
        }
        tmr_wait_period (TIMER1);
        
        
    }
    /**/
    
    /*part 2
    float dist_value;
    //configure adc
    AD1CON3bits.ADCS = 8;
    AD1CON1bits.ASAM = 0;
    AD1CON1bits.SSRC = 7;
    AD1CON3bits.SAMC = 16;
    AD1CON2bits.CHPS = 0; //only channel 0 used
    AD1CHS0bits.CH0SA = 0b00101; //positive input is AN 5
    AD1CSSLbits.CSS5 = 1;
    AD1CON2bits.CSCNA = 1;
    AD1CON1bits.ADON = 1;
    //set en bit of sensore 
    LATBbits.LATB4 = 1;
    while (1) {
        AD1CON1bits.SAMP = 1;
        while (AD1CON1bits.DONE == 0);
        dist_value = ADC1BUF0;
        //n : 1024 = x : 2.1
        //dist_value = 0.4 + 1.7 * dist_value / 1024;
        //dist_value = 2.1 / 1024 * dist_value;
        //at 10 cm  about level 650
        //dist_value = 2.1 / 670 * dist_value;
        dist_value = 3.3 / 1024 * dist_value;
        //dist_value = 2 * dist_value;
        //dist_value = compute_dist_from_volt (dist_value);
        //send value through the uart
        sprintf(out_msg, "dist volt=%f", dist_value);
        for (int i = 0; i < strlen(out_msg); i++) {
            while (U1STAbits.UTXBF == 1);
            U1TXREG = out_msg [i];
        }
        tmr_wait_period (TIMER1);
        
    }
    /**/
    
    /*part 3
    //configure adc
    //ADCS has 8 bits,  maximum is 255 (+1)
    //1kHz : sampling rate + conversion rate = (Samc + 12) * Tad = 1ms-> Tad * (12 + SAMC) = 1ms
    //Tad = adcs * 13.9 ns
    //ADCS * 13.9 ns * (SAMC + 12) = 1ms 
    //ADCS * (SAMC + 12) = 1ms /13.9 ns = 72000 = 2^6 * 3^2 * 5^3
    //if ADCS = 2^6 * 3 = 192: SAMC + 12 = 375 , too big
    //reading once every 16 conversion 
    //SAMC + 12 = 23.43
    //SAMC = 11
    //or adcs = 128 and samc = 23
    
    
    AD1CON3bits.ADCS = 128; 
    AD1CON1bits.ASAM = 1;
    AD1CON1bits.SSRC = 7;
    AD1CON3bits.SAMC = 23; 
    //to finish
    AD1CON2bits.CHPS = 0; //only channel 0 used
    AD1CHS0bits.CH0SA = 0b00101; //positive input is AN 5
    AD1CSSLbits.CSS5 = 1;
    AD1CON2bits.CSCNA = 1;
    AD1CON1bits.ADON = 1;
     * */
    
    return 0;
}
