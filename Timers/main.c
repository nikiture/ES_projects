/* 
 * File:   main.c
 * Author: nikit
 *
 * Created on 5 marzo 2024, 12.36
 */
#include "xc.h"
#include "timer.h"



/*
 * 
 */

int main () {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0X0000;
    TRISAbits.TRISA0 = 0; //set led pin to output
    TRISGbits.TRISG9 = 0;
    //TRISEbits.TRISE8 = 1;
    int delay;
    //delay = 50;
    //delay = 200;
    delay = 2000;
    int res;
    
    tmr_setup_period (TIMER1, 200);
    
    while (1) {
        /*part 1*/
        /*
        LATAbits.LATA0 = !LATAbits.LATA0;
        tmr_wait_period (TIMER1);
        /**/
        /*part 2*/
        /*
        LATAbits.LATA0 = 1;
        tmr_wait_ms (TIMER2, 20);
        LATAbits.LATA0 = 0;
        tmr_wait_ms (TIMER2, 200);
        /**/
        /*part 3*/
        /**/
        
        tmr_wait_ms (TIMER2, delay);
        LATAbits.LATA0 = !LATAbits.LATA0;
        //tmr_wait_ms (TIMER2, delay);
        res = tmr_wait_period(TIMER1);
        if (res) {
            LATGbits.LATG9 = 1;
        } else {
            LATGbits.LATG9 = 0;
        }
        /**/
    }
}
