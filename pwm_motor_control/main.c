/*
 * File:   main.c
 * Author: nikit
 *
 * Created on 2 maggio 2024, 14.40
 */


#include "xc.h"
#include "timer.h"

#define OC_1 1
#define OC_2 2
#define OC_3 3
#define OC_4 4

#define pwm_period 10000

int toggle_motion = 0;

void set_pwm (int oc, int duty_cycle) {
    switch (oc) {
        case OC_1:
            OC1R = duty_cycle * (pwm_period / 100);
            break;
        case OC_2:
            OC2R = duty_cycle * (pwm_period / 100);
            break;
        case OC_3:
            OC3R = duty_cycle * (pwm_period / 100);
            break;
        case OC_4:
            OC4R = duty_cycle * (pwm_period / 100);
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
    set_pwm (OC_2, 100);
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

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt () {
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 0;
    if (PORTEbits.RE8 == 1) {   
        toggle_motion = !toggle_motion;
    }
}
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt () {
    IFS1bits.INT1IF = 0;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    //10 ms "wait" for debouncing
    tmr_setup_period(TIMER2, 10);    
} 

int main(void) {
    int motion;
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0;
    TRISDbits.TRISD1 = TRISDbits.TRISD2 = TRISDbits.TRISD3 = TRISDbits.TRISD4 = 0;
    //timer3 (set at 10 kHz)  with sync with timer 3 
    //OC1CON1bits.OCTSEL = OC2CON1bits.OCTSEL = OC3CON1bits.OCTSEL = OC4CON1bits.OCTSEL = 7;
    //OC1CON2bits.SYNCSEL = OC2CON2bits.SYNCSEL = OC3CON2bits.SYNCSEL = OC4CON2bits.SYNCSEL = 0x1F;
    OC1CON1bits.OCTSEL = OC2CON1bits.OCTSEL = OC3CON1bits.OCTSEL = OC4CON1bits.OCTSEL = 1;
    OC1CON2bits.SYNCSEL = OC2CON2bits.SYNCSEL = OC3CON2bits.SYNCSEL = OC4CON2bits.SYNCSEL = 13;
    
    /*motor control:
     * oc3: right wheels backward
     * oc1: left wheels backward
     * oc2: left wheels forward
     * oc4: right wheels forward
    */
    //pin remap 
    RPOR0bits.RP65R = 0b010000;
    RPOR1bits.RP66R = 0b010001;
    RPOR1bits.RP67R = 0b010010;
    RPOR2bits.RP68R = 0b010011;
    
    OC1CON1bits.OCM = OC2CON1bits.OCM = OC3CON1bits.OCM = OC4CON1bits.OCM = 6;
    
    //map INT1 to pin RE8
    RPINR0bits.INT1R = 0x58; //map interrupt INT1 to RPI88 (pin of button T2)
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;
    
    //setup timer 3 to 10kHz
    //Fcy = 72000000
    //0.0001 s: 7200 clocks, acceptable value for PR
    TMR3 = 0;
    PR3 = 7200;
    T3CONbits.TCKPS = 0;
    T3CONbits.TON = 1;
    tmr_setup_period (TIMER1, 200);
    
    while (1) {
        IEC1bits.INT1IE = 0;
        motion = toggle_motion;
        IEC1bits.INT1IE = 1;
        if (motion == 1) {
            move_forward (90);
            //steer_right ();
            //steer_left ();
        } else {
            stop ();
        }
        
        
        tmr_wait_period(TIMER1);
    }
    
    
    
    
    
    
    return 0;
}
