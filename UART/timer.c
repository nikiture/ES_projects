#include "xc.h"
#include "timer.h"

void tmr_setup_period (int timer, int ms) {
    //fosc = 144Mhz, Fcy = 72Mhz = 4000000
    //TMAX = 0.2 S 14400000 CLOCKS MAX
    //TO FIT 200 in one single 16-bit timer we need the prescaler set at more than 220, we set the prescaler to 256)
    if (timer == TIMER1) { 
        TMR1 = 0; //reset timer
        //set prescaler to 1:256
        T1CONbits.TCKPS = 3; //11
        //WITH PRESCALER AT 256 WE HAVE ONE UPDATE OF TMR EVERY 16 us 
        //Fcy = 4Mhz -> tclock = 1/72M = 0.0135 us -> tupdate = 256 * 0.013 us = 3.56 us
        //PR = Fcy * wait_period (S) / prescaler
        PR1 = (72000000 / 1000) / 256 * ms;
        IFS0bits.T1IF = 0; //set flag to zero
        T1CONbits.TON = 1; //start the timer
              
    } else if (timer == TIMER2) {
        //set prescaler to 1:256
        T2CONbits.TCKPS = 3; //11
        //WITH PRESCALER AT 256 WE HAVE ONE UPDATE OF TMR EVERY 16 us 
        //Fcy = 4Mhz -> tclock = 1/72M = 0.0135 us -> tupdate = 256 * 0.013 us = 3.56 us
        //PR = Fcy * wait_period (S) / prescaler
        PR2 = (72000000 / 1000) / 256 * ms;
        TMR2 = 0; //reset timer
        IFS0bits.T2IF = 0; //set flag to zero
        T2CONbits.TON = 1; //start the timer
        
    }
    else if (timer == TIMER3) {
        //set prescaler to 1:256
        T3CONbits.TCKPS = 3; //11
        //WITH PRESCALER AT 256 WE HAVE ONE UPDATE OF TMR EVERY 16 us 
        //Fcy = 4Mhz -> tclock = 1/72M = 0.0135 us -> tupdate = 256 * 0.013 us = 3.56 us
        //PR = Fcy * wait_period (S) / prescaler
        PR3 = (72000000 / 1000) / 256 * ms;
        TMR3 = 0; //reset timer
        IFS0bits.T3IF = 0; //set flag to zero
        T3CONbits.TON = 1; //start the timer
    }
    else if (timer == TIMER4) {
        //set prescaler to 1:256
        T4CONbits.TCKPS = 3; //11
        //WITH PRESCALER AT 256 WE HAVE ONE UPDATE OF TMR EVERY 16 us 
        //Fcy = 4Mhz -> tclock = 1/72M = 0.0135 us -> tupdate = 256 * 0.013 us = 3.56 us
        //PR = Fcy * wait_period (S) / prescaler
        PR4 = (72000000 / 1000) / 256 * ms;
        TMR4 = 0; //reset timer
        IFS1bits.T4IF = 0; //set flag to zero
        T4CONbits.TON = 1; //start the timer
    }
}


int tmr_wait_period (int timer) {
    if (timer == TIMER1) {
        
        //IFS0bits.T1IF = 0; //set flag to zero
        //TMR1 = 0; //reset timer
        if (IFS0bits.T1IF != 0) {
            IFS0bits.T1IF = 0;
            return 1;
        }
        while (IFS0bits.T1IF ==  0) {} //BUSY WAITING of the timer to finish 
        IFS0bits.T1IF = 0;
    } else if (timer == TIMER2) {
        //TMR2 = 0; //reset timer
        if (IFS0bits.T2IF != 0) {
            IFS0bits.T2IF = 0;
            return 1;
        }
        //IFS0bits.T2IF = 0; //set flag to zero
        //T2CONbits.TON = 1; //start the timer
        while (IFS0bits.T2IF ==  0) {} //BUSY WAITING of the timer to finish
        IFS0bits.T2IF = 0;
    }
    else if (timer == TIMER3) {
        if (IFS0bits.T3IF != 0) {
            IFS0bits.T3IF = 0;
            return 1;
        }
        //IFS0bits.T2IF = 0; //set flag to zero
        //T2CONbits.TON = 1; //start the timer
        while (IFS0bits.T3IF ==  0) {} //BUSY WAITING of the timer to finish
        IFS0bits.T3IF = 0;
    }
    else if (timer == TIMER4) {
        if (IFS1bits.T4IF != 0) {
            IFS1bits.T4IF = 0;
            return 1;
        }
        //IFS0bits.T2IF = 0; //set flag to zero
        //T2CONbits.TON = 1; //start the timer
        while (IFS1bits.T4IF ==  0) {} //BUSY WAITING of the timer to finish
        IFS1bits.T4IF = 0;
    }
    return 0;
}

void tmr_wait_ms (int timer, int ms) {
    int max_single_timer = (72000000 / 256) * 200 / 1000; //max value acceptable for PR with a single timer cycle
    if (timer == TIMER1) {
        T1CONbits.TON = 0; //TURN OFF TIMER IF ACTIVE
        T1CONbits.TCKPS = 3; //set prescaler to 256
        //long cycle_num = (72000000 / 1000) / 256 * ms;
        int repeat = ms / 200;
        int remainder = ms % 200;
        IFS0bits.T1IF = 0;
        PR1 = max_single_timer;
        T1CONbits.TON = 1;
        for (int i = 0; i < repeat; i++) {
            while (IFS0bits.T1IF == 0) {}
            IFS0bits.T1IF = 0;
        }
        T1CONbits.TON = 0;
        //if (remainder == 0) return; //if the remainder is 0 early return or else perma-block
        if(remainder != 0) {
            TMR1 = 0;
            PR1 = (72000000 / 256) * ((float) remainder / 1000);
            IFS0bits.T1IF = 0;
            T1CONbits.TON = 1;
            while (IFS0bits.T1IF == 0) {}
            IFS0bits.T1IF = 0;
            T1CONbits.TON = 0;
        }
    } else if (timer == TIMER2) {
        T2CONbits.TON = 0; //TURN OFF TIMER IF ACTIVE
        T2CONbits.TCKPS = 3; //set prescaler to 256
        //long cycle_num = (72000000 / 1000) / 256 * ms;
        int repeat = ms / 200;
        int remainder = ms % 200;
        IFS0bits.T2IF = 0;
        PR2 = max_single_timer;
        T2CONbits.TON = 1;
        for (int i = 0; i < repeat; i++) {
            while (IFS0bits.T2IF == 0) {}
            IFS0bits.T2IF = 0;
        }
        T2CONbits.TON = 0;
        //if (remainder == 0) return; //if the remainder is 0 early return or else perma-block
        if(remainder != 0) {
            TMR2 = 0;
            PR2 = (72000000 / 256) * ((float) remainder / 1000);
            IFS0bits.T2IF = 0;
            T2CONbits.TON = 1;
            while (IFS0bits.T2IF == 0) {}
            IFS0bits.T2IF = 0;
            T2CONbits.TON = 0;
        }
    } 
    else if (timer == TIMER3) {
        T3CONbits.TON = 0; //TURN OFF TIMER IF ACTIVE
        T3CONbits.TCKPS = 3; //set prescaler to 256
        //long cycle_num = (72000000 / 1000) / 256 * ms;
        int repeat = ms / 200;
        int remainder = ms % 200;
        IFS0bits.T3IF = 0;
        PR3 = max_single_timer;
        T3CONbits.TON = 1;
        for (int i = 0; i < repeat; i++) {
            while (IFS0bits.T3IF == 0) {}
            IFS0bits.T3IF = 0;
        }
        T3CONbits.TON = 0;
        //if (remainder == 0) return; //if the remainder is 0 early return or else perma-block
        if(remainder != 0) {
            TMR3 = 0;
            PR3 = (72000000 / 256) * ((float) remainder / 1000);
            IFS0bits.T3IF = 0;
            T3CONbits.TON = 1;
            while (IFS0bits.T3IF == 0) {}
            IFS0bits.T3IF = 0;
            T3CONbits.TON = 0;
        }
    }
    else if (timer == TIMER4) {
        T4CONbits.TON = 0; //TURN OFF TIMER IF ACTIVE
        T4CONbits.TCKPS = 3; //set prescaler to 256
        //long cycle_num = (72000000 / 1000) / 256 * ms;
        int repeat = ms / 200;
        int remainder = ms % 200;
        IFS1bits.T4IF = 0;
        PR4 = max_single_timer;
        T4CONbits.TON = 1;
        for (int i = 0; i < repeat; i++) {
            while (IFS1bits.T4IF == 0) {}
            IFS1bits.T4IF = 0;
        }
        T4CONbits.TON = 0;
        //if (remainder == 0) return; //if the remainder is 0 early return or else perma-block
        if(remainder != 0) {
            TMR4 = 0;
            PR4 = (72000000 / 256) * ((float) remainder / 1000);
            IFS1bits.T4IF = 0;
            T4CONbits.TON = 1;
            while (IFS1bits.T4IF == 0) {}
            IFS1bits.T4IF = 0;
            T4CONbits.TON = 0;
        }
    }
}
