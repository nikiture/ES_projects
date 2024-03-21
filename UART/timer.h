/* 
 * File:   timer.h
 * Author: nikit
 *
 * Created on 5 marzo 2024, 12.36
 */

#ifndef TIMER_H
#define	TIMER_H
#include "xc.h"

#ifdef	__cplusplus
extern "C" {
#endif
#define TIMER1 1
#define TIMER2 2 
    void tmr_setup_period (int timer, int ms);
    int tmr_wait_period (int timer);
    void tmr_wait_ms (int timer, int ms);



#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_H */

