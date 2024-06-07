/*
 * UDHAL_PWM.c
 *
 *  Created on: 10 May 2024
 *      Author: Chee
 */


#include "UDHAL_PWM.h"

/* Period and duty in microseconds */
uint32_t pwmHLPeriod = LIGHT_PWM_PERIOD;
uint16_t duty      = 0;

/* Frequency in Hertz */
//uint32_t pwmBUZFrequency = BUZZER_PWM_FREQUENCY; // in Hertz

PWM_Handle pwm_headlight = NULL;
PWM_Handle pwm_buzzer = NULL;
PWM_Params params_headlight;
PWM_Params params_buzzer;

    /* Call driver init functions. */
extern void UDHAL_PWM_init() {
    PWM_init();
}

uint8_t pwm_headlight_openStatus = 0;
uint8_t pwm_buzzer_openStatus = 0;
uint8_t pwm_openStatus;

extern uint8_t UDHAL_PWM_paramInit(){
    PWM_Params_init(&params_headlight);
    params_headlight.dutyUnits   = PWM_DUTY_US;   // in microsecond
    params_headlight.dutyValue   = 0;             // initial value
    params_headlight.periodUnits = PWM_PERIOD_US; // Period in microseconds
    params_headlight.periodValue = pwmHLPeriod;     // Defines one Period
    pwm_headlight = PWM_open(CONFIG_PWM_0, &params_headlight);
    if (pwm_headlight == NULL){
        /* CONFIG_PWM_0 did not open */
        pwm_headlight_openStatus = 0;
        while (1) {}
    }
    else {
        pwm_headlight_openStatus = 1;
        PWM_start(pwm_headlight);
    }

    PWM_Params_init(&params_buzzer);
    params_buzzer.dutyUnits   = PWM_DUTY_FRACTION;   // in microsecond
    params_headlight.dutyValue   = 0;             // initial value
    params_headlight.periodUnits = PWM_PERIOD_HZ; // Period in microseconds
    params_headlight.periodValue = BUZZER_PWM_FREQUENCY; //pwmBUZFrequency;     // Defines frequency
    pwm_buzzer = PWM_open(CONFIG_PWM_1, &params_buzzer);
    if (pwm_buzzer == NULL){
        /* CONFIG_PWM_1 did not open */
        pwm_buzzer_openStatus = 0;
        while (1) {}
    }
    else {
        pwm_buzzer_openStatus = 1;
        PWM_start(pwm_buzzer);
    }

    if ((pwm_headlight_openStatus) && (pwm_buzzer_openStatus)) {
        pwm_openStatus = 1;
    }
    else {
        pwm_openStatus = 0;
    }

    return (pwm_openStatus);
}

/*********************************************************************
 * @fn      UDHAL_PWM_setBUZDutyAndPeriod
 *
 * @brief   set PWM duty and Period in Frequency mode
 *
 * @param   pwm_DutyPercent, pwm_Period
 *
 * @return  none
 */
uint32_t pwmBUZduty;
extern void UDHAL_PWM_setBUZDutyAndPeriod(uint8_t dutyPercent, uint32_t pwmBUZFrequency)
{
    /* Calculate pwmDuty % */
    if (!pwm_buzzer) {
        /* What kind of error handling should be activated if PWM open fails? */
//        ledControl_ErrorPriority(PWM1_OPEN_NULL);
    }
    else {
        pwmBUZduty = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * dutyPercent) / 100);
        PWM_setDutyAndPeriod(pwm_buzzer, pwmBUZduty, pwmBUZFrequency);        // set duty cycle to UDHAL_PWM1_pwmDuty % and set Period to pwmPeriod
    }
}

uint32_t pwmHLduty;
extern void UDHAL_PWM_setHLDutyAndPeriod(uint8_t dutyPercent){
    if (dutyPercent == 0){
        pwmHLduty = 0;
    }
    else {
        pwmHLduty = pwmHLPeriod * dutyPercent /100;
    }
    PWM_setDutyAndPeriod(pwm_headlight, pwmHLduty, pwmHLPeriod);
}



