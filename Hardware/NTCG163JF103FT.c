/*
 * NTCG163JF103FT.c
 *
 *  Created on: 24 Jan 2025
 *      Author: Chee
 */

#include "Hardware/NTCG163JF103FT.h"


extern int heatSinkTempOffset50C(uint32_t R_value)
{
    if (R_value > UPPER_RESISTANCE) {
        R_value = UPPER_RESISTANCE;
    }
    if (R_value < LOWER_RESISTANCE) {
        R_value = LOWER_RESISTANCE;
    }

    double Temp = TEMP0;    // absolute temperature Kelvin
    double temp_offSet50C = 75;

    /*  NTCG163JF103FT Thermistor temperature as a function of resistance  */
    Temp = (BCONSTANT_AV * TEMP0) / (TEMP0 * log((double) R_value/RESISTANCE0) + BCONSTANT_AV);

    temp_offSet50C = Temp - ABSOLUTE_TEMP + 50; // degree Celcius + 50 degree Celcius

    return ((int) round(temp_offSet50C));

}


