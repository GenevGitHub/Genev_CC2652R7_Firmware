/*
 * CMFAX103_thermal_sensor.c
 *
 *  CMFAX103F3950FB - Thermal sensor
 *  Temperature – Resistance Interpolation Curves
 *
 *  Created on: 23 Jan 2025
 *      Author: Chee
 */

#include "Hardware/CMFAX103_thermal_sensor.h"


extern int motorTempOffset50C(uint32_t R_value) {
    float temp_value;
    uint8_t region;
    if (R_value <= 341){
        region = 1;
        temp_value = 126;
    }
    else if ((R_value <= 1249) && (R_value > 341)) {
        region = 2;
        temp_value = 925.83 * pow(R_value,(-0.342));
    }
    else if ((R_value <= 5313) && (R_value > 1249)) {
        region = 3;
        temp_value = -27.63 * log(R_value) + 276.32;
    }
    else if ((R_value <= 34548) && (R_value > 5313)) {
        region = 4;
        temp_value = -21.97 * log(R_value) + 227.71;
    }
    else if ((R_value <= 345275) && (R_value > 34548)) {
        region = 5;
        temp_value = -17.01 * log(R_value) + 175.95;
    }
    else { // R_value > 345275
        region = 6;
        temp_value = -41;
    }

    float temp_value_Offset50_Celcius = temp_value + 50;
    return ((int) round(temp_value_Offset50_Celcius));

}


