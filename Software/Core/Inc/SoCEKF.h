#ifndef INC_SoCEKF_H_
#define INC_SoCEKF_H_

/**
 * @brief EKF implementation for calculating and monitoring SoC of the Accumulator
 * @author Sam Haines & Tom Fraser
**/

#include "main.h"
#include <stdint.h>
#include <string.h>

#include "Non_Volatile.h"
#include "HandmadeMath.h"

void SoC_reset(); /**< Resets the SoC record. Used after the accumulator is recharged, activated by steering wheel */
float SoC_pull(); /**< Pulls the latest SoC record from flash */
void SoC_push(float SoC); /**< Pushed the latest SoC record to flash */

void SoCEKF_init(); /**< Initialises the SoC EKF */


#endif
