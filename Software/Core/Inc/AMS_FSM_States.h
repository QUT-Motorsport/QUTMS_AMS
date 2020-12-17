/**
 ******************************************************************************
 * @file AMS_FSM_States.h
 * @brief AMS FSM States
 ******************************************************************************
 */

#ifndef INC_AMS_FSM_STATES_H_
#define INC_AMS_FSM_STATES_H_

#include "FSM.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <memory.h>
#include <stdbool.h>
#include <stdbool.h>
#include "AMS_CAN_Messages.h"
#include "BMS_CAN_Messages.h"
#include "can.h"
#include <math.h>

/**
 * @brief AMS Global State
 * @note AMS Global State is shared across threads, so use the semaphore to gain control
 */
typedef struct
{
	//CAN
	uint32_t CAN2_TxMailbox;
	uint32_t CAN2_RxMailbox;
	uint32_t CAN4_TxMailbox;
	uint32_t CAN4_RxMailbox;

	uint32_t startupTicks; /**< The Tick count at the initial startup time */

	bool BMSStartupSoc[BMS_COUNT];
	uint16_t BMSVoltages[BMS_COUNT][BMS_VOLTAGE_COUNT]; /**< Globally stores the 10 Voltages for each BMS*/
	uint8_t BMSTemperatures[BMS_COUNT][BMS_TEMPERATURE_COUNT]; /**< Globally stores the 12 Temperatures for each BMS*/

	int64_t CoulombCountuA;
	float CoulombCount;

	int32_t VoltageuV;
	float Voltage;

	int32_t HVACurrentuA;
	float HVACurrent;

	int32_t HVBCurrentuA;
	float HVBCurrent;

	osMessageQueueId_t CANQueue;
	osMessageQueueId_t CANForwardQueue;

	osTimerId_t heartbeatTimer;
	osTimerId_t IDC_AlarmTimer;
	osTimerId_t heartbeatTimerAMS;
	osTimerId_t ccTimer;
	osTimerId_t cTimer;
	osTimerId_t debugTimer;
	osSemaphoreId_t sem;
	int readyCount;
} AMS_GlobalState_t;

AMS_GlobalState_t *AMS_GlobalState;

/**
 * @brief Dead state enter function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_enter(fsm_t *fsm);

/**
 * @brief Dead state iterate function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_iterate(fsm_t *fsm);

/**
 * @brief Dead state exit function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_exit(fsm_t *fsm);

/**
 * @brief deadState ie. startup state for the fsm
 * @note Initial FSM state, has no functionality
 * @details Next: idleState (Instantly)
 */
state_t deadState;

/**
 * @brief Idle state enter function. Initialises the AMS_GlobalState, starts sending hearbeats
 * @param fsm A pointer to the FSM object
 */
void state_idle_enter(fsm_t *fsm);

/**
 * @brief Idle state iterate function. Waits for RTD, if RTD is pressed, change the FSM state to precharge
 * @param fsm A pointer to the FSM object
 */
void state_idle_iterate(fsm_t *fsm);

/**
 * @brief Idle state exit function. Cleans up any timers or semaphores not being used
 * @param fsm A pointer to the FSM object
 */
void state_idle_exit(fsm_t *fsm);

/**
 * @brief idleState ie. idle state before RTD request is recieved
 * @note Idle FSM state, waiting for RTD or charging CAN messages. Publishing heartbeats. Waking up, Monitoring BMSs
 * @details Next: prechargeState (CAN RTD Recieved), chargingState (CAN Charge Start Recieved), errorState (Error)
 */
state_t idleState;

/**
 * Precharge state enter function. Creates a new precharge timer so we wait long enough for precharge to occur
 * @param fsm A pointer to the FSM object
 */
void state_precharge_enter(fsm_t *fsm);

/**
 * Precharge state iterate function. Waits for the precharge timer callback to occur
 * @param fsm A pointer to the FSM object
 */
void state_precharge_iterate(fsm_t *fsm);

/**
 * Precharge state exit function. Clean up the timer
 * @param fsm A pointer to the FSM object
 */
void state_precharge_exit(fsm_t *fsm);

/**
 * Precharge timer callback. calls a state change after precharge has occured
 * @param fsm A pointer to the FSM object
 */
void prechargeTimer_cb(void *fsm);

/**
 * @brief prechargeState ie. idle state after RTD is recieved
 * @note Precharge FSM state, performing precharge by connecting precharge resistor. Publishing heartbeats / BMSs
 * @details Next: drivingState (precharge timer done (>300ms), errorState(Error)
 */
state_t prechargeState;
/**
 * @brief prechargeTimer object
 */
osTimerId_t prechargeTimer;

/**
 * Driving state enter function. Open the precharge contactor, close the HV contactors.
 * @param fsm A pointer to the FSM object
 */
void state_driving_enter(fsm_t *fsm);

/**
 * Driving state iterate function. Sends heartbeats, queries BMSs.
 * @param fsm A pointer to the FSM object
 */
void state_driving_iterate(fsm_t *fsm);

/**
 * Driving state exit function. Close HV contactors, broadcast state
 * @param fsm A pointer to the FSM object
 */
void state_driving_exit(fsm_t *fsm);

/**
 * @brief drivingState ie. idle state after precharge has occured. Moving state.
 * @note Driving FSM state, Publishing heartbeats / BMSs
 * @details Next: errorState (Error), resetState (CAN reset)
 */
state_t drivingState;

/**
 * Error state enter function. Broadcast error over CAN, break the alarm line
 * @param fsm A pointer to the FSM object
 */
void state_error_enter(fsm_t *fsm);

/**
 * Error state iterate function. Recursively broadcast error over CAN.
 * @param fsm A pointer to the FSM object
 */
void state_error_iterate(fsm_t *fsm);

/**
 * Error state exit function. We should never get here
 * @param fsm A pointer to the FSM object
 */
void state_error_exit(fsm_t *fsm);

/**
 * @brief errorState ie. an inescapable state without power cycle
 * @note Error state, Publishing CAN Error, driving Alarm Line LOW
 * @details Next: None (Power Cycle)
 */
state_t errorState;

void state_reset_enter(fsm_t *fsm);
void state_reset_iterate(fsm_t *fsm);
void state_reset_exit(fsm_t *fsm);

/**
 * @brief resetState ie. Return to idle state without power cycle
 * @note Reset state, Open contactors, publish heartbeats
 * @details Next: idleState (instantly), errorState (On error)
 */
state_t resetState;

void state_charging_enter(fsm_t *fsm);
void state_charging_iterate(fsm_t *fsm);
void state_charging_exit(fsm_t *fsm);

/**
 * @brief chargingState ie. Gets charge request from CAN
 * @note Charging state, close contactors, monitor BMSs
 * @details Next: errorState (On error), resetState (CAN), idleState (Stop charging)
 */
state_t chargingState;

void state_SoC_enter(fsm_t *fsm);
void state_SoC_iterate(fsm_t *fsm);
void state_SoC_exit(fsm_t *fsm);

/**
 * @brief SoCState ie. sets up the initial SoC
 * @note SoC State, get initial average voltage from BMSs, map to SoC, monitor SoC using Current Sensor
 * @details Next: errorState (On error), resetState (CAN), idleState (done SoC Setup)
 */
state_t SoCState;


void state_init_enter(fsm_t *fsm);

void state_init_iterate(fsm_t *fsm);

void state_init_exit(fsm_t *fsm);
/**
 * @brief initState ie. sets up intial globalState
 * @notes Init State, create and init initial globalState
 * @details Next: errorState (On error), resetState (CAN), SoCState (CAN RTD)
 */
state_t initState;

void state_charging_enter(fsm_t *fsm);

void state_charging_iterate(fsm_t *fsm);

void state_charging_exit(fsm_t *fsm);
/**
 * @brief chargingState ie. charges the bats
 * @notes Charging State, make sure the BMSs don't go bang
 * @details Next: errorState (on error)
 */
state_t chargingState;

/** Handlers for generic CAN messages that are repeated in multiple states */

/** BMS Handlers */
void BMS_handleVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg);
void BMS_handleTemperature(fsm_t *fsm, AMS_CAN_Generic_t msg);
void BMS_handleBadCellVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg);
void BMS_handleBadCellTemperature(fsm_t *fsm, AMS_CAN_Generic_t msg);

/** Sendyne Sensor Handlers */
void Sendyne_handleVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg);
void Sendyne_handleColoumbCount(fsm_t *fsm, AMS_CAN_Generic_t msg);
void Sendyne_handleCurrent(fsm_t *fsm, AMS_CAN_Generic_t msg);

#endif /* INC_AMS_FSM_STATES_H_ */
