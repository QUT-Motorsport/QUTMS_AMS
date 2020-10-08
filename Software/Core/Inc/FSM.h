/**
 ******************************************************************************
 * @file FSM.h
 * @brief FSM
 ******************************************************************************
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include "main.h"
#include <stdlib.h>
#include <memory.h>
#include "cmsis_os.h"

/**
 * @brief Typedef state as state_t
 */
typedef struct state state_t;

/**
 * @brief Typedef fsm as fsm_t
 */
typedef struct fsm fsm_t;

/**
 * @brief fsm_function Typedef
 * @param A pointer to the FSM object
 */
typedef void (*fsm_function)(fsm_t*);

/**
 * @brief FSM state
 */
struct state {
	fsm_function enter; /**< State enter function */
	fsm_function iter; /**< State iterate function */
	fsm_function exit; /**< State exit function */
	char *stateName;
};

/**
 * @brief FSM
 */
struct fsm {
	state_t *currentState; /**< Current FSM State */
	osSemaphoreId_t sem; /**< FSM change state semaphore */
	osSemaphoreId_t updating; /**< FSM iterating semaphore */
};

/**
 * @brief Creates a new FSM object
 * @param beginState The inital state to be set for the FSM
 * @return A pointer to the FSM object
 */
fsm_t *fsm_new(state_t *beginState);

/**
 * @brief Iterates the FSM by calling fsm->currentState->iter(fsm)
 * @param fsm A pointer to the FSM object
 */
void fsm_iterate(fsm_t *fsm);

/**
 * @brief Changes the state of the FSM
 * @param fsm A pointer to the FSM object
 * @param newState A pointer to the new state to change to
 */
void fsm_changeState(fsm_t *fsm, state_t *newState);

/**
 * @brief Gets a pointer to the current state of the FSM
 * @param fsm A pointer to the FSM object
 * @return A pointer to the current state object
 */
state_t *fsm_getState_t(fsm_t *fsm);

/**
 * @brief Gets the name of the current state of the FSM
 * @param fsm A pointer to the FSM object
 * @return A string of the name of the current FSM state
 */
char* fsm_getState(fsm_t *fsm);

/**
 * @brief Resets the FSM to a state without calling exit and enter function
 * @param fsm A pointer to the FSM object
 * @param resetState A pointer to the state to reset to
 */
void fsm_reset(fsm_t *fsm, state_t *resetState);

/**
 * @brief Logs the FSM to UART or SD
 * @param fsm A pointer to the FSM object
 */
void fsm_log(fsm_t *fsm);

/**
 * @brief Delete by memory freeing the FSM object
 * @param fsm A pointer to the FSM object
 */
void fsm_delete(fsm_t *fsm);

#endif /* INC_FSM_H_ */
