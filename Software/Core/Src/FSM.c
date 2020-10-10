/**
 ******************************************************************************
 * @file FSM.c
 * @brief FSM
 ******************************************************************************
 */

#include <FSM.h>

fsm_t *fsm_new(state_t *beginState)
{
	// malloc, 0 memory then set state
	AMS_LogInfo("Creating New FSM\r\n", strlen("Creating New FSM\r\n"));
	fsm_t *fsm = malloc(sizeof(fsm_t));
	memset(fsm, 0, sizeof(fsm_t));
	fsm->currentState = beginState;

	// Set semaphores
	fsm->sem = osSemaphoreNew(3U, 3U, NULL);
	fsm->updating = osSemaphoreNew(3U, 3U, NULL);

	return fsm;
}

void fsm_iterate(fsm_t *fsm)
{
	if(osSemaphoreAcquire(fsm->updating, SEM_ACQUIRE_TIMEOUT) == osOK) {
		fsm->currentState->iter(fsm);
		osSemaphoreRelease(fsm->updating);
	} else
	{
		Error_Handler();
	}
}

void fsm_changeState(fsm_t *fsm, state_t *newState)
{
	if(fsm->currentState == newState)
	{
		return;
	}
	if(osSemaphoreAcquire(fsm->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		char x[80];
		int len = sprintf(x, "Changing FSM State: %s->%s\r\n", fsm->currentState->stateName, newState->stateName);
		AMS_LogInfo(x, len);
		fsm->currentState->exit(fsm);

		fsm->currentState = newState;
		fsm->currentState->enter(fsm);

		osSemaphoreRelease(fsm->sem);
	} else
	{
		Error_Handler();
	}
}

state_t *fsm_getState_t(fsm_t *fsm)
{
	if(osSemaphoreAcquire(fsm->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		state_t *s = fsm->currentState;
		osSemaphoreRelease(fsm->sem);
		return s;
	} else
	{
		Error_Handler();
	}
	return NULL;
}

char* fsm_getState(fsm_t *fsm)
{
	if(osSemaphoreAcquire(fsm->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		char *n = fsm->currentState->stateName;
		osSemaphoreRelease(fsm->sem);
		return n;
	} else
	{
		Error_Handler();
	}
	return NULL;
}

void fsm_reset(fsm_t *fsm, state_t *resetState)
{
	char x[80];
	int len = sprintf(x, "Resetting FSM to: %s\r\n", resetState->stateName);
	AMS_LogInfo(x, len);
	memset(fsm, 0, sizeof(fsm_t));
	fsm->currentState = resetState;

	// Set semaphores
	fsm->sem = osSemaphoreNew(3U, 3U, NULL);
	fsm->updating = osSemaphoreNew(3U, 3U, NULL);
}

void fsm_log(fsm_t *fsm)
{
	//TODO, how are we logging?
	return;
}

void fsm_delete(fsm_t *fsm)
{
	osSemaphoreDelete(fsm->sem);
	osSemaphoreDelete(fsm->updating);
	free(fsm);
	fsm = NULL;
}
