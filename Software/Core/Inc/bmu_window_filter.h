/*
 * bmu_window_filter.h
 *
 *  Created on: 9 Mar 2021
 *      Author: Calvin
 */

#ifndef INC_BMU_WINDOW_FILTER_H_
#define INC_BMU_WINDOW_FILTER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define MAX_FILTER_SIZE 8

typedef struct bmu_window_filter {
	uint32_t current_filtered;
	uint32_t prev_values[MAX_FILTER_SIZE];
	uint32_t running_sum;
	uint32_t current_idx;
	bool initialized;
	uint16_t window_size;
} bmu_window_filter_t;

void bmu_window_filter_initialize(bmu_window_filter_t *filter, uint32_t initial_value, uint16_t window_size);

void bmu_window_filter_update(bmu_window_filter_t *filter, uint32_t new_value);

#endif /* INC_BMU_WINDOW_FILTER_H_ */
