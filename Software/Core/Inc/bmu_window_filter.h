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

#define BMU_MAX_FILTER_SIZE 8

typedef struct bmu_window_filter {
	uint16_t current_filtered;
	uint16_t prev_values[BMU_MAX_FILTER_SIZE];
	uint32_t running_sum;
	uint16_t current_idx;
	bool initialized;
	uint16_t window_size;
} bmu_window_filter_t;

void bmu_window_filter_initialize(bmu_window_filter_t *filter, uint16_t initial_value, uint16_t window_size);

void bmu_window_filter_update(bmu_window_filter_t *filter, uint16_t new_value);

#endif /* INC_BMU_WINDOW_FILTER_H_ */