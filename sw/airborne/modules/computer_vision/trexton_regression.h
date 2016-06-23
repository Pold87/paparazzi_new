/*
 * Copyright (C) Volker Strobel
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/trexton/trexton.regression.h"
 * @author Volker Strobel
 * treXton regression localization
 */

#ifndef TREXTON_REG_H
#define TREXTON_REG_H

#include <stdio.h>
#include "texton_settings.h"
#include "lib/vision/image.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/particle_filter/particle_filter.h"

/* static char training_data_path[] = "training_data/"; */

void init_positions(void);
void send_pos_to_ground_station(int x, int y);

void predict_position(struct measurement pos[], float hist[], int hist_size);
struct measurement predict_fann(double hist[], int size_hist);
struct measurement linear_regression_prediction(int texton_histogram[]);

static void send_trexton_position(struct transport_tx *trans, struct link_device *dev);
extern void trexton_init(void);
extern uint8_t isInTargetPos(uint8_t wp_id);
extern uint8_t isCertain(void);

#endif
