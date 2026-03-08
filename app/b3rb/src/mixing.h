/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CEREBRI_B3RB_MIXING_H
#define CEREBRI_B3RB_MIXING_H

#include <synapse_topic_list.h>
uint32_t filter_refs(double speed_ref, double steer_ref,double *speed_ref_f, double *steer_ref_f, uint64_t *last_ticks);
void b3rb_set_actuators(synapse_msgs_Actuators* msg, double turn_angle, double omega_fwd);

#endif // CEREBRI_B3RB_MIXING_H
/* vi: ts=4 sw=4 et */
