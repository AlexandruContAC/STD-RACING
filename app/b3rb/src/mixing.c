/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mixing.h"
#include <zephyr/kernel.h>

static uint32_t elapsed_ms(uint64_t *start_ticks) {
     uint64_t current_ticks = k_uptime_ticks();
     uint64_t ticks = k_uptime_ticks() - *start_ticks;
     
     if(ticks>0x7fffffffffffffff) ticks += 0xffffffffffffffff;
     uint32_t ms = k_ticks_to_ms_ceil32(ticks);
     if(ms>0) {
        *start_ticks = current_ticks;
     }
     return ms;
}
static void filter_ref(double ref, double *filt, uint32_t dt, double min, double max, double a_max, double b_max) {
    double f = *filt;
    double df = ref - f;
    if(df>0 && f >= 0) {
        double Df = a_max * dt * 0.001;
        if(df>Df) {
            f += Df;
        } else {
            f = ref;
        }
    } else if (df<0 && f >= 0) {
        double Df = -b_max * dt * 0.001;
        if(df<Df) {
            f += Df;
        } else {
            f = ref;
        }
    } else if (df>0 && f <= 0) {
        double Df = b_max * dt * 0.001;
        if(df>Df) {
            f += Df;
        } else {
            f = ref;
        }
    } else if (df<0 && f <= 0) {
        double Df = -a_max * dt * 0.001;
        if(df<Df) {
            f += Df;
        } else {
            f = ref;
        }
    } else {
        //WHAT???
    }
    if(f<min) *filt = min;
    else if(f>max) *filt = max;
    else *filt = f;
}

uint32_t filter_refs(double speed_ref, double steer_ref, double *speed_ref_f, double *steer_ref_f, uint64_t *last_ticks) {
    uint32_t dt_ms = elapsed_ms(last_ticks);

    if(dt_ms>0) {
        filter_ref(speed_ref, speed_ref_f, dt_ms, -1, 1, 2, 4);
        filter_ref(steer_ref, steer_ref_f, dt_ms, -1.15, 1.15, 5, 5);
    }
    return dt_ms;
}

void b3rb_set_actuators(synapse_msgs_Actuators* msg, double steer_ref, double speed_ref)
{
    msg->has_header = true;
    stamp_header(&msg->header, k_uptime_ticks());
    msg->header.seq++;
    strncpy(msg->header.frame_id, "odom", sizeof(msg->header.frame_id) - 1);

    msg->position_count = 0;//1; originally was 1... why? it is hard to calibrate servo center, min, max values in position mode...
    msg->velocity_count = 1;
    msg->normalized_count = 1;//2; originally was 2... why? expected 0...
    //msg->position[0] = turn_angle; //why did they user position mode when it is so difficult to calibrate?
    msg->normalized[0] = steer_ref; //better use normalized mode, hence the setting of 1 for normalized count
    msg->velocity[0] = speed_ref;
}

/* vi: ts=4 sw=4 et */
