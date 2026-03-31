/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "casadi/gen/b3rb.h"

/* Local math helpers to avoid linking libm */
static inline double my_fabs(double x) { return x < 0.0 ? -x : x; }
static double my_sqrt(double x) {
  if (x <= 0.0) return 0.0;
  double guess = x;
  for (int i = 0; i < 20; i++) {
    guess = 0.5 * (guess + x / guess);
  }
  return guess;
}

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>
#include <cerebri/core/casadi.h>

#include "mixing.h"

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(b3rb_velocity, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

/* ── Lap detection state machine ──────────────────────────────────────── */
typedef enum {
    STATE_NORMAL,
    STATE_LAP_STARTED,
    STATE_LAP_DONE
} challenge_state_t;

static double g_start_x = 0.0;
static double g_start_y = 0.0;
static bool g_initial_position_set = false;

typedef struct _context {
  struct zros_node node;
  synapse_msgs_Twist cmd_vel;
  synapse_msgs_Status status;
  synapse_msgs_Actuators actuators;
  synapse_msgs_Actuators actuators_manual;
  synapse_msgs_PixyVector pixy_vector;
  synapse_msgs_Odometry odom;
  struct zros_sub sub_status, sub_cmd_vel, sub_actuators_manual,
      sub_pixy_vector, sub_odom;
  struct zros_pub pub_actuators;
  const double wheel_radius;
  const double wheel_base;
  challenge_state_t state;
} context;

static context g_ctx = {
    .node = {},
    .cmd_vel = synapse_msgs_Twist_init_default,
    .status = synapse_msgs_Status_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .actuators_manual = synapse_msgs_Actuators_init_default,
    .pixy_vector = synapse_msgs_PixyVector_init_default,
    .odom = synapse_msgs_Odometry_init_default,
    .sub_status = {},
    .sub_cmd_vel = {},
    .sub_actuators_manual = {},
    .sub_odom = {},
    .pub_actuators = {},
    .wheel_radius = CONFIG_CEREBRI_B3RB_WHEEL_RADIUS_MM / 1000.0,
    .wheel_base = CONFIG_CEREBRI_B3RB_WHEEL_BASE_MM / 1000.0,
    .state = STATE_NORMAL,
};

static void update_state(context *ctx) {
  double x = ctx->odom.pose.pose.position.x;
  double y = ctx->odom.pose.pose.position.y;

  if (!g_initial_position_set) {
    /* Wait until the car is actually driving (main.py sending speed) */
    if (my_fabs(ctx->cmd_vel.linear.x) < 0.1) {
      return;
    }
    g_start_x = x;
    g_start_y = y;
    g_initial_position_set = true;
    //LOG_INF("Start position captured: x=%.2f y=%.2f", g_start_x, g_start_y);
  }

  double dx = x - g_start_x;
  double dy = y - g_start_y;
  double dist = my_sqrt(dx * dx + dy * dy);


  switch (ctx->state) {
  case STATE_NORMAL:
    if (dist > 0.4) {
      ctx->state = STATE_LAP_STARTED;
      //LOG_INF("Lap started (dist=%.2f)", dist);
    }
    break;
  case STATE_LAP_STARTED:
    if (dist < 0.25) {
      ctx->state = STATE_LAP_DONE;
      //LOG_INF("Lap done (dist=%.2f) — slowing to 0.2", dist);
    }
    break;
  case STATE_LAP_DONE:
    break;
  }
}

static void init_b3rb_vel(context *ctx) {
  LOG_DBG("init vel");
  zros_node_init(&ctx->node, "b3rb_velocity");
  zros_sub_init(&ctx->sub_cmd_vel, &ctx->node, &topic_cmd_vel, &ctx->cmd_vel,
                10);
  zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
  zros_sub_init(&ctx->sub_actuators_manual, &ctx->node, &topic_actuators_manual,
                &ctx->actuators_manual, 10);
  zros_sub_init(&ctx->sub_pixy_vector, &ctx->node, &topic_pixy_vector,
                &ctx->pixy_vector, 10);
  zros_sub_init(&ctx->sub_odom, &ctx->node, &topic_estimator_odometry,
                &ctx->odom, 10);
  zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators,
                &ctx->actuators);
}

// computes rc_input from V, omega
static void update_cmd_vel(context *ctx) {
  double V = ctx->cmd_vel.linear.x;
  double omega = ctx->cmd_vel.angular.z;

  // b3rb_set_actuators(&ctx->actuators, omega, V);
  static double steer_ref_f = 0, speed_ref_f = 0;
  static uint64_t last_ticks;
  static bool do_init_static = true;
  if (do_init_static) {
    do_init_static = false;
    last_ticks = k_uptime_ticks();
  }
  // LOG_ERR("auto_dt_ms %d",
  filter_refs(V, omega, &speed_ref_f, &steer_ref_f, &last_ticks);
  b3rb_set_actuators(&ctx->actuators, steer_ref_f, speed_ref_f);
}
static void follow_line(context *ctx) {
  // copiere valori steer si speed din cmd_vel, primite de la nav q plus
  double steer = ctx->cmd_vel.angular.z;
  double speed = ctx->cmd_vel.linear.x;

  update_state(ctx);

  if (ctx->state == STATE_LAP_DONE) {
    if (my_fabs(speed) < 0.01) {
      // LIDAR IS ACTIVE! Leave speed exactly at 0.0
      speed = 0.0; 
    } else {
      // Normal driving. Cap speed to 0.2 because lap is done
      speed = 0.2;
    }
  }

  printf("steer %f speed %f state %d\n", steer, speed, ctx->state);

  static double steer_ref_f = 0, speed_ref_f = 0;
  static uint64_t last_ticks;
  static bool do_init_static = true;
  if (do_init_static) {
    do_init_static = false;
    last_ticks = k_uptime_ticks();
  }
  LOG_INF("steer %f speed %f state %d\n", steer, speed, ctx->state);

  filter_refs(speed, steer, &speed_ref_f, &steer_ref_f, &last_ticks);
  b3rb_set_actuators(&ctx->actuators, steer_ref_f, speed_ref_f);
}

static void stop(context *ctx) { b3rb_set_actuators(&ctx->actuators, 0, 0); }

static void b3rb_velocity_entry_point(void *p0, void *p1, void *p2) {
  LOG_INF("init");
  context *ctx = p0;
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);

  init_b3rb_vel(ctx);

  synapse_msgs_Status_Mode prev_mode = synapse_msgs_Status_Mode_MODE_UNKNOWN;

  static int64_t cmd_vel_last_ticks = 0;
  static bool cmd_vel_ever_received = false;

  while (true) {
    synapse_msgs_Status_Mode mode = ctx->status.mode;

    int rc = 0;
    if (mode == synapse_msgs_Status_Mode_MODE_MANUAL) {
      struct k_poll_event events[] = {
          *zros_sub_get_event(&ctx->sub_actuators_manual),
      };
      rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
      if (rc != 0) {
        LOG_DBG("not receiving manual actuators");
      }
    } else if (mode == synapse_msgs_Status_Mode_MODE_CMD_VEL) {
      struct k_poll_event events[] = {
          *zros_sub_get_event(&ctx->sub_cmd_vel),
      };
      rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
      if (rc != 0) {
        LOG_DBG("not receiving cmd_vel");
      }
    } else { /// Cazul auto NE INTERESEAZA
      struct k_poll_event events[] = {
          *zros_sub_get_event(&ctx->sub_cmd_vel), // Modificat: ascultam cmd_vel
                                                  // in loc de pixy_vector
      };
      rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
      if (rc != 0) {
        LOG_DBG("not receiving cmd_vel in auto mode");
      }
    }

    if (zros_sub_update_available(&ctx->sub_status)) {
      zros_sub_update(&ctx->sub_status);
    }

    if (zros_sub_update_available(&ctx->sub_cmd_vel)) {
      zros_sub_update(&ctx->sub_cmd_vel);
      cmd_vel_last_ticks = k_uptime_ticks();
      cmd_vel_ever_received = true;
    }

    /* Reset lap state when cmd_vel goes stale (main.py was closed) */
    if (cmd_vel_ever_received) {
      int64_t now = k_uptime_ticks();
      if ((now - cmd_vel_last_ticks) > 2 * CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
        g_initial_position_set = false;
        ctx->state = STATE_NORMAL;
        cmd_vel_ever_received = false;
        LOG_INF("cmd_vel lost - lap state reset");
      }
    }

    if (zros_sub_update_available(&ctx->sub_actuators_manual)) {
      zros_sub_update(&ctx->sub_actuators_manual);
    }

    if (zros_sub_update_available(&ctx->sub_odom)) {
      zros_sub_update(&ctx->sub_odom);
    }
    // if (zros_sub_update_available(&ctx->sub_pixy_vector)) {
    //   zros_sub_update(&ctx->sub_pixy_vector);
    // }

    // handle modes
    if (ctx->status.arming != synapse_msgs_Status_Arming_ARMING_ARMED) {
      stop(ctx);
      LOG_DBG("not armed, stopped");
    } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_MANUAL) {
      LOG_DBG("manual mode");
      ctx->actuators = ctx->actuators_manual;
    } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_AUTO) {
      /* Reset lap detection when re-entering AUTO from another mode */
      if (prev_mode != synapse_msgs_Status_Mode_MODE_AUTO) {
        g_initial_position_set = false;
        ctx->state = STATE_NORMAL;
        LOG_INF("AUTO re-entered, lap state reset");
      }
      LOG_DBG("auto mode");
      follow_line(ctx);
    } else {
      LOG_DBG("cmd_vel mode");
      update_cmd_vel(ctx);
    }

    // publish
    zros_pub_update(
        &ctx->pub_actuators); // aici se copiaza valorile de steer si speed in
                              // actuators,in memoria globala zephyr,
    // lucru care trezeste toate nodurile abonate la acest topic

    prev_mode = mode;
  }
}

K_THREAD_DEFINE(b3rb_velocity, MY_STACK_SIZE, b3rb_velocity_entry_point, &g_ctx,
                NULL, NULL, MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */