/**
 * @file Drv_UpFlow.c
 * @brief 优象光流模块驱动
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-07-20
 *
 * THINK DIFFERENTLY
 */
#include "Drv_UpFlow.h"
_up_flow_raw_t up_flow_raw;
_up_flow_t up_flow = {
    .x_inv = 0,
    .y_inv = 0,
    .xy_swap = 1,
};

void UpFlow_DataAnl(void) {
  if (up_flow_raw.flow_confidence < 0xF5 || up_flow_raw.tof_confidence < 0x64) {
    up_flow.available = 0;
    up_flow.x_speed = 0x8000;
    up_flow.y_speed = 0x8000;
    up_flow.height = 0xFFFFFFFF;
  } else {
    up_flow.available = 1;
    int x_speed = up_flow_raw.flow_x_integral * up_flow_raw.tof_distance /
                  10000 * 10 * 1000000 / up_flow_raw.integration_timespan;
    int y_speed = up_flow_raw.flow_y_integral * up_flow_raw.tof_distance /
                  10000 * 10 * 1000000 / up_flow_raw.integration_timespan;
    if (up_flow.x_inv) x_speed = -x_speed;
    if (up_flow.y_inv) y_speed = -y_speed;
    if (up_flow.xy_swap) {
      up_flow.x_speed = y_speed;
      up_flow.y_speed = x_speed;
    } else {
      up_flow.x_speed = x_speed;
      up_flow.y_speed = y_speed;
    }
    up_flow.height = up_flow_raw.tof_distance / 10;
    up_flow.update_cnt++;
  }
}

void UpFlow_GetOneByte(uint8_t data) {
  static uint8_t state = 0;
  static uint8_t len = 0;
  static uint8_t xnor = 0;
  if (state == 0 && data == 0xFE) {
    state = 1;
  } else if (state == 1 && data == 0x0A) {
    state = 2;
    len = 0;
    xnor = 0;
  } else if (state == 2) {
    ((uint8_t*)&up_flow_raw)[len++] = data;
    xnor ^= data;
    if (len == 10) {
      state = 3;
    }
  } else if (state == 3 && data == xnor) {
    state = 4;
  } else if (state == 4 && data == 0x55) {
    UpFlow_DataAnl();
    state = 0;
  } else {
    state = 0;
  }
}
