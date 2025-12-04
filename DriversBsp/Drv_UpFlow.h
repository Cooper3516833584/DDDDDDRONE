/**
 * @file Drv_UpFlow.h
 * @brief 优象光流模块驱动
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-07-20
 *
 * THINK DIFFERENTLY
 */

#ifndef __DRV_UPFLOW_H
#define __DRV_UPFLOW_H
#include "SysConfig.h"

typedef struct {            // 光流原始数据
  int16_t flow_x_integral;  // 累计位移(/10000*height(mm)->actual move(mm))
  int16_t flow_y_integral;  // 累计位移(/10000*height(mm)->actual move(mm))
  uint16_t integration_timespan;  // 累计时间(us)
  uint16_t tof_distance;          // 激光高度(mm)
  uint8_t flow_confidence;        // 光流可信度(0xF5可信->0x00不可信)
  uint8_t tof_confidence;         // 激光可信度(0x64可信->0x00不可信)
} __attribute__((__packed__)) _up_flow_raw_t;

typedef struct {      // 光流数据
  uint8_t x_inv;      // x轴反向
  uint8_t y_inv;      // y轴反向
  uint8_t xy_swap;    // xy轴交换
  int16_t x_speed;    // x轴速度(cm/s)
  int16_t y_speed;    // y轴速度(cm/s)
  uint32_t height;    // 高度(cm)
  uint8_t available;  // 可用
  uint8_t update_cnt; // 更新计数
} _up_flow_t;

extern _up_flow_raw_t up_flow_raw;
extern _up_flow_t up_flow;

void UpFlow_GetOneByte(uint8_t data);

#endif
