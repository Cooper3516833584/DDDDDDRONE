/**
 * @file User_Com.c
 * @brief 用户下位机通信模块
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-08
 *
 * THINK DIFFERENTLY
 */

#include "User_Com.h"

#include <stdarg.h>
#include <stdio.h>

#include "ANO_DT_LX.h"
#include "ANO_LX.h"
#include "Drv_BSP.h"
#include "Drv_Led.h"
#include "Drv_Misc.h"
#include "Drv_Uart.h"
#include "Drv_WS2812.h"
#include "LX_FC_EXT_Sensor.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "Uart_Screen.h"
#include "Wireless_Com.h"
#include "string.h"
void UserCom_DataExchange(void);
void UserCom_SendData(u8* dataToSend, u8 Length);
void UserCom_CheckAck(void);
void UserCom_SendAck(uint8_t option, uint8_t* data_p, uint8_t data_len);

u8 user_connected = 0;                  // 用户下位机是否连接
static u16 user_heartbeat_cnt = 0;      // 用户下位机心跳计数
static u8 realtime_control_enable = 0;  // 实时控制是否开启
static u16 realtime_control_cnt = 0;    // 实时控制超时计数
static uint32_t pod_target_time = 0;    // 吊舱目标时间
static uint32_t pod_start_time = 0;     // 吊舱开始时间
static u8 pod_state = 0;                // 吊舱状态
s16 user_pwm[4] = {0};                  // 范围0-10000
_user_pos_st user_pos = {0};            // 用户下位机位置数据
_user_vel_st user_vel = {0};            // 用户下位机速度数据
_user_alt_st user_alt = {0};            // 用户下位机高度数据
_to_user_un to_user_data;               // 回传状态数据
static uint8_t user_ack_buf[32];        // ACK数据
static uint16_t user_ack_cnt = 0;       // ACK计数
uint8_t user_data_temp[280];            // 数据接受缓存

/***接收****/
static u8 _user_data_cnt = 0;  // 数据计数
static u8 _data_len = 0;       // 数据长度
static u8 state = 0;           // 状态机
/**
 * @brief 用户协议数据获取,在串口中断中调用,解析完成后调用UserCom_DataAnl
 * @param  data             数据
 */
void UserCom_GetOneByte(u8 data) {
  if (state == 0 && data == 0xAA) {
    state = 1;
    user_data_temp[0] = data;
  } else if (state == 1 && data == 0x22) {
    state = 2;
    user_data_temp[1] = data;
  } else if (state == 2)  // 功能字
  {
    state = 3;
    user_data_temp[2] = data;
  } else if (state == 3)  // 长度
  {
    state = 4;
    user_data_temp[3] = data;
    _data_len = data;  // 数据长度
    _user_data_cnt = 0;
  } else if (state == 4 && _data_len > 0) {
    _data_len--;
    user_data_temp[4 + _user_data_cnt++] = data;  // 数据
    if (_data_len == 0) state = 5;
  } else if (state == 5) {
    state = 0;
    user_data_temp[4 + _user_data_cnt] = data;  // check sum
    user_data_temp[5 + _user_data_cnt] = 0;
    UserCom_DataAnl(user_data_temp, 4 + _user_data_cnt);
  } else
    state = 0;
}

void UserCom_GetBuffer(u8* data_buf, u32 len) {
  for (u32 i = 0; i < len; i++) {
    if (state == 0 && data_buf[i] == 0xAA) {
      state = 1;
      user_data_temp[0] = data_buf[i];
    } else if (state == 1 && data_buf[i] == 0x22) {
      state = 2;
      user_data_temp[1] = data_buf[i];
    } else if (state == 2)  // 功能字
    {
      state = 3;
      user_data_temp[2] = data_buf[i];
    } else if (state == 3)  // 长度
    {
      state = 4;
      user_data_temp[3] = data_buf[i];
      _data_len = data_buf[i];  // 数据长度
      _user_data_cnt = 0;
    } else if (state == 4 && _data_len > 0) {
      for (; i < len; i++) {
        _data_len--;
        user_data_temp[4 + _user_data_cnt++] = data_buf[i];  // 数据
        if (_data_len == 0) {
          state = 5;
          break;
        }
      }
    } else if (state == 5) {
      state = 0;
      user_data_temp[4 + _user_data_cnt] = data_buf[i];  // check sum
      user_data_temp[5 + _user_data_cnt] = 0;
      UserCom_DataAnl(user_data_temp, 4 + _user_data_cnt);
    } else
      state = 0;
  }
}

/**
 * @brief 用户命令解析执行,数据接收完成后自动调用
 * @param  data_buf         数据缓存
 * @param  data_len         数据长度
 */
void UserCom_DataAnl(u8* data_buf, u16 data_len) {
  s8* p_s8;
  s16* p_s16;
  s32* p_s32;
  u32* p_u32;
  u8 u8_temp;
  u32 u32_temp;
  u8* p_data = (uint8_t*)(data_buf + 4);
  u8 option = (data_buf[2] & 127);  // 功能字(最高位为1表示需要ACK)
  u8 send_ack = (data_buf[2] & 128);
  u8 len = data_buf[3];
  u8 calc_check = 0;
  for (u8 i = 0; i < len + 4; i++) {
    calc_check += data_buf[i];
  }
  if (calc_check != data_buf[data_len]) {
    LxStringSend(LOG_COLOR_RED, "ERR: usercom checksum error");
    return;
  }
  switch (option) {
    case 0x00:  // 心跳包
      if (p_data[0] == 0x01) {
        if (!user_connected) {
          user_connected = 1;
          LxStringSend(LOG_COLOR_GREEN, "INFO: user connected");
        }
        user_heartbeat_cnt = 0;
        return;
      }
    case 0x01:  // 转发到IMU, 命令格式应遵循匿名通信协议, 此命令需要返回ACK
      if (dt.wait_ck == 0) {
        dt.cmd_send.CID = p_data[0];
        for (u8 i = 0; i < len - 1 && i < 10; i++) {
          dt.cmd_send.CMD[i] = p_data[i + 1];
        }
        CMD_Send(0xFF, &dt.cmd_send);
        // LxPrintf("DBG: to imu: 0x%02X 0x%02X 0x%02X", dt.cmd_send.CID,
        //          dt.cmd_send.CMD[0], dt.cmd_send.CMD[1]);
      } else {
        LxStringSend(LOG_COLOR_RED, "ERR: cmd to imu dropped for wait_ck");
        send_ack = 0;
      }
      break;
    case 0x02:  // WS2812控制
      u32_temp = 0xff000000;
      u8_temp = p_data[0];  // R
      u32_temp |= u8_temp << 16;
      u8_temp = p_data[1];  // G
      u32_temp |= u8_temp << 8;
      u8_temp = p_data[2];  // B
      u32_temp |= u8_temp;
      WS2812_SetAll(u32_temp);
      WS2812_SendBuf();
      break;
    case 0x03:  // 位置信息回传
      p_s32 = (s32*)(p_data);
      user_pos.pos_x = *p_s32;
      p_s32++;
      user_pos.pos_y = *p_s32;
      p_s32++;
      user_pos.pos_z = *p_s32;
      user_pos.update_cnt++;  // 触发发送
      break;
    case 0x04:  // 实时控制帧
      p_s16 = (s16*)(p_data);
      rt_tar.st_data.vel_x = *p_s16;  // 头向速度，厘米每秒
      p_s16++;
      rt_tar.st_data.vel_y = *p_s16;  // 左向速度，厘米每秒
      p_s16++;
      rt_tar.st_data.vel_z = *p_s16;  // 天向速度，厘米每秒
      p_s16++;
      rt_tar.st_data.yaw_dps = *p_s16;  // 航向角速度，度每秒，逆时针为正
      dt.fun[0x41].WTS = 1;             // 触发发送
      // 此处启用实时控制安全检查, 实时控制命令发送应持续发送且间隔小于1秒
      // 超时会自动停止运动
      realtime_control_enable = 1;
      realtime_control_cnt = 0;
      if (rt_tar.st_data.vel_x == 0 && rt_tar.st_data.vel_y == 0 &&
          rt_tar.st_data.vel_z == 0 && rt_tar.st_data.yaw_dps == 0) {
        realtime_control_enable = 0;
      }
      break;
    case 0x05:                     // 用户PWM控制
      u8_temp = p_data[0];         // 设置通道
      p_s16 = (s16*)(p_data + 1);  // 设置PWM值
      if (u8_temp <= 3) {          // 有效通道0-3
        user_pwm[u8_temp] = *p_s16;
      }
      break;
    case 0x06:  // IO控制
      DOut_Set(p_data[0], p_data[1]);
      break;
    case 0x07:  // 吊舱 控制
      pod_state = p_data[0];
      p_u32 = (u32*)(p_data + 1);
      pod_target_time = *p_u32;
      pod_start_time = GetSysRunTimeMs();
      break;
    case 0x08:  // 速度信息回传
      p_s16 = (s16*)(p_data);
      user_vel.vel_x = *p_s16;
      p_s16++;
      user_vel.vel_y = *p_s16;
      p_s16++;
      user_vel.vel_z = *p_s16;
      user_vel.update_cnt++;  // 触发发送
      break;
    case 0x09:  // GPS信息回传
      memcpy(ext_sens.fc_gps.byte, p_data, 23);
      dt.fun[0x30].WTS = 1;
      break;
    case 0x0A:  // 设置指示灯
      p_s8 = (s8*)(p_data);
      user_led.brightness[0] = *p_s8;
      p_s8++;
      user_led.brightness[1] = *p_s8;
      p_s8++;
      user_led.brightness[2] = *p_s8;
      break;
    case 0x0B:  // 高度信息回传
      p_s32 = (s32*)(p_data);
      user_alt.alt = *p_s32;
      user_alt.update_cnt++;  // 触发发送
      break;
    case 0x0C:  // 转发到串口屏
      Send_To_Screen(p_data, len);
      break;
    case 0x0D:  // 转发到无线串口
      Send_To_Wireless(p_data, len);
      break;
    default:
      break;
  }
  if (send_ack) {
    UserCom_SendAck(option, p_data, len);
  }
}

void UserCom_SendAck(uint8_t option, uint8_t* data_p, uint8_t data_len) {
  uint8_t ack_data;
  if (user_ack_cnt >= 32) return;
  ack_data = option;
  for (uint8_t i = 0; i < data_len; i++) {
    ack_data += data_p[i];
  }
  user_ack_buf[user_ack_cnt] = ack_data;
  user_ack_cnt++;
}

/**
 * @brief 用户通讯持续性任务，在调度器中调用
 * @param  dT_s
 */
void UserCom_Task(float dT_s) {
  static uint16_t data_exchange_cnt = 0;

  if (user_connected) {
    // 心跳超时检查
    user_heartbeat_cnt++;
    if (user_heartbeat_cnt * dT_s >= USER_HEARTBEAT_TIMEOUT_S) {
      user_connected = 0;
      LxStringSend(LOG_COLOR_RED, "WARN: user disconnected");
      user_led.brightness[0] = 0;
      user_led.brightness[1] = 0;
      user_led.brightness[2] = 0;
      if (fc_sta.unlock_sta == 1) {  // 如果是解锁状态，则采取安全措施
        // OneKey_Land(); //降落
        OneKey_Stable();  // 恢复悬停
        realtime_control_enable = 0;
      }
    }

    // ACK发送检查
    UserCom_CheckAck();

    // 数据交换
    data_exchange_cnt++;
    if (data_exchange_cnt * dT_s >= USER_DATA_EXCHANGE_TIMEOUT_S) {
      data_exchange_cnt = 0;
      UserCom_DataExchange();
    }
  }

  // 实时控制安全检查
  if (realtime_control_enable) {
    realtime_control_cnt++;
    if (realtime_control_cnt * dT_s >= REALTIME_CONTROL_TIMEOUT_S ||
        fc_sta.fc_mode_sta == 3) {
      // 超时, 停止运动
      realtime_control_cnt = 0;
      realtime_control_enable = 0;
      rt_tar.st_data.vel_x = 0;
      rt_tar.st_data.vel_y = 0;
      rt_tar.st_data.vel_z = 0;
      rt_tar.st_data.yaw_dps = 0;
      dt.fun[0x41].WTS = 1;  // 触发发送
      LxStringSend(LOG_COLOR_RED, "WARN: realtime control stop");
    }
  }

  // 吊舱状态检测
  if (pod_state == 0x01) {  // 放线开始
    user_pwm[1] = 7000;
    if (GetSysRunTimeMs() - pod_start_time > 1000) {  // 超时
      pod_state = 0x03;
    }
  } else if (pod_state == 0x02) {  // 收线
    if (Button_Get(0x02) == 0) {
      user_pwm[1] = 4800;
    }
    if (GetSysRunTimeMs() - pod_start_time > pod_target_time) {  // 超时
      pod_state = 0x00;
      user_pwm[1] = 6000;
    } else if (Button_Get(0x02) == 1) {  // 限位按钮按下
      pod_state = 0x00;
      user_pwm[1] = 6000;
    }
  } else if (pod_state == 0x03) {  // 放线等待
    user_pwm[1] = 7000;
    if (GetSysRunTimeMs() - pod_start_time > pod_target_time) {  // 超时
      pod_state = 0x00;
      user_pwm[1] = 6000;
    } else if (Button_Get(0x02) == 1) {  // 限位按钮按下
      pod_state = 0x00;
      user_pwm[1] = 6000;
    }
  }
}

/**
 * @brief 交换飞控数据
 */
void UserCom_DataExchange(void) {
  const u8 user_data_size = sizeof(to_user_data.byte_data);

  // 初始化数据
  to_user_data.st_data.head1 = 0xAA;
  to_user_data.st_data.head2 = 0x55;
  to_user_data.st_data.length = user_data_size - 4;
  to_user_data.st_data.cmd = 0x01;

  // 数据赋值
  to_user_data.st_data.rol_x100 = fc_att.st_data.rol_x100;
  to_user_data.st_data.pit_x100 = fc_att.st_data.pit_x100;
  to_user_data.st_data.yaw_x100 = fc_att.st_data.yaw_x100;
  to_user_data.st_data.alt_fused = fc_alt.st_data.alt_fused;
  to_user_data.st_data.alt_add = fc_alt.st_data.alt_add;
  to_user_data.st_data.vel_x = fc_vel.st_data.vel_x;
  to_user_data.st_data.vel_y = fc_vel.st_data.vel_y;
  to_user_data.st_data.vel_z = fc_vel.st_data.vel_z;
  to_user_data.st_data.pos_x = fc_pos.st_data.pos_x;
  to_user_data.st_data.pos_y = fc_pos.st_data.pos_y;
  to_user_data.st_data.voltage_100 = fc_bat.st_data.voltage_100;
  to_user_data.st_data.fc_mode_sta = fc_sta.fc_mode_sta;
  to_user_data.st_data.unlock_sta = fc_sta.unlock_sta;
  to_user_data.st_data.CID = fc_sta.cmd_fun.CID;
  to_user_data.st_data.CMD_0 = fc_sta.cmd_fun.CMD_0;
  to_user_data.st_data.CMD_1 = fc_sta.cmd_fun.CMD_1;

  // 校验和
  to_user_data.st_data.check_sum = 0;
  for (u8 i = 0; i < user_data_size - 1; i++) {
    to_user_data.st_data.check_sum += to_user_data.byte_data[i];
  }

  UserCom_SendData(to_user_data.byte_data, user_data_size);
}

static u8 data_to_send[12];

/**
 * @brief 检查ACK队列并发送
 */
void UserCom_CheckAck() {
  while (user_ack_cnt) {
    user_ack_cnt--;
    data_to_send[0] = 0xAA;                        // head1
    data_to_send[1] = 0x55;                        // head2
    data_to_send[2] = 0x02;                        // length
    data_to_send[3] = 0x02;                        // cmd
    data_to_send[4] = user_ack_buf[user_ack_cnt];  // data
    data_to_send[5] = 0;                           // check_sum
    for (uint8_t i = 0; i < 5; i++) {
      data_to_send[5] += data_to_send[i];
    }
    UserCom_SendData(data_to_send, 6);
  }
}

/**
 * @brief 发送事件
 * @param  event            事件代码
 * @param  op               操作代码
 */
void UserCom_SendEvent(u8 event, u8 op) {
  data_to_send[0] = 0xAA;   // head1
  data_to_send[1] = 0x55;   // head2
  data_to_send[2] = 0x03;   // length
  data_to_send[3] = 0x03;   // cmd
  data_to_send[4] = event;  // event code
  data_to_send[5] = op;     // op code
  data_to_send[6] = 0;      // check_sum
  for (u8 i = 0; i < 6; i++) {
    data_to_send[6] += data_to_send[i];
  }
  UserCom_SendData(data_to_send, 7);
}

extern void cdc_acm_data_send(uint8_t* buf, uint32_t len);
/**
 * @brief 用户通讯数据发送
 */
void UserCom_SendData(u8* dataToSend, u8 Length) {
  cdc_acm_data_send(dataToSend, Length);
  // DrvUart2SendBuf(dataToSend, Length);
}

static u8 strBuf[STRLENMAX + 5];
static u8 strBufPrintf[STRLENMAX];

/**
 * @brief 发送LOG到用户端
 * @param  str             字符串
 */
void UserStringSend(char* str) {
  u8 len = strlen(str);
  strBuf[0] = 0xAA;     // head1
  strBuf[1] = 0x55;     // head2
  strBuf[2] = len + 1;  // length
  strBuf[3] = 0x06;     // cmd
  if (len > STRLENMAX) len = STRLENMAX;
  memcpy(strBuf + 4, str, len);
  strBuf[4 + len] = 0;  // check_sum
  for (u8 j = 0; j < 4 + len; j++) {
    strBuf[4 + len] += strBuf[j];
  }
  UserCom_SendData(strBuf, 5 + len);
}

/**
 * @brief 发送LOG到用户端
 */
void UserPrintf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf((char*)strBufPrintf, STRLENMAX, fmt, ap);
  va_end(ap);
  UserStringSend((char*)strBufPrintf);
}
