/*==========================================================================
 * 描述    ：凌霄飞控外置传感器处理
 * 更新时间：2020-02-06
 * 作者		 ：匿名科创-Jyoun
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 项目合作：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群互相交流、讨论、学习。
 * 若您觉得匿名有不好的地方，欢迎您拍砖提意见。
 * 若您觉得匿名好，请多多帮我们推荐，支持我们。
 * 匿名开源程序代码欢迎您的引用、延伸和拓展，不过在希望您在使用时能注明出处。
 * 君子坦荡荡，小人常戚戚，匿名坚决不会请水军、请喷子，也从未有过抹黑同行的行为。
 * 开源不易，生活更不容易，希望大家互相尊重、互帮互助，共同进步。
 * 只有您的支持，匿名才能做得更好。
===========================================================================*/
#include "LX_FC_EXT_Sensor.h"

#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_UpFlow.h"
#include "User_Com.h"


_fc_ext_sensor_st ext_sens;
u8 using_of = 1;      // 使用光流数据而非用户速度数据
u8 using_of_alt = 1;  // 使用光流高度而非用户高度

// 这里把光流数据打包成通用速度传感器数据
static inline void General_Velocity_Data_Handle() {
  static u8 of_update_cnt = 0, user_vel_update_cnt = 0;
  static u16 dT_ms = 0;
  if (using_of && user_vel_update_cnt != user_vel.update_cnt) {
    LxStringSend(LOG_COLOR_GREEN, "INFO: switch to velocity data");
    using_of = 0;
  }
  if (using_of) {
#if !OF_USE_UP_FLOW
    // 检查OF数据是否更新
    if (of_update_cnt != ano_of.of_update_cnt) {
      of_update_cnt = ano_of.of_update_cnt;
      // XY_VEL
      if (ano_of.of1_sta && ano_of.work_sta)  // 光流有效
      {
        ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = ano_of.of1_dx;
        ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = ano_of.of1_dy;
      } else  // 无效
      {
        ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = 0x8000;
        ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = 0x8000;
      }
#else
    if (of_update_cnt != up_flow.update_cnt) {
      of_update_cnt = up_flow.update_cnt;
      ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = up_flow.x_speed;
      ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = up_flow.y_speed;
#endif  // !OF_USE_UP_FLOW
      // 不输入z轴速度，将z速度赋值为无效
      ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = 0x8000;
      // 触发发送
      dt.fun[0x33].WTS = 1;
    }
  } else {
    // 每一毫秒dT_ms+1，用来判断是否长时间无数据
    dT_ms++;
    // 检查速度数据是否更新
    if (user_vel_update_cnt != user_vel.update_cnt) {
      user_vel_update_cnt = user_vel.update_cnt;
      // XY_VEL
      ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = user_vel.vel_x;
      ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = user_vel.vel_y;
      ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = user_vel.vel_z;
      // 触发发送
      dt.fun[0x33].WTS = 1;
      dT_ms = 0;
    }
    if (dT_ms > 100) {  // 应以至少10Hz发送数据, 否则切换回光流
      using_of = 1;
      dT_ms = 0;
      LxStringSend(LOG_COLOR_RED, "WARN: velocity data lost, switch to OF");
    }
  }
}

static inline void General_Distance_Data_Handle() {
  static u8 of_alt_update_cnt = 0, user_alt_update_cnt = 0;
  static u16 dT_ms = 0;
  if (using_of_alt && user_alt_update_cnt != user_alt.update_cnt) {
    LxStringSend(LOG_COLOR_GREEN, "INFO: switch to distance data");
    using_of_alt = 0;
  }
  if (using_of_alt) {
#if !OF_USE_UP_FLOW
    if (of_alt_update_cnt != ano_of.alt_update_cnt) {
      of_alt_update_cnt = ano_of.alt_update_cnt;
      ext_sens.gen_dis.st_data.distance_cm = ano_of.of_alt_cm;
#else
    if (of_alt_update_cnt != up_flow.update_cnt) {
      of_alt_update_cnt = up_flow.update_cnt;
      ext_sens.gen_dis.st_data.distance_cm = up_flow.height;
#endif  // !OF_USE_UP_FLOW
      ext_sens.gen_dis.st_data.direction = 0;
      ext_sens.gen_dis.st_data.angle_100 = 270;
      // 触发发送
      dt.fun[0x34].WTS = 1;
    }
  } else {
    // 每一毫秒dT_ms+1，用来判断是否长时间无数据
    dT_ms++;
    // 检查速度数据是否更新
    if (user_alt_update_cnt != user_alt.update_cnt) {
      user_alt_update_cnt = user_alt.update_cnt;
      // XY_VEL
      ext_sens.gen_dis.st_data.direction = 0;
      ext_sens.gen_dis.st_data.angle_100 = 270;
      ext_sens.gen_dis.st_data.distance_cm = user_alt.alt;
      // 触发发送
      dt.fun[0x34].WTS = 1;
      dT_ms = 0;
    }
    if (dT_ms > 100) {  // 应以至少10Hz发送数据, 否则切换回光流
      using_of_alt = 1;
      dT_ms = 0;
      LxStringSend(LOG_COLOR_RED, "WARN: distance data lost, switch to OF");
    }
  }
}

static inline void General_Position_Data_Handle() {
  static u8 pos_update_cnt;
  if (pos_update_cnt != user_pos.update_cnt) {
    //
    pos_update_cnt = user_pos.update_cnt;
    //
    ext_sens.gen_pos.st_data.ulhca_pos_cm[0] = user_pos.pos_x;
    ext_sens.gen_pos.st_data.ulhca_pos_cm[1] = user_pos.pos_y;
    ext_sens.gen_pos.st_data.ulhca_pos_cm[2] = user_pos.pos_z;
    // 触发发送
    dt.fun[0x32].WTS = 1;
  }
}
void LX_FC_EXT_Sensor_Task(float dT_s)  // 1ms
{
  //
  General_Velocity_Data_Handle();
  //
  General_Distance_Data_Handle();
  //
  General_Position_Data_Handle();
}
