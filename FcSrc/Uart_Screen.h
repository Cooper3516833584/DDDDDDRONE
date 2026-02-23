/**
 * @file Uart_Screen.h
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-07-18
 *
 * THINK DIFFERENTLY
 */

#ifndef _UART_SCREEN_H_
#define _UART_SCREEN_H_

#include "SysConfig.h"
void Uart_Screen_GetOneByte(u8 data);
void Send_To_Screen(u8* data, u8 len);
#endif
