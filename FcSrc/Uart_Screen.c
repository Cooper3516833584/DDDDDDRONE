/**
 * @file Uart_Screen.c
 * @brief 串口屏通讯
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-07-18
 *
 * THINK DIFFERENTLY
 */

#include "Uart_Screen.h"

#include "Drv_Uart.h"
#include "User_Com.h"

static u8 screen_buf[260] = {0};
u8* screen_buf_ptr = screen_buf + 4;
u8 data_cnt = 0;

extern void cdc_acm_data_send(uint8_t* buf, uint32_t len);
void Uart_Screen_DataSend(void) {
  screen_buf[0] = 0xAA;          // header1
  screen_buf[1] = 0x55;          // header2
  screen_buf[2] = data_cnt + 1;  // length
  screen_buf[3] = 0x05;          // custom id
  screen_buf[data_cnt + 4] = 0;  // checksum
  for (int i = 0; i < data_cnt + 4; i++) {
    screen_buf[data_cnt + 4] += screen_buf[i];
  }
  cdc_acm_data_send(screen_buf, data_cnt + 5);
  data_cnt = 0;
}

void Uart_Screen_GetOneByte(u8 data) {
  if (!user_connected) return;
  screen_buf_ptr[data_cnt++] = data;
  if (data_cnt == 254) {  // overflow
    Uart_Screen_DataSend();
  } else if (data_cnt >= 2 && screen_buf_ptr[data_cnt - 2] == '\r' &&
             screen_buf_ptr[data_cnt - 1] == '\n') {
    Uart_Screen_DataSend();
  }
}

void Send_To_Screen(u8* data, u8 len) { DrvUart3SendBuf(data, len); }
