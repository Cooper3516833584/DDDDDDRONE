/******************** (C) COPYRIGHT 2017 ANO Tech
 ********************************* 作者    ：匿名科创 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：主循环
 **********************************************************************************/
#include "Ano_Scheduler.h"
#include "SysConfig.h"
#include "Drv_PwmOut.h"
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  TIM1->CCR1 = 0;  // 4
  TIM1->CCR2 = 0;  // 3
  TIM1->CCR3 = 0;  // 2
  TIM1->CCR4 = 0;  // 1
  // 硬重置
  NVIC_SystemReset();
  while (1) {
    //当系统出错后，会进入这个死循环
  }
}
#endif

void HardFault_Handler(void) {
  TIM1->CCR1 = 0;  // 4
  TIM1->CCR2 = 0;  // 3
  TIM1->CCR3 = 0;  // 2
  TIM1->CCR4 = 0;  // 1
  // 硬重置
  NVIC_SystemReset();
  while (1) {
  }
}

/**
 * 初始化USB-FS-Device的gpio和时钟电源
 */
void mcu_usb_device_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


    /* Configure DM DP Pins */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    //开启 OTG_FS_IRQ中断，此中断的回调函数，已经在协议栈中实现，我们无需关注，非常轻松
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void usb_dc_low_level_init(void) {
    mcu_usb_device_init();
}

extern void cdc_acm_init();
//=======================================================================================
//=======================================================================================
int main(void) {
  //进行所有设备的初始化，并将初始化结果保存
  All_Init();
  //初始化CherryUSB协议栈
  cdc_acm_init();
  //调度器初始化，系统为裸奔，这里人工做了一个时分调度器
  Scheduler_Setup();
  while (1) {
    //运行任务调度器，所有系统功能，除了中断服务函数，都在任务调度器内完成
    Scheduler_Run();
  }
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
