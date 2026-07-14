/**
  ******************************************************************************
  * @file    JY901S.h
  * @brief   JY901S / 维特类模块串口接收
  *
  * 接线：模块 TX -> STM32 PB11 ，GND 共地，VCC 供电
  *       模块 RX -> PB10（可选）
  ******************************************************************************
  */
#ifndef __JY901S_H
#define __JY901S_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct {
    int32_t  roll_x10;     /* 0.1° */
    int32_t  pitch_x10;
    int32_t  yaw_x10;
    uint32_t baud;
    uint32_t rx_bytes;
    uint32_t cnt_55;       /* 流里出现 0x55 的次数 */
    uint32_t frame_ok;
    uint32_t angle_frames;
    uint8_t  last_type;
    uint8_t  last8[8];     /* 最近收到的 8 个原始字节 */
} JY901_Info_t;

extern JY901_Info_t JY901;

void JY901_Init(uint32_t baudrate);

/** 清空统计与环形缓冲（切换波特率前调用） */
void JY901_ResetStats(void);

/** 主循环调用：解包 */
void JY901_Parse(void);

/**
  * @brief  在多种波特率下试听，选「有效帧最多」或「0x55 最多」的
  * @retval 选中的波特率
  */
uint32_t JY901_AutoBaud(void);

void JY901_EnableAngleOutput(void);

#endif
