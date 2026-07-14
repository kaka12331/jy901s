/**
  ******************************************************************************
  * @file    main.c
  * @brief   JY901S 显示 Roll / Pitch / Yaw（真实角度，1 位小数）
  *
  * 说明：驱动内部仍用 0.1° 整数（roll_x10），显示时换成 ±xxx.x°
  *       以前直接显示 roll_x10，会出现“好几百”，其实是 几十度×10
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "JY901S.h"

/**
  * @brief  显示 ±整数.小数  例如 +12.3
  * @param  line, col  起始行列（江科大 OLED：1~4 行，1~16 列）
  * @param  x10        0.1° 单位的有符号整数（123 → 12.3°）
  */
static void OLED_ShowDeg1(uint8_t line, uint8_t col, int32_t x10)
{
	int32_t v = x10;
	int32_t ip;
	int32_t fp;

	if (v >= 0) {
		OLED_ShowChar(line, col, '+');
	} else {
		OLED_ShowChar(line, col, '-');
		v = -v;
	}

	ip = v / 10;   /* 整数度 */
	fp = v % 10;   /* 小数 1 位 */

	/* 符号后：3 位整数 + '.' + 1 位小数  共占 6 列（含符号） */
	OLED_ShowNum(line, col + 1, (uint32_t)ip, 3);
	OLED_ShowChar(line, col + 4, '.');
	OLED_ShowNum(line, col + 5, (uint32_t)fp, 1);
}

int main(void)
{
	uint32_t baud;
	uint8_t i;

	OLED_Init();
	OLED_ShowString(1, 1, "Scan baud...");
	Delay_ms(200);

	baud = JY901_AutoBaud();

	OLED_Clear();
	OLED_ShowString(1, 1, "Baud:");
	OLED_ShowNum(1, 6, baud, 6);
	Delay_ms(400);

	if (JY901.frame_ok > 0 || JY901.cnt_55 > 5) {
		JY901_EnableAngleOutput();
		for (i = 0; i < 30; i++) {
			JY901_Parse();
			Delay_ms(20);
		}
	}

	OLED_Clear();
	OLED_ShowString(1, 1, "R:");
	OLED_ShowString(2, 1, "P:");
	OLED_ShowString(3, 1, "Y:");
	OLED_ShowString(4, 1, "unit: deg");

	while (1)
	{
		JY901_Parse();

		/* 真实角度，例如 +12.3  不再显示 ×10 的“好几百” */
		OLED_ShowDeg1(1, 3, JY901.roll_x10);
		OLED_ShowDeg1(2, 3, JY901.pitch_x10);
		OLED_ShowDeg1(3, 3, JY901.yaw_x10);

		Delay_ms(50);
	}
}
