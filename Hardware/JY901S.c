/**
  ******************************************************************************
  * @file    JY901S.c
  * @brief   USART3 中断收数 + 宽松滑动窗口解帧
  *
  * B 增加而 F=0 的常见原因：波特率不对（乱码里偶尔也有字节，但凑不出 0x55 帧）
  ******************************************************************************
  */

#include "JY901S.h"
#include "Delay.h"
#include <math.h>

JY901_Info_t JY901;

#define RING_SIZE 512
static volatile uint8_t  s_ring[RING_SIZE];
static volatile uint16_t s_w = 0;
static volatile uint16_t s_r = 0;
static uint8_t s_last8_idx = 0;

static void usart3_hw(uint32_t baud);
static void ring_push(uint8_t b);
static uint16_t ring_count(void);
static uint8_t  ring_peek(uint16_t off);
static void ring_drop(uint16_t n);
static int16_t  combine(uint8_t h, uint8_t l);
static void handle_frame(const uint8_t *f);
static void usart3_send_reg(uint8_t reg, uint8_t dl, uint8_t dh);

static void ring_push(uint8_t b)
{
    uint16_t next = (uint16_t)((s_w + 1) % RING_SIZE);
    if (next == s_r) {
        s_r = (uint16_t)((s_r + 1) % RING_SIZE);
    }
    s_ring[s_w] = b;
    s_w = next;

    JY901.rx_bytes++;
    JY901.last8[s_last8_idx] = b;
    s_last8_idx = (uint8_t)((s_last8_idx + 1) % 8);

    if (b == 0x55) {
        JY901.cnt_55++;
    }
}

static uint16_t ring_count(void)
{
    if (s_w >= s_r) {
        return (uint16_t)(s_w - s_r);
    }
    return (uint16_t)(RING_SIZE - s_r + s_w);
}

static uint8_t ring_peek(uint16_t off)
{
    return s_ring[(s_r + off) % RING_SIZE];
}

static void ring_drop(uint16_t n)
{
    s_r = (uint16_t)((s_r + n) % RING_SIZE);
}

static int16_t combine(uint8_t h, uint8_t l)
{
    return (int16_t)(((uint16_t)h << 8) | l);
}

static int32_t raw_to_x10(int16_t raw)
{
    return ((int32_t)raw * 1800) / 32768;
}

static void handle_frame(const uint8_t *f)
{
    int16_t ax, ay, az;
    int16_t r0, r1, r2, r3;
    float fax, fay, faz;
    float w, x, y, z;
    float roll, pitch, yaw;
    float sinr, cosr, sinp, siny, cosy;

    JY901.last_type = f[1];
    JY901.frame_ok++;

    switch (f[1]) {
        case 0x53: /* 角度 */
            JY901.angle_frames++;
            JY901.roll_x10  = raw_to_x10(combine(f[3], f[2]));
            JY901.pitch_x10 = raw_to_x10(combine(f[5], f[4]));
            JY901.yaw_x10   = raw_to_x10(combine(f[7], f[6]));
            break;

        case 0x51: /* 加速度 → 估算 roll/pitch */
            ax = combine(f[3], f[2]);
            ay = combine(f[5], f[4]);
            az = combine(f[7], f[6]);
            fax = (float)ax / 32768.0f * 16.0f;
            fay = (float)ay / 32768.0f * 16.0f;
            faz = (float)az / 32768.0f * 16.0f;
            roll  = (float)atan2((double)fay, (double)faz) * 57.29578f;
            pitch = (float)atan2((double)(-fax),
                        (double)sqrt((double)(fay * fay + faz * faz))) * 57.29578f;
            JY901.roll_x10  = (int32_t)(roll * 10.0f);
            JY901.pitch_x10 = (int32_t)(pitch * 10.0f);
            break;

        case 0x59: /* 四元数 */
            r0 = combine(f[3], f[2]);
            r1 = combine(f[5], f[4]);
            r2 = combine(f[7], f[6]);
            r3 = combine(f[9], f[8]);
            w = (float)r0 / 32768.0f;
            x = (float)r1 / 32768.0f;
            y = (float)r2 / 32768.0f;
            z = (float)r3 / 32768.0f;
            sinr = 2.0f * (w * x + y * z);
            cosr = 1.0f - 2.0f * (x * x + y * y);
            sinp = 2.0f * (w * y - z * x);
            siny = 2.0f * (w * z + x * y);
            cosy = 1.0f - 2.0f * (y * y + z * z);
            if (sinp > 1.0f)  sinp = 1.0f;
            if (sinp < -1.0f) sinp = -1.0f;
            roll  = (float)atan2((double)sinr, (double)cosr) * 57.29578f;
            pitch = (float)asin((double)sinp) * 57.29578f;
            yaw   = (float)atan2((double)siny, (double)cosy) * 57.29578f;
            JY901.roll_x10  = (int32_t)(roll * 10.0f);
            JY901.pitch_x10 = (int32_t)(pitch * 10.0f);
            JY901.yaw_x10   = (int32_t)(yaw * 10.0f);
            break;

        default:
            break;
    }
}

static void usart3_hw(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, DISABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, DISABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_Cmd(USART3, DISABLE);
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    (void)USART_ReceiveData(USART3);
    USART_ClearFlag(USART3, USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

void JY901_ResetStats(void)
{
    s_w = 0;
    s_r = 0;
    s_last8_idx = 0;
    JY901.rx_bytes = 0;
    JY901.cnt_55 = 0;
    JY901.frame_ok = 0;
    JY901.angle_frames = 0;
    JY901.last_type = 0;
    JY901.roll_x10 = 0;
    JY901.pitch_x10 = 0;
    JY901.yaw_x10 = 0;
    {
        uint8_t i;
        for (i = 0; i < 8; i++) {
            JY901.last8[i] = 0;
        }
    }
}

void JY901_Init(uint32_t baudrate)
{
    JY901_ResetStats();
    JY901.baud = baudrate;
    usart3_hw(baudrate);
}

/**
  * 滑动窗口：
  * 1) 找 0x55
  * 2) 不限制类型字节
  * 3) 校验和 = 前 10 字节累加低 8 位
  * 若类型在 0x50~0x5F 则解析数据；否则只记 frame_ok（证明协议对齐了）
  */
void JY901_Parse(void)
{
    uint8_t frame[11];
    uint8_t sum;
    uint8_t i;

    while (ring_count() >= 11) {
        if (ring_peek(0) != 0x55) {
            ring_drop(1);
            continue;
        }

        sum = 0;
        for (i = 0; i < 10; i++) {
            frame[i] = ring_peek(i);
            sum = (uint8_t)(sum + frame[i]);
        }
        frame[10] = ring_peek(10);

        if (sum == frame[10]) {
            handle_frame(frame);
            ring_drop(11);
        } else {
            ring_drop(1);
        }
    }
}

static void usart3_send_byte(uint8_t b)
{
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, b);
}

static void usart3_send_reg(uint8_t reg, uint8_t dl, uint8_t dh)
{
    usart3_send_byte(0xFF);
    usart3_send_byte(0xAA);
    usart3_send_byte(reg);
    usart3_send_byte(dl);
    usart3_send_byte(dh);
    Delay_ms(40);
}

void JY901_EnableAngleOutput(void)
{
    usart3_send_reg(0x69, 0x88, 0xB5);
    Delay_ms(80);
    usart3_send_reg(0x02, 0xFF, 0x07);
    Delay_ms(80);
    usart3_send_reg(0x03, 0x06, 0x00);
    Delay_ms(80);
    usart3_send_reg(0x00, 0x00, 0x00);
    Delay_ms(120);
}

/**
  * @brief  扫描常见波特率，优先选 frame_ok 最多，其次 cnt_55 最多
  */
uint32_t JY901_AutoBaud(void)
{
    static const uint32_t bauds[] = {
        9600, 115200, 4800, 57600, 38400, 19200, 230400
    };
    uint8_t n = (uint8_t)(sizeof(bauds) / sizeof(bauds[0]));
    uint8_t i;
    uint8_t t;
    uint32_t best_baud = 9600;
    uint32_t best_frames = 0;
    uint32_t best_55 = 0;

    for (i = 0; i < n; i++) {
        JY901_Init(bauds[i]);

        /* 每种波特率听 500ms */
        for (t = 0; t < 25; t++) {
            JY901_Parse();
            Delay_ms(20);
        }

        if (JY901.frame_ok > best_frames) {
            best_frames = JY901.frame_ok;
            best_55 = JY901.cnt_55;
            best_baud = bauds[i];
        } else if (JY901.frame_ok == best_frames && JY901.cnt_55 > best_55) {
            best_55 = JY901.cnt_55;
            best_baud = bauds[i];
        }
    }

    /* 用最佳波特率正式初始化，并再听 600ms 填统计（供主界面显示） */
    JY901_Init(best_baud);
    for (t = 0; t < 30; t++) {
        JY901_Parse();
        Delay_ms(20);
    }
    return best_baud;
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint16_t dr = USART_ReceiveData(USART3);
        ring_push((uint8_t)dr);
    }
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) {
        (void)USART_ReceiveData(USART3);
    }
}
