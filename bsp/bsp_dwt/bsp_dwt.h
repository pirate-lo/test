#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"
#include "stdint.h"

typedef struct 
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
}DWT_Time_t;

/**
 * @brief 该宏用于计算代码运行时间，单位为秒/s，返回值为float类型
 *         首先需要创建一个float类型的变量，用于储存时间间隔
 *        计算得到的时间间隔同时还会通过RTT打印到日志终值
 * 
*/
#define TIME_ELAPSE(dt, code)                   \
    do                                          \
    {                                           \
        float tstart = DWT_GetTimeline_s();     \
        code;                                   \
        dt = DWT_GetTimeline_s() - tstart;      \
    } while (0);                                
    
/**
 * @brief 初始化DWT，传入参数为CPU的频率。单位MHz
 * @param CPU_Freq_mHz A板为180MHz C板为168 MHz
 * */    
void DWT_Init(uint32_t CPU_Freq_mhz);

/**
 * @brief 获取两次调用之间的时间间隔，单位s
 * @param cnt_last 上次调用的时间戳
 * @return float 时间间隔，单位s
*/
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔，单位s
 * @param cnt_last 上次调用的时间戳
 * @return double 时间间隔，单位s
*/
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief 获取当前时间，单位s，即初始化后时间
 * @return float
*/
float DWT_GetTimeline_s(void);

/**
 * @brief 获取当前时间，单位ms，即初始化后时间
 * @return float
*/
float DWT_GetTimeline_ms(void);

/**
 * @brief 获取当前时间，单位us，即初始化后时间
 * @return uint64_t
*/
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief DWT延时函数，单位s
 * @attention 该函数不受中断是否开启的影响，可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()应用此函数
 * @param Delay 延时时间，单位为s
 */
 void DWT_Delay(float Delay);

/**
 * @brief DWT更新时间轴函数，会被三个timeline函数调用 
 * @attention 如果长时间不调用timeline函数，则需要手动调用该函数更新时间轴，否则CYCCNT溢出后定时和时间轴不准确
 */ 
void DWT_SysTimeUpdate(void);  

#endif //!__BSP_DWT_H
