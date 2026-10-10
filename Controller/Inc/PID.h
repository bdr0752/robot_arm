#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include <stdbool.h>

/* PID 参数：所有时间使用秒，积分限幅针对 I 项输出而非误差累计值。 */
typedef struct {
    float K_P;             /* 比例增益。 */
    float K_I;             /* 积分增益。 */
    float K_D;             /* 微分增益。 */

    float Out_Min;         /* PID 总输出下限。 */
    float Out_Max;         /* PID 总输出上限。 */
    float I_Out_Min;       /* I 项输出下限。 */
    float I_Out_Max;       /* I 项输出上限。 */
    bool D_Filter_Enable; /* true 启用微分滤波，false 使用原始微分。 */
    float D_Filter_Time;   /* 滤波时间常数，单位秒；启用且大于 0 时生效。 */
} Struct_PID_Config;

/* 每个控制对象独立保存参数和历史；可直接监视各项输出。 */
typedef struct {
    Struct_PID_Config Config;

    float Target;
    float Now;
    float Error;

    float P_Out;
    float I_Out;
    float D_Out;
    float Out;

    float Pre_Now;
    float D_Filter;
    bool Has_Pre_Now;
} Struct_PID;

bool PID_Init(Struct_PID *pid, const Struct_PID_Config *config);
void PID_Reset(Struct_PID *pid);
bool PID_Adjust(Struct_PID *pid, float Target, float Now, float D_T, float *Out);

#endif
