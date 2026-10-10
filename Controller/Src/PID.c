#include "PID.h"

#include <math.h>
#include <stddef.h>

/* 参数检查与限幅 ----------------------------------------------------------- */

/**
  * @brief  将输入值限制在指定区间内。
  * @param  value: 输入值。
  * @param  minimum: 下限。
  * @param  maximum: 上限。
  * @retval 限幅后的值。
  */
static float PID_Limit(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/**
  * @brief  检查增益、输出范围、积分范围和微分滤波参数。
  * @param  config: PID 参数。
  * @retval true 为有效，false 为无效。
  */
static bool PID_Check_Config(const Struct_PID_Config *config)
{
    if (config == NULL) {
        return false;
    }

    /* 增益必须是有限的非负数。 */
    if (!isfinite(config->K_P) || config->K_P < 0.0f ||
        !isfinite(config->K_I) || config->K_I < 0.0f ||
        !isfinite(config->K_D) || config->K_D < 0.0f) {
        return false;
    }

    /* 总输出上下限有效，积分输出范围包含零。 */
    if (!isfinite(config->Out_Min) || !isfinite(config->Out_Max) ||
        config->Out_Min >= config->Out_Max ||
        !isfinite(config->I_Out_Min) || config->I_Out_Min > 0.0f ||
        !isfinite(config->I_Out_Max) || config->I_Out_Max < 0.0f) {
        return false;
    }

    return !config->D_Filter_Enable ||
           (isfinite(config->D_Filter_Time) && config->D_Filter_Time >= 0.0f);
}

/* 初始化与复位 ------------------------------------------------------------- */

/**
  * @brief  保存 PID 参数并清除计算历史，不访问硬件。
  * @param  pid: PID 对象。
  * @param  config: 增益、限幅和滤波参数。
  * @retval true 为初始化成功；false 为参数无效，对象保持不变。
  */
bool PID_Init(Struct_PID *pid, const Struct_PID_Config *config)
{
    if (pid == NULL || !PID_Check_Config(config)) {
        return false;
    }

    pid->Config = *config;
    PID_Reset(pid);
    return true;
}

/**
  * @brief  清除目标、反馈、各项输出和历史，保留 PID 参数。
  * @param  pid: 已初始化的 PID 对象，NULL 时不操作。
  * @retval 无。
  */
void PID_Reset(Struct_PID *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->Target = 0.0f;
    pid->Now = 0.0f;
    pid->Error = 0.0f;

    pid->P_Out = 0.0f;
    pid->I_Out = 0.0f;
    pid->D_Out = 0.0f;
    pid->Out = 0.0f;

    pid->Pre_Now = 0.0f;
    pid->D_Filter = 0.0f;
    pid->Has_Pre_Now = false;
}

/* PID 计算 ----------------------------------------------------------------- */

/**
  * @brief  依次计算 P、I、D 输出，处理积分抗饱和并更新历史。
  * @param  pid: 已成功初始化的对象，不直接修改内部历史状态。
  * @param  Target: 目标值，与 Now 使用相同单位。
  * @param  Now: 当前测量值。
  * @param  D_T: 两次有效测量之间的时间，单位秒，必须大于零。
  * @param  Out: 限幅后的 PID 输出地址。
  * @retval true 为成功；false 为输入或计算无效，不改变状态和输出。
  * @note   微分针对测量值，避免目标阶跃造成微分冲击。
  *         D_Filter_Enable 选择是否滤波；D_Filter_Time 为 0 时使用原始微分。
  */
bool PID_Adjust(Struct_PID *pid, float Target, float Now, float D_T, float *Out)
{
    if (pid == NULL || Out == NULL || !PID_Check_Config(&pid->Config) ||
        !isfinite(Target) || !isfinite(Now) ||
        !isfinite(D_T) || D_T <= 0.0f) {
        return false;
    }

    const Struct_PID_Config *config = &pid->Config;
    float error = Target - Now;

    /* 1. 比例项：根据当前误差计算输出。 */
    float p_out = config->K_P * error;

    /* 2. 积分项：累计误差对应的输出，并限制积分幅值。 */
    float i_increment = config->K_I * error * D_T;
    float i_out = pid->I_Out + i_increment;
    if (!isfinite(i_out)) {
        return false;
    }
    i_out = PID_Limit(i_out, config->I_Out_Min, config->I_Out_Max);

    /* 3. 微分项：首次采样不微分，后续按配置选择原始微分或低通滤波。 */
    float d_filter = 0.0f;
    if (pid->Has_Pre_Now && config->K_D > 0.0f) {
        float now_rate = (Now - pid->Pre_Now) / D_T;
        d_filter = -now_rate;
        if (config->D_Filter_Enable && config->D_Filter_Time > 0.0f) {
            float filter_ratio = D_T / (config->D_Filter_Time + D_T);
            d_filter = pid->D_Filter + filter_ratio * (-now_rate - pid->D_Filter);
        }
    }
    float d_out = config->K_D * d_filter;

    /* 4. 合成输出：饱和时，禁止积分继续向饱和方向增加。 */
    float out = p_out + i_out + d_out;
    if (!isfinite(error) || !isfinite(d_filter) || !isfinite(out)) {
        return false;
    }

    bool upper_saturation =
        out > config->Out_Max && i_increment > 0.0f;
    bool lower_saturation =
        out < config->Out_Min && i_increment < 0.0f;

    if (upper_saturation || lower_saturation) {
        i_out = pid->I_Out;
        out = p_out + i_out + d_out;
    }
    if (!isfinite(out)) {
        return false;
    }
    out = PID_Limit(out, config->Out_Min, config->Out_Max);

    /* 5. 更新结果和历史：各项输出可直接用于调试观察。 */
    pid->Target = Target;
    pid->Now = Now;
    pid->Error = error;
    pid->P_Out = p_out;
    pid->I_Out = i_out;
    pid->D_Out = d_out;
    pid->Out = out;
    pid->Pre_Now = Now;
    pid->D_Filter = d_filter;
    pid->Has_Pre_Now = true;

    *Out = out;
    return true;
}
