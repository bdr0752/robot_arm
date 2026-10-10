#include "PID.h"

#include <assert.h>
#include <math.h>

/**
  * @brief  检查浮点输出是否在允许误差内。
  * @param  actual: 实际输出。
  * @param  expected: 期望输出。
  * @retval 无，偏差超限时断言失败。
  */
static void expect_value(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

/**
  * @brief  验证比例、积分限幅、抗饱和、测量微分和复位行为。
  * @param  无。
  * @retval 0 表示所有检查通过。
  */
int main(void)
{
    Struct_PID_Config config = {
        .K_P = 2.0f,
        .K_I = 0.0f,
        .K_D = 0.0f,
        .Out_Min = -10.0f,
        .Out_Max = 10.0f,
        .I_Out_Min = -2.0f,
        .I_Out_Max = 2.0f,
        .D_Filter_Enable = false,
        .D_Filter_Time = 0.0f
    };
    Struct_PID pid;
    float output;

    assert(PID_Init(&pid, &config));
    assert(PID_Adjust(&pid, 5.0f, 3.0f, 0.1f, &output));
    expect_value(output, 4.0f);
    expect_value(pid.P_Out, 4.0f);

    config.K_I = 1.0f;
    assert(PID_Init(&pid, &config));
    for (unsigned i = 0U; i < 20U; ++i) {
        assert(PID_Adjust(&pid, 100.0f, 0.0f, 0.1f, &output));
    }
    expect_value(output, 10.0f);
    expect_value(pid.I_Out, 0.0f);

    config.K_P = 0.0f;
    assert(PID_Init(&pid, &config));
    for (unsigned i = 0U; i < 10U; ++i) {
        assert(PID_Adjust(&pid, 10.0f, 0.0f, 0.1f, &output));
    }
    expect_value(output, 2.0f);
    assert(PID_Adjust(&pid, -10.0f, 0.0f, 0.1f, &output));
    expect_value(output, 1.0f);

    config.K_I = 0.0f;
    config.K_D = 1.0f;
    config.D_Filter_Enable = true;
    config.D_Filter_Time = 0.1f;
    assert(PID_Init(&pid, &config));
    assert(PID_Adjust(&pid, 0.0f, 0.0f, 0.1f, &output));
    assert(PID_Adjust(&pid, 100.0f, 0.0f, 0.1f, &output));
    expect_value(output, 0.0f);  /* 目标阶跃不引入微分冲击。 */
    assert(PID_Adjust(&pid, 100.0f, 1.0f, 0.1f, &output));
    expect_value(output, -5.0f);

    /* 关闭滤波后，即使时间常数非零，也直接使用测量微分。 */
    config.D_Filter_Enable = false;
    assert(PID_Init(&pid, &config));
    assert(PID_Adjust(&pid, 100.0f, 0.0f, 0.1f, &output));
    assert(PID_Adjust(&pid, 100.0f, 1.0f, 0.1f, &output));
    expect_value(output, -10.0f);

    assert(!PID_Adjust(&pid, 100.0f, 2.0f, 0.0f, &output));
    expect_value(pid.Now, 1.0f);
    PID_Reset(&pid);
    expect_value(pid.I_Out, 0.0f);
    expect_value(pid.Out, 0.0f);
    assert(!pid.Has_Pre_Now);
    return 0;
}
