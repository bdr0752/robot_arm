#ifndef FT_SERVO_DEFS_H
#define FT_SERVO_DEFS_H

/* FT-SCS 协议指令码 */
#define FT_SCS_INST_PING          0x01U
#define FT_SCS_INST_READ          0x02U
#define FT_SCS_INST_WRITE         0x03U
#define FT_SCS_INST_REG_WRITE     0x04U
#define FT_SCS_INST_REG_ACTION    0x05U
#define FT_SCS_INST_SYNC_READ     0x82U
#define FT_SCS_INST_SYNC_WRITE    0x83U

/* SMS/STS 舵机寄存器地址 */
#define FT_SMS_REG_ID             0x05U
#define FT_SMS_REG_BAUD_RATE      0x06U
#define FT_SMS_REG_MODE           0x21U
#define FT_SMS_REG_TORQUE_ENABLE  0x28U
#define FT_SMS_REG_ACC            0x29U
#define FT_SMS_REG_GOAL_POS_L     0x2AU
#define FT_SMS_REG_GOAL_SPEED_L   0x2EU
#define FT_SMS_REG_LOCK           0x37U
#define FT_SMS_REG_PRESENT_POS_L  0x38U
#define FT_SMS_REG_PRESENT_CUR_L  0x45U

#endif