#ifndef FT_SERVO_H
#define FT_SERVO_H

#include <stdint.h>
#include <stddef.h>

/* DMA 接收缓冲区为 32 字节，应答额外占 6 字节。 */
#define FT_SERVO_MAX_READ_BYTES 26U

typedef enum {
    FT_SERVO_OK = 0,
    FT_SERVO_TIMEOUT = -1,
    FT_SERVO_IO_ERROR = -2,
    FT_SERVO_BAD_REPLY = -3,
    FT_SERVO_BAD_CHECKSUM = -4,
    FT_SERVO_BAD_ARGUMENT = -5
} FtServoResult;

FtServoResult ft_servo_ping(uint8_t id, uint8_t *status);

/* 通用寄存器操作；写入广播 ID 0xFE 时无应答，status 可以为 NULL。 */
FtServoResult ft_servo_read(uint8_t id, uint8_t address,
                            uint8_t *data, uint8_t length, uint8_t *status);
FtServoResult ft_servo_write(uint8_t id, uint8_t address,
                             const uint8_t *data, uint8_t length, uint8_t *status);
FtServoResult ft_servo_reg_write(uint8_t id, uint8_t address,
                                 const uint8_t *data, uint8_t length, uint8_t *status);
FtServoResult ft_servo_action(uint8_t id, uint8_t *status);
FtServoResult ft_servo_sync_write(const uint8_t *ids, uint8_t count,
                                  uint8_t address, const uint8_t *data,
                                  uint8_t bytes_per_servo);
FtServoResult ft_servo_read_byte(uint8_t id, uint8_t address,
                                 uint8_t *value, uint8_t *status);
FtServoResult ft_servo_read_word(uint8_t id, uint8_t address,
                                 uint16_t *value, uint8_t *status);
FtServoResult ft_servo_write_byte(uint8_t id, uint8_t address,
                                  uint8_t value, uint8_t *status);
FtServoResult ft_servo_write_word(uint8_t id, uint8_t address,
                                  uint16_t value, uint8_t *status);

/* SMS/STS 常用控制；位置和速度负值沿用飞特例程的 bit15 符号编码。 */
FtServoResult ft_servo_set_torque(uint8_t id, uint8_t enabled, uint8_t *status);
FtServoResult ft_servo_set_mode(uint8_t id, uint8_t mode, uint8_t *status);
FtServoResult ft_servo_set_lock(uint8_t id, uint8_t locked, uint8_t *status);
FtServoResult ft_servo_calibrate_offset(uint8_t id, uint8_t *status);
FtServoResult ft_servo_write_position(uint8_t id, int16_t position,
                                      uint16_t speed, uint8_t acceleration,
                                      uint8_t *status);
FtServoResult ft_servo_reg_write_position(uint8_t id, int16_t position,
                                          uint16_t speed, uint8_t acceleration,
                                          uint8_t *status);
FtServoResult ft_servo_write_speed(uint8_t id, int16_t speed,
                                   uint8_t acceleration, uint8_t *status);
FtServoResult ft_servo_reg_write_speed(uint8_t id, int16_t speed,
                                       uint8_t acceleration, uint8_t *status);

typedef struct {
    uint8_t id;
    int16_t position;
    uint16_t speed;
    uint8_t acceleration;
} FtServoPositionCommand;

typedef struct {
    uint8_t id;
    int16_t speed;
    uint8_t acceleration;
} FtServoSpeedCommand;

/* 批量同步写采用广播帧，没有逐个舵机应答。 */
FtServoResult ft_servo_sync_write_position(const FtServoPositionCommand *commands,
                                            uint8_t count);
FtServoResult ft_servo_sync_write_speed(const FtServoSpeedCommand *commands,
                                         uint8_t count);

/* 反馈值是寄存器原始值；单位换算应按实际舵机型号手册处理。 */

FtServoResult ft_servo_read_position(
    uint8_t id, uint16_t *position, uint8_t *status
);
FtServoResult ft_servo_read_speed(uint8_t id, uint16_t *speed, uint8_t *status);
FtServoResult ft_servo_read_load(uint8_t id, uint16_t *load, uint8_t *status);
FtServoResult ft_servo_read_current(uint8_t id, uint16_t *current, uint8_t *status);
FtServoResult ft_servo_read_voltage(uint8_t id, uint8_t *voltage, uint8_t *status);
FtServoResult ft_servo_read_temperature(uint8_t id, uint8_t *temperature, uint8_t *status);
FtServoResult ft_servo_read_moving(uint8_t id, uint8_t *moving, uint8_t *status);

#endif
