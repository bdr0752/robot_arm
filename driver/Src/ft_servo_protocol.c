#include "ft_servo_protocol.h"

#include "servo_bus.h"

#include <string.h>

/* 协议 LENGTH 是 8 位：写命令长度 = 数据字节数 + 3。 */
#define FT_SERVO_MAX_WRITE_BYTES 252U
#define FT_SERVO_MAX_FRAME_BYTES 259U

/**
  * @brief  将总线层错误转换成舵机驱动结果。
  * @param  result: 总线层返回值。
  * @retval 对应的 FtServoResult。
  */
static FtServoResult map_bus_result(ServoBusResult result)
{
    if (result == SERVO_BUS_TIMEOUT) {
        return FT_SERVO_TIMEOUT;
    }
    if (result == SERVO_BUS_LENGTH) {
        return FT_SERVO_BAD_REPLY;
    }
    return FT_SERVO_IO_ERROR;
}

/**
  * @brief  计算从 ID 到最后一个数据字节的反码校验和。
  * @param  data: 待求和的字节序列。
  * @param  length: 字节数。
  * @retval 8 位反码校验和。
  */
static uint8_t frame_checksum(const uint8_t *data, uint16_t length)
{
    uint32_t sum = 0U;
    for (uint16_t i = 0U; i < length; ++i) {
        sum += data[i];
    }
    return (uint8_t)~sum;
}

/**
  * @brief  检查指定 ID 的应答帧头、长度、ID 和校验和。
  * @param  id: 请求时使用的舵机 ID。
  * @param  frame: 应答帧。
  * @param  data_length: 期望的数据区字节数。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval FT_SERVO_OK、格式错误或校验错误。
  */
static FtServoResult check_reply(uint8_t id, const uint8_t *frame,
                                 uint8_t data_length, uint8_t *status)
{
    if (frame[0] != 0xFFU || frame[1] != 0xFFU || frame[2] != id ||
        frame[3] != (uint8_t)(data_length + 2U)) {
        return FT_SERVO_BAD_REPLY;
    }
    if (frame[data_length + 5U] != frame_checksum(&frame[2],
                                                   (uint16_t)data_length + 3U)) {
        return FT_SERVO_BAD_CHECKSUM;
    }
    if (status != NULL) {
        *status = frame[4];
    }
    return FT_SERVO_OK;
}

/**
  * @brief  使用 WRITE 或 REG_WRITE 指令写连续寄存器。
  * @param  id: 目标 ID；广播 ID 无应答。
  * @param  instruction: 写指令码。
  * @param  address: 首个寄存器地址。
  * @param  data: 待写数据。
  * @param  length: 待写字节数。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
static FtServoResult write_registers(uint8_t id, uint8_t instruction,
                                     uint8_t address, const uint8_t *data,
                                     uint8_t length, uint8_t *status)
{
    if (id > FT_SCS_BROADCAST_ID || data == NULL || length == 0U ||
        length > FT_SERVO_MAX_WRITE_BYTES ||
        (uint16_t)address + length > 256U) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    uint8_t tx[FT_SERVO_MAX_FRAME_BYTES];
    uint8_t rx[6];
    tx[0] = 0xFFU;
    tx[1] = 0xFFU;
    tx[2] = id;
    tx[3] = (uint8_t)(length + 3U);
    tx[4] = instruction;
    tx[5] = address;
    memcpy(&tx[6], data, length);
    tx[length + 6U] = frame_checksum(&tx[2], (uint16_t)length + 4U);

    uint16_t reply_length = id == FT_SCS_BROADCAST_ID ? 0U : sizeof(rx);
    ServoBusResult bus = servo_bus_exchange(tx, (uint16_t)length + 7U,
                                             reply_length == 0U ? NULL : rx,
                                             reply_length);
    if (bus != SERVO_BUS_OK) {
        return map_bus_result(bus);
    }
    return reply_length == 0U ? FT_SERVO_OK : check_reply(id, rx, 0U, status);
}

/**
  * @brief  从指定舵机读取连续寄存器。
  * @param  id: 单播舵机 ID，范围 0～253。
  * @param  address: 起始寄存器地址。
  * @param  data: 输出缓冲区，至少容纳 length 字节。
  * @param  length: 读取字节数，范围 1～FT_SERVO_MAX_READ_BYTES。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read(uint8_t id, uint8_t address,
                            uint8_t *data, uint8_t length, uint8_t *status)
{
    if (id > 253U || data == NULL || length == 0U ||
        length > FT_SERVO_MAX_READ_BYTES ||
        (uint16_t)address + length > 256U) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    uint8_t tx[8] = {0xFFU, 0xFFU, id, 0x04U,
                     FT_SCS_INST_READ, address, length, 0U};
    uint8_t rx[FT_SERVO_MAX_READ_BYTES + 6U];
    tx[7] = frame_checksum(&tx[2], 5U);
    ServoBusResult bus = servo_bus_exchange(tx, sizeof(tx), rx,
                                             (uint16_t)length + 6U);
    if (bus != SERVO_BUS_OK) {
        return map_bus_result(bus);
    }
    FtServoResult result = check_reply(id, rx, length, status);
    if (result == FT_SERVO_OK) {
        memcpy(data, &rx[5], length);
    }
    return result;
}

/**
  * @brief  立即写入指定舵机的连续寄存器。
  * @param  id: 单播 ID 或广播 ID 0xFE。
  * @param  address: 起始寄存器地址。
  * @param  data: 待写数据缓冲区。
  * @param  length: 写入字节数，范围 1～252。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果；广播只表示发送成功，不代表各舵机已执行。
  */
FtServoResult ft_servo_write(uint8_t id, uint8_t address,
                             const uint8_t *data, uint8_t length, uint8_t *status)
{
    return write_registers(id, FT_SCS_INST_WRITE, address, data, length, status);
}

/**
  * @brief  暂存连续寄存器写入，等待 ACTION 指令统一执行。
  * @param  id: 单播 ID 或广播 ID 0xFE。
  * @param  address: 起始寄存器地址。
  * @param  data: 待写数据缓冲区。
  * @param  length: 写入字节数，范围 1～252。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_reg_write(uint8_t id, uint8_t address,
                                 const uint8_t *data, uint8_t length, uint8_t *status)
{
    return write_registers(id, FT_SCS_INST_REG_WRITE, address, data, length, status);
}

/**
  * @brief  执行先前通过 REG_WRITE 暂存的写入。
  * @param  id: 单播 ID 或广播 ID 0xFE。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果；广播只表示发送成功。
  */
FtServoResult ft_servo_action(uint8_t id, uint8_t *status)
{
    if (id > FT_SCS_BROADCAST_ID) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    uint8_t tx[6] = {0xFFU, 0xFFU, id, 0x02U,
                     FT_SCS_INST_REG_ACTION, 0U};
    uint8_t rx[6];
    tx[5] = frame_checksum(&tx[2], 3U);
    uint16_t reply_length = id == FT_SCS_BROADCAST_ID ? 0U : sizeof(rx);
    ServoBusResult bus = servo_bus_exchange(tx, sizeof(tx),
                                             reply_length == 0U ? NULL : rx,
                                             reply_length);
    if (bus != SERVO_BUS_OK) {
        return map_bus_result(bus);
    }
    return reply_length == 0U ? FT_SERVO_OK : check_reply(id, rx, 0U, status);
}

/**
  * @brief  通过广播帧向多个 ID 写入同一寄存器区间。
  * @param  ids: 目标 ID 数组，元素必须在 0～253。
  * @param  count: 目标数量，受 8 位协议长度字段限制。
  * @param  address: 起始寄存器地址。
  * @param  data: 顺序排列的每个舵机数据，长度为 count * bytes_per_servo。
  * @param  bytes_per_servo: 每个舵机的数据字节数。
  * @retval 参数错误、总线错误或发送成功；广播无应答。
  */
FtServoResult ft_servo_sync_write(const uint8_t *ids, uint8_t count,
                                  uint8_t address, const uint8_t *data,
                                  uint8_t bytes_per_servo)
{
    uint32_t packet_length = 4U + (uint32_t)count * (bytes_per_servo + 1U);
    if (ids == NULL || data == NULL || count == 0U || bytes_per_servo == 0U ||
        (uint16_t)address + bytes_per_servo > 256U || packet_length > 255U) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    uint8_t tx[FT_SERVO_MAX_FRAME_BYTES];
    tx[0] = 0xFFU;
    tx[1] = 0xFFU;
    tx[2] = FT_SCS_BROADCAST_ID;
    tx[3] = (uint8_t)packet_length;
    tx[4] = FT_SCS_INST_SYNC_WRITE;
    tx[5] = address;
    tx[6] = bytes_per_servo;
    uint16_t offset = 7U;
    for (uint8_t i = 0U; i < count; ++i) {
        if (ids[i] > 253U) {
            return FT_SERVO_BAD_ARGUMENT;
        }
        tx[offset++] = ids[i];
        memcpy(&tx[offset], &data[(uint16_t)i * bytes_per_servo], bytes_per_servo);
        offset += bytes_per_servo;
    }
    tx[offset] = frame_checksum(&tx[2], offset - 2U);
    ServoBusResult bus = servo_bus_exchange(tx, offset + 1U, NULL, 0U);
    return bus == SERVO_BUS_OK ? FT_SERVO_OK : map_bus_result(bus);
}

/**
  * @brief  读取一个 8 位寄存器。
  * @param  id: 单播舵机 ID。
  * @param  address: 寄存器地址。
  * @param  value: 成功时写入寄存器值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_byte(uint8_t id, uint8_t address,
                                 uint8_t *value, uint8_t *status)
{
    return ft_servo_read(id, address, value, 1U, status);
}

/**
  * @brief  按 SMS/STS 低字节在前的顺序读取 16 位寄存器。
  * @param  id: 单播舵机 ID。
  * @param  address: 低字节寄存器地址。
  * @param  value: 成功时写入 16 位原始值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_word(uint8_t id, uint8_t address,
                                 uint16_t *value, uint8_t *status)
{
    if (value == NULL) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    uint8_t bytes[2];
    FtServoResult result = ft_servo_read(id, address, bytes, 2U, status);
    if (result == FT_SERVO_OK) {
        *value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    }
    return result;
}

/**
  * @brief  写入一个 8 位寄存器。
  * @param  id: 单播 ID 或广播 ID。
  * @param  address: 寄存器地址。
  * @param  value: 待写数值。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_write_byte(uint8_t id, uint8_t address,
                                  uint8_t value, uint8_t *status)
{
    return ft_servo_write(id, address, &value, 1U, status);
}

/**
  * @brief  按低字节在前的顺序写入 16 位寄存器。
  * @param  id: 单播 ID 或广播 ID。
  * @param  address: 低字节寄存器地址。
  * @param  value: 待写的 16 位原始值。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_write_word(uint8_t id, uint8_t address,
                                  uint16_t value, uint8_t *status)
{
    uint8_t bytes[2] = {(uint8_t)value, (uint8_t)(value >> 8)};
    return ft_servo_write(id, address, bytes, sizeof(bytes), status);
}

/**
  * @brief  启用或关闭舵机扭矩输出。
  * @param  id: 单播 ID 或广播 ID。
  * @param  enabled: 1 为启用，0 为关闭。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_set_torque(uint8_t id, uint8_t enabled, uint8_t *status)
{
    if (enabled > 1U) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    return ft_servo_write_byte(id, FT_SMS_REG_TORQUE_ENABLE, enabled, status);
}

/**
  * @brief  写入 SMS/STS 工作模式寄存器。
  * @param  id: 单播 ID 或广播 ID。
  * @param  mode: 模式原始值；飞特例程用 1 表示恒速模式。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_set_mode(uint8_t id, uint8_t mode, uint8_t *status)
{
    return ft_servo_write_byte(id, FT_SMS_REG_MODE, mode, status);
}

/**
  * @brief  读取 SMS/STS 当前工作模式寄存器。
  * @param  id: 单播舵机 ID。
  * @param  mode: 成功时写入模式寄存器原始值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_mode(uint8_t id, uint8_t *mode, uint8_t *status)
{
    return ft_servo_read_byte(id, FT_SMS_REG_MODE, mode, status);
}

/**
  * @brief  设置 EEPROM 锁定寄存器。
  * @param  id: 单播 ID 或广播 ID。
  * @param  locked: 1 为锁定，0 为解锁。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_set_lock(uint8_t id, uint8_t locked, uint8_t *status)
{
    if (locked > 1U) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    return ft_servo_write_byte(id, FT_SMS_REG_LOCK, locked, status);
}

/**
  * @brief  按飞特例程执行中位校准（扭矩寄存器写入 128）。
  * @param  id: 单播舵机 ID。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_calibrate_offset(uint8_t id, uint8_t *status)
{
    return ft_servo_write_byte(id, FT_SMS_REG_TORQUE_ENABLE, 128U, status);
}

/**
  * @brief  将有符号位置或速度编码为飞特例程使用的 bit15 符号格式。
  * @param  value: 待编码数值。
  * @param  encoded: 输出的 16 位编码值。
  * @retval FT_SERVO_OK；-32768 无法按此格式编码，返回参数错误。
  */
static FtServoResult encode_signed(int16_t value, uint16_t *encoded)
{
    if (value == INT16_MIN) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    *encoded = value < 0 ? (uint16_t)(-(int32_t)value) | 0x8000U : (uint16_t)value;
    return FT_SERVO_OK;
}

/**
  * @brief  生成从加速度寄存器开始的 7 字节控制数据。
  * @param  encoded_value: 已编码的位置或速度数值。
  * @param  speed: 位置模式下的速度；恒速模式下此值为零。
  * @param  acceleration: 加速度寄存器原始值。
  * @param  data: 输出的 7 字节数组。
  * @retval 无
  */
static void make_motion_data(uint16_t encoded_value, uint16_t speed,
                              uint8_t acceleration, uint8_t data[7])
{
    data[0] = acceleration;
    data[1] = (uint8_t)encoded_value;
    data[2] = (uint8_t)(encoded_value >> 8);
    data[3] = 0U;
    data[4] = 0U;
    data[5] = (uint8_t)speed;
    data[6] = (uint8_t)(speed >> 8);
}

/**
  * @brief  立即写入目标位置、速度和加速度。
  * @param  id: 单播 ID 或广播 ID。
  * @param  position: 目标位置原始值；负值按 bit15 符号格式编码。
  * @param  speed: 目标速度原始值。
  * @param  acceleration: 加速度原始值。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_write_position(uint8_t id, int16_t position,
                                      uint16_t speed, uint8_t acceleration,
                                      uint8_t *status)
{
    uint16_t encoded;
    FtServoResult result = encode_signed(position, &encoded);
    if (result != FT_SERVO_OK) {
        return result;
    }
    uint8_t data[7];
    make_motion_data(encoded, speed, acceleration, data);
    return ft_servo_write(id, FT_SMS_REG_ACC, data, sizeof(data), status);
}

/**
  * @brief  暂存目标位置、速度和加速度，等待 ACTION 执行。
  * @param  id: 单播 ID 或广播 ID。
  * @param  position: 目标位置原始值。
  * @param  speed: 目标速度原始值。
  * @param  acceleration: 加速度原始值。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_reg_write_position(uint8_t id, int16_t position,
                                          uint16_t speed, uint8_t acceleration,
                                          uint8_t *status)
{
    uint16_t encoded;
    FtServoResult result = encode_signed(position, &encoded);
    if (result != FT_SERVO_OK) {
        return result;
    }
    uint8_t data[7];
    make_motion_data(encoded, speed, acceleration, data);
    return ft_servo_reg_write(id, FT_SMS_REG_ACC, data, sizeof(data), status);
}

/**
  * @brief  在恒速模式下写入目标速度和加速度。
  * @param  id: 单播 ID 或广播 ID。
  * @param  speed: 速度原始值；负值按 bit15 符号格式编码。
  * @param  acceleration: 加速度原始值。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_write_speed(uint8_t id, int16_t speed,
                                   uint8_t acceleration, uint8_t *status)
{
    uint16_t encoded;
    FtServoResult result = encode_signed(speed, &encoded);
    if (result != FT_SERVO_OK) {
        return result;
    }
    uint8_t data[7];
    make_motion_data(0U, encoded, acceleration, data);
    return ft_servo_write(id, FT_SMS_REG_ACC, data, sizeof(data), status);
}

/**
  * @brief  暂存恒速模式的速度和加速度，等待 ACTION 执行。
  * @param  id: 单播 ID 或广播 ID。
  * @param  speed: 速度原始值；负值按 bit15 符号格式编码。
  * @param  acceleration: 加速度原始值。
  * @param  status: 非 NULL 时写入单播应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_reg_write_speed(uint8_t id, int16_t speed,
                                       uint8_t acceleration, uint8_t *status)
{
    uint16_t encoded;
    FtServoResult result = encode_signed(speed, &encoded);
    if (result != FT_SERVO_OK) {
        return result;
    }
    uint8_t data[7];
    make_motion_data(0U, encoded, acceleration, data);
    return ft_servo_reg_write(id, FT_SMS_REG_ACC, data, sizeof(data), status);
}

/**
  * @brief  在一帧广播指令中为多个舵机写入位置、速度和加速度。
  * @param  commands: 每个舵机的 ID 和目标参数数组。
  * @param  count: 指令数量，范围 1～31。
  * @retval 操作结果；广播无应答。
  */
FtServoResult ft_servo_sync_write_position(const FtServoPositionCommand *commands,
                                            uint8_t count)
{
    if (commands == NULL || count == 0U || count > 31U) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    uint8_t ids[31];
    uint8_t data[31U * 7U];
    for (uint8_t i = 0U; i < count; ++i) {
        uint16_t encoded;
        if (commands[i].id > 253U ||
            encode_signed(commands[i].position, &encoded) != FT_SERVO_OK) {
            return FT_SERVO_BAD_ARGUMENT;
        }
        ids[i] = commands[i].id;
        make_motion_data(encoded, commands[i].speed, commands[i].acceleration,
                         &data[(uint16_t)i * 7U]);
    }
    return ft_servo_sync_write(ids, count, FT_SMS_REG_ACC, data, 7U);
}

/**
  * @brief  在一帧广播指令中为多个舵机写入恒速模式参数。
  * @param  commands: 每个舵机的 ID、速度和加速度数组。
  * @param  count: 指令数量，范围 1～31。
  * @retval 操作结果；广播无应答。
  */
FtServoResult ft_servo_sync_write_speed(const FtServoSpeedCommand *commands,
                                         uint8_t count)
{
    if (commands == NULL || count == 0U || count > 31U) {
        return FT_SERVO_BAD_ARGUMENT;
    }
    uint8_t ids[31];
    uint8_t data[31U * 7U];
    for (uint8_t i = 0U; i < count; ++i) {
        uint16_t encoded;
        if (commands[i].id > 253U ||
            encode_signed(commands[i].speed, &encoded) != FT_SERVO_OK) {
            return FT_SERVO_BAD_ARGUMENT;
        }
        ids[i] = commands[i].id;
        make_motion_data(0U, encoded, commands[i].acceleration,
                         &data[(uint16_t)i * 7U]);
    }
    return ft_servo_sync_write(ids, count, FT_SMS_REG_ACC, data, 7U);
}

/**
  * @brief  读取当前速度寄存器的 16 位原始值。
  * @param  id: 单播舵机 ID。
  * @param  speed: 成功时写入原始速度值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_speed(uint8_t id, uint16_t *speed, uint8_t *status)
{
    return ft_servo_read_word(id, FT_SMS_REG_PRESENT_SPEED_L, speed, status);
}

/**
  * @brief  读取当前负载寄存器的 16 位原始值。
  * @param  id: 单播舵机 ID。
  * @param  load: 成功时写入原始负载值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_load(uint8_t id, uint16_t *load, uint8_t *status)
{
    return ft_servo_read_word(id, FT_SMS_REG_PRESENT_LOAD_L, load, status);
}

/**
  * @brief  读取当前电流寄存器的 16 位原始值。
  * @param  id: 单播舵机 ID。
  * @param  current: 成功时写入原始电流值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_current(uint8_t id, uint16_t *current, uint8_t *status)
{
    return ft_servo_read_word(id, FT_SMS_REG_PRESENT_CUR_L, current, status);
}

/**
  * @brief  读取当前电压寄存器的 8 位原始值。
  * @param  id: 单播舵机 ID。
  * @param  voltage: 成功时写入原始电压值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_voltage(uint8_t id, uint8_t *voltage, uint8_t *status)
{
    return ft_servo_read_byte(id, FT_SMS_REG_PRESENT_VOLTAGE, voltage, status);
}

/**
  * @brief  读取当前温度寄存器的 8 位原始值。
  * @param  id: 单播舵机 ID。
  * @param  temperature: 成功时写入原始温度值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_temperature(uint8_t id, uint8_t *temperature, uint8_t *status)
{
    return ft_servo_read_byte(id, FT_SMS_REG_PRESENT_TEMPERATURE, temperature, status);
}

/**
  * @brief  读取运动状态寄存器的 8 位原始值。
  * @param  id: 单播舵机 ID。
  * @param  moving: 成功时写入运动状态原始值。
  * @param  status: 非 NULL 时写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_moving(uint8_t id, uint8_t *moving, uint8_t *status)
{
    return ft_servo_read_byte(id, FT_SMS_REG_MOVING, moving, status);
}

/**
  * @brief  向指定 ID 发送 PING，并验证应答帧。
  * @param  id: 舵机 ID，范围 0～253。
  * @param  status: 成功时写入应答中的状态字节。
  * @retval FT_SERVO_OK、超时、收发错误、应答格式错误、校验错误或参数错误。
  */
FtServoResult ft_servo_ping(uint8_t id, uint8_t *status)
{
    if (id > 253U || status == NULL) {
        return FT_SERVO_BAD_ARGUMENT;
    }

    uint8_t tx[6] = {0xFFU, 0xFFU, id, 0x02U, FT_SCS_INST_PING, 0U};
    uint8_t rx[6];
    tx[5] = frame_checksum(&tx[2], 3U);
    ServoBusResult bus = servo_bus_exchange(tx, sizeof(tx), rx, sizeof(rx));
    if (bus != SERVO_BUS_OK) {
        return map_bus_result(bus);
    }
    return check_reply(id, rx, 0U, status);
}

/**
  * @brief  读取指定舵机当前位置寄存器的 16 位原始值。
  * @param  id: 舵机 ID，范围 0～253。
  * @param  position: 成功时写入原始位置值。
  * @param  status: 非 NULL 时在成功后写入应答状态字节。
  * @retval 操作结果。
  */
FtServoResult ft_servo_read_position(uint8_t id, uint16_t *position, uint8_t *status)
{
    return ft_servo_read_word(id, FT_SMS_REG_PRESENT_POS_L, position, status);
}
