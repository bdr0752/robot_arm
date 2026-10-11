#pragma once

#include "usart.h"

namespace BSP {
    // 中断中调用，数据只在回调期间有效；data 为 0 表示接收错误。
    typedef void (*UartCallback)(const uint8_t *data, uint16_t size);

    template <typename T>
    class UART {
    public:
        /**
         * @brief 启动普通模式 DMA 空闲接收。
         * @retval 无
         */
        static void Init() {
            HAL_UARTEx_ReceiveToIdle_DMA(T::Handle(), T::rxbuf, T::RXBUF_SIZE);
            __HAL_DMA_DISABLE_IT(T::Handle()->hdmarx, DMA_IT_HT);
        }

        /**
         * @brief 停止当前 DMA 接收。
         * @retval 无
         */
        static void Stop() { HAL_UART_AbortReceive(T::Handle()); }

        /**
         * @brief 阻塞发送，不返回 HAL 状态。
         * @param data 有效数据地址。
         * @param size 数据字节数。
         * @param timeout_ms HAL 等待超时，单位毫秒。
         * @retval 无
         */
        static void Transmit(const uint8_t *data, uint16_t size, uint32_t timeout_ms = 100U) {
            HAL_UART_Transmit(T::Handle(), const_cast<uint8_t *>(data), size, timeout_ms);
        }

        /**
         * @brief 注册唯一接收回调，0 表示取消。
         * @param callback 接收函数。
         * @retval 无
         */
        static void RegisterCallback(UartCallback callback) { T::callback = callback; }

        /**
         * @brief 交付数据后重新接收，仅由 HAL 回调调用。
         * @param size 实际长度。
         * @retval 无
         */
        static void OnReceive(uint16_t size) {
            if (T::callback) T::callback(T::rxbuf, size);
            Init();
        }

        /**
         * @brief 通知接收错误并恢复接收，仅由 HAL 回调调用。
         * @retval 无
         */
        static void OnError() {
            Stop();
            if (T::callback) T::callback(0, 0);
            Init();
        }
    };

    class UART1 : public UART<UART1> {
        friend class UART<UART1>;
    private:
        enum { RXBUF_SIZE = 128 };
        /**
         * @brief 获取本板 USART1 的 HAL 句柄。
         * @retval 已由 CubeMX 初始化的句柄地址。
         */
        static UART_HandleTypeDef *Handle() { return &huart1; }
        static uint8_t rxbuf[RXBUF_SIZE];
        static UartCallback callback;
    };

    class UART3 : public UART<UART3> {
        friend class UART<UART3>;
    private:
        enum { RXBUF_SIZE = 32 };
        /**
         * @brief 获取本板 USART3 的 HAL 句柄。
         * @retval 已由 CubeMX 初始化的句柄地址。
         */
        static UART_HandleTypeDef *Handle() { return &huart3; }
        static uint8_t rxbuf[RXBUF_SIZE];
        static UartCallback callback;
    };
}
