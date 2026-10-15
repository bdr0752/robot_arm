#pragma once

#include "usart.h"
#include <string.h>

namespace BSP {
    // 中断中调用，数据只在回调期间有效；data 为 0 表示接收错误。
    typedef void (*UartCallback)(const uint8_t *data, uint16_t size);
    typedef void (*UartTxCallback)(void);

    template <typename T>
    class UART {
    public:
        /**
         * @brief 启动普通模式 DMA 空闲接收；已在接收时保持现有传输。
         * @retval HAL 接收启动结果。
         */
        static HAL_StatusTypeDef Init() {
            if (T::Handle()->RxState == HAL_UART_STATE_BUSY_RX) return HAL_OK;
            HAL_StatusTypeDef result = HAL_UARTEx_ReceiveToIdle_DMA(
                T::Handle(), T::rxbuf, T::RXBUF_SIZE);
            if (result == HAL_OK) __HAL_DMA_DISABLE_IT(T::Handle()->hdmarx, DMA_IT_HT);
            return result;
        }

        /**
         * @brief 停止当前 DMA 接收。
         * @retval 无
         */
        static void Stop() { HAL_UART_AbortReceive(T::Handle()); }

        /**
         * @brief 复制数据到内部缓冲区并启动非阻塞 DMA 发送。
         * @param data 有效数据地址。
         * @param size 数据字节数。
         * @retval true 表示启动成功；false 表示忙、长度无效或 HAL 启动失败。
         * @note 单个调用者使用；函数返回后原数据缓冲区可以修改或释放。
         */
        static bool Transmit(const uint8_t *data, uint16_t size) {
            if (T::tx_busy || size == 0U || size > T::TXBUF_SIZE) return false;
            T::tx_busy = true;
            memcpy(T::txbuf, data, size);
            if (HAL_UART_Transmit_DMA(T::Handle(), T::txbuf, size) == HAL_OK) return true;
            T::tx_busy = false;
            return false;
        }

        /**
         * @brief 查询 DMA 发送是否尚未完成。
         * @retval 发送忙标志。
         */
        static bool IsBusy() { return T::tx_busy; }

        /**
         * @brief 注册发送完成回调，在中断中调用。
         * @param callback 完成函数，0 表示取消。
         * @retval 无
         */
        static void RegisterTxCallback(UartTxCallback callback) { T::tx_callback = callback; }

        /**
         * @brief 释放发送状态并通知使用者，仅由 HAL 回调调用。
         * @retval 无
         */
        static void OnTransmit() {
            T::tx_busy = false;
            if (T::tx_callback) T::tx_callback();
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
         * @brief 通知 UART 错误、释放已终止的发送并恢复接收。
         * @retval 无
         */
        static void OnError() {
            // RX 错误不提前结束仍在进行的 TX；DMA TX 错误已由 HAL 结束发送。
            if (T::tx_busy && T::Handle()->gState != HAL_UART_STATE_BUSY_TX) OnTransmit();
            Stop();
            if (T::callback) T::callback(0, 0);
            Init();
        }
    };

    class UART1 : public UART<UART1> {
        friend class UART<UART1>;
    private:
        enum { RXBUF_SIZE = 128, TXBUF_SIZE = 256 };
        /**
         * @brief 获取本板 USART1 的 HAL 句柄。
         * @retval 已由 CubeMX 初始化的句柄地址。
         */
        static UART_HandleTypeDef *Handle() { return &huart1; }
        static uint8_t rxbuf[RXBUF_SIZE];
        static uint8_t txbuf[TXBUF_SIZE];
        static volatile bool tx_busy;
        static UartTxCallback tx_callback;
        static UartCallback callback;
    };

    class UART3 : public UART<UART3> {
        friend class UART<UART3>;
    private:
        enum { RXBUF_SIZE = 32, TXBUF_SIZE = 260 };
        /**
         * @brief 获取本板 USART3 的 HAL 句柄。
         * @retval 已由 CubeMX 初始化的句柄地址。
         */
        static UART_HandleTypeDef *Handle() { return &huart3; }
        static uint8_t rxbuf[RXBUF_SIZE];
        static uint8_t txbuf[TXBUF_SIZE];
        static volatile bool tx_busy;
        static UartTxCallback tx_callback;
        static UartCallback callback;
    };
}
