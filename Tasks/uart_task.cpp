#include "uart_task.hpp"
#include <cmath> 
#include <cstring>

static UartTask g_uart_task;

UartTask::UartTask() :
    tx_huart_(nullptr),
    rx_huart_(nullptr),
    data_tx_({0, 0.0f}),
    data_rx_({0, 0.0f}),
    rx_state_(DecodeState::WAIT_HEADER_1),
    decode_count_(0),
    last_send_tick_(0)
{
    memset(tx_buffer_, 0, sizeof(tx_buffer_));
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    memset(decode_buffer_, 0, sizeof(decode_buffer_));
}

// 初始化函数
void UartTask::init(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart) {
    tx_huart_ = tx_huart;
    rx_huart_ = rx_huart;

    if (rx_huart_ != nullptr) {
        HAL_UARTEx_ReceiveToIdle_DMA(rx_huart_, rx_buffer_, RX_BUFFER_SIZE);
    }
}

void UartTask::run(uint32_t current_tick) {
    data_tx_.tick = current_tick;
    data_tx_.value = sinf(static_cast<float>(current_tick) / 1000.0f);

    if (current_tick - last_send_tick_ >= 100) {
        last_send_tick_ = current_tick;

        encode();

        if (tx_huart_ != nullptr) {
            HAL_UART_Transmit(tx_huart_, tx_buffer_, TX_BUFFER_SIZE, 50);
        }
    }
}

void UartTask::rx_event_callback(uint16_t size) {
    for (uint16_t i = 0; i < size; ++i) {
        decode_byte(rx_buffer_[i]);
    }
    
    if (rx_huart_ != nullptr) {
        HAL_UARTEx_ReceiveToIdle_DMA(rx_huart_, rx_buffer_, RX_BUFFER_SIZE);
    }
}

UartCommData UartTask::get_received_data() const {
    return data_rx_;
}

void UartTask::encode() {
    tx_buffer_[0] = 0xAA;
    tx_buffer_[1] = 0xBB;
    tx_buffer_[2] = 0xCC;
    tx_buffer_[3] = (data_tx_.tick >> 24) & 0xFF;
    tx_buffer_[4] = (data_tx_.tick >> 16) & 0xFF;
    tx_buffer_[5] = (data_tx_.tick >> 8) & 0xFF;
    tx_buffer_[6] = data_tx_.tick & 0xFF;
    int16_t value_encoded = static_cast<int16_t>(data_tx_.value * 30000.0f);
    tx_buffer_[7] = (value_encoded >> 8) & 0xFF;
    tx_buffer_[8] = value_encoded & 0xFF;
}

void UartTask::reset_decoder() {
    rx_state_ = DecodeState::WAIT_HEADER_1;
    decode_count_ = 0;
}

void UartTask::decode_byte(uint8_t byte) {
    switch (rx_state_) {
        case DecodeState::WAIT_HEADER_1:
            if (byte == 0xAA) {
                decode_buffer_[0] = byte;
                rx_state_ = DecodeState::WAIT_HEADER_2;
            }
            break;
        case DecodeState::WAIT_HEADER_2:
            if (byte == 0xBB) {
                decode_buffer_[1] = byte;
                rx_state_ = DecodeState::WAIT_HEADER_3;
            } else {
                reset_decoder();
            }
            break;
        case DecodeState::WAIT_HEADER_3:
            if (byte == 0xCC) {
                decode_buffer_[2] = byte;
                decode_count_ = 3;
                rx_state_ = DecodeState::RECEIVING_DATA;
            } else {
                reset_decoder();
            }
            break;
        case DecodeState::RECEIVING_DATA:
            decode_buffer_[decode_count_++] = byte;
            if (decode_count_ >= DECODE_BUFFER_SIZE) {
                data_rx_.tick = (static_cast<uint32_t>(decode_buffer_[3]) << 24) |
                                (static_cast<uint32_t>(decode_buffer_[4]) << 16) |
                                (static_cast<uint32_t>(decode_buffer_[5]) << 8)  |
                                (static_cast<uint32_t>(decode_buffer_[6]));

                int16_t value_encoded = (static_cast<int16_t>(decode_buffer_[7]) << 8) | decode_buffer_[8];
                data_rx_.value = static_cast<float>(value_encoded) / 30000.0f;
                
                reset_decoder();
            }
            break;
    }
}

void uart_task_init(void) {
    extern UART_HandleTypeDef huart1;
    extern UART_HandleTypeDef huart2;
    g_uart_task.init(&huart1, &huart2);
}

void uart_task_run(uint32_t current_tick) {
    g_uart_task.run(current_tick);
}

void UartTask_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
         g_uart_task.rx_event_callback(Size);
    }
}

UartCommData uart_task_get_received_data(void) {
    return g_uart_task.get_received_data();
}