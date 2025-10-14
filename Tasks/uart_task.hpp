#ifndef UART_TASK_HPP
#define UART_TASK_HPP

#include "main.h" 

typedef struct {
    uint32_t tick;
    float value;
}UartCommData;

#ifdef __cplusplus

class UartTask {
public:
    UartTask();
    void init(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart);
    void run(uint32_t current_tick);
    void rx_event_callback(uint16_t size);
    UartCommData get_received_data() const;

private:
    static constexpr uint8_t TX_BUFFER_SIZE = 9;
    static constexpr uint8_t RX_BUFFER_SIZE = 64;
    static constexpr uint8_t  DECODE_BUFFER_SIZE = 9;

    enum class DecodeState {
        WAIT_HEADER_1,
        WAIT_HEADER_2,
        WAIT_HEADER_3,
        RECEIVING_DATA
    };

    UART_HandleTypeDef* tx_huart_;
    UART_HandleTypeDef* rx_huart_;
    UartCommData data_tx_;
    UartCommData data_rx_;
    uint8_t tx_buffer_[TX_BUFFER_SIZE];
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    DecodeState rx_state_;
    uint8_t decode_buffer_[DECODE_BUFFER_SIZE];
    uint8_t decode_count_;
    uint32_t last_send_tick_;

    void encode();
    void decode_byte(uint8_t byte);
    void reset_decoder();
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

void uart_task_init(void);
void uart_task_run(uint32_t current_tick);
void UartTask_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
UartCommData uart_task_get_received_data(void);

#ifdef __cplusplus
}
#endif

#endif // UART_TASK_HPP