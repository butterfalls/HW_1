#include "can_task.hpp"
#include <cmath>
#include <cstring>

static CanTask g_can_task;

void can_task_init(void) {
    extern CAN_HandleTypeDef hcan;
    g_can_task.init(&hcan);
}

void can_task_run(uint32_t current_tick) {
    g_can_task.run(current_tick);
}

CANCommData can_task_get_received_data(void) {
    return g_can_task.get_received_data();
}

CANCommData CanTask::get_received_data() const {
    return data_rx_;
}

CanTask::CanTask() :
    hcan_(nullptr),
    last_send_tick_(0)
{
    memset(&data_tx_, 0, sizeof(CANCommData));
    memset(&data_rx_, 0, sizeof(CANCommData));
}

void CanTask::config_filter() {
    CAN_FilterTypeDef canfilter;
    canfilter.FilterBank = 0;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = (0x100 << 5);
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = (0x7FF << 5); 
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilter.FilterActivation = ENABLE;
    canfilter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan_, &canfilter) != HAL_OK) {
        Error_Handler();
    }
}

void CanTask::init(CAN_HandleTypeDef* hcan) {
    hcan_ = hcan;

    config_filter();

    if (HAL_CAN_Start(hcan_) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

void CanTask::run(uint32_t current_tick) {
    if (current_tick - last_send_tick_ >= 100) {
        last_send_tick_ = current_tick;

        data_tx_.tick = current_tick;
        data_tx_.value1 = sinf(static_cast<float>(current_tick) / 1000.0f);
        data_tx_.value2 = static_cast<uint8_t>(current_tick % 256);
        data_tx_.flag1 = (current_tick / 100) % 2 == 0;
        data_tx_.flag2 = (current_tick / 200) % 2 == 0;
        data_tx_.flag3 = (current_tick / 300) % 2 == 0;
        data_tx_.flag4 = (current_tick / 400) % 2 == 0;

        uint8_t tx_buffer[8];
        encode(tx_buffer);

        CAN_Send_Msg(hcan_, tx_buffer, 0x100, 
            8);
    }
}

void CanTask::rx_message_callback() {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (rx_header.StdId == 0x100 && rx_header.DLC == 8) {
            decode(rx_data);
        }
    }
}

void CanTask::encode(uint8_t* tx_data) {
    tx_data[0] = (data_tx_.tick >> 24) & 0xFF;
    tx_data[1] = (data_tx_.tick >> 16) & 0xFF;
    tx_data[2] = (data_tx_.tick >> 8) & 0xFF;
    tx_data[3] = data_tx_.tick & 0xFF;
    int16_t value1_encoded = static_cast<int16_t>(data_tx_.value1 * 30000.0f);
    tx_data[4] = (value1_encoded >> 8) & 0xFF;
    tx_data[5] = value1_encoded & 0xFF;
    tx_data[6] = data_tx_.value2;
    tx_data[7] = 0;
    if (data_tx_.flag1) tx_data[7] |= (1 << 0);
    if (data_tx_.flag2) tx_data[7] |= (1 << 1);
    if (data_tx_.flag3) tx_data[7] |= (1 << 2);
    if (data_tx_.flag4) tx_data[7] |= (1 << 3);
}

void CanTask::decode(const uint8_t* rx_data) {
    data_rx_.tick = (static_cast<uint32_t>(rx_data[0]) << 24) |
                    (static_cast<uint32_t>(rx_data[1]) << 16) |
                    (static_cast<uint32_t>(rx_data[2]) << 8)  |
                    (static_cast<uint32_t>(rx_data[3]));
    int16_t value1_encoded = (static_cast<int16_t>(rx_data[4]) << 8) | rx_data[5];
    data_rx_.value1 = static_cast<float>(value1_encoded) / 30000.0f;
    data_rx_.value2 = rx_data[6];
    data_rx_.flag1 = (rx_data[7] & (1 << 0));
    data_rx_.flag2 = (rx_data[7] & (1 << 1));
    data_rx_.flag3 = (rx_data[7] & (1 << 2));
    data_rx_.flag4 = (rx_data[7] & (1 << 3));
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    g_can_task.rx_message_callback();
}
