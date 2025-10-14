#ifndef CAN_TASK_HPP
#define CAN_TASK_HPP

#include "main.h"
#include <stdbool.h>

#include "HW_can.hpp"

#ifdef __cplusplus

struct CANCommData {
    uint32_t tick;
    float    value1;
    uint8_t  value2;
    bool     flag1;
    bool     flag2;
    bool     flag3;
    bool     flag4;
};

class CanTask {
public:
    CanTask();
    void init(CAN_HandleTypeDef* hcan);
    void run(uint32_t current_tick);

    void rx_message_callback();

private:
    CAN_HandleTypeDef* hcan_;
    uint32_t last_send_tick_;

    CANCommData data_tx_; 
    CANCommData data_rx_; 

    void encode(uint8_t* tx_data);
    void decode(const uint8_t* rx_data);
    void config_filter(); 
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

void can_task_init(void);
void can_task_run(uint32_t current_tick);

#ifdef __cplusplus
}
#endif

#endif // CAN_TASK_HPP