/* 
 * scout_can_parser.h
 * 
 * Created on: Aug 31, 2019 04:23
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef SCOUT_CAN_PARSER_H
#define SCOUT_CAN_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "scout_sdk/scout_protocol/scout_protocol.h"

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame
{
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8]__attribute__((aligned(8)));
};
#endif

bool DecodeScoutStatusMsgFromCAN(const struct can_frame *rx_frame, ScoutStatusMessage *msg);
bool DecodeScoutControlMsgFromCAN(const struct can_frame *rx_frame, ScoutControlMessage *msg);

void EncodeScoutStatusMsgToCAN(const ScoutStatusMessage *msg, struct can_frame *tx_frame);
void EncodeScoutControlMsgToCAN(const ScoutControlMessage *msg, struct can_frame *tx_frame);

void EncodeScoutMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame);
void EncodeScoutLightControlMsgToCAN(const LightControlMessage *msg, struct can_frame *tx_frame);

uint8_t CalcScoutCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* SCOUT_CAN_PARSER_H */
