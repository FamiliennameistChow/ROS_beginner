/* 
 * scout_uart_parser.h
 * 
 * Created on: Aug 14, 2019 12:01
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_UART_PARSER_H
#define SCOUT_UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "scout_sdk/scout_protocol/scout_protocol.h"

bool DecodeScoutStatusMsgFromUART(uint8_t c, ScoutStatusMessage *msg);
bool DecodeScoutControlMsgFromUART(uint8_t c, ScoutControlMessage *msg);

void EncodeScoutStatusMsgToUART(const ScoutStatusMessage *msg, uint8_t *buf, uint8_t *len);
void EncodeScoutControlMsgToUART(const ScoutControlMessage *msg, uint8_t *buf, uint8_t *len);

void EncodeMotionControlMsgToUART(const MotionControlMessage *msg, uint8_t *buf, uint8_t *len);
void EncodeLightControlMsgToUART(const LightControlMessage *msg, uint8_t *buf, uint8_t *len);

uint8_t CalcScoutUARTChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* SCOUT_UART_PARSER_H */
