/*
 * Copyright (c) 2011 Ambuj Varshney <ambuj_varshney@daiict.ac.in>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * TigerCENSE - Wireless Camera Sensor Node
 * BitCloud stack and application configuration parameters.
 */

#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include <BoardConfig.h>

/******************************************************************************
                    Interface selection
******************************************************************************/
#define APP_INTERFACE_USART 0x01
#define APP_INTERFACE_VCP   0x02
#define APP_INTERFACE_SPI   0x03
#define APP_INTERFACE_UART  0x04

/* APS fragmentation support (0 = disabled, 1 = enabled) */
#define APP_FRAGMENTATION        0

/* Link failure detection (0 = disabled, 1 = enabled) */
#define APP_DETECT_LINK_FAILURE  0

/* Primary serial interface type */
#define APP_INTERFACE            APP_INTERFACE_USART

/* USART channel for host communication */
#define APP_USART_CHANNEL        USART_CHANNEL_1

/******************************************************************************
                    Board-specific configuration
******************************************************************************/
#ifdef BOARD_RCB
  /* Enable TTL-to-RS232 converter control pin on RCB boards */
  #define BSP_ENABLE_RS232_CONTROL 1
#endif

/******************************************************************************
                    RF channel configuration
******************************************************************************/
#ifdef AT86RF212
  /*
   * 900 MHz band channel mask (sub-GHz radio).
   * Valid channels: 0x00 - 0x0a
   */
  #define CS_CHANNEL_MASK  (1L << 0x01)

  /*
   * Channel page:
   *   0 - 915MHz BPSK-40 (ch 1-10), 868MHz BPSK-20 (ch 0)
   *   2 - 915MHz O-QPSK-250 (ch 1-10), 868MHz O-QPSK-100 (ch 0)
   */
  #define CS_CHANNEL_PAGE  0
#else
  /*
   * 2.4 GHz band channel mask.
   * Valid channels: 0x0b - 0x1a
   */
  #define CS_CHANNEL_MASK  (1L << 0x0f)
#endif

/******************************************************************************
                    Network parameters
******************************************************************************/

/* Extended PAN ID */
#define CS_EXT_PANID                  0xAAAAAAAAAAAAAAAALL

/* 64-bit Unique Identifier (0 = read from hardware if available) */
#define CS_UID                        0x0LL

/* Short network address (used when CS_NWK_UNIQUE_ADDR is enabled) */
#define CS_NWK_ADDR                   0x0001

/* Maximum children per coordinator/router */
#define CS_MAX_CHILDREN_AMOUNT        4

/* Maximum router children */
#define CS_MAX_CHILDREN_ROUTER_AMOUNT 4

/* Neighbor table size */
#define CS_NEIB_TABLE_SIZE            5

/* Route table size */
#define CS_ROUTE_TABLE_SIZE           8

/* Network depth (max hops = depth * 2) */
#define CS_MAX_NETWORK_DEPTH          3

/******************************************************************************
                    Fragmentation parameters
******************************************************************************/
#if (APP_FRAGMENTATION == 1)
  #define CS_APS_MAX_BLOCKS_AMOUNT    4
  #define CS_APS_BLOCK_SIZE           0
#endif

/******************************************************************************
                    RF transmit power
******************************************************************************/
/*
 * TX power range:
 *   AT86RF230/231/230B: -17 to +3 dBm
 *   AT86RF212:          -11 to +11 dBm
 */
#define CS_RF_TX_POWER                3

/******************************************************************************
                    Security configuration
******************************************************************************/
#ifdef STANDARD_SECURITY_MODE
  #define CS_ZDO_SECURITY_STATUS          0
  #define CS_NETWORK_KEY                  {0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC}
  #define CS_APS_TRUST_CENTER_ADDRESS     0xAAAAAAAAAAAAAAAALL
  #define CS_APS_SECURITY_TIMEOUT_PERIOD  10000
  #define CS_APS_SECURITY_BUFFERS_AMOUNT  4
#endif

#endif /* _CONFIGURATION_H_ */
