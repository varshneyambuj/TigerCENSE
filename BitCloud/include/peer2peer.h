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
 * Application header with type definitions, constants, and macros.
 */

#ifndef _PEER2PEER_H
#define _PEER2PEER_H

#if TEST_NETWORK == 1
  #undef _SLIDERS_
  #undef _BUTTONS_
#endif

/******************************************************************************
                    Includes section
******************************************************************************/
#ifdef _SLIDERS_
  #include "sliders.h"
#endif

#ifdef _LEDS_
  #include "leds.h"
#endif

#ifdef _BUTTONS_
  #include "buttons.h"
#endif

/******************************************************************************
                    Defines section
******************************************************************************/

#ifndef APP_USART_CHANNEL
  #define APP_USART_CHANNEL               USART_CHANNEL_0
#endif

/* LED blink period (ms) during network joining */
#define APP_JOINING_INDICATION_PERIOD     500L

/* ZigBee endpoint, profile, and cluster IDs */
#define APP_ENDPOINT                      1
#define APP_PROFILE_ID                    1
#define APP_CLUSTER_ID                    1

/*
 * Data transfer tuning parameters:
 *
 * APP_TRANSMISSION_DELAY (ms)
 *   Delay between transmitted packets. Reduces collisions and increases
 *   throughput. Set to 0 to disable.
 *
 * APP_MAX_PACKET_SIZE (bytes)
 *   Maximum useful data per packet.
 *
 * APP_DELAY_BEFORE_SEND (ms)
 *   Delay between first UART byte received and actual air transmission.
 *   Higher values increase throughput but add latency. Set to 0 to disable.
 */
#define APP_TRANSMISSION_DELAY            20

#if APP_FRAGMENTATION
  #define APP_MAX_PACKET_SIZE             200
  #define APP_DELAY_BEFORE_SEND           3000
#else
  #define APP_MAX_PACKET_SIZE             60
  #define APP_DELAY_BEFORE_SEND           0
#endif

/* APS payload = user data + 1 byte message ID */
#define APP_APS_PAYLOAD_SIZE              (APP_MAX_PACKET_SIZE + 1)

/* FIFO buffer for data received from air (must be >= APP_MAX_PACKET_SIZE) */
#define APP_DATA_IND_BUFFER_SIZE          APP_APS_PAYLOAD_SIZE

/* USART buffer sizes */
#define APP_USART_RX_BUFFER_SIZE          6
#define APP_USART_TX_BUFFER_SIZE          100

/******************************************************************************
                    Compile-time sanity checks
******************************************************************************/
#if APP_DATA_IND_BUFFER_SIZE < APP_MAX_PACKET_SIZE
  #error APP_DATA_IND_BUFFER_SIZE must be >= APP_MAX_PACKET_SIZE
#endif

#if APP_FRAGMENTATION
  #ifndef _APS_FRAGMENTATION_
    #error BitCloud must be built with fragmentation feature enabled
  #endif
  #if APP_APS_PAYLOAD_SIZE > (CS_APS_MAX_BLOCKS_AMOUNT * APS_MAX_ASDU_SIZE)
    #error APP_MAX_PACKET_SIZE must be <= (CS_APS_MAX_BLOCKS_AMOUNT * APS_MAX_ASDU_SIZE)
  #endif
#else
  #if APP_APS_PAYLOAD_SIZE > APS_MAX_ASDU_SIZE
    #error APP_APS_PAYLOAD_SIZE must be <= APS_MAX_ASDU_SIZE
  #endif
#endif

/******************************************************************************
                    Type definitions
******************************************************************************/

/* Application state machine states */
typedef enum
{
    APP_INITIAL_STATE,                /* Power-on initialization */
    APP_SYNC_CAMERA,                  /* Synchronizing with C328 camera */
    APP_ECHO,                         /* USART echo test mode */
    APP_SET_BAUD,                     /* Baud rate change (unused) */
    APP_SET_BAUD1,                    /* Baud rate change step 2 (unused) */
    APP_CAMERA_INITIAL,               /* Setting camera packet size */
    APP_CAMERA_IMAGESIZE,             /* Requesting image from camera */
    APP_CAMERA_IMAGE,                 /* Taking snapshot */
    APP_CAMERA_GETIMAGE,              /* Configuring camera initial params */
    APP_CAMERA_RIMAGE,                /* Reading image data packets */
    APP_CAMERA_SEND_RIMAGE,           /* Sending image data over network */
    APP_NETWORK_JOINING_STATE,        /* Joining ZigBee network */
    APP_NETWORK_JOINED_STATE,         /* Successfully joined network */
    APP_NETWORK_LEAVING_STATE,        /* Leaving the network */
    APP_ERROR_STATE,                  /* Runtime error occurred */
    APP_STALL                         /* Image transfer complete, idle */
} AppState_t;

/* Network data transmission states */
typedef enum
{
    APP_DATA_TRANSMISSION_SENDING_STATE,  /* Ready to submit APS request */
    APP_DATA_TRANSMISSION_BUSY_STATE,     /* Waiting for APS confirm */
    APP_DATA_TRANSMISSION_WAIT_STATE,     /* Waiting for USART data */
    APP_DATA_TRANSMISSION_READY_STATE,    /* Ready for new transmission */
    APP_DATA_TRANSMISSION_STOP_STATE      /* Inter-frame delay active */
} AppDataTransmissionState_t;

/* Application network message (over-the-air format) */
BEGIN_PACK
typedef struct PACK
{
    uint8_t messageId;                    /* Message sequence ID */
    uint8_t data[APP_MAX_PACKET_SIZE];    /* Payload data */
} PACK AppMessage_t;

/* Message buffer with stack-required header/footer space */
typedef struct PACK
{
    uint8_t header[APS_ASDU_OFFSET];                    /* Stack header */
    AppMessage_t message;                               /* Application data */
    uint8_t footer[APS_AFFIX_LENGTH - APS_ASDU_OFFSET]; /* Stack footer */
} PACK AppMessageBuffer_t;
END_PACK

/******************************************************************************
                    LED macros
******************************************************************************/
#ifdef _LEDS_
  #define APP_NETWORK_STATUS_LED          LED_GREEN
  #define APP_RECEIVING_STATUS_LED        LED_YELLOW
  #define APP_SENDING_STATUS_LED          LED_RED
#endif

#ifdef _LEDS_
  #define appOpenLeds()      BSP_OpenLeds()
  #define appCloseLeds()     BSP_CloseLeds()
  #define appOnLed(id)       BSP_OnLed(id)
  #define appOffLed(id)      BSP_OffLed(id)
  #define appToggleLed(id)   BSP_ToggleLed(id)
#else
  #define appOpenLeds()
  #define appCloseLeds()
  #define appOnLed(id)
  #define appOffLed(id)
  #define appToggleLed(id)
#endif

#endif /* _PEER2PEER_H */

/* eof peer2peer.h */
