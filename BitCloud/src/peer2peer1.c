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
 *
 * This firmware runs on an ATmega1281-based ZigBit module using the
 * Atmel BitCloud ZigBee stack. It interfaces with a C328 serial camera
 * module to capture JPEG images and transmit them over a ZigBee
 * wireless sensor network.
 *
 * Hardware:
 *   - ZigBit ATmega1281 module with AT86RF230 radio
 *   - C328 serial JPEG camera (connected via USART0 at 57600 baud)
 *   - Debug/host USART1 at 115200 baud
 */

/******************************************************************************
                    Includes section
******************************************************************************/
#include <types.h>
#include <configServer.h>
#include <appTimer.h>
#include <zdo.h>
#include <peer2peer.h>
#include <serialInterface.h>

/******************************************************************************
                    Defines section
******************************************************************************/

/* Set to 1 to enable debug messages over USART, 0 to disable */
#define DEBUG_MSG 1

/* USART transmit buffer size (bytes) */
#define APP_USART_TX_BUFFER_SIZE  600

/* Convenience macro for USART writes */
#define USART_Write HAL_WriteUsart

/*
 * C328 Camera Module - Color Type Constants
 *
 * GS2  - 2-bit greyscale
 * GS4  - 4-bit greyscale
 * GS8  - 8-bit greyscale
 * C12  - 12-bit colour
 * C16  - 16-bit colour
 * CJPEG - JPEG compressed colour
 */
#define GS2   0x01
#define GS4   0x02
#define GS8   0x03
#define C12   0x05
#define C16   0x06
#define CJPEG 0x07

/*
 * C328 Camera Module - Resolution Constants
 *
 * Preview resolutions:
 *   P8060  - 80 x 60 pixels
 *   P1612  - 160 x 120 pixels
 *
 * JPEG capture resolutions:
 *   J8064    - 80 x 64
 *   J160128  - 160 x 128
 *   J320240  - 320 x 240
 *   J640480  - 640 x 480
 */
#define P8060    0x01
#define P1612    0x03
#define J8064    0x01
#define J160128  0x03
#define J320240  0x05
#define J640480  0x07

/******************************************************************************
                    Static variables section
******************************************************************************/

/* --- Network state --- */
static uint16_t nwkAddr;                          /* Network address */
static AppMessageBuffer_t appMessageBuffer;       /* Application message buffer */
static uint8_t messageIdTx = 0;                   /* Transmitted message sequence ID */
static uint8_t messageIdRx = 0;                   /* Expected received message ID */
static uint16_t actualDataLength = 0;             /* Data length for network transmission */

/* --- APS data indication FIFO --- */
static uint8_t apsDataIndFifo[APP_DATA_IND_BUFFER_SIZE];
static uint16_t apsDataIndFifoStart = 0;
static uint16_t apsDataIndFifoEnd = 0;

/* --- Application state --- */
static AppDataTransmissionState_t appDataTransmissionState;
static ZDO_StartNetworkReq_t networkParams;
static APS_DataReq_t apsDataReq;
static AppState_t appState = APP_INITIAL_STATE;
static bool noIndications = true;

/* --- Endpoint parameters --- */
static SimpleDescriptor_t simpleDescriptor = {
    APP_ENDPOINT, APP_PROFILE_ID, 1, 1, 0, 0, NULL, 0, NULL
};
static APS_RegisterEndpointReq_t endpointParams;

/* --- Link failure detection --- */
#if APP_DETECT_LINK_FAILURE == 1
static uint8_t retryCounter = 0;
static ZDO_ZdpReq_t leaveReq;
#endif

/* --- Camera state flags --- */
static bool initialVGA = false;    /* Initial VGA setup command sent */
static bool synSent = false;       /* Sync command sent to camera */
static bool initialSent = false;   /* Packet size command sent */
static bool imageSent = false;     /* Image size command sent */
static bool takeImage = false;     /* Snapshot command sent */
static bool dummy = false;         /* Double-callback guard for snapshot response */
static bool sendImage = true;      /* Ready to read next image packet */

/* --- Image transfer tracking --- */
static unsigned int filesz = 0;        /* Total image file size (bytes) */
static int bytes_image_read = 0;       /* Bytes read in current packet */
static int no_pkts = 0;               /* Total image packets to receive */
static uint8_t sent_pkt = 0;          /* Packets received so far */

/* --- USART buffers and descriptors --- */
static bool usartTxBusy = false;
static uint8_t usartRxBuffer[APP_USART_RX_BUFFER_SIZE];   /* USART1 (debug) RX buffer */
static uint8_t usartRxBuffer1[512];                        /* USART0 (camera) RX buffer */
static uint8_t usartTxBuffer[APP_USART_TX_BUFFER_SIZE];    /* USART1 TX buffer */
static uint8_t usartTxBuffer1[APP_USART_TX_BUFFER_SIZE];   /* USART0 TX buffer */

static HAL_UsartDescriptor_t appUsartDescriptor;   /* USART1 - debug/host serial */
static HAL_UsartDescriptor_t appUsartDescriptor1;  /* USART0 - camera serial */

/* --- Timers --- */
static HAL_AppTimer_t delayTimer;

/* --- Camera image receive buffer --- */
static uint8_t rxImageBuffer[100];

/******************************************************************************
                    Forward declarations section
******************************************************************************/

/* Network functions */
static void initNetwork(void);
static void startNetwork(void);
static void networkSendData(bool newTransmission);
static void APS_DataIndication(APS_DataInd_t *dataInd);
static void APS_DataConf(APS_DataConf_t *confInfo);
static void ZDO_StartNetworkConf(ZDO_StartNetworkConf_t *confirmInfo);

/* Timer callbacks */
static void startBlinkTimer(void);
static void startingNetworkTimerFired(void);
static void delayTimerFired(void);

/* FIFO operations */
static uint16_t fifoFreeSpace(void);
static void fifoWriteData(uint8_t *data, uint16_t size);
static uint16_t fifoReadData(uint8_t *data, uint16_t size);

/* Serial / debug output */
static void initSerialInterface(void);
static void displayText(char *text);
static void displayNumber(unsigned int no);

/* C328 camera commands */
static void synCamera(void);
static void synCameraResponse(void);
static void setPacketCamera(void);
static void setCameraPacketResponse(void);
static void setImageCamera(void);
static void setImageCameraResponse(void);
static void initialSetup(void);
static void initialSetupResponse(void);
static void takeSnapshot(void);
static void snapshotResponse(void);
static void recvImage(void);
static void getimage_statechange(void);

#if APP_DETECT_LINK_FAILURE == 1
static void leaveNetwork(void);
static void zdpLeaveResp(ZDO_ZdpResp_t *zdpResp);
#endif

/******************************************************************************
                    Button handler
******************************************************************************/

/*
 * Called when a board button is pressed. Triggers camera sync sequence.
 */
static void buttonPressed(uint8_t pressedButtonId)
{
    if (pressedButtonId == BSP_KEY0 || pressedButtonId == BSP_KEY1)
    {
        appState = APP_SYNC_CAMERA;
        SYS_PostTask(APL_TASK_ID);
    }
}

/******************************************************************************
                    Main application task handler
******************************************************************************/

/*
 * APL_TaskHandler - Main state machine for the TigerCENSE application.
 *
 * States flow:
 *   INITIAL -> SYNC_CAMERA -> CAMERA_GETIMAGE -> CAMERA_INITIAL ->
 *   CAMERA_IMAGESIZE -> CAMERA_IMAGE -> CAMERA_RIMAGE -> STALL
 *
 * Network path (unused in standalone camera mode):
 *   INITIAL -> NETWORK_JOINING -> NETWORK_JOINED
 */
void APL_TaskHandler(void)
{
    unsigned char camera_ack[6] = {0xAA, 0x0E, 0x00, 0x00, 0x00, 0x00};

    switch (appState)
    {
        /*
         * APP_INITIAL_STATE
         * Initialize serial interfaces, open LEDs, register button handler.
         * Stays in this state until a button press triggers camera sync.
         */
        case APP_INITIAL_STATE:
            initSerialInterface();
            appOpenLeds();
            BSP_OpenButtons(buttonPressed, NULL);
            appState = APP_INITIAL_STATE;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_NETWORK_JOINED_STATE
         * Network has been joined. Currently a placeholder for future
         * wireless image transmission.
         */
        case APP_NETWORK_JOINED_STATE:
            appState = APP_NETWORK_JOINED_STATE;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_NETWORK_JOINING_STATE
         * Attempt to join the ZigBee network. Blink LED while joining.
         */
        case APP_NETWORK_JOINING_STATE:
            HAL_WriteUsart(&appUsartDescriptor,
                           "Joining State\r\n", strlen("Joining State\r\n"));
            startNetwork();
            startBlinkTimer();
            break;

        case APP_NETWORK_LEAVING_STATE:
            break;

        /*
         * APP_SYNC_CAMERA
         * Send sync command (0xAA 0x0D) to C328 camera and wait for
         * ACK + SYNC response. Retries until camera responds.
         */
        case APP_SYNC_CAMERA:
            if (!synSent)
            {
                synCamera();
                delayTimer.interval = 10;
                delayTimer.mode = TIMER_ONE_SHOT_MODE;
                delayTimer.callback = synCameraResponse;
                HAL_StartAppTimer(&delayTimer);
            }
            appState = APP_SYNC_CAMERA;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_CAMERA_GETIMAGE
         * Send INITIAL command (0xAA 0x01) to configure camera for
         * JPEG capture at 640x480.
         */
        case APP_CAMERA_GETIMAGE:
            if (!initialVGA)
            {
                initialSetup();
                delayTimer.interval = 10;
                delayTimer.mode = TIMER_ONE_SHOT_MODE;
                delayTimer.callback = initialSetupResponse;
                HAL_StartAppTimer(&delayTimer);
            }
            appState = APP_CAMERA_GETIMAGE;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_CAMERA_INITIAL
         * Set camera packet size (64 bytes per packet).
         */
        case APP_CAMERA_INITIAL:
            if (!initialSent)
            {
                setPacketCamera();
                delayTimer.interval = 10;
                delayTimer.mode = TIMER_ONE_SHOT_MODE;
                delayTimer.callback = setCameraPacketResponse;
                HAL_StartAppTimer(&delayTimer);
            }
            appState = APP_CAMERA_INITIAL;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_CAMERA_IMAGESIZE
         * Send GET PICTURE command (0xAA 0x05) to request image data.
         */
        case APP_CAMERA_IMAGESIZE:
            if (!imageSent)
            {
                setImageCamera();
                delayTimer.interval = 10;
                delayTimer.mode = TIMER_ONE_SHOT_MODE;
                delayTimer.callback = setImageCameraResponse;
                HAL_StartAppTimer(&delayTimer);
            }
            appState = APP_CAMERA_IMAGESIZE;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_CAMERA_IMAGE
         * Send SNAPSHOT command (0xAA 0x04) to capture a frame.
         */
        case APP_CAMERA_IMAGE:
            if (!takeImage)
            {
                takeSnapshot();
                delayTimer.interval = 800;
                delayTimer.mode = TIMER_ONE_SHOT_MODE;
                delayTimer.callback = snapshotResponse;
                HAL_StartAppTimer(&delayTimer);
            }
            appState = APP_CAMERA_IMAGE;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_CAMERA_RIMAGE
         * Read image data packets from camera, 64 bytes at a time.
         * Sends each packet's payload to the debug USART and ACKs
         * back to the camera to request the next packet.
         */
        case APP_CAMERA_RIMAGE:
            if (sent_pkt < no_pkts)
            {
                if (sendImage && (HAL_IsTxEmpty(&appUsartDescriptor1) == 1))
                {
                    bytes_image_read = HAL_ReadUsart(&appUsartDescriptor1,
                                                     rxImageBuffer, 64);

                    if (bytes_image_read == 64 || (sent_pkt == (no_pkts - 1)))
                    {
                        uint8_t pktsize;
                        uint16_t pktid;

                        /* Parse packet header: ID (2 bytes) + size (2 bytes) */
                        pktid = (rxImageBuffer[0]) | (rxImageBuffer[1] << 8);
                        pktsize = (rxImageBuffer[2]) | (rxImageBuffer[3] << 8);

                        /* ACK current packet, request next */
                        sent_pkt++;
                        camera_ack[4] = sent_pkt;
                        HAL_WriteUsart(&appUsartDescriptor1, camera_ack, 6);

                        /* Forward image payload to debug USART */
                        HAL_WriteUsart(&appUsartDescriptor,
                                       &rxImageBuffer[4], pktsize);

                        /* Schedule next read after a short delay */
                        sendImage = false;
                        delayTimer.interval = 50;
                        delayTimer.mode = TIMER_ONE_SHOT_MODE;
                        delayTimer.callback = recvImage;
                        HAL_StartAppTimer(&delayTimer);

                        appState = APP_CAMERA_RIMAGE;
                    }
                }
            }
            else
            {
                /* All packets received - send end-of-transfer ACK to camera */
                camera_ack[4] = 0xF0;
                camera_ack[5] = 0xF0;
                HAL_WriteUsart(&appUsartDescriptor1, camera_ack, 6);
                appState = APP_STALL;
            }
            SYS_PostTask(APL_TASK_ID);
            break;

        case APP_CAMERA_SEND_RIMAGE:
            /* Reserved for future wireless image transmission */
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_STALL
         * Image transfer complete. Stays here until reset.
         */
        case APP_STALL:
            appState = APP_STALL;
            SYS_PostTask(APL_TASK_ID);
            break;

        /*
         * APP_ECHO
         * Simple USART echo mode for testing: reads from USART1,
         * writes to USART0.
         */
        case APP_ECHO:
        {
            char ch;
            if (HAL_ReadUsart(&appUsartDescriptor, &ch, 1) > 0)
                HAL_WriteUsart(&appUsartDescriptor1, &ch, 1);
            appState = APP_ECHO;
            SYS_PostTask(APL_TASK_ID);
            break;
        }

        default:
            break;
    }
}

/******************************************************************************
                    Network functions
******************************************************************************/

/*
 * Initialize network parameters and determine device role
 * (coordinator or router) based on the assigned network address.
 */
static void initNetwork(void)
{
    DeviceType_t deviceType;

    nwkAddr = 1;

    if (0 == nwkAddr)
    {
#ifdef _SECURITY_
        {
            ExtAddr_t extAddr;
            CS_ReadParameter(CS_APS_TRUST_CENTER_ADDRESS_ID, &extAddr);
            CS_WriteParameter(CS_UID_ID, &extAddr);
        }
#endif
        deviceType = DEVICE_TYPE_COORDINATOR;
        displayText("COORDINATOR");
    }
    else
    {
        deviceType = DEVICE_TYPE_ROUTER;
    }

    CS_WriteParameter(CS_DEVICE_TYPE_ID, &deviceType);
    appState = APP_NETWORK_JOINING_STATE;
    SYS_PostTask(APL_TASK_ID);
}

/*
 * Start network join procedure at the ZDO layer.
 */
static void startNetwork(void)
{
    networkParams.ZDO_StartNetworkConf = ZDO_StartNetworkConf;
    ZDO_StartNetworkReq(&networkParams);
}

/*
 * Callback when network join attempt completes.
 * On success, registers the application endpoint for data exchange.
 */
void ZDO_StartNetworkConf(ZDO_StartNetworkConf_t *confirmInfo)
{
    HAL_StopAppTimer(&delayTimer);

    if (confirmInfo->status == ZDO_SUCCESS_STATUS)
    {
        appDataTransmissionState = APP_DATA_TRANSMISSION_READY_STATE;
        appState = APP_NETWORK_JOINED_STATE;

#if APP_DETECT_LINK_FAILURE == 1
        retryCounter = 0;
#endif
        actualDataLength = 0;
        appOnLed(APP_NETWORK_STATUS_LED);

        /* Register application endpoint for data indications */
        endpointParams.simpleDescriptor = &simpleDescriptor;
        endpointParams.APS_DataInd = APS_DataIndication;
        APS_RegisterEndpointReq(&endpointParams);
        displayText("APS-Configured");
        SYS_PostTask(APL_TASK_ID);
    }
    else
    {
        SYS_PostTask(APL_TASK_ID);
    }
}

/*
 * Send data over the ZigBee network to the paired peer node.
 * If newTransmission is true, sets up a fresh APS data request.
 * Otherwise retransmits the previous request.
 */
static void networkSendData(bool newTransmission)
{
    if (APP_DATA_TRANSMISSION_SENDING_STATE == appDataTransmissionState)
    {
        appDataTransmissionState = APP_DATA_TRANSMISSION_BUSY_STATE;
        appOnLed(APP_SENDING_STATUS_LED);

        if (newTransmission)
        {
            appMessageBuffer.message.messageId = messageIdTx++;
            apsDataReq.dstAddrMode = APS_SHORT_ADDRESS;
            apsDataReq.dstAddress.shortAddress = nwkAddr ^ 1;
            apsDataReq.profileId = simpleDescriptor.AppProfileId;
            apsDataReq.dstEndpoint = simpleDescriptor.endpoint;
            apsDataReq.clusterId = APP_CLUSTER_ID;
            apsDataReq.srcEndpoint = simpleDescriptor.endpoint;
            apsDataReq.asdu = (uint8_t *)&appMessageBuffer.message;
            apsDataReq.asduLength = actualDataLength +
                                    sizeof(appMessageBuffer.message.messageId);
            apsDataReq.txOptions.acknowledgedTransmission = 1;

#if APP_FRAGMENTATION
            apsDataReq.txOptions.fragmentationPermitted = 1;
#else
            apsDataReq.txOptions.fragmentationPermitted = 0;
#endif
            apsDataReq.radius = 0;
            apsDataReq.APS_DataConf = APS_DataConf;

            HAL_WriteUsart(&appUsartDescriptor,
                           "Sending Data\r\n", strlen("Sending Data\r\n"));
        }

        APS_DataReq(&apsDataReq);
    }
}

/*
 * APS data confirmation callback. Handles transmission success/failure
 * and advances the camera state machine on successful delivery.
 */
void APS_DataConf(APS_DataConf_t *confInfo)
{
    appOffLed(APP_SENDING_STATUS_LED);

    if (APS_SUCCESS_STATUS != confInfo->status)
    {
#if APP_DETECT_LINK_FAILURE == 1
        retryCounter++;
        if (MAX_RETRIES_BEFORE_REJOIN == retryCounter)
        {
            leaveNetwork();
        }
        else
#endif
        {
            appDataTransmissionState = APP_DATA_TRANSMISSION_SENDING_STATE;
            networkSendData(false);
        }
        return;
    }

    if (nwkAddr == 0)
        sendImage = true;

#if APP_DETECT_LINK_FAILURE == 1
    retryCounter = 0;
#endif
    actualDataLength = 0;

    /* Advance camera state after successful network send */
    if (nwkAddr != 0)
    {
        if (appState == APP_SYNC_CAMERA && synSent)
        {
            HAL_WriteUsart(&appUsartDescriptor,
                           "Syn Sent Confirmation\r\n",
                           strlen("Syn Sent Confirmation\r\n"));
            appState = APP_CAMERA_GETIMAGE;
        }
    }

#if APP_TRANSMISSION_DELAY > 0
    appDataTransmissionState = APP_DATA_TRANSMISSION_STOP_STATE;
    delayTimer.interval = APP_TRANSMISSION_DELAY;
    delayTimer.mode = TIMER_ONE_SHOT_MODE;
    delayTimer.callback = delayTimerFired;
    HAL_StartAppTimer(&delayTimer);
#else
    appDataTransmissionState = APP_DATA_TRANSMISSION_READY_STATE;
#endif
}

/*
 * APS data indication callback. Called when data is received from a
 * peer node. Writes received data to the debug USART.
 */
void APS_DataIndication(APS_DataInd_t *indData)
{
    AppMessage_t *appMessage = (AppMessage_t *)indData->asdu;
    appOnLed(APP_RECEIVING_STATUS_LED);

    if (appMessage->messageId == messageIdRx)
    {
        fifoWriteData(appMessage->data, indData->asduLength - 1);
        HAL_WriteUsart(&appUsartDescriptor,
                       appMessage->data, indData->asduLength - 1);

        if (fifoFreeSpace() < APP_APS_PAYLOAD_SIZE)
        {
            APS_StopEndpointIndication(APP_ENDPOINT);
            noIndications = true;
        }

        APS_ResumeEndpointIndication(APP_ENDPOINT);
        noIndications = false;
    }

    messageIdRx = appMessage->messageId + 1;
    appOffLed(APP_RECEIVING_STATUS_LED);
}

/******************************************************************************
                    Timer callbacks
******************************************************************************/

/*
 * Start the LED blink timer during network joining phase.
 */
static void startBlinkTimer(void)
{
    delayTimer.interval = APP_JOINING_INDICATION_PERIOD;
    delayTimer.mode = TIMER_REPEAT_MODE;
    delayTimer.callback = startingNetworkTimerFired;
    HAL_StartAppTimer(&delayTimer);
}

/*
 * Toggle the network status LED while joining.
 */
static void startingNetworkTimerFired(void)
{
    appToggleLed(APP_NETWORK_STATUS_LED);
}

/*
 * Inter-frame delay timer callback for network data transmission.
 */
static void delayTimerFired(void)
{
    if (APP_DATA_TRANSMISSION_STOP_STATE == appDataTransmissionState)
        appDataTransmissionState = APP_DATA_TRANSMISSION_READY_STATE;
    else
        appDataTransmissionState = APP_DATA_TRANSMISSION_SENDING_STATE;
}

/******************************************************************************
                    ZDO notification callbacks
******************************************************************************/

/*
 * Handle network management update notifications (start, loss, etc.).
 */
void ZDO_MgmtNwkUpdateNotf(ZDO_MgmtNwkUpdateNotf_t *nwkParams)
{
    switch (nwkParams->status)
    {
        case ZDO_NETWORK_STARTED_STATUS:
            break;

        case ZDO_NETWORK_LOST_STATUS:
        {
            APS_UnregisterEndpointReq_t unregEndpoint;
            unregEndpoint.endpoint = endpointParams.simpleDescriptor->endpoint;
            APS_UnregisterEndpointReq(&unregEndpoint);
            appOffLed(APP_NETWORK_STATUS_LED);
            appState = APP_NETWORK_JOINING_STATE;
            SYS_PostTask(APL_TASK_ID);
            break;
        }

        case ZDO_NWK_UPDATE_STATUS:
            break;

        default:
            break;
    }
}

void ZDO_WakeUpInd(void)
{
    /* Required by stack - nothing to do */
}

/******************************************************************************
                    FIFO operations
******************************************************************************/

/*
 * Return the number of free bytes in the APS data indication FIFO.
 */
static uint16_t fifoFreeSpace(void)
{
    uint16_t free = apsDataIndFifoStart - apsDataIndFifoEnd;
    if (apsDataIndFifoStart <= apsDataIndFifoEnd)
        free += sizeof(apsDataIndFifo);
    return free;
}

/*
 * Write data into the circular FIFO buffer.
 */
static void fifoWriteData(uint8_t *data, uint16_t size)
{
    uint16_t i;
    for (i = 0; i < size; i++)
    {
        apsDataIndFifo[apsDataIndFifoEnd++] = data[i];
        if (apsDataIndFifoEnd == sizeof(apsDataIndFifo))
            apsDataIndFifoEnd = 0;
    }
}

/*
 * Read data from the circular FIFO buffer. Returns bytes actually read.
 */
static uint16_t fifoReadData(uint8_t *data, uint16_t size)
{
    uint16_t read = 0;
    uint16_t i;

    for (i = 0; i < size; i++)
    {
        if (apsDataIndFifoStart == apsDataIndFifoEnd)
            break;
        data[i] = apsDataIndFifo[apsDataIndFifoStart++];
        read++;
        if (apsDataIndFifoStart == sizeof(apsDataIndFifo))
            apsDataIndFifoStart = 0;
    }

    return read;
}

/******************************************************************************
                    Debug output helpers
******************************************************************************/

/*
 * Print a text string followed by CRLF to the debug USART.
 */
static void displayText(char *text)
{
    char str[60];
    snprintf(str, sizeof(str), "%s\r\n", text);
    USART_Write(&appUsartDescriptor, (uint8_t *)str, strlen(str));
}

/*
 * Print an unsigned integer followed by CRLF to the debug USART.
 */
static void displayNumber(unsigned int no)
{
    char str[60];
    snprintf(str, sizeof(str), "%d\r\n", no);
    USART_Write(&appUsartDescriptor, (uint8_t *)str, strlen(str));
}

/******************************************************************************
                    C328 Camera command functions
******************************************************************************/

/*
 * Send SYNC command (0xAA 0x0D) to the C328 camera module.
 * The camera must be synchronized before any other commands.
 */
static void synCamera(void)
{
    unsigned char sync_command[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};

    if (HAL_IsTxEmpty(&appUsartDescriptor1) == 1)
    {
        int bytes_sent = USART_Write(&appUsartDescriptor1, sync_command, 6);
        if (bytes_sent != 6)
        {
            synSent = false;
            return;
        }
    }
    synSent = true;
}

/*
 * Handle sync response from camera. Expects ACK (0xAA 0x0E 0x0D)
 * followed by SYNC (0xAA 0x0D). Sends ACK back and advances to
 * CAMERA_GETIMAGE state on success.
 */
static void synCameraResponse(void)
{
    unsigned char camera_response[12];
    unsigned char camera_ack[6] = {0xAA, 0x0E, 0x0D, 0x00, 0x00, 0x00};

    HAL_StopAppTimer(&delayTimer);
    int bytes_read = HAL_ReadUsart(&appUsartDescriptor1, camera_response, 12);

    if (bytes_read < 4)
    {
        appState = APP_SYNC_CAMERA;
        synSent = false;
    }
    else
    {
        /* Check for ACK (bytes 0-2) and SYNC (bytes 6-8) */
        if (camera_response[0] == 0xAA && camera_response[1] == 0x0E &&
            camera_response[2] == 0x0D)
        {
            if (camera_response[6] == 0xAA && camera_response[7] == 0x0D &&
                camera_response[8] == 0x00)
            {
                /* Send ACK back to camera to complete handshake */
                HAL_WriteUsart(&appUsartDescriptor1, camera_ack, 6);

                if (DEBUG_MSG)
                    HAL_WriteUsart(&appUsartDescriptor,
                                   "SYNCDONE!\r\n", strlen("SYNCDONE!\r\n"));

                appState = APP_CAMERA_GETIMAGE;
                synSent = true;
                initialVGA = false;
                return;
            }
        }

        /* Sync failed - retry */
        appState = APP_SYNC_CAMERA;
        synSent = false;
    }
}

/*
 * Send INITIAL command (0xAA 0x01) to configure camera:
 * JPEG colour type, 640x480 resolution.
 */
static void initialSetup(void)
{
    unsigned char initial_setup[6] = {0xAA, 0x01, 0x00, 0x07, 0x07, 0x07};

    if (HAL_IsTxEmpty(&appUsartDescriptor1) == 1)
    {
        int bytes_sent = USART_Write(&appUsartDescriptor1, initial_setup, 6);
        if (bytes_sent != 6)
        {
            initialVGA = false;
            return;
        }
    }
    initialVGA = true;
}

/*
 * Handle response to INITIAL command. Reads ACK and advances to
 * CAMERA_INITIAL (packet size setup) state.
 */
static void initialSetupResponse(void)
{
    unsigned char camera_response[6];
    int bytes_read;

    bytes_read = HAL_WriteUsart(&appUsartDescriptor1, camera_response, 6);
    if (DEBUG_MSG)
        displayNumber(bytes_read);

    appState = APP_CAMERA_INITIAL;
    initialSent = false;
}

/*
 * Send SET PACKAGE SIZE command (0xAA 0x06) to set camera packet
 * size to 64 bytes per packet for image data transfer.
 */
static void setPacketCamera(void)
{
    unsigned char setPktSize[6] = {0xAA, 0x06, 0x08, 0x40, 0x00, 0x00};

    if (HAL_IsTxEmpty(&appUsartDescriptor1) == 1)
    {
        int bytes_sent = HAL_WriteUsart(&appUsartDescriptor1, setPktSize, 6);
        if (bytes_sent != 6)
        {
            initialSent = false;
            appState = APP_CAMERA_INITIAL;
            return;
        }
    }
    initialSent = true;
}

/*
 * Handle response to SET PACKAGE SIZE command. On success (ACK 0x06),
 * advances to CAMERA_IMAGESIZE state.
 */
static void setCameraPacketResponse(void)
{
    unsigned char camera_response[6];

    HAL_StopAppTimer(&delayTimer);
    int bytes_read = HAL_ReadUsart(&appUsartDescriptor1, camera_response, 6);

    if (bytes_read < 4)
    {
        appState = APP_CAMERA_INITIAL;
        initialSent = false;
    }
    else
    {
        if (camera_response[0] == 0xAA && camera_response[1] == 0x0E &&
            camera_response[2] == 0x06)
        {
            if (DEBUG_MSG)
                HAL_WriteUsart(&appUsartDescriptor,
                               "PACKETSET\r\n", strlen("PACKETSET\r\n"));
            appState = APP_CAMERA_IMAGESIZE;
            synSent = true;
            initialSent = true;
            imageSent = false;
            return;
        }
        else
        {
            appState = APP_CAMERA_INITIAL;
            initialSent = false;
        }
    }
}

/*
 * Send GET PICTURE command (0xAA 0x05) to request the captured image.
 */
static void setImageCamera(void)
{
    unsigned char image_command[6] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00};

    if (HAL_IsTxEmpty(&appUsartDescriptor1) == 1)
    {
        int bytes_sent = USART_Write(&appUsartDescriptor1, image_command, 6);
        if (bytes_sent != 6)
        {
            imageSent = false;
            appState = APP_CAMERA_IMAGE;
            return;
        }
    }
    imageSent = true;
}

/*
 * Handle response to GET PICTURE command. On success (ACK 0x05),
 * advances to CAMERA_IMAGE (snapshot) state.
 * On NAK (0x0F), retries from CAMERA_IMAGESIZE.
 */
static void setImageCameraResponse(void)
{
    unsigned char camera_response[6];

    HAL_StopAppTimer(&delayTimer);
    int bytes_read = HAL_ReadUsart(&appUsartDescriptor1, camera_response, 6);

    if (bytes_read < 4)
    {
        appState = APP_CAMERA_IMAGESIZE;
        imageSent = false;
    }
    else
    {
        if (camera_response[0] == 0xAA && camera_response[1] == 0x0E &&
            camera_response[2] == 0x05)
        {
            if (DEBUG_MSG)
                HAL_WriteUsart(&appUsartDescriptor,
                               "Initial Parameters Set\r\n",
                               strlen("Initial Parameters Set\r\n"));
            appState = APP_CAMERA_IMAGE;
            imageSent = true;
            takeImage = false;
            return;
        }
        else if (camera_response[0] == 0xAA && camera_response[1] == 0x0F &&
                 camera_response[2] == 0x00)
        {
            if (DEBUG_MSG)
                HAL_WriteUsart(&appUsartDescriptor,
                               "NAK\r\n", strlen("NAK\r\n"));
            appState = APP_CAMERA_IMAGESIZE;
            imageSent = false;
            initialSent = false;
            return;
        }
        else
        {
            if (DEBUG_MSG)
            {
                HAL_WriteUsart(&appUsartDescriptor,
                               "Failed to set initial Parameter\r\n",
                               strlen("Failed to set initial Parameter\r\n"));
                HAL_WriteUsart(&appUsartDescriptor, camera_response, 6);
            }
            appState = APP_CAMERA_IMAGESIZE;
            imageSent = false;
        }
    }
}

/*
 * Send SNAPSHOT command (0xAA 0x04 0x01) to capture a compressed frame.
 */
static void takeSnapshot(void)
{
    unsigned char snapshot_command[6] = {0xAA, 0x04, 0x01, 0x00, 0x00, 0x00};

    if (HAL_IsTxEmpty(&appUsartDescriptor1) == 1)
    {
        int bytes_sent = USART_Write(&appUsartDescriptor1, snapshot_command, 6);
        if (bytes_sent != 6)
        {
            takeImage = false;
            appState = APP_CAMERA_IMAGE;
            return;
        }
    }
    takeImage = true;
}

/*
 * Handle snapshot response. Camera sends ACK (0x0E 0x04) followed by
 * DATA response (0xAA 0x0A 0x01) containing the image file size.
 * Calculates number of packets and begins image data transfer.
 */
static void snapshotResponse(void)
{
    unsigned char camera_response[12];
    unsigned char camera_ack[6] = {0xAA, 0x0E, 0x00, 0x00, 0x00, 0x00};

    HAL_StopAppTimer(&delayTimer);

    /* First call is a delayed retry to let camera prepare */
    if (!dummy)
    {
        dummy = true;
        HAL_StartAppTimer(&delayTimer);
        return;
    }

    int bytes_read = HAL_ReadUsart(&appUsartDescriptor1, camera_response, 12);

    if (bytes_read < 6)
    {
        /* Not enough data - retry from image size state */
        appState = APP_CAMERA_IMAGESIZE;
        takeImage = false;
        imageSent = false;
        return;
    }

    /* Check for ACK (0xAA 0x0E 0x04) + DATA header (0xAA 0x0A 0x01) */
    if (camera_response[0] == 0xAA && camera_response[1] == 0x0E &&
        camera_response[2] == 0x04)
    {
        if (camera_response[6] == 0xAA && camera_response[7] == 0x0A &&
            camera_response[8] == 0x01)
        {
            /* Extract 3-byte file size from DATA response */
            filesz = camera_response[9] +
                     (camera_response[10] << 8) +
                     (camera_response[11] << 16);
            no_pkts = (filesz / 64) + 1;

            if (DEBUG_MSG)
                displayNumber(no_pkts);

            /* ACK the DATA response to begin packet transfer */
            HAL_WriteUsart(&appUsartDescriptor1, camera_ack, 6);

            /* Reset all flags for the transfer */
            takeImage = false;
            imageSent = false;
            initialSent = false;
            synSent = false;

            /* Begin reading image packets after a delay */
            appState = APP_CAMERA_RIMAGE;
            delayTimer.interval = 1800;
            delayTimer.mode = TIMER_ONE_SHOT_MODE;
            delayTimer.callback = getimage_statechange;
            HAL_StartAppTimer(&delayTimer);
            return;
        }
    }

    /* Unexpected response - restart from sync */
    takeImage = false;
    imageSent = false;
    initialSent = false;
    synSent = false;
    appState = APP_SYNC_CAMERA;
}

/*
 * Timer callback to begin reading image packets from camera.
 */
static void getimage_statechange(void)
{
    HAL_StopAppTimer(&delayTimer);
    HAL_WriteUsart(&appUsartDescriptor,
                   "INSTATECHANGE\r\n", strlen("INSTATECHANGE\r\n"));
    sendImage = true;
    SYS_PostTask(APL_TASK_ID);
}

/*
 * Timer callback that signals readiness to read the next image packet.
 */
static void recvImage(void)
{
    sendImage = true;
}

/******************************************************************************
                    Serial interface initialization
******************************************************************************/

/*
 * Initialize both USART channels:
 *   USART1 (appUsartDescriptor)  - 115200 baud, debug/host interface
 *   USART0 (appUsartDescriptor1) - 57600 baud, C328 camera interface
 */
static void initSerialInterface(void)
{
    /* USART1 - Debug/host serial (programming port) */
    usartTxBusy = false;
    appUsartDescriptor.tty            = USART_CHANNEL_1;
    appUsartDescriptor.mode           = USART_MODE_ASYNC;
    appUsartDescriptor.baudrate       = USART_BAUDRATE_115200;
    appUsartDescriptor.dataLength     = USART_DATA8;
    appUsartDescriptor.parity         = USART_PARITY_NONE;
    appUsartDescriptor.stopbits       = USART_STOPBIT_1;
    appUsartDescriptor.rxBuffer       = usartRxBuffer;
    appUsartDescriptor.rxBufferLength = sizeof(usartRxBuffer);
    appUsartDescriptor.txBuffer       = NULL;
    appUsartDescriptor.txBufferLength = 0;
    appUsartDescriptor.rxCallback     = NULL;
    appUsartDescriptor.txCallback     = NULL;
    appUsartDescriptor.flowControl    = USART_FLOW_CONTROL_NONE;
    OPEN_USART(&appUsartDescriptor);

    /* USART0 - Camera serial interface */
    usartTxBusy = false;
    appUsartDescriptor1.tty            = USART_CHANNEL_0;
    appUsartDescriptor1.mode           = USART_MODE_ASYNC;
    appUsartDescriptor1.baudrate       = USART_BAUDRATE_57600;
    appUsartDescriptor1.dataLength     = USART_DATA8;
    appUsartDescriptor1.parity         = USART_PARITY_NONE;
    appUsartDescriptor1.stopbits       = USART_STOPBIT_1;
    appUsartDescriptor1.rxBuffer       = usartRxBuffer1;
    appUsartDescriptor1.rxBufferLength = sizeof(usartRxBuffer1);
    appUsartDescriptor1.txBuffer       = usartTxBuffer1;
    appUsartDescriptor1.txBufferLength = sizeof(usartTxBuffer1);
    appUsartDescriptor1.rxCallback     = NULL;
    appUsartDescriptor1.txCallback     = NULL;
    appUsartDescriptor1.flowControl    = USART_FLOW_CONTROL_NONE;
    OPEN_USART(&appUsartDescriptor1);
}

/******************************************************************************
                    Stack-required callback stubs
******************************************************************************/

#ifdef _BINDING_
void ZDO_BindIndication(ZDO_BindInd_t *bindInd)
{
    (void)bindInd;
}

void ZDO_UnbindIndication(ZDO_UnbindInd_t *unbindInd)
{
    (void)unbindInd;
}
#endif

/******************************************************************************
                    Entry point
******************************************************************************/

int main(void)
{
    SYS_SysInit();

    for (;;)
    {
        SYS_RunTask();
    }
}

/* eof peer2peer1.c */
