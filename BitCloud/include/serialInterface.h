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
 * Serial interface abstraction layer (USART / VCP).
 */

#ifndef _SERIALINTERFACE_H
#define _SERIALINTERFACE_H

/******************************************************************************
                    Includes section
******************************************************************************/
#include <usart.h>

/******************************************************************************
                    Defines section
******************************************************************************/
#define APP_INTERFACE_USART 0x01
#define APP_INTERFACE_VCP   0x02

#if APP_INTERFACE == APP_INTERFACE_VCP
  #include <vcpVirtualUsart.h>
#endif

/* Map generic USART macros to the selected interface implementation */
#if APP_INTERFACE == APP_INTERFACE_USART
  #define OPEN_USART    HAL_OpenUsart
  #define CLOSE_USART   HAL_CloseUsart
  #define WRITE_USART   HAL_WriteUsart
  #define READ_USART    HAL_ReadUsart
  #define USART_CHANNEL APP_USART_CHANNEL
#endif

#if APP_INTERFACE == APP_INTERFACE_VCP
  #define OPEN_USART    VCP_OpenUsart
  #define CLOSE_USART   VCP_CloseUsart
  #define WRITE_USART   VCP_WriteUsart
  #define READ_USART    VCP_ReadUsart
  #define USART_CHANNEL USART_CHANNEL_VCP
#endif

#ifndef OPEN_USART
  #error USART interface is not defined.
#endif

#endif /* _SERIALINTERFACE_H */

/* eof serialInterface.h */
