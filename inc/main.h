/**
  ******************************************************************************
  * @file    USB_Example/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"
#include "usb_istr.h"
#include "stm32f30x_it.h"
#include "usb_desc.h"


/* Exported variables ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define cgX 0		//Arrangements of the sensor coordinates in each array
#define cgY 1		//( used in packFoculusdataBlock() )
#define cgZ 2
#define caX 0
#define caY 1
#define caZ 2
#define cmX 0
#define cmY 2
#define cmZ 1
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void USB_Config (void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
