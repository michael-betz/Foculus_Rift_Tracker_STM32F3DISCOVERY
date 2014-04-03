/**
 ******************************************************************************
 * @file    USB_Example/usb_endp.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    20-September-2012
 * @brief   Endpoint routines
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

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_istr.h"
#include "hw_config.h"
#include "globalVariables.h"

/**
 * @brief  EP1 OUT Callback Routine.
 * @param  None
 * @retval None
 */
void EP1_IN_Callback(void) {
	/* Set the transfer complete token to inform upper layer that the current
	 transfer has been complete */
//	STM_EVAL_LEDToggle(LED10);
	EP1flagTXcomplete = 1;
}

void EP1_OUT_Callback(void) {
//	STM_EVAL_LEDToggle(LED8);
	EP1flagRXcomplete = 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

