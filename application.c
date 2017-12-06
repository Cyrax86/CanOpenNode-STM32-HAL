/*
 * Header for application interface for CANopenNode stack.
 *
 * @file        application.c
 * @ingroup     application
 * @version     SVN: \$Id: application.c 31 2013-03-08 17:57:40Z jani22 $
 * @author      Janez Paternoster
 * @copyright   2012 - 2013 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <http://canopennode.sourceforge.net>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "CANopen.h"
#include "ds401.h"

#include "wwdg.h"
#include "can.h"

extern uint8_t digital_input[2];
extern uint8_t digital_output[2];

extern uint32_t can_error;
	

/*******************************************************************************/
void programStart(void)
{
	/* Verify, if OD structures have proper alignment of initial values */
    if (CO_OD_RAM.FirstWord != CO_OD_RAM.LastWord)
        {
            while(1){}  
        }
    if (CO_OD_EEPROM.FirstWord != CO_OD_EEPROM.LastWord)
        {
            while(1){}
        }
    if (CO_OD_ROM.FirstWord != CO_OD_ROM.LastWord)
        {
            while(1){}
        }
		/* increase variable each startup. Variable is stored in eeprom. */
		OD_powerOnCounter++;
}


/*******************************************************************************/
void communicationReset(void)
{
	CO_ReturnError_t err;
	
  /* initialize CAN interface and CANopen */
  do
	{
		err = CO_init(0/* CAN module address */, OD_CANNodeID/* NodeID */, OD_CANBitRate /* bit rate */);
			
		if(err != CO_ERROR_NO)
		{
			CO_errorReport(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, CO_EMC_SOFTWARE_INTERNAL, err);
		}
		else
		{
			CO_errorReset(CO->em, CO_EM_MEMORY_ALLOCATION_ERROR, err);
		}
	}
	while (err!= CO_ERROR_NO);
}


/*******************************************************************************/
void programEnd(void){

}


/*******************************************************************************/
void programAsync(uint16_t timer1msDiff)
{	
	uint8_t i;
	bool_t error_em;
	static uint32_t can_tec = 0;
	static uint16_t CO_timer_10000ms = 0;
	static uint16_t CO_timer_100ms = 0;
	static uint16_t wwdg_timer = 0;
	
	if (LED_GREEN_RUN(CO->NMT) > 0)	HAL_GPIO_WritePin( LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET);
	else	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
	
	if (LED_RED_ERROR(CO->NMT)>0)	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
	else	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
	
	/* calculate cycle time for performance measurement */
	OD_performance[ODA_performance_mainCycleTime] = timer1msDiff;
	if(timer1msDiff > OD_performance[ODA_performance_mainCycleMaxTime])
	{
		OD_performance[ODA_performance_mainCycleMaxTime] = timer1msDiff;
	}
	
	wwdg_timer += timer1msDiff;
	if (wwdg_timer >= 33 )
	{
		HAL_WWDG_Refresh(&hwwdg);
		wwdg_timer = 0;
	}
	
	CO_timer_100ms += timer1msDiff;
	if (CO_timer_100ms >= 100 )
	{
		// check if CAN bus errors disappear and reset self for safety
		can_tec = hcan.Instance->ESR & CAN_ESR_TEC;
		if ((can_error != HAL_CAN_ERROR_NONE) && (!can_tec))
		{
			/* Application interface */
			programEnd();
			/* delete objects from memory */
			CO_delete(0/* CAN module address */);
			/* reset */
			NVIC_SystemReset();
		}
		
		/* read input ports*/
		digital_input = get_digital_inputs();
		digital_input_handler(CO, &digital_input[0], sizeof(digital_input), &timer1msDiff);
	
		/* write output ports*/
		digital_output_handler(CO, digital_output, sizeof(digital_output));
		set_digital_outputs((uint16_t*)digital_output);
		
		CO_timer_100ms = 0;
	}
	
	
	// Check if all monitored nodes != CO_NMT_OPERATIONAL every 10s
	// Reset self for safety and go to CO_NMT_PREOPERATIONAL state(default state after startup).
	CO_timer_10000ms += timer1msDiff;
	if (CO_timer_10000ms >= 10000 )
	{		
		CO_DISABLE_INTERRUPTS();
		error_em = CO_false;
		if (CO->HBcons->allMonitoredOperational != CO_NMT_OPERATIONAL) error_em = CO_true;				

		if(error_em)
		{
			/* Application interface */
			programEnd();

			/* delete objects from memory */
			CO_delete(0/* CAN module address */);

			/* reset */
			NVIC_SystemReset();
		}
		CO_timer_10000ms = 0;

		CO_ENABLE_INTERRUPTS();
	}
	
}


/*******************************************************************************/
void program1ms(void)
{
	
	if(CO->CANmodule[0]->CANnormal) 
	{
		bool_t syncWas;

		/* Process RPDO Sync and read inputs */
		syncWas = CO_process_SYNC_RPDO(CO, 1000);
	
		/* I/O or nonblocking application code may go here. ************************/
		
		/* I/O or nonblocking application code end. ************************/
			
		/*  Process TPDO */
		CO_process_TPDO(CO, syncWas, 1000);
	
		/* verify timer overflow */
		if(0) 
		{
			CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U);
		}
	}
}
