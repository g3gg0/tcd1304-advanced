
#include <string.h>

#include "main.h"

extern __IO uint32_t SH_period;
extern __IO uint32_t ICG_period;
extern __IO uint16_t aTxBuffer[CCDSize];
extern __IO uint8_t aRxBuffer[RxDataSize];
extern __IO uint8_t nRxBuffer[RxDataSize];

extern __IO uint8_t change_exposure_flag;
extern __IO uint8_t data_flag;
extern __IO uint8_t pulse_counter;
extern __IO uint8_t CCD_flushed;
extern __IO uint8_t avg_exps;
extern __IO uint8_t exps_left;
extern __IO uint8_t coll_mode;

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/

/*	This interrupt is invoked when UART-reception is completed */
void DMA1_Stream5_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
    {
        /* Clear DMA Stream Transfer Complete interrupt */
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

        /* Wait a short while - to let DMA finish ..or something */
        for (int i = 0; i < 8400; i++)
        {
        }

        /* Sort aRxBuffer into nRxBuffer */
        sort_aRxBuffer();

        /* Check the key before doing anything */
        if ((nRxBuffer[0] == 69) && (nRxBuffer[1] == 82))
        {
            /* reset the key */
            nRxBuffer[0] = 0;
            nRxBuffer[1] = 0;

            if (nRxBuffer[10] == 2)
            {
                extern __IO uint32_t trigger_delay;
                extern __IO uint8_t trigger_level;

                trigger_delay = (nRxBuffer[4] << 8) | nRxBuffer[5];

                if(nRxBuffer[6] == 0xFF)
                {
                    UART2_SendData("[g3gg0.de]", 10);
                }
                else
                {
                    trigger_level = nRxBuffer[6];
                }
            }
            else
            {
                /* set flags for main-loop */
                change_exposure_flag = 1;
                data_flag = 0;

                /* disable averaging by default */
                avg_exps = 1;

                /* continous or one-shot mode? */
                coll_mode = nRxBuffer[10];

                /* check if user averaging-request is valid */
                if ((nRxBuffer[11] < 16) && (nRxBuffer[11] > 0))
                {
                    avg_exps = nRxBuffer[11];
                }

                exps_left = avg_exps;
            }
        }
    }
}

/* This interrupt is invoked when the ADC has finished reading the CCD.
   The function's two tasks is to:
        1) Disable TIM4
        2) Set the appropiate value for the data_flag:
            1: Send data back immediately after 1st integration
            2: Overwrite avgBuffer with the latest integration
            3: Add the latest integration to avgBuffer
            4: Calculate the averaged integration and transmit the result	*/
void DMA2_Stream0_IRQHandler(void)
{
    /* Test for DMA Stream Transfer Complete interrupt */
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
        /* Clear DMA Stream Transfer Complete interrupt pending bit */
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

        /* Stop TIM4 and thus the ADC */
        TIM4->CR1 &= (uint16_t)~TIM_CR1_CEN;

        /* Keep track of the number of integrations performed */
        /* Are we collecting just once? */
        if (avg_exps == 1)
        {
            /* Set the data_flag to transmit */
            data_flag = 1;
        }
        else if (avg_exps > 1)
        {
            /* Is this the first collection of several? */
            if (exps_left == avg_exps)
            {
                // exps_left--;
                /* Set the pulse counter to 6 to start the ADC again
                   at next ICG-pulse. */
                pulse_counter = 6;
                /* Set the data_flag to overwrite avgBuffer */
                data_flag = 2;
            }

            /* Is this a collection in the middle? */
            else if ((exps_left < avg_exps) && (exps_left > 1))
            {
                // exps_left--;
                /* Set the pulse counter to 6 to start the ADC again
                   at next ICG-pulse. */
                pulse_counter = 6;
                /* Set the data_flag to sum integrations */
                data_flag = 3;
            }

            /* Is this the last collection of several? */
            else if (exps_left == 1)
            {
                // exps_left--;
                /* Set the data_flag to average integrations and tx */
                data_flag = 4;
            }
            exps_left--;
        }
    }
}

/* 	This interrupt is invoked when the ICG-pulse is sent,
    ie. when the CCD starts to output.

    At the 3rd pulse the CCD has been flushed.
    At the 6th pulse the CCD has been collecting once
    for the desired integration time.

    The pulse counter is (re)set to 6 by the DMA2-TC interrupt
    if more than one integration has been requested.	 */
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update))
    {
        /* Clear TIM5 update interrupt */
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        if (pulse_counter == 6)
        {
            /* Restart TIM4 as this gets the ADC running again */
            TIM4->CR1 |= TIM_CR1_CEN;
        }
        else if (pulse_counter == 3)
        {
            CCD_flushed = 1;
        }

        pulse_counter++;

        /* prevent overflow */
        if (pulse_counter > 32)
            pulse_counter = 32;

        /* Flash the led to the beat of ICG */
        GPIOA->ODR ^= GPIO_Pin_5;
    }
}

void EXTI15_10_IRQHandler(void)
{
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
        trigger_fired();
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
}

