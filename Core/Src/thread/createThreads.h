#include "extern.h"


void createThreads(void)
{
    baseThread = new pruThread(TIM1, TIM1_UP_TIM10_IRQn, PRU_BASEFREQ);
    //NVIC_SetVector(TIM1_UP_TIM10_IRQn, (uint32_t)TIM1_IRQHandler);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2);

    servoThread = new pruThread(TIM2, TIM2_IRQn , PRU_SERVOFREQ);
    //NVIC_SetVector(TIM2_IRQn , (uint32_t)TIM2_IRQHandler);
    NVIC_SetPriority(TIM2_IRQn , 3);
}
