#include <stdio.h>
#include <stdlib.h>

#include "init.h"
#include "misc.h"
#include <stm32f4xx_usart.h>
/*#include "FreeRTOS.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "actions.h"*/

//#include "define.h"

//#include <AUSBEE/encoder.h>

//extern xSemaphoreHandle USART1ReceiveHandle;

void init_usart1_interrupt()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void init_lidar()
{
  platform_usart_init(USART1, 115200);
  init_usart1_interrupt();


  /*USART1ReceiveHandle=xSemaphoreCreateMutex();
  if(USART1ReceiveHandle== NULL)
  {
    //platform_led_toggle(PLATFORM_LED6);
  }*/
}


