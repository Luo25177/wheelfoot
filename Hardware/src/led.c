#include "led.h"

void LedInit() {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_DeInit(GPIOA);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  LED_BLUE_OFF;
  LED_GREEN_OFF;
  LED_YELLOW_OFF;
  LED_RED_OFF;
}

void LedShow() {
  LED_RED_ON;
  OSTimeDly(3000);
  LED_GREEN_OFF;
  OSTimeDly(3000);
  LED_YELLOW_ON;
  OSTimeDly(3000);
  LED_RED_OFF;
  OSTimeDly(3000);
  LED_BLUE_ON;
  OSTimeDly(3000);
  LED_YELLOW_OFF;
  OSTimeDly(3000);
  LED_GREEN_ON;
  OSTimeDly(3000);
  LED_BLUE_OFF;
}