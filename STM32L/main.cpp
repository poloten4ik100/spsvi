#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_i2c.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define SERVO_180 5243
#define SERVO_0 1050

char buf[32]={0};
int counter=0;

// ������� �������� ������� ����� USART
void Usart_Transmit(uint8_t data)
{
  while(!(USART1->SR & USART_SR_TC)); //���� ��������� ����� TC - ���������� ��������
  USART1->DR = data;
}

// ������� �������� ������ ����� USART
void Usart_Transmit_str(char* str)
{
  uint8_t i=0;
  while(str[i])
  {
    Usart_Transmit(str[i]);
    i++;
  }
}

void set_pos(uint8_t pos) 
{
  uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;
  TIM2->CCR2 = SERVO_0 + tmp * pos;
 }

GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;

void InitUSART(void)
{
  // ������������� �������: PA9 - USART1_TX, PA10 - USART1_RX
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); //�������� ������������ GPIO�
  // ��� ������� PA9, PA10 �������� �������������� ������� ������ � USART1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  // ������������� ������ PA10 - USART1_Rx �� ��������� �� Input floating
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; //��������� ������ PA10
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //����� �������������� �������
  GPIO_Init(GPIOA, &GPIO_InitStruct); //�������� ��������� ��������� � ��������� GPIO�
  // ������������� ������ PA9 - USART1_Tx
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; //��������� ������ PA9
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //����� �������������� �������
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; //����� Push-Pull
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; //����� ��� ������������� ����������
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz; //�������� ����� ������������
  GPIO_Init(GPIOA, &GPIO_InitStruct); //�������� ��������� ��������� � ��������� GPIO�
  // ������������� USART1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //�������� ������������ USART1
  USART_InitStruct.USART_BaudRate = 9600; //�������� ������ 9600 ���
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; //����� ����� 8 ���
  USART_InitStruct.USART_StopBits = USART_StopBits_1; //1 ����-���
  USART_InitStruct.USART_Parity = USART_Parity_No ; //��� �������� ��������
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��� ����������� ��������
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //������� ���������� � �������� USART1
  USART_Init(USART1, &USART_InitStruct); //�������� ��������� ��������� � ��������� USART1
  USART_Cmd(USART1, ENABLE); //�������� USART1
}

void SERVO(void)
  {
    /* 
      ����������� �������� ������� ���������� �� HSI 
      HSI oscillator clock � ���������� ��������������� RC ���������
      � �������� 16 ��� 
    */
    /*
    RCC->CR |= RCC_CR_HSION; //�������� �������� ��������� HSI
    while(!(RCC_CR_HSIRDY)); //���� ��� ������������
    RCC->CFGR |= RCC_CFGR_SW_HSI; //�������� ���������� �������� ������� SYSCLK ��������� HSI
    RCC->CR &= ~RCC_CR_MSION; //��������� ��������� MSI.
    */
    
    /*������������� GPIOA. ����� PA1 ������������� ��� ������ � ������� TIM2_CH2*/
    RCC->AHBENR |=RCC_AHBENR_GPIOAEN;//������������ GPIOA
        
    GPIOA->MODER |= GPIO_MODER_MODER1_1;//PA1 - output AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1;//PA1 - Push-Pull
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;//PA1 - Nopull
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;//PA1 - 40MHz
    
    GPIOA->MODER |= GPIO_MODER_MODER2_1;  //Alternate function mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_2;  //Output push-pull
    GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR2;  //40 MHz
    GPIOA->PUPDR &=~GPIO_PUPDR_PUPDR2;  //No pull-up, pull-down
    
    GPIOA->AFR[0] = 0x00000110;//PA1 - AFIO1 (TIM2_CH2 � TIM2_CH3)

    /*������������� ������� TIM2
    ��� ������������ ��� ������������ ����� �������/��������� 2 (TIM2_CH2)*/
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//������������ TIM2
    //TIM2->PSC = 0xA3D4;//��������� ������������ �������
    TIM2->CR1 |= TIM_CR1_ARPE;//������� ����� ��������������� ������ �������� ����������������
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;//������� ����� ��������������� �������� �������� ���������
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 );//OC2M = 110 - PWM mode 1
    TIM2->ARR = 41939;//������ ��������� ������� T = 20mS (���� 20ms = 50��)
    /*
    1048,5 - 90 �������� 1200 ������ =/
    2097 - 45 ��������
    3145 - 0 ��������
    4193 - -45 ��������
    5242,5 - -90 ��������
    */
    TIM2->CCR2 = 3145;//������������ �������� (� ������ ������ Duty cycle = 80%)
    TIM2->CCR3 = 2097;
    //TIM2->CCER |= TIM_CCER_CC2P;//���������� ��������� �������
    TIM2->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3E);//����� ������ �������/��������� �������
    TIM2->CR1 |= TIM_CR1_CEN;//����� ����� �������
  }

int main()
{
  InitUSART(); 
  Usart_Transmit_str("START\r\n");
  SERVO();
  while(1)
  {
    
  }
  return 0;
}
