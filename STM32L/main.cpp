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

// Функция передачи символа через USART
void Usart_Transmit(uint8_t data)
{
  while(!(USART1->SR & USART_SR_TC)); //Ждем установки флага TC - завершения передачи
  USART1->DR = data;
}

// Функция передачи строки через USART
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
  // Инициализация выводов: PA9 - USART1_TX, PA10 - USART1_RX
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); //Включаем тактирование GPIOА
  // Для выводов PA9, PA10 выбираем альтернативную функцию работы с USART1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  // Инициализации вывода PA10 - USART1_Rx По умолчанию он Input floating
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; //Настройки вывода PA10
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //Режим альтернативной функции
  GPIO_Init(GPIOA, &GPIO_InitStruct); //Заданные настройки сохраняем в регистрах GPIOА
  // Инициализации вывода PA9 - USART1_Tx
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; //Настройки вывода PA9
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //Режим альтернативной функции
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; //Выход Push-Pull
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; //Выход без подтягивающих резисторов
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz; //Скорость порта максимальная
  GPIO_Init(GPIOA, &GPIO_InitStruct); //Заданные настройки сохраняем в регистрах GPIOА
  // Инициализация USART1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Включаем тактирование USART1
  USART_InitStruct.USART_BaudRate = 9600; //Скорость обмена 9600 бод
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; //Длина слова 8 бит
  USART_InitStruct.USART_StopBits = USART_StopBits_1; //1 стоп-бит
  USART_InitStruct.USART_Parity = USART_Parity_No ; //Без проверки четности
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Без аппаратного контроля
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Включен передатчик и приемник USART1
  USART_Init(USART1, &USART_InitStruct); //Заданные настройки сохраняем в регистрах USART1
  USART_Cmd(USART1, ENABLE); //Включаем USART1
}

void SERVO(void)
  {
    /* 
      Настраиваем такирвую частоту процессора от HSI 
      HSI oscillator clock – внутренний высокочастотный RC генератор
      с частотой 16 МГц 
    */
    /*
    RCC->CR |= RCC_CR_HSION; //Включаем тактовый генератор HSI
    while(!(RCC_CR_HSIRDY)); //Ждем его стабилизации
    RCC->CFGR |= RCC_CFGR_SW_HSI; //Выбираем источником тактовой частоты SYSCLK генератор HSI
    RCC->CR &= ~RCC_CR_MSION; //Отключаем генератор MSI.
    */
    
    /*Инициализация GPIOA. Вывод PA1 настраивается для работы с выходом TIM2_CH2*/
    RCC->AHBENR |=RCC_AHBENR_GPIOAEN;//Тактирование GPIOA
        
    GPIOA->MODER |= GPIO_MODER_MODER1_1;//PA1 - output AF
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1;//PA1 - Push-Pull
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;//PA1 - Nopull
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;//PA1 - 40MHz
    
    GPIOA->MODER |= GPIO_MODER_MODER2_1;  //Alternate function mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_2;  //Output push-pull
    GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR2;  //40 MHz
    GPIOA->PUPDR &=~GPIO_PUPDR_PUPDR2;  //No pull-up, pull-down
    
    GPIOA->AFR[0] = 0x00000110;//PA1 - AFIO1 (TIM2_CH2 и TIM2_CH3)

    /*Инициализация таймера TIM2
    Для формирования ШИМ используется канал захвата/сравнения 2 (TIM2_CH2)*/
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//Тактирование TIM2
    //TIM2->PSC = 0xA3D4;//Настройка предделителя таймера
    TIM2->CR1 |= TIM_CR1_ARPE;//Включен режим предварительной записи регистра автоперезагрузки
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;//Включен режим предварительной загрузки регистра сравнения
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 );//OC2M = 110 - PWM mode 1
    TIM2->ARR = 41939;//Период выходного сигнала T = 20mS (надо 20ms = 50Гц)
    /*
    1048,5 - 90 градусов 1200 предел =/
    2097 - 45 градусов
    3145 - 0 градусов
    4193 - -45 градусов
    5242,5 - -90 градусов
    */
    TIM2->CCR2 = 3145;//Длительность импульса (в данном случае Duty cycle = 80%)
    TIM2->CCR3 = 2097;
    //TIM2->CCER |= TIM_CCER_CC2P;//Полярность выходного сигнала
    TIM2->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3E);//Выход канала захвата/сравнения включен
    TIM2->CR1 |= TIM_CR1_CEN;//Старт счета таймера
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
