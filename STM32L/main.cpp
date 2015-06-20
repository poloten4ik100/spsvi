#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_i2c.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define SERVO_180 40000
#define SERVO_0 8000

int j=1,l=0;

double servo_angle_pitch = 90;
double pitch_angle = 0;
double servo_angle_roll = 90;
double roll_angle = 0;
double servo_angle_yaw = 90;
double yaw_angle = 0;
int tim_angle_pitch = 3000;
int tim_angle_roll = 3000;
int tim_angle_yaw = 3000;
/*
I2C адресс L3G4200D
HEX: 0x69
int: 105
110100xb, x=1 => SDO лог. 1 => LSB = 1
*/
uint8_t L3G4200D_Address = 0x69;
uint8_t L3G4200D_Address_r = 0xD3; // (1-чтение)
uint8_t L3G4200D_Address_w = 0xD2; 

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int16_t x_g;
int16_t y_g;
int16_t z_g;

/*
I2C адресс ADXL345
HEX: 0x53
int: 83
1010011b
*/
uint8_t ADXL345_Address = 0x53;
uint8_t ADXL345_Address_r = 0xA7; // (1-чтение)
uint8_t ADXL345_Address_w = 0xA6;

#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define BW_RATE 0x2C

int16_t x_a;
int16_t y_a;
int16_t z_a;

/*
I2C адресс Honeywell MC5883L
HEX: 0x1E
*/
uint8_t MC5883L_Address = 0x1E;
uint8_t MC5883L_Address_r = 0x3D; // (1-чтение)
uint8_t MC5883L_Address_w = 0x3C; 

#define ConfigR_A 0x00
#define ConfigR_B 0x01
#define ModeR 0x02

int16_t x_m;
int16_t y_m;
int16_t z_m;

// Данные для анализа в матлабе
uint8_t xa_1;
uint8_t xa_2;
uint8_t ya_1;
uint8_t ya_2;
uint8_t za_1;
uint8_t za_2;

uint8_t xg_1;
uint8_t xg_2;
uint8_t yg_1;
uint8_t yg_2;
uint8_t zg_1;
uint8_t zg_2;

uint8_t xm_1;
uint8_t xm_2;
uint8_t ym_1;
uint8_t ym_2;
uint8_t zm_1;
uint8_t zm_2;

char buf[32]={0};
int counter=0;

// GLOBAL область видимости
  char str[10]; 
  uint8_t data;
  //double servo_angle_pitch_integral = 90;
  double pre_servo_res_pitch = 0;
  double pre_servo_res_roll = 0;
  double pre_servo_res_yaw = 0;
  double servo_res_pitch = 0;
  double servo_res_roll = 0;
  double servo_res_yaw = 0;
  double pitch;
  double roll;
  double pitch_tmp;
  double roll_tmp;
  double yaw;
  double FilterRoll;
  double FilterPitch;
  double FilterYaw;
  //double tmp1 = 0;
  //double tmp2 = 0;
  int32_t zaderzhka = 10; //мс 10^-3 c
  double K = 0.04;
  double angle;
  double Xcal2;  
  double Ycal2;
  double xa,ya,za;
  int f=0;
  
  double PKp = 1.24; //0.4
  double PKi = 0.38;//0.0006
  double PKd = 0.34; //0.005
  double RKp = 1.24; //0.4
  double RKi = 0.38; //0.0006
  double RKd = 0.34; //0.005
  double YKp = 0.35; //0.4
  double YKi = 0.1; //0.0006
  double YKd = 0.35; //0.005
  double error_pitch = 0;
  double pre_error_pitch = 0;
  double pre_error_pitch2 =0;
  double integral_pitch = 0;
  double error_roll = 0;
  double pre_error_roll = 0;
  double pre_error_roll2 = 0;
  double integral_roll = 0;
  double error_yaw = 0;
  double pre_error_yaw = 0;
  double pre_error_yaw2 = 0;
  double integral_yaw = 0;
  int flag = 0;
// Функция передачи символа через USART
void Usart_Transmit(uint8_t data)
{
  while(!(USART1->SR & USART_SR_TC)); //Ждем установки флага TC - завершения передачи
  USART1->DR = data;
}

// Функция передачи строки через USART
void Usart_Transmit_str(char* str)
{
  uint8_t i=0;  while(str[i])
  {
    Usart_Transmit(str[i]);
    i++;
  }
}

// Функции очистки PID параметров
void clear_pitch() 
{
   pre_error_pitch = 0;
   pre_servo_res_pitch = 0;
   pre_error_pitch = 0;
   pre_error_pitch2 = 0;
   error_pitch = 0;
   integral_pitch = 0;
   servo_res_pitch = 0;
}

void clear_roll() 
{
   pre_error_roll = 0;
   pre_servo_res_roll = 0;
   pre_error_roll = 0;
   pre_error_roll2 = 0;
   error_roll = 0;
   integral_roll = 0;
   servo_res_roll = 0;
}

void clear_yaw() 
{
   pre_error_yaw = 0;
   pre_servo_res_yaw = 0;
   pre_error_yaw = 0;
   pre_error_yaw2 = 0;
   error_yaw = 0;
   integral_yaw = 0;
   servo_res_yaw = 0;
}


void set_pos_servo1(double pos) 
{
  if ((pos>=-90) && (pos<=90)){
  uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;
  TIM2->CCR2 = SERVO_0 + tmp * ((int)pos+90);}else
  {
    clear_pitch();
  }
 }

void set_pos_servo2(double pos) 
{
  if ((pos>=-90) && (pos<=90)){
  uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;
  TIM2->CCR3 = SERVO_0 + tmp * ((int)pos+90);}else
  {
    clear_roll();
  }
 }

void set_pos_servo3(double pos) 
{
  if ((pos>=-180) && (pos<=180)){
  uint32_t tmp=(SERVO_180 - SERVO_0) /180 ;
  TIM2->CCR4 = SERVO_0 + tmp * ((int)pos+90);}else
  {
    clear_yaw();
  }
 }

void set_pos_servo1_tim(int pos) 
{
  TIM2->CCR2 = pos;
 }

void set_pos_servo2_tim(int pos) 
{
  TIM2->CCR3 = pos;
 }

void set_pos_servo3_tim(int pos) 
{
  TIM2->CCR4 = pos;
 }

GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;
I2C_InitTypeDef I2C_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

void InitUSART(void)
{
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
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
  USART_InitStruct.USART_BaudRate = 115200; //Скорость обмена 9600 бод
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; //Длина слова 8 бит
  USART_InitStruct.USART_StopBits = USART_StopBits_1; //1 стоп-бит
  USART_InitStruct.USART_Parity = USART_Parity_No ; //Без проверки четности
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Без аппаратного контроля
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //Включен передатчик и приемник USART1
  USART_Init(USART1, &USART_InitStruct); //Заданные настройки сохраняем в регистрах USART1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE); //Включаем USART1

}

void InitI2C(void)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); //Включаем тактирование GPIOB
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); //Включаем тактирование I2C 
  
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_ClockSpeed = 100000; // частота тактового сигнала (100кГц), максимум – 400 КГц
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; // Режим работы
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; // настройки для работы в быстром режиме
  I2C_InitStructure.I2C_OwnAddress1 = 0x15; // собственный адрес устройства
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // включено или нет использование бита подтверждения Ack
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // выбор формата адреса, 7 бит или 10 бит
  I2C_Init(I2C1, &I2C_InitStructure);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;		
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
  I2C_Cmd(I2C1, ENABLE);
}

void I2C_single_write(uint8_t HW_address, uint8_t addr, uint8_t data)
{
  //while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  // Шлем адрес устройства
  I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  // Шлем адрес нужного регистра
  I2C_SendData(I2C1, addr);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  // Шлем данные в регистр
  I2C_SendData(I2C1, data);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
}

uint8_t I2C_single_read(uint8_t HW_address_w, uint8_t HW_address_r, uint8_t addr)
{
  uint8_t data;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, HW_address_w, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C1, addr);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, HW_address_r, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
  data = I2C_ReceiveData(I2C1);
  I2C_AcknowledgeConfig(I2C1, DISABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  return data;
}

// Акселерометр ADXL345
void Init_ADXL345(void)
{
  // POWER_CTL
  //I2C_single_write(ADXL345_Address_w,POWER_CTL,0); 
  //I2C_single_write(ADXL345_Address_w,POWER_CTL,16);
  I2C_single_write(ADXL345_Address_w,POWER_CTL,8);
  // left-justified, +/-16g, FULL_RES 
  I2C_single_write(ADXL345_Address_w,DATA_FORMAT,0x0b);
  //I2C_single_write(ADXL345_Address_w,BW_RATE,0x0b);
}

// Гироскоп L3G4200D
void Init_L3G4200D(void)
{
  // CTRL_REG1 (00001111) Скорость оцифровки сигнала 100Гц (Cut-Off 12.5), все оси включены, Power Down - нормальный режим
  I2C_single_write(L3G4200D_Address_w,CTRL_REG1,0x2f); 
  // CTRL_REG2 (00000000) Нормальный режим ФВЧ (сброс чтением HP_RESET_FILTER), частота среза 8Гц (ODR=100Гц)
  I2C_single_write(L3G4200D_Address_w,CTRL_REG2,0x24);
  // CTRL_REG3 (00001000) Вывод состояния Data Ready на DRDY/INT2
  I2C_single_write(L3G4200D_Address_w,CTRL_REG3,0x00);
    // CTRL_REG4 (00001000)  Выбор полной шкалы 2000 dps
  I2C_single_write(L3G4200D_Address_w,CTRL_REG4,0x10);
    // CTRL_REG5 (00001000)  откл фильтр фвч и че-то еще
  I2C_single_write(L3G4200D_Address_w,CTRL_REG5,0x00);
}

//Компас Honeywell MC5883L
void Init_MC5883L(void)
{
  I2C_single_write(MC5883L_Address_w, ConfigR_A, 0x70); //8-среднее число выборок, 15 Hz default, normal measurement
  I2C_single_write(MC5883L_Address_w, ConfigR_B,0xA0); //Gain=5, or any other desired gain
  I2C_single_write(MC5883L_Address_w, ModeR, 0x00); 
}

void getGyroValues(void)
{
/*  uint8_t xH;
  uint8_t xL;
  uint8_t yH;
  uint8_t yL;
  uint8_t zH;
  uint8_t zL;*/
  xg_1 = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x28); //28 L
  xg_2 = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x29); //29 H 
  yg_1 = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2a);
  yg_2 = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2b);
  zg_1 = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2c);
  zg_2 = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2d);
  
  //xL = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x28); //28 L
 // xH = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x29); //29 H 
  /*yL = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2a);
  yH = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2b);
  zL = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2c);
  zH = I2C_single_read(L3G4200D_Address_w,L3G4200D_Address_r,0x2d);
 */ 
  x_g = ((xg_2 << 8) | xg_1);
  y_g = ((yg_2 << 8) | yg_1);
  z_g = ((zg_2 << 8) | zg_1);
  /*
  x_g = x_g*0.07*0.1;
  y_g = y_g*0.07*0.1;
  z_g = z_g*0.07*0.1;
  */
  //xg_1 = (xL | (xH << 8));
}

void getAccelValues(void)
{
  uint8_t xMSB;
  uint8_t xLSB;
  uint8_t yMSB;
  uint8_t yLSB;
  uint8_t zMSB;
  uint8_t zLSB;
  
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, ADXL345_Address_w, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C1, 0x32);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, ADXL345_Address_r, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  xLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); //X0
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  xMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); //X1
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  yLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  yMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  zLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, DISABLE);
  zMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

  
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  xa_1 = xLSB;
  xa_2 = xMSB;
  ya_1 = yLSB;
  ya_2 = yMSB;
  za_1 = zLSB;
  za_2 = zMSB;
  
  x_a = ((xMSB << 8) | xLSB);
  y_a = ((yMSB << 8) | yLSB);
  z_a = ((zMSB << 8) | zLSB);
}

void getMagValues(void)
{
  uint8_t xMSB;
  uint8_t xLSB;
  uint8_t yMSB;
  uint8_t yLSB;
  uint8_t zMSB;
  uint8_t zLSB;
  
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, MC5883L_Address_w, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C1, 0x03);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, MC5883L_Address_r, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  xLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); //X0
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  xMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); //X1
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  zLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  zMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  yLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, DISABLE);
  yMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  x_m = (xLSB << 8) | xMSB;
  y_m = (yLSB << 8) | yMSB;
  z_m = (zLSB << 8) | zMSB;
  
  xm_1 = xMSB;
  xm_2 = xLSB;
  ym_1 = yMSB;
  ym_2 = yLSB;
  zm_1 = zMSB;
  zm_2 = zLSB;
}

// Функция задержки
void Delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}


void TIM_ZAD(void)
  { 
    
     NVIC_SetPriority(TIM3_IRQn, 1); //Приоритет прерывания
  NVIC_EnableIRQ(TIM3_IRQn); //Разрешаем обработку прерывания от таймера 2
     //TIM2 init
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Подаем тактовые импульсы на таймер 2
  TIM3->PSC = 15999; //Коэффициент деления 2096 (1 msec)
  TIM3->ARR = 3000; //Перезагружаемое значение соответствует вызову прерывания через 500 msec
  TIM3->DIER |= TIM_DIER_UIE; //Разрешаем прерывание при переполнении счетчика
  TIM3->CR1 |= TIM_CR1_CEN; //Запуск счета   
  
    
  }

void SERVO(void)
  { 
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
    
    GPIOA->MODER |= GPIO_MODER_MODER3_1;  //Alternate function mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_3;  //Output push-pull
    GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR3;  //40 MHz
    GPIOA->PUPDR &=~GPIO_PUPDR_PUPDR3;  //No pull-up, pull-down
    
    GPIOA->AFR[0] = 0x00001110;//PA1 - AFIO1 (TIM2_CH2 и TIM2_CH3 и TIM2_CH4)

    /*Инициализация таймера TIM2
    Для формирования ШИМ используется канал захвата/сравнения 2 (TIM2_CH2)*/
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//Тактирование TIM2
    //TIM2->PSC = 0xA3D4;//Настройка предделителя таймера
    TIM2->CR1 |= TIM_CR1_ARPE;//Включен режим предварительной записи регистра автоперезагрузки
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE;//Включен режим предварительной загрузки регистра сравнения
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
    TIM2->ARR = 319999;//Период выходного сигнала T = 20mS (надо 20ms = 50Гц)
    /*
    8000 - 90 градусов 1200 предел =/
    16000 - 45 градусов
    24000 - 0 градусов
    32000 - -45 градусов
    40000 - -90 градусов
    */
    TIM2->CCR2 = 24000;//Длительность импульса (в данном случае Duty cycle = 80%)
    TIM2->CCR3 = 24000;
    TIM2->CCR4 = 24000;
    //TIM2->CCER |= TIM_CCER_CC2P;//Полярность выходного сигнала
    TIM2->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);//Выход канала захвата/сравнения включен
    TIM2->CR1 |= TIM_CR1_CEN;//Старт счета таймера
  }

void reverse(char s[])
{
int i, j;
char c;
for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
c = s[i];
s[i] = s[j];
s[j] = c;
}
}

void itoa(int n, char s[])
{
int i, sign;
if ((sign = n) < 0) //записываем знак 
n = -n; // делаем n положительным числом 
i = 0;
do { //генерируем цифры в обратном порядке 
s[i++] = n % 10 + '0'; //берем следующую цифру 
} while ((n /= 10) > 0); // удаляем 
if (sign < 0)
s[i++] = '-';
s[i] = '\0';
reverse(s);
}

// Выпрямляем данные с магнитометра, калибруем...
//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
double calibrated_values[3];   
//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction 
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc
void transformation(double uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {1.038, -0.009, 0.011},
    {0.021, 1.065, -0.046},
    {0.027, 0.019, 1.241}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    15.67,
    14.585,
    -33.419
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

 extern "C" void TIM3_IRQHandler(void) //Функция обработчика прерывания от таймера 3
  {
    TIM3->SR &= ~TIM_SR_UIF; //Сбрасываем бит вызова прерывания.
    flag++;
  }

extern "C" void USART1_IRQHandler(void)
{
   if (USART1->SR & USART_SR_RXNE) 
    {
          if((USART1->SR & USART_SR_RXNE)) //Проверяем поступление данных от компьютера
          {
            data = USART1->DR; //Считываем принятые данные 
            if (data!=0x0D)
            {
              buf[counter]=data; 
              counter++;
            }else
            {
              char subStr[3];
              char param[32];
              strncpy(subStr, buf+0, 2);
              memset(param, 0, sizeof(buf));   
              strncpy(param, buf+2, sizeof(buf));
               
              subStr[2] = 0;    
              //param[sizeof(buf)] = 0;
              //r-p
              if (strcmp("RP",subStr)==0)
              {
                PKp= atof (param);
                clear_pitch();
              }
              if (strcmp("RI",subStr)==0)
              {
                PKi= atof (param);
                clear_pitch();
              }
              if (strcmp("RD",subStr)==0)
              {
                PKd= atof (param);
                clear_pitch();
              }
              //p-r
              if (strcmp("PP",subStr)==0)
              {
                RKp= atof (param);
                clear_roll();
              }
              if (strcmp("PI",subStr)==0)
              {
                RKi= atof (param);
                clear_roll();
              }
              if (strcmp("PD",subStr)==0)
              {
                RKd= atof (param);
                clear_roll();
              }
              //yaw
              if (strcmp("YP",subStr)==0)
              {
                YKp= atof (param);
                clear_yaw();
              }
              if (strcmp("YI",subStr)==0)
              {
                YKi= atof (param);
                clear_yaw();
              }
              if (strcmp("YD",subStr)==0)
              {
                YKd= atof (param);
                clear_yaw();
              }
              memset(buf, 0, sizeof(buf));
              counter=0;
            }
          }
    }
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

main()
{
  /* 
  Настраиваем такирвую частоту процессора от HSI 
  HSI oscillator clock – внутренний высокочастотный RC генератор
  с частотой 16 МГц 
  */
  RCC->CR |= RCC_CR_HSION; //Включаем тактовый генератор HSI
  while(!(RCC_CR_HSIRDY)); //Ждем его стабилизации
  RCC->CFGR |= RCC_CFGR_SW_HSI; //Выбираем источником тактовой частоты SYSCLK генератор HSI
  RCC->CR &= ~RCC_CR_MSION; //Отключаем генератор MSI.

  InitUSART(); 
  SERVO();  
  InitI2C();  
  Init_L3G4200D();
  Init_ADXL345();
  Init_MC5883L();
  TIM_ZAD();
  set_pos_servo1(-30);
  //Usart_Transmit_str("SSkS=>");
  //Delay_ms(3000);
  
  while(1){
    
    getAccelValues();
    getGyroValues();
    getMagValues();
 
    //xa =x_a*0.0039; 
    //ya =y_a*0.0039;
    //za =z_a*0.0039;
    xa = (x_a *0.0039-(-0.01035))/0.99265;
    ya = (y_a  *0.0039-0.0402)/1.0067;
    za = (z_a *0.00390-0.10525)/0.97225;
    //roll_tmp = atan2(ya ,za);
    //pitch_tmp = atan2(xa ,sqrt(za*za + ya*ya));
    roll = (atan2(ya ,sqrt(za*za + xa*xa)) * 180) / 3.14;
    pitch = (atan2(xa ,sqrt(za*za + ya*ya)) * 180) / 3.14;
    FilterRoll = (1-K)*(FilterRoll+(x_g*0.0175 + 0.3705)*0.01)+K*roll;
    FilterPitch = (1-K)*(FilterPitch+(-1)*(y_g*0.0175 + 0.3705)*0.01)+K*pitch; //-1
    roll_angle = FilterRoll+90; //-1
    pitch_angle = FilterPitch+90;  
    
   if (flag>=0) 
   {
     // Новый крутой регулятор. Экономим вычислительные ресурсы    
     error_pitch = 90-pitch_angle;
     servo_res_pitch = pre_servo_res_pitch+PKp*(error_pitch-pre_error_pitch)+ PKi*(error_pitch+pre_error_pitch)/2+PKd*(y_g*0.0175 + 0.3705)*0.01; //PKd*(error_pitch-2*pre_error_pitch+pre_error_pitch2); //
     set_pos_servo1(servo_res_pitch);
     pre_servo_res_pitch = servo_res_pitch;
     pre_error_pitch = error_pitch;
     pre_error_pitch2 = pre_error_pitch;
 sprintf(str, "%f", FilterPitch); 
 Usart_Transmit_str(str);
 Usart_Transmit_str("\r");
    } else 
    {
     clear_pitch();
    } 
    //RA 
    /*Usart_Transmit_str("RA");
    sprintf(str, "%f", pitch_angle ); 
    Usart_Transmit_str(str);
    Usart_Transmit_str("E");
    sprintf(str, "%f", error_pitch); 
    Usart_Transmit_str(str);
    Usart_Transmit_str("S");
    sprintf(str, "%f", servo_res_pitch); 
    Usart_Transmit_str(str);*/

    error_roll = 90-roll_angle;
    servo_res_roll = pre_servo_res_roll+RKp*(error_roll-pre_error_roll) + RKi*(error_roll+pre_error_roll)/2+(-1)*RKd*(x_g*0.0175 + 0.3705)*0.01;//RKd*(error_roll-2*pre_error_roll+pre_error_roll2); 
    set_pos_servo2(servo_res_roll);
    pre_servo_res_roll = servo_res_roll;
    pre_error_roll = error_roll;
    pre_error_roll2 = pre_error_roll;
    // PA
    /*Usart_Transmit_str("PA");
    sprintf(str, "%f", roll_angle ); 
    Usart_Transmit_str(str);
    Usart_Transmit_str("E");
    sprintf(str, "%f", error_roll); 
    Usart_Transmit_str(str);
    Usart_Transmit_str("S");
    sprintf(str, "%f", servo_res_roll); 
    Usart_Transmit_str(str);*/
    /*
    double input_data [3] = {(double)x_m,(double)y_m,(double)z_m};
    transformation(input_data);     
    //Xcal2 = ((double)calibrated_values[0]*0.92)*cos((double)pitch_tmp) + ((double)calibrated_values[1]*0.92)*sin((double)roll_tmp)*sin((double)pitch_tmp) + ((double)calibrated_values[2]*0.92)*cos((double)roll_tmp)*sin((double)pitch_tmp);
   // Ycal2 = ((double)calibrated_values[1]*0.92)*cos((double)roll_tmp) - ((double)calibrated_values[2]*0.92)*sin((double)roll_tmp);
    Xcal2 = (calibrated_values[0]*0.92)*cos(pitch_tmp) + (calibrated_values[1]*0.92)*sin(roll_tmp)*sin(pitch_tmp) + (calibrated_values[2]*0.92)*cos(roll_tmp)*sin(pitch_tmp);
    Ycal2 = (calibrated_values[1]*0.92)*cos(roll_tmp) - (calibrated_values[2]*0.92)*sin(roll_tmp);
    angle = atan2( -Ycal2, Xcal2 );
    //angle = atan2( calibrated_values[1], calibrated_values[0]);
    if (angle < 0) 
    angle += 2*3.14;
    if (angle > 2*3.14)
    angle -= 2*3.14;        
    yaw = angle * (180 / 3.14);
    //FilterYaw =(1-K)*(FilterYaw+z_g*0.07*0.01)+K*yaw;
    
    
    error_yaw = 180-yaw;
    servo_res_yaw = pre_servo_res_yaw+YKp*(error_yaw-pre_error_yaw) + YKi*(error_yaw+pre_error_yaw)/2+(-1)*YKd*z_g*0.07*0.01;//+YKd*(error_yaw-2*pre_error_yaw+pre_error_yaw2);
    set_pos_servo3(servo_res_yaw);
    pre_servo_res_yaw = servo_res_yaw;
    pre_error_yaw = error_yaw;
    pre_error_yaw2 = pre_error_yaw;*/
    
    //YA
    /*Usart_Transmit_str("YA");
    sprintf(str, "%f", yaw ); 
    Usart_Transmit_str(str);
    Usart_Transmit_str("E");
    sprintf(str, "%f", error_yaw); 
    Usart_Transmit_str(str);
    Usart_Transmit_str("S");
    sprintf(str, "%f", servo_res_yaw); 
    Usart_Transmit_str(str);*/

   /* sprintf(str, "%d ", (int)x_m);
    Usart_Transmit_str(str);
    Usart_Transmit(',');
    sprintf(str, "%d", (int)y_m);
    Usart_Transmit_str(str);
    Usart_Transmit(',');
    sprintf(str, "%d", (int)z_m);
    Usart_Transmit_str(str);
    Usart_Transmit_str("\r\n");*/
    // Вывод калиброваных значений*/
   /* float input_data [3] = {(double)x_m,(double)y_m,(double)z_m};
    transformation(input_data);
    sprintf(str, "%d ", (int)calibrated_values[0]);
    Usart_Transmit_str(str);
    Usart_Transmit(',');
    sprintf(str, "%d", (int)calibrated_values[1]);
    Usart_Transmit_str(str);
    Usart_Transmit(',');
    sprintf(str, "%d", (int)calibrated_values[2]);
    Usart_Transmit_str(str);
    Usart_Transmit_str("\r\n");*/
    //sprintf(str, "%f ", xa);
    //Usart_Transmit_str(str);
    /*Usart_Transmit(' ');
    sprintf(str, "%f", ya);
    Usart_Transmit_str(str);
    Usart_Transmit(' ');
    sprintf(str, "%f", za);
    Usart_Transmit_str(str);*/
    //Usart_Transmit_str("\r");
      //Отладка в матлабе
 /*  Usart_Transmit(xa_1);
    Usart_Transmit(xa_2);
    Usart_Transmit(ya_1);
    Usart_Transmit(ya_2);
    Usart_Transmit(za_1);
    Usart_Transmit(za_2);
    //
    Usart_Transmit(xg_1);  
    Usart_Transmit(xg_2);
    Usart_Transmit(yg_1);  
    Usart_Transmit(yg_2);
    Usart_Transmit(zg_1);  
    Usart_Transmit(zg_2);*/
    
   Delay_ms(zaderzhka); 
    /*Usart_Transmit(xm_1);  
    Usart_Transmit(xm_2);
    Usart_Transmit(ym_1);  
    Usart_Transmit(ym_2);
    Usart_Transmit(zm_1); 
    Usart_Transmit(zm_2);
    uint8_t data;
    l=0;
    while(!(l==1))
    {
      while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==RESET)
      {
      data=USART_ReceiveData(USART1);
      if (data==0x0D) l=1;
      }
    }*/
   /* int i=0;
   for (i=10;i<170;i++)
   {
     Delay_ms(100);
     set_pos_servo3(i);
   }*/
  }

}
