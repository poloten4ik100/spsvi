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

/*
I2C ������ L3G4200D
HEX: 0x69
int: 105
110100xb, x=1 => SDO ���. 1 => LSB = 1
*/
uint8_t L3G4200D_Address = 0x69;
uint8_t L3G4200D_Address_r = 0xD3; // (1-������)
uint8_t L3G4200D_Address_w = 0xD2; 

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int x_g;
int y_g;
int z_g;

/*
I2C ������ ADXL345
HEX: 0x53
int: 83
1010011b
*/
uint8_t ADXL345_Address = 0x53;
uint8_t ADXL345_Address_r = 0xA7; // (1-������)
uint8_t ADXL345_Address_w = 0xA6;

#define POWER_CTL 0x2D

int16_t x_a;
int16_t y_a;
int16_t z_a;

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
I2C_InitTypeDef I2C_InitStructure;

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

void InitI2C(void)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); //�������� ������������ GPIOB
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); //�������� ������������ I2C
  
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_ClockSpeed = 100000; // ������� ��������� ������� (100���), �������� � 400 ���
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; // ����� ������
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; // ��������� ��� ������ � ������� ������
  I2C_InitStructure.I2C_OwnAddress1 = 0x15; // ����������� ����� ����������
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // �������� ��� ��� ������������� ���� ������������� Ack
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // ����� ������� ������, 7 ��� ��� 10 ���
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
  // ���� ����� ����������
  I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  // ���� ����� ������� ��������
  I2C_SendData(I2C1, addr);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  // ���� ������ � �������
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

// ������������ ADXL345
void Init_ADXL345(void)
{
  // POWER_CTL
  I2C_single_write(ADXL345_Address_w,POWER_CTL,0); 
  I2C_single_write(ADXL345_Address_w,POWER_CTL,16);
  I2C_single_write(ADXL345_Address_w,POWER_CTL,8);
}

// �������� L3G4200D
void Init_L3G4200D(void)
{
  // CTRL_REG1 (00001111) �������� ��������� ������� 100�� (Cut-Off 12.5), ��� ��� ��������, Power Down - ���������� �����
  I2C_single_write(L3G4200D_Address_w,CTRL_REG1,0x0f); 
  // CTRL_REG2 (00000000) ���������� ����� ��� (����� ������� HP_RESET_FILTER), ������� ����� 8�� (ODR=100��)
  I2C_single_write(L3G4200D_Address_w,CTRL_REG2,0x00);
  // CTRL_REG3 (00001000) ����� ��������� Data Ready �� DRDY/INT2
  I2C_single_write(L3G4200D_Address_w,CTRL_REG3,0x00);
    // CTRL_REG4 (00001000)  ����� ������ ����� 250 dps
  I2C_single_write(L3G4200D_Address_w,CTRL_REG4,0x30);
    // CTRL_REG5 (00001000)  ���� ������ ��� � ��-�� ���
  I2C_single_write(L3G4200D_Address_w,CTRL_REG5,0x00);
}

void getGyroValues(void)
{
  uint8_t xH;
  uint8_t xL;
  uint8_t yH;
  uint8_t yL;
  uint8_t zH;
  uint8_t zL;
  
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, L3G4200D_Address_w, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C1, 0x28);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, L3G4200D_Address_r, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  xL = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  xH = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  
  yL = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  yH = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  
  zL = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); 
  I2C_AcknowledgeConfig(I2C1, DISABLE);
  zH = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
   
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  x_g = ((xH << 8) | xL);
  y_g = ((yH << 8) | yL);
  z_g = ((zH << 8) | zL);
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
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  xMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  
  yLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  yMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  
  zLSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  
  zMSB = I2C_ReceiveData(I2C1);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  I2C_AcknowledgeConfig(I2C1, DISABLE);
  
  I2C_GenerateSTOP(I2C1, ENABLE);
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  
  x_a = ((xMSB << 8) | xLSB);
  y_a = ((yMSB << 8) | yLSB);
  z_a = ((zMSB << 8) | zLSB);
}

// ������� ��������
void Delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
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
if ((sign = n) < 0) //���������� ���� 
n = -n; // ������ n ������������� ������ 
i = 0;
do { //���������� ����� � �������� ������� 
s[i++] = n % 10 + '0'; //����� ��������� ����� 
} while ((n /= 10) > 0); // ������� 
if (sign < 0)
s[i++] = '-';
s[i] = '\0';
reverse(s);
}

int main()
{
  char str[10]; 

  InitUSART(); 
  //Usart_Transmit_str("Start!!!\r\n");
  //SERVO();
  
  InitI2C();
  
  //Init_L3G4200D();
  
  Init_ADXL345();
   
  //Usart_Transmit_str("OK\r\n");
    
  Delay_ms(20);
  //uint8_t adr = I2C_single_read(0xD3, 0x0F);
  //Usart_Transmit(adr);
  /*while(1)
  {
    getGyroValues();
    //Usart_Transmit_str("X: ");
    Usart_Transmit(y_g);
    //Usart_Transmit_str("Y: ");
    //Usart_Transmit_str("Z: ");
    //Usart_Transmit(z_g);
    //Usart_Transmit_str("\r\n");
    Delay_ms(50);
  }*/
  
  while(1)
  {
    getAccelValues();
    //Usart_Transmit_str("X: ");
    //sprintf(str, "%d", (int)x_a);
    //Usart_Transmit_str(str);

    //Usart_Transmit_str("Y: ");
    //Usart_Transmit(y_a);
    //deg = (int16_t)(180); 
   // sprintf(str, "Angle: %d", deg);
    //Usart_Transmit_str("Z: ");
    //Usart_Transmit(z);
    //Usart_Transmit_str(str);
    Usart_Transmit_str("X: ");
    sprintf(str, "%d", x_a);
    Usart_Transmit_str(str);
    
    Usart_Transmit_str(" Y: ");
    sprintf(str, "%d", y_a);
    Usart_Transmit_str(str);
    
    Usart_Transmit_str(" Z: ");
    sprintf(str, "%d", z_a);
    Usart_Transmit_str(str);
    
    Usart_Transmit_str("\r\n");
    
    Delay_ms(300);
  }

}
