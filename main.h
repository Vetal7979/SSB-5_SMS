// SIM900 || SIM800

#define SIM900
//#define SIM800
//------------------------------------------------------------------------------
#define ATE1 

#define max_numb_len      25      // ������������ ����� ��������� ����������� ������

#define u8 unsigned char
#define uint8_t unsigned char

#define BUFF_SIZE       512     // ������ ������������ ������
#define Rx_buff_size    256
#define SMS_buff_size   256
#define ADC_VREF_TYPE 0x40


typedef enum 
{
  STATUS_OK       = 0x00,
  STATUS_ERROR    = 0x01,
  STATUS_TIMEOUT  = 0x02
} StatusTypeDef;

//�������� � �������
signed int __flash  InTab[]= {212,229,246,264,282,300,318,336,354,372,389,407,424,441,458,474,490,505,521,532,544};
signed int __flash  OutTab[]={-50,-40,-30,-20,-10,  0, 10, 20, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150};

__eeprom __no_init unsigned char setup; // 1-� ���������
__eeprom __no_init unsigned char ust_m;                                         // �������
__eeprom __no_init unsigned char bit_temp_zapret;                               // ���� ������� ������� ���������������(�������� � 1, � �� �� ������� ��������)
__eeprom __no_init unsigned char hyst_m;                                        // ������������ ���������� 30
__eeprom __no_init unsigned char relay_zapret;                                  // ������ ���������������



//------------------------------------------------------------------------------
unsigned int read_adc(unsigned char adc_input);
void putchar_debug(unsigned char send_char);
void put_message_modem_RAM (char *s);
u8 putchar_modem(unsigned char send_char);
void put_message_modem (char const __flash  *s);
void Get_Number(char *buff, char *mass);
int ADC2Temp(unsigned int Temp_adc1);
void say_numeric(int num);
void say_gradus(int num);

unsigned char Get_Char(void);
void Clean_buff(void);
void Parser(void);
void System_Error(unsigned char err);
void Hardware_PWR_ON(void);
StatusTypeDef SIM800_PWR_ON(void);
StatusTypeDef SIM800_Init(void);
StatusTypeDef SSB_Config(void);
StatusTypeDef Write_SIM_default(void);
StatusTypeDef SIM800_SMS_Send(unsigned char number, char *text, char type);
StatusTypeDef SIM800_SMS_Read(void);
void Execute_SMS_Command(char *text);
StatusTypeDef SIM800_Write_to_SIM(char *text, char cell, char *name);
void Check_Incoming_Call(void);
void Check_DTMF_Cmd(void);
void Execute_DTMF_Cmd(void);
void call_master(unsigned char who,unsigned char message, unsigned char channel);



