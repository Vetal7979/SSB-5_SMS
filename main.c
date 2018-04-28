//   Сигналка версия 4 основная
// 
// 
// 
// 
#define CPU_CLK_Hz 7372800
#include <iom64.h>
#include "main.h"
#include "compiler.h"
#include "delayIAR.h"
#include "delay.c"
#include <ioavr.h>
#include <string.h>  
#include <stdlib.h>
#include <stdio.h>
#include "uart.h"
#include "init.c"
#include "voice.c"

//#include "lcd.c" 
//#include "lcd.h"
//#include "pff.h" //?
//#include "diskio.h" //?

char cycle_buff[BUFF_SIZE];  // кольцевой буфер
u16 r_index=0, w_index=0, rx_count=0;  // индекс чтения, записи, кол-во байт в буфере

char rx_buff[Rx_buff_size]; // входной буфер
char sms_buff[SMS_buff_size]; // буфер смс
char answer[70]; // ответное смс

u8 rx_cnt=0;
unsigned char mess; 
char dtmf_buff[5], dtmf_cnt=0; // буфер dtmf команд
u16 timeout; // таймаут ответа от модема
char task=0xFF;  // номер задачи, FF - не определено
                // 0 - основной цикл

/* Флаги (можно потом сделать чтоб все хранилось в 1 переменной битами) */   
unsigned char ok_flag, err_flag=0, sms_flag=0, cpin_rdy=0, ring_flag=0, cmgs_flag=0;
unsigned char call_rdy=0, sms_rdy=0, n_pwr_dn=0, cfun=0;


char mass_number[max_numb_len];
char mass_inc_number[max_numb_len]; // номер при входящем звонке
char mass_master_number[max_numb_len]; // мастер номер (считанный)
char ring_master=0; // 1 если определили что звонит мастер номер


unsigned char recived_byte;
unsigned char message_ready;

unsigned char pa[11]; // Настройки ячейки CONFIG *000011**11 (11 символов) так и будет по порядку, звездочки тоже.
unsigned char vv=0; // если в 1 то авария датчика
char p[4]; // хранимый пароль из симки (массив 4 байта в ASCii)

unsigned long overflow; // счетчик для вешания трубки
unsigned char ring_cnt=0;  // чтоб трубку брало со 2-го RING, а не сразу
unsigned char wait_on,wait_off;    // байт на задержку включения и задержку выключения
unsigned char retry_count;         // байт на счетчик попыток
unsigned char relay1_status=0,relay2_status=0,relay3_status=0;
unsigned char relay4_status=0,relay5_status=0,relay6_status=0;
unsigned char relay7_status=0;


unsigned char security_stand = 0;  // 0 - не стоит 1 стоит на охране ( верхняя плата )
unsigned char sec_stat=0; // нижняя плата на охране (1) или нет (0)
unsigned char sec_stat1,sec_stat2,sec_stat3,sec_stat4;
unsigned char cm[5]={0,0,0,0,0}; // надо ли звонить на номера 1..5
unsigned char ignor1,ignor2,ignor3,ignor4;

unsigned char temperature_status;
unsigned char baterry_bit = 1;
unsigned char zb=0,t_off,t_on,ust,hyst;

unsigned int u,chan_stat1,chan_stat2,chan_stat3,chan_stat4;  //  сохраним там игнорируемые каналы

char flag_rele3=0; // флаг того, что реле включим по срабатыванию сигналки
unsigned long rele3_count=0; // счетчик времени работы сирены 

unsigned int reztemp;//результат замера АЦП
int temperature;



//------------------------------------------------------------------------------
// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
  ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
  // Delay needed for the stabilization of the ADC input voltage
  delay_us(10);
  // Start the AD conversion
  ADCSRA|=0x40;
  // Wait for the AD conversion to complete
  while ((ADCSRA & 0x10)==0);
  ADCSRA|=0x10;
  return ADC;
}
//------------------------------------------------------------------------------

void putchar_debug(unsigned char send_char)
{
  //if (send_char == 0x0d) mess=0; // конец сообщения
  while (!(UCSR0A & 0x20))
    ; /* wait xmit ready */
  UDR0 = send_char;
  //return(send_char);
}
//------------------------------------------------------------------------------
/*
void put_message_UCS2(char *b)
{
// не работает, бяда с указателями
 char aaa[5]={0,0,0,0,0};
 u16 tmp=0;
// lcd_at(0,0); lcd_put_char(*b,0);
 while(*b);
 {
  lcd_at(0,1); lcd_put_char(*b,0);
//  Delay_ms(500);
  __watchdog_reset();
  if (*s>=192) tmp=*s;
  else tmp=*s + 0x350;
  sprintf(aaa,"%4.4X",tmp);
 put_message_modem_RAM(aaa);
  *s++;
 
 }
}
*/
//------------------------------------------------------------------------------
void put_message_modem_RAM (char *s)
{
  while (*s)
    putchar_modem(*s++);
  Delay_ms(10);
}
//------------------------------------------------------------------------------
u8 putchar_modem(unsigned char send_char)
{
  while (!(UCSR1A & 0x20))
    ; /* wait xmit ready */
  UDR1 = (unsigned char) send_char;
  return(send_char);
}
//------------------------------------------------------------------------------
void put_message_modem (char const __flash  *s)
{
  while (*s)
    putchar_modem(*s++);
  Delay_ms(10); // без задержек периодически подглючивает
}

//------------------------------------------------------------------------------
void Get_Number(char *buff, char *mass)  // выдирает номер их *buff и кладет его в mass
{
//  char i=0;
  while(*buff++!='"');
  while(*buff!='"') // начинаем читать номер до след "
   {
    *mass++=*buff++; // пишем номер в mass_number
   }
  *mass=0x00; // завершаем строку \0
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
int ADC2Temp(unsigned int Temp_adc1)     // ----
{  
  register unsigned int cnt;    
  
  if ( Temp_adc1 < 222 ) return -100;
  if ( Temp_adc1 > 514 ) return 1000;
  //setup counter
  cnt=0xffff;
  
  while ( InTab[++cnt] < Temp_adc1 );
  
  if(cnt) --cnt;
  Temp_adc1= ( (double)OutTab[cnt]+((long)OutTab[cnt+1]-OutTab[cnt])*(Temp_adc1-InTab[cnt])/
              (InTab[cnt+1]-InTab[cnt]) );
  return Temp_adc1; }  
//------------------------------------------------------------------------------
// Говорит любое число от -199 до 199
void say_numeric(int num)
{
  if (num==0) say_zero();
  if (num<0){ minus(); num=num*(-1); } //если отрицательная температура скажем минус и сделаем ее положительной
  if (num>=100) { say_100(); num=num-100; Delay_ms(100); } // больше  100 - скажем сто, и отнимем....   
  if (num>=20)
  {
    switch (num/10)
    {
    case 2:{say_20();}break;
    case 3:{say_30();}break;
    case 4:{say_40();}break;
    case 5:{say_50();}break;
    case 6:{say_60();}break;
    case 7:{say_70();}break;
    case 8:{say_80();}break;
    case 9:{say_90();}break;
    }
    Delay_ms(100);
    num%=10;
  }
  else if(num>=10 && num<=19)
  {
   switch (num)
   {
   case 10:{ say_10(); }break; 
   case 11:{ say_11(); }break; 
   case 12:{ say_12(); }break; 
   case 13:{ say_13(); }break; 
   case 14:{ say_14(); }break; 
   case 15:{ say_15(); }break; 
   case 16:{ say_16(); }break; 
   case 17:{ say_17(); }break; 
   case 18:{ say_18(); }break; 
   case 19:{ say_19(); }break; 
   }
   num/=100; // чтоб получился 0 и вышли
  }
  
  if (num<10)
  {
    switch (num)
    {
    case 1:{say_one();}break;  
    case 2:{say_two();}break;
    case 3:{say_three();}break;
    case 4:{say_four();}break;
    case 5:{say_five();}break;
    case 6:{say_six();}break;
    case 7:{say_seven();}break;
    case 8:{say_eight();}break;
    case 9:{say_nine();}break;
    }    
  }
}
//------------------------------------------------------------------------------
// говорит градус, градусов, градуса
void say_gradus(int num)
{
 if (num<0) num*=-1;
 num%=10;
 if (num==1) gradus();
 else if(num==2 || num==3 || num==4) gradusa();
 else gradusov();

}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//---------------------------Прерывание по приходу данных из модема-------------
#pragma vector = USART1_RXC_vect   
__interrupt void _USART1_RXC (void)
{
/* Read one byte from the receive data register */
unsigned  char status, data;
  status=UCSR1A;
  data=UDR1;
  if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
  {
    cycle_buff[w_index]=data;
    w_index++; rx_count++;
    if (w_index==BUFF_SIZE) w_index=0;
  }  
}
//------------------------------------------------------------------------------
//-------------------------Прерывание по приходу данных от сопроцессора---------
#pragma vector = USART0_RXC_vect  
__interrupt void _USART0_RXC (void)
{
  char status,data;
  status=UCSR0A;  // считали статус
  data=UDR0;      // cчитали данные
  if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0) // если посылка пришла нормальная
  {
    recived_byte = data;
    message_ready = 1;   //   Можно читать месягу потихоньку
  };
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Timer 0 overflow interrupt service routine
// 10.0 ms
#pragma vector = TIMER0_OVF_vect  
__interrupt void _TIMER0_OVF_vect (void)
{
// Reinitialize Timer 0 value
 TCNT0=0xB8; // 10.00 ms

 if(overflow!=0) overflow--;

 if (timeout!=0) timeout--;
 
 if(rele3_count!=0) rele3_count--;
 
//PORTC_Bit6=~PORTC_Bit6;// test
}// Timer 0 
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
unsigned char Get_Char(void)
{
   unsigned char sym = 0;
   if (rx_count > 0)  //если буфер не пустой
   {                            
      sym = cycle_buff[r_index];              //считываем символ из буфера
      rx_count--;                                   //уменьшаем счетчик символов
      r_index++;                                  //инкрементируем индекс головы буфера
      if (r_index == BUFF_SIZE) r_index = 0;
   }
   return sym;
}
//------------------------------------------------------------------------------
void Clean_buff(void)
{
  rx_count=0;
  r_index=0;
  w_index=0;
}
//------------------------------------------------------------------------------
void Parser(void)
{ 
 unsigned char sym = 0;
  // для SIM900 символ на 6 позиции, для SIM800 - на 7 (эти суки добавили пробел)
  // define в main.h
#ifdef SIM800
 #ifndef SIM900
  char dtmf_sym_pos=7;
 #endif 
#else
 #ifdef SIM900
  #ifndef SIM800 
   char dtmf_sym_pos=6;
  #endif 
 #endif 
#endif

 while(!mess && rx_count!=0)
 {  
   sym=Get_Char();
//   if(sym==0x0A && (rx_cnt!=0 && rx_buff[rx_cnt-1]==0x0D))                      // пришло 0x0A, а перед этим было 0x0D
   if (sym==0x0D)                                                               // работаем только по /r
   { 
     mess=1;                                                                    // поднимаем флаг - пока флаг - след сообщение не прочитается, поэтому mess=0; только после обработки строки, а то может заглючить
     rx_buff[rx_cnt]=0; /*rx_buff[rx_cnt-1]=0;*/                                    // заменяем \r\n на \0\0 (завершаем строку) - чтоб нормально работать со строками и не мешался \r в конце) 
     rx_cnt=0;
     if (rx_buff[0]==0 || !strncmp(rx_buff,"AT",2)) mess=0;                     // пришло только \r\n  или пришло эхо от модема (начинается с AT) - его пропускаем                                         
//     if (rx_buff[rx_cnt-2]==0x0D) rx_buff[rx_cnt-2]=0;                          // при эхе от модема может придти \r\r\n, но начинаться будет не с AT - вроде не должно такого быть!! 
     
     // Далее парсим на URC                                                     // 
     if (rx_buff[0]=='+')                                                       // пришло что-то, начинающееся с + -> разбираем его 
     {
       if (!strncmp_P(rx_buff,cpin,6))                                          // +CPIN:
       { 
         if (!strncmp_P(rx_buff,cpin_ready,12)) cpin_rdy=1;                     // +CPIN: READY                      
         if (!strncmp_P(rx_buff,cpin_not_ins,19) || 
             !strncmp_P(rx_buff,cpin_not_ready,16)) System_Error(1);            // +CPIN: NOT READY придет если отвалится симка
         
         mess=0; 
       } 
       
       else if (!strncmp(rx_buff,"+CFUN:",6))                                   // +CFUN:
       { 
         if (!strncmp(rx_buff,"+CFUN: 1",12)) cfun=1;  
         mess=0; 
       } 
       
       else if (!strncmp_P(rx_buff,dtmf,6))                                     // +DTMF:
       { 
         if (pa[10]==0x31)// проговариваем нажимаемые цифры
           if (rx_buff[dtmf_sym_pos]>=0x30 && rx_buff[dtmf_sym_pos]<=0x39) say_numeric(rx_buff[dtmf_sym_pos]-0x30); else gudok(); 
           
           overflow=15*100; dtmf_buff[dtmf_cnt++]=rx_buff[dtmf_sym_pos];  // каждое нажатие кнопки дает еще +15 сек до того как повесим трубку
           if(dtmf_cnt==5) dtmf_cnt=0;
		   mess=0;
       }
       
       else if (!strncmp_P(rx_buff,cmti,6))                                     // +CMTI:
       { if (!sms_flag) { sms_flag=1; strcpy(sms_buff,rx_buff); }  mess=0; }    //строку копируем для дальнейшего разбора (если уже обрабатываем какую-то смс, то эту пропустим)
       
       else if (!strncmp_P(rx_buff,clip_p,6))                                   // +CLIP:
       { 
         Get_Number(rx_buff,mass_inc_number);                                   // выдираем номер
         if (!strncmp(mass_master_number,mass_inc_number,strlen(mass_master_number))) ring_master=1; else ring_master=0;
         mess=0;
       }
     }// '+'
     else
     {
       if (!strncmp_P(rx_buff,ring,4) && task!=0) { ring_flag=1; mess=0; }      // ринг надо как-то по другому обрабатывать       // mess=0 - выкидываем RING чтоб не висело в буфере 
       else if (!strncmp_P(rx_buff,call_ready,10)) { call_rdy=1; mess=0; }      // Call Ready
       else if (!strncmp_P(rx_buff,sms_ready,9)) { sms_rdy=1; mess=0; }         // SMS Ready
       else if (!strncmp_P(rx_buff,power_dn,17)) { n_pwr_dn=1; mess=0; }        // NORMAL POWER DOWN
       //       else if (!strncmp_P(rx_buff,init_str,4)) { mess=0; rx_cnt=0; }           // пропускаем IIIIюююю (смотрим тока 1-е 4 символа, т.к. больше - глючит)
       
       
     }
   }                 
   else if (sym!=0 && sym!=0x0A) rx_buff[rx_cnt++]=sym;                         // еще не конец - пишем в буфер (0x0A пропускаем)
   
   if (rx_cnt==255) rx_cnt=0;  // защита от переполнения (хотя т.к. rx_cnt = u8 - оно сработает автоматом по переполнению )
   
 }//while
}
//------------------------------------------------------------------------------
// мигаем код ошибки
void System_Error(unsigned char err)
{
 while (1)
 {
  __watchdog_reset();  
  switch (err)
  {
  case 0:{}break;
  case 1:             // Нет SIM или SIM отвалилась
    { 
      putchar_debug(0x11); putchar_debug(0x31); putchar_debug(0x20); Delay_ms(300);
      putchar_debug(0x10); putchar_debug(0x30); putchar_debug(0x21); Delay_ms(300);
    }break;
  case 2:{}break;
  }
 }
}
//------------------------------------------------------------------------------
void Hardware_PWR_ON(void)
{
 Delay_ms(100);                                                                 // без этой задержки почему-то криво обрабатывает флаг n_pwr_dn
 n_pwr_dn=0; call_rdy=0; cpin_rdy=0;  sms_rdy=0; // сбросим флаги
 modem_power = 1;
 Delay_s(3);
 modem_power = 0; 

}
//------------------------------------------------------------------------------

StatusTypeDef SIM800_PWR_ON(void)
{
  
  if (setup==1) // не первое включение
    Hardware_PWR_ON();
  else
    setup=1;    // включили 1-й раз

 mess=0;

 char op_present=0;
 char st=0;
 char end=0;
 char err_cnt=0;

timeout=2000; // ждем появления Call Ready 20 сек

 while(!end)
 {
  __watchdog_reset();  

  switch (st)
  { // появление Call Ready задается AT+CIURC=1 (default)
  case 0: {  if (mess) mess=0; if(call_rdy && cpin_rdy/* && sms_rdy */) st++; }break; // ждем прихода +CPIN: Ready, Call Ready и SMS Ready 

  case 1: { put_message_modem (ipr_115200); st++; timeout=200; }break;
  case 2: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,strlen_P(error))) {st--; err_cnt++;} }  mess=0;} } break;
  
  case 3: // ATE 0
    { 
#ifdef ATE1
st=10;
#else
put_message_modem (echo_off); st++; timeout=200; 
#endif
    }break;
  case 4:  // OK
    { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st=10; else { if (!strncmp_P(rx_buff,error,strlen_P(error))) {st--; err_cnt++;} }  mess=0;} } break;   
    
  case 10:
    { put_message_modem(cops); st++; timeout=200;}break;
  case 11:
    { 
      if (mess) 
      {  
        if (!strncmp_P(rx_buff,cops_p,6)) // пришел +COPS:
        {
          if (!strncmp_P(rx_buff,cops_ans,12)) op_present=1; // пришел +COPS: 0,0,"
          else if (!strncmp_P(rx_buff,cops_no_ans,8)) op_present=0; // пришел +COPS: 0
          st++;
        }
        mess=0;
      } 
    }break;
  case 12:
    {
      if (mess) 
      {
        if (!strncmp_P(rx_buff,ok,2) && op_present==1) st++;  
        else {st=10; Delay_s(1); timeout=200;} 
        mess=0;
      }      
    }break;
  case 13:{ end=1; }break;
  }// switch
  if (!mess) Parser();  
  if (n_pwr_dn) { n_pwr_dn=0;   st=0;  Hardware_PWR_ON(); }
  if (timeout==0) { end=1; return STATUS_TIMEOUT; }
  if (err_cnt>10) return STATUS_ERROR;

}
return STATUS_OK;
}
//------------------------------------------------------------------------------
StatusTypeDef SIM800_First_Setup(void)
{
  char end=0, st=0, err_cnt=0;
  Hardware_PWR_ON();
  while(!end)
  {
    __watchdog_reset();  
   switch (st)
   {
   case 0:{ put_message_modem (at); st++; timeout=500; }break; // вначале большой таймаут (5 сек), т.к. долго включается и т.п.
   case 1:{ if(mess){ if(!strncmp_P(rx_buff,ok,2)) st++; else st--;  mess=0;} }break;
   case 2:{ put_message_modem(ipr_115200); st++; timeout=200; }break;
   case 3:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break;
   case 4:{ put_message_modem(ciurc); st++; timeout=200; }break;
   case 5:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break; 
   case 6:{ put_message_modem(atw); st++; timeout=200; }break;
   case 7:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break;   
   case 8:{ end=1; }break;
   
   }
   if (!mess) Parser();
   if (n_pwr_dn) { st=0;  Hardware_PWR_ON();  }
   if (timeout==0) { end=1;  return STATUS_TIMEOUT; }
   if (err_cnt>10) return STATUS_ERROR;
  }
return STATUS_OK;
}
//------------------------------------------------------------------------------
StatusTypeDef SIM800_Init(void)
{
  /*  Возможно стоит добавить:
  AT+CSCB - запрет широковещательных сообщений
  */
  char st=0, end=0, err_cnt=0;
  while(!end)
  {  
    __watchdog_reset(); 
    switch (st)
    {
    case 0:{ put_message_modem (_atl3); st++; timeout=200; }break; 
    case 1:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break;
    case 2:{ put_message_modem (atm1); st++; timeout=200; }break;
    case 3:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break;
    case 4:{ put_message_modem (_atclvl); st++; timeout=200; }break;
    case 5:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break;  
    case 6:{ put_message_modem (mic_lvl); st++; timeout=200; }break;
    case 7:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break; 
    case 8:{ put_message_modem (set_gsm); st++; timeout=200; }break;
    case 9:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break; 
    case 10:{ put_message_modem (_atcpbs); st++; timeout=200; }break;
    case 11:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break; 
    case 12:{ put_message_modem (colp); st++; timeout=200; }break;
    case 13:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break;     
    case 14:{ put_message_modem (sms_del_all); st++; timeout=200; }break; // сотрем все смс
    case 15:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break; 
    case 16:{ put_message_modem(sms_storage_sm); st++; timeout=300; } // может оно и не надо
    case 17:{ if (mess) { if (!strncmp_P(rx_buff,cpms,6)) {mess=0; st++; } else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} } } }break;
    case 18:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st-=2; err_cnt++;} }  mess=0;} }break;
    case 19:{ put_message_modem (sms_format_text); st++; timeout=200; }break; // выполняем 1 раз при настройке
    case 20:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break;     
    case 21:{ put_message_modem (dtmf_en); st++; timeout=200; }break;
    case 22:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} }  mess=0;} }break; 
    case 23:{ put_message_modem(sms_mess_ind); st++; timeout=200; }break; // реакция на смс
    case 24:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st-=2; err_cnt++;} }  mess=0;} }break; 
    case 25:{ put_message_modem(clip); st++; timeout=200; }break; // реакция на смс
    case 26:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st-=2; err_cnt++;} }  mess=0;} }break;     
    case 27:{ end=1; }break;   
    }
    if (!mess) Parser();
    if (timeout==0) { end=1;  return STATUS_TIMEOUT; }
    if (err_cnt>10) return STATUS_ERROR;
    
    
  }
return STATUS_OK;
}
//----------------------------------------------------------------------------------
StatusTypeDef SSB_Config(void) // добавить выдергивание номеров 1..5, а лучше чтоб номера дергало только при отправке смс и само переводило в юникод
{
 char st=0;
 char err_cnt=0;
 char empty_sim=0;
 char end=0;
 char i=0;
 mess=0;
 
 while (!end)
 {
  __watchdog_reset();
  switch (st)
  {
  case 0:{ put_message_modem (check_parol); st++; timeout=200; }break; 
  case 1:
    { 
      if (mess)
      {      
        if (!strncmp_P(rx_buff,parol_ch,strlen_P(parol_ch))) { empty_sim=0; st++; } // на симке есть ячейка PAROL, перейдем к st=2 (считаем пароль и конфиг)
        else if (!strncmp_P(rx_buff,ok,2)) { empty_sim=1; st+=3;}                // если нет записи, то модем ответит только ОК и надо перескочить на st=4
        else if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; } 
        mess=0;
//        delay_ms(10);
      }
    }break;
  case 2: 
    { 
     p[0]=rx_buff[17]; p[1]=rx_buff[18]; p[2]=rx_buff[19]; p[3]=rx_buff[20];
     wait_off = rx_buff[11]-0x30; // задержка включения тревоги  
     wait_on = rx_buff[13]-0x30; // задержка включения режима охраны
     retry_count = rx_buff[15]-0x30; // количество попыток
     wait_off = wait_off+0xa0;  // а0 - признак задержки тревоги для славы
     putchar_debug(wait_on); // отправили сопроцессору задержку на постановку на охрану
     Delay_ms(200);//???
     putchar_debug(wait_off); // отправили сопроцессору задержку на снятие с охраны
     st++;     
    }break;
  case 3: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st-=2; err_cnt++;} }  mess=0;} }break;
  case 4: { if(empty_sim==1) st++; else st+=2; }break;
  case 5: { Write_SIM_default(); st++; }
 
  case 6: { put_message_modem(check_config); st++; timeout=200; }break;
  case 7:
    { 
      if (mess)
      { 
        if (!strncmp_P(rx_buff,cbpr,6)) // ответ от SIM
        {
          pa[1] = rx_buff[11] ;  // реле срабатывает при срабатывании охраны
          pa[2] = rx_buff[12] ;  // дозвон при пропадании питания
          pa[3] = rx_buff[13];   // дозвон при постановке / снятии с кнопки
          pa[4] = rx_buff[14];   // автопостановка на охрану при срабатывании канала
          pa[5] = rx_buff[15];   // работа с датчиком температуры
          pa[6] = rx_buff[16];   // дозвон по достижении уставки   
          pa[9] = rx_buff[19];   // если 0x30 - не спрашиваем пароль, 0x31 - не спрашиваем у мастера, все остальное - всегда спрашиваем 
          pa[10] = rx_buff[20];   // если 0x31, то говорим вводимые цифры голосом, любое другое - молчим

          st++; 
        }
        else if (!strncmp_P(rx_buff,error,5)) {st--; err_cnt++;} 
        mess=0; 
        delay_ms(10);
      }
    }break;
  case 8: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) {st-=2; err_cnt++;} }  mess=0;} }break;
  case 9: { put_message_modem(check_master_num); st++; timeout=300; }break;
  case 10: 
    { 
     if (mess)
     {      
       if (!strncmp_P(rx_buff,cbpr,6))  
       {
         Get_Number(rx_buff,mass_master_number);
         st++; 
       }
       else if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++;}  
       mess=0;
     }
    }break;
  case 11: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) { st-=2; err_cnt++; } }  mess=0;} }break;  
  case 12: 
    {
      switch(i)
      {
      case 0:{ put_message_modem(check_number1); }break; 
      case 1:{ put_message_modem(check_number2); }break; 
      case 2:{ put_message_modem(check_number3); }break; 
      case 3:{ put_message_modem(check_number4); }break; 
      case 4:{ put_message_modem(check_number5); }break;  
      }
       st++; timeout=300; 
    }break;
  case 13: 
    {
      if (mess)
      {
       if (!strncmp_P(rx_buff,cbpr,6)) 
       {
        if (rx_buff[10]=='*') cm[i]=0; else cm[i]=1; // здесь начало номера точно известно и смотрим только 1-й символ
        st++;
       }
       else if (!strncmp_P(rx_buff,error,2)) { st--; err_cnt++; } 
       mess=0;
      }
    }break;
  case 14: 
    { 
      if (mess) 
      { if (!strncmp_P(rx_buff,ok,2)) 
         { if(i==4) st++; else { i++; st-=2; } } // читаем 5 номеров
        else  if (!strncmp_P(rx_buff,error,5)) { st-=2; err_cnt++; }  
       mess=0;
      } 
     }break;   
  case 15: { end=1;  }
  
  }//switch
  if (!mess) Parser();
  if (timeout==0) return STATUS_TIMEOUT;
  if (err_cnt>10) return STATUS_ERROR;
 }//while
 return STATUS_OK;
}
//----------------------------------------------------------------------------------
StatusTypeDef Write_SIM_default(void)
{
  char st=0, end=0;
  mess=0;
  while(!end)
  {
    __watchdog_reset();
    switch (st)
    {
    case 0: { put_message_modem (set_gsm); st++; timeout=100; }break; // пишем настройки SIM по умолчанию
    case 1: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 2: { put_message_modem (set_master); st++; timeout=100; }break;
    case 3: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 4: { put_message_modem (set_parol); st++; timeout=100; }break;
    case 5: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 6: { put_message_modem (set_config); st++; timeout=100; }break;
    case 7: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 8: { put_message_modem (set_number1); st++; timeout=100; }break;
    case 9: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 10: { put_message_modem (set_number2); st++; timeout=100; }break;
    case 11: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 12: { put_message_modem (set_number3); st++; timeout=100; }break;
    case 13: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 14: { put_message_modem (set_number4); st++; timeout=100; }break;
    case 15: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break;
    case 16: { put_message_modem (set_number5); st++; timeout=100; }break;
    case 17: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) st--; }  mess=0;} }break; // закончили новые настройки   
    case 18: { end=1; }
    
    }
    if (!mess) Parser();
    if (timeout==0) return STATUS_TIMEOUT;
  }
return STATUS_OK;  
}
//------------------------------------------------------------------------------
// number - номер на который шлем (0-master, 1..5), text - текст (русский), type - тип смс
StatusTypeDef SIM800_SMS_Send(unsigned char number, char *text, char type) 
{
 char end=0;
 char st=0;
 char err_cnt=0; 
 char err_flag=0;

 while(!end)
 { 
  __watchdog_reset(); 
  switch(st)
  {
  case 0:
    {
     switch (number)
     {
     case 0:{ put_message_modem (check_master_num); }break; 
     case 1:{ put_message_modem (check_number1); }break; 
     case 2:{ put_message_modem (check_number2); }break; 
     case 3:{ put_message_modem (check_number3); }break; 
     case 4:{ put_message_modem (check_number4); }break; 
     case 5:{ put_message_modem (check_number5); }break; 
     }
     timeout=300; st++;
    }break; 
  case 1:
    {
     if (mess)
      {
       if (!strncmp_P(rx_buff,cbpr,6)) 
       {
         Get_Number(rx_buff,mass_number);
         st++;
       }
       mess=0;
      }
    }break;
  case 2:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) { st-=2; err_cnt++; } }  mess=0;} }break;
  case 3:{ put_message_modem (set_unicod); st++; timeout=200; }break;
  case 4:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; } }  mess=0;} }break; 
  case 5:{ if(type) put_message_modem (sms_parameters_soxr); else put_message_modem (sms_parameters); st++; timeout=200; }
  case 6:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; } }  mess=0;} }break; 
  case 7:
    {       
     put_message_modem (sms_number); // AT+CMGS="
     char i=0;
     while(mass_number[i]!=0) // пошлем считанный номер
     {
      putchar_modem(0x30); putchar_modem(0x30); // 00
      if (mass_number[i]=='+') { putchar_modem(0x32); putchar_modem(0x42); } //2B      
      else { putchar_modem(0x33); putchar_modem(mass_number[i]); }       
      i++;
     }
     putchar_modem(0x22); putchar_modem(0x0D); // "/r
     Delay_ms(10);
     st++; 
     timeout=200;
    }break;// теперь надо поймать ">" 
  case 8:{ if (!strncmp(rx_buff,"> ",2)){ st++;  rx_cnt=0; mess=0; } }break; // ждем >
  case 9:
    { 
      
      char aaa[5]; u16 tmp; // aaa[5] вместо aaa[4] т.к. надо учесть конец строки, иначе глюки с памятью вылезают
      while(*text)
      {
       if (*text>=192) tmp=*text+0x350; else tmp=*text; // со 192 начинаются русские символы, +0x350 - переводим в юникод
       sprintf(aaa,"%4.4X",tmp);
       put_message_modem_RAM(aaa);
       *text++;
      }
      //put_message_UCS2(text); // не работает, бяда с указателями...
      putchar_modem(0x1A);  // Ctrl-Z
      Delay_ms(10);
      st++;
      timeout=6000; // 40 s
      mess=0; //??    
    }break;
  case 10:
    { //должно придти +CMGS:
      if (mess) 
       { 
         if (!strncmp(rx_buff,"+CMGS:",6)) st++;
         if (!strncmp_P(rx_buff,error,5) || !strncmp(rx_buff,"+CME ERROR:",11)) {mess=0; err_flag=1; st=12; } // может придти ERROR или +CME ERROR: выход с ошибкой
         mess=0;
       }  
    }break;
  case 11:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; } }  mess=0;} }break; 
  case 12:{ put_message_modem (set_gsm); st++; timeout=200; }break; // вернем кодировку GSM
  case 13:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; } }  mess=0;} }break;
  case 14:{ end=1; }break;
  }
  if (!mess) Parser();
  if(timeout==0) { while(1); /*return STATUS_TIMEOUT;*/ } // по таймауту циклимся 
  if (err_cnt>10) return STATUS_ERROR;  
 }//while
 if (err_flag) return STATUS_ERROR;
 else return STATUS_OK; 
}
//------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
StatusTypeDef SIM800_SMS_Read(void)
{
  char st=0, end=0;
  char err_cnt=0;
  char *s;
  char sms_index;
  char corr_sms=0; // флаг что смс наша
   
  // сюда переходим по +CMTI: и в sms_buff строка вида +CMTI: "SM",1 
  s=sms_buff;
  while(*s && *s++!=','); // ищем запятую
  sms_index=*s;                                  // символ после запятой - sms_index  (1..9) /индекс sms может быть только 1..9, т.к. все смс мы постоянно стираем (99% это будет 1, но возможно все :) )
  if (sms_index<0x31 || sms_index>0x39) st=20; // если индекс не 1..9, то стираем все - заглушка
  
  
  while(!end)
  {
    __watchdog_reset();
    switch(st)
    {
    case 0:{ put_message_modem (set_gsm); st++; timeout=200; }break;
    case 1:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; }   mess=0;} }break; 

    case 2:{ put_message_modem(sms_read); putchar_modem(sms_index); putchar_modem(0x0D); Delay_ms(10); st++; timeout=300;}break;
    case 3:
      { 
        if (mess)
        {
          if (!strncmp_P(rx_buff,cmgr,6)) st=10; // тут можно добавить выдирание номера отправителя и проверку на белый список и т.п.
          else if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; }; 
          mess=0;
        }
      }break;
    case 10:  // тут в буфере имеем текст смс
      { 
        if (mess)
        {
          if(rx_buff[0]=='P' || rx_buff[0]=='p') { strcpy(sms_buff,rx_buff); corr_sms=1;  } // в sms_buff копируем текст для дальнейшего разбора
          else  corr_sms=0;  // смс начинается не с P (p) - стираем его нахер 
          mess=0;     
          st++;
        }
      }break;
    case 11:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else if (!strncmp_P(rx_buff,error,5)) { st-=2; err_cnt++; }   mess=0;} }break;  
    case 12:{ if (corr_sms) st++; else st=20; }break;
    case 13:{ if(p[0]==sms_buff[1] && p[1]==sms_buff[2] && p[2]==sms_buff[3] && p[3]==sms_buff[4]) st++; else st=20; }break; // проверяем пароль, если не подходит - стираем смс
    case 14:{ Execute_SMS_Command(sms_buff); st=20; }break; // выполняем команду и стираем смс
    
    case 20:{ put_message_modem (sms_del_all); st++; timeout=200; }break;
    case 21:{ if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; };   mess=0;} }break;
    case 22:{ end=1; }break; 

    }//switch
    if (!mess) Parser();
    if (timeout==0) return STATUS_TIMEOUT;
    if (err_cnt>10) return STATUS_ERROR;
  }//while
return STATUS_OK;
}
//----------------------------------------------------------------------------------
void Execute_SMS_Command(char *text)
{
  
  memmove(text,text+5,strlen(text));  // сдвигаем строку на 5 символов влево - в теории не должно заглючить, непонятно тольно почему копируется strlen(text) символов 
  
  sprintf_P(answer,err_cmd); // "Ошибка в команде" - если не будет совпадений по командам - отправится эта СМС
  
  if (!strncmp_P(text,cmd_11,3) || !strncmp_P(text,RELE1_ON,8) || !strncmp_P(text,rele1_on,8)) 
    { rele1=1; putchar_debug(0x11); relay1_status=1; sprintf_P(answer,rele_on,1); }     //  Реле 1 включено
  else if (!strncmp_P(text,cmd_10,3) || !strncmp_P(text,RELE1_OFF,9) || !strncmp_P(text,rele1_off,9)) 
    { rele1=0; putchar_debug(0x10); relay1_status=0; sprintf_P(answer,rele_off,1); }    //  Реле 1 выключено
  else if (!strncmp_P(text,cmd_21,3) || !strncmp_P(text,RELE2_ON,8) || !strncmp_P(text,rele2_on,8)) 
      { 
        rele2=1; putchar_debug(0x21); relay2_status=1; sprintf_P(answer,rele_on,2);
        if (pa[6]==0x31 && !vv && relay_zapret==0) { relay_zapret=1; strcat_P(answer,t_reg_off); } //если вкл т-рег, то отключим его и допишем строчку что он отключен
      }
  else if (!strncmp_P(text,cmd_20,3) || !strncmp_P(text,RELE2_OFF,9) || !strncmp_P(text,rele2_off,9)) 
      {
        rele2=0; putchar_debug(0x20); relay2_status=0; sprintf_P(answer,rele_off,2); 
        if (pa[6]==0x31 && !vv && relay_zapret==0) { relay_zapret=1; strcat_P(answer,t_reg_off); } //если вкл т-рег, то отключим его и допишем строчку что он отключен
      }
  else if (!strncmp_P(text,cmd_31,3) || !strncmp_P(text,RELE3_ON,8) || !strncmp_P(text,rele3_on,8)) 
    { rele3=1; putchar_debug(0x31); relay3_status=1; sprintf_P(answer,rele_on,3); }
  else if (!strncmp_P(text,cmd_30,3) || !strncmp_P(text,RELE3_OFF,9) || !strncmp_P(text,rele3_off,9)) 
    { rele3=0; putchar_debug(0x30); relay3_status=0; sprintf_P(answer,rele_off,3); }
  else if (!strncmp_P(text,cmd_41,3) || !strncmp_P(text,RELE4_ON,8) || !strncmp_P(text,rele4_on,8)) 
    { rele4=1; relay4_status=1; sprintf_P(answer,rele_on,4); }
  else if (!strncmp_P(text,cmd_40,3) || !strncmp_P(text,RELE4_OFF,9) || !strncmp_P(text,rele4_off,9)) 
    { rele4=0; relay4_status=0; sprintf_P(answer,rele_off,4); }
  else if (!strncmp_P(text,cmd_51,3) || !strncmp_P(text,RELE5_ON,8) || !strncmp_P(text,rele5_on,8)) 
    { rele5=1; relay5_status=1; sprintf_P(answer,rele_on,5); }
  else if (!strncmp_P(text,cmd_50,3) || !strncmp_P(text,RELE5_OFF,9) || !strncmp_P(text,rele5_off,9)) 
    { rele5=0; relay5_status=0; sprintf_P(answer,rele_off,5); }
  else if (!strncmp_P(text,cmd_61,3) || !strncmp_P(text,RELE6_ON,8) || !strncmp_P(text,rele6_on,8)) 
    { rele6=1; relay6_status=1; sprintf_P(answer,rele_on,6); }
  else if (!strncmp_P(text,cmd_60,3) || !strncmp_P(text,RELE6_OFF,9) || !strncmp_P(text,rele6_off,9)) 
    { rele6=0; relay6_status=0; sprintf_P(answer,rele_off,6); }
  else if (!strncmp_P(text,cmd_71,3) || !strncmp_P(text,RELE7_ON,8) || !strncmp_P(text,rele7_on,8)) 
    { rele7=1; relay7_status=1; sprintf_P(answer,rele_on,7); }
  else if (!strncmp_P(text,cmd_70,3) || !strncmp_P(text,RELE7_OFF,9) || !strncmp_P(text,rele7_off,9)) 
    { rele7=0; relay7_status=0; sprintf_P(answer,rele_off,7); }
  
  else if (!strncmp_P(text,cmd_81,3) || !strncmp_P(text,DIAL_ON,7) || !strncmp_P(text,dial_on,7)) 
    { bit_temp_zapret=0; sprintf_P(answer,ctrl_on); } // Контроль включен
  else if (!strncmp_P(text,cmd_80,3) || !strncmp_P(text,DIAL_OFF,8) || !strncmp_P(text,dial_off,8)) 
    { bit_temp_zapret=1; sprintf_P(answer,ctrl_off); } // Контроль выключен
  
  else if (!strncmp_P(text,cmd_821,4) || !strncmp_P(text,TERM_ON,7) || !strncmp_P(text,term_on,7)) // можно вкл/выкл если только включено в симке
   { 
     if (pa[6]==0x31) { if(relay_zapret==1) relay_zapret=0; sprintf_P(answer,t_reg_on); }
     else sprintf_P(answer,t_reg_off_sim);   
   }
  else if (!strncmp_P(text,cmd_820,4) || !strncmp_P(text,TERM_OFF,8) || !strncmp_P(text,term_off,8))  // можно вкл/выкл если только включено в симке
    { 
     if (pa[6]==0x31) { if(relay_zapret==0) relay_zapret=1; sprintf_P(answer,t_reg_off); }
     else sprintf_P(answer,t_reg_off_sim);
    }
  
  
  else if (!strncmp_P(text,cmd_91,3) || !strncmp_P(text,SEC_ON,6) || !strncmp_P(text,sec_on,6)) 
    { putchar_debug(0x91); sec_stat = 1; sprintf_P(answer,security_on); } // Охрана вкл
  else if (!strncmp_P(text,cmd_90,3) || !strncmp_P(text,SEC_OFF,7) || !strncmp_P(text,sec_off,7)) 
    { putchar_debug(0x90); sec_stat = 0; sprintf_P(answer,security_off); } // Охрана выкл

  //--------------------------------------      

  else if (!strncmp_P(text,cmd_00,3) || !strncmp_P(text,STATUS,6) || !strncmp_P(text,status,6)) 
    sprintf_P(answer,all_temp,temperature,ust_m,hyst,relay1_status,relay2_status,relay3_status);
  

  //--------------------------------------
  else if (text[0]=='#' || !strncmp_P(text,SET,3) || !strncmp_P(text,set,3)) // #025
  {
    char temp[4]={0,0,0,0};  // 4 т.к. строка завершается \0
    char tmp_ust=0, err_ust=0;
    char k=0;
    if (text[0]=='#') k=1; else k=3; // #123 и SET123 - уставка начинается или с 2 или с 4 символа
    for(int i = 0; i < 3; ++i)
    {
      if (text[i+k]>=0x30 && text[i+k]<=0x39) temp[i]=text[i+k]; // проверяем что в уставке только цифры
      else err_ust=1;
    }
    if (!err_ust) // если уставка нормальная
    {
      tmp_ust=atoi(temp); // декодируем уставку в число
      if (tmp_ust>120) tmp_ust=120; // ограничение уставки 120 град
      if (tmp_ust<1) tmp_ust=1; 
      ust_m=tmp_ust; ust=tmp_ust; sprintf_P(answer,ustavka,ust_m); 
    }//err_ust
  }//#
  //---------------------------------------
  else if (text[0]=='h' || text[0]=='H' || !strncmp_P(text,HYS,3) || !strncmp_P(text,hys,3)) // гистерезис  h05   H30
  {
    char temp[3]={0,0,0};  // 3 т.к. строка завершается \0
    char tmp_h=0, err_h=0; 
    char k=0;
    if (text[0]=='h'|| text[0]=='H') k=1; else k=3; // H12 и HYS12 - уставка начинается или с 2 или с 4 символа
    for(int i = 0; i < 2; ++i)
    {
      if (text[i+k]>=0x30 && text[i+k]<=0x39) temp[i]=text[i+k]; // проверяем что в уставке только цифры
      else err_h=1;
    }
    if (!err_h) // если уставка нормальная
    {
      tmp_h=atoi(temp); // декодируем уставку в число
      if (tmp_h>30)  tmp_h=30;  // ограничение уставки 120 град
      if (tmp_h<2)  tmp_h=2; 
      hyst=tmp_h; hyst_m=hyst; sprintf_P(answer,hysterezis,hyst); 
    }   
  }
  //--------------------------------------
  else if(!strncmp_P(text,master,6) || !strncmp_P(text,master_sm,6)) 
   {
    memmove(text,text+6,strlen(text)); 
    if (SIM800_Write_to_SIM(text,0,"MASTER")==STATUS_OK) sprintf_P(answer,mas_ok); // если все норм
   } 
  else if(!strncmp_P(text,number1,7) || !strncmp_P(text,number1_sm,7)) { memmove(text,text+7,strlen(text)); if (SIM800_Write_to_SIM(text,1,"NUMBER1")==STATUS_OK) sprintf_P(answer,n_ok,1); } 
  else if(!strncmp_P(text,number2,7) || !strncmp_P(text,number2_sm,7)) { memmove(text,text+7,strlen(text)); if (SIM800_Write_to_SIM(text,1,"NUMBER2")==STATUS_OK) sprintf_P(answer,n_ok,2); }  
  else if(!strncmp_P(text,number3,7) || !strncmp_P(text,number3_sm,7)) { memmove(text,text+7,strlen(text)); if (SIM800_Write_to_SIM(text,1,"NUMBER3")==STATUS_OK) sprintf_P(answer,n_ok,3); }
  else if(!strncmp_P(text,number4,7) || !strncmp_P(text,number4_sm,7)) { memmove(text,text+7,strlen(text)); if (SIM800_Write_to_SIM(text,1,"NUMBER4")==STATUS_OK) sprintf_P(answer,n_ok,4); }
  else if(!strncmp_P(text,number5,7) || !strncmp_P(text,number5_sm,7)) { memmove(text,text+7,strlen(text)); if (SIM800_Write_to_SIM(text,1,"NUMBER5")==STATUS_OK) sprintf_P(answer,n_ok,5); }
  else if(!strncmp_P(text,config,6) || !strncmp_P(text,config_sm,6)) 
   { 
     memmove(text,text+6,strlen(text)); 
     if (SIM800_Write_to_SIM(text,2,"CONFIG")==STATUS_OK) 
     { // *000011****
      pa[1] = text[1] ;  // реле срабатывает при срабатывании охраны
      pa[2] = text[2] ;  // дозвон при пропадании питания
      pa[3] = text[3];   // дозвон при постановке / снятии с кнопки
      pa[4] = text[4];   // автопостановка на охрану при срабатывании канала
      pa[5] = text[5];   // работа с датчиком температуры
      pa[6] = text[6];   // дозвон по достижении уставки
      pa[9] = text[9];   // спрашивание пароля
      pa[10] = text[10];  // проговаривание цифр голосом     
      sprintf_P(answer,cfg_ok); 
     }
   }
  
  else if(!strncmp_P(text,parol,5) || !strncmp_P(text,parol_sm,5)) 
   { //  *1*1*1*1234
     memmove(text,text+5,strlen(text)); 
     if (SIM800_Write_to_SIM(text,3,"PAROL")==STATUS_OK) 
     {
       p[0]=text[7]; p[1]=text[8]; p[2]=text[9]; p[3]=text[10];
       wait_off = text[1]-0x30; // задержка включения тревоги  
       wait_on = text[3]-0x30; // задержка включения режима охраны
       retry_count = text[5]-0x30; // количество попыток
       wait_off = wait_off+0xa0;  // а0 - признак задержки тревоги для славы
       putchar_debug(wait_on); // отправили сопроцессору задержку на постановку на охрану
       Delay_ms(200);
       putchar_debug(wait_off); // отправили сопроцессору задержку на снятие с охраны
       sprintf_P(answer,pass_ok); 
     }
   }
  
  else if(!strncmp_P(text,rst,5)) 
   { 
     ust_m=25; ust=ust_m; hyst_m=2; hyst=hyst_m; bit_temp_zapret=0; relay_zapret=0;
     if(Write_SIM_default()==STATUS_OK) 
     { 
      SSB_Config(); // здесь после сброса надо перечитать все
      sprintf_P(answer,default_settings);  
     }
   } // сбросим все
  
  else if (!strncmp_P(text,restart,7)) while(1);  // экстренная команда, вдруг глючит, а смс пробъется. ответной смс тут не будет, а прибор перезапустится
 
  SIM800_SMS_Send(0,answer,1);  // отошлем ответ мастеру
   

}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
StatusTypeDef SIM800_Write_to_SIM(char *text, char cell, char *name) 
{
  char end=0, st=0, corr_cell=0;
  char err_cnt=0;
  char n[max_numb_len]; // записываемый в ячейку номер
  char cnt=0;
  char write_cell[60]; // AT+CPBW=5,"",129,"NUMBER2" - (26 символов + max_numb_len) 60 хватит
  // делаем защиту от дебилов - проверяем корректность пришедшей смс в зависимости от того, какую ячейку пишем
  switch (cell)
  {
  case 0: // проверяем Master
    {
     while(*text)
     {
      if ((*text>=0x30 || *text<=0x39 || *text=='+') && *text!=0) { n[cnt++]=*text; corr_cell=1; } else corr_cell=0;
      *text++;
     }
     if (cnt<=(max_numb_len-1)) n[cnt]=0; else n[max_numb_len-1]=0; // завершим строку (с защитой от переполнения) 
     sprintf_P(write_cell,write_number,1,n,name);
    }break; 
  case 1: // проверяем Number1..5
    {
      if (text[0]=='*') // если 1-я * - то пишем только её
      {
       n[0]='*'; n[1]=0x00; corr_cell=1;
      }
      else // или пишем нормальный номер, начинающийся с 8 или +
      {
       while(*text)
       {
        if ((*text>=0x30 || *text<=0x39 || *text=='+') && *text!=0) { n[cnt++]=*text; corr_cell=1; } else corr_cell=0;
        *text++;
       }
       if (cnt<=(max_numb_len-1)) n[cnt]=0; else n[max_numb_len-1]=0; // завершим строку (с защитой от переполнения)
      }
     switch(name[6])
     {
     case '1':{sprintf_P(write_cell,write_number,4,n,name);}break;
     case '2':{sprintf_P(write_cell,write_number,5,n,name);}break;
     case '3':{sprintf_P(write_cell,write_number,6,n,name);}break;
     case '4':{sprintf_P(write_cell,write_number,7,n,name);}break;
     case '5':{sprintf_P(write_cell,write_number,8,n,name);}break;
     }
    }break;
  case 2: // проверяем Config
    {
     for(int i = 0; i < 11; ++i)
      {
        if ((*text>=0x30 || *text<=0x39 || *text=='*') && *text!=0) { n[i]=*text; corr_cell=1; } else corr_cell=0;
       *text++;
      }
      n[11]=0;  
     sprintf_P(write_cell,write_number,3,n,name);
     if ( n[0]!='*' || n[7]!='*' || n[8]!='*' || 
          (n[9]!='*' && n[9]!='0' && n[9]!='1') || 
            (n[10]!='*' && n[10]!='1') || 
             (n[1]!='0' && n[1]!='1') || (n[2]!='0' && n[2]!='1') || (n[3]!='0' && n[3]!='1') ||
               (n[4]!='0' && n[4]!='1') || (n[5]!='0' && n[5]!='1') || (n[6]!='0' && n[6]!='1') ) corr_cell=0; // // проверили корректность ячейки CONFIG
    }break;
  case 3: // проверяем Parol
    {
     for(int i = 0; i < 11; ++i)
      {
        if ((*text>=0x30 || *text<=0x39 || *text=='*') && *text!=0) { n[i]=*text; corr_cell=1; } else corr_cell=0;
       *text++;
      }
      n[11]=0;   
      sprintf_P(write_cell,write_number,2,n,name);  
     if ( n[0]!='*' || n[2]!='*' || n[4]!='*' || n[6]!='*' || 
          (n[1]<0x31 && n[1]>0x39) || (n[3]<0x31 && n[3]>0x39) || (n[5]<0x31 && n[5]>0x39) ||
            (n[7]<0x30 && n[7]>0x39) || (n[8]<0x30 && n[8]>0x39) || (n[9]<0x30 && n[9]>0x39) || (n[10]<0x30 && n[10]>0x39) ) corr_cell=0; // проверили корректность ячейки PAROL
                                                                                                                 
    }break;
  }//switch
  
if (corr_cell)
{
  while (!end)
  {
    switch(st) //AT+CSCS="GSM" уже сделано при чтении смс
    {
    case 0: { put_message_modem_RAM(write_cell); st++; timeout=100;}break;
    case 1: { if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { if (!strncmp_P(rx_buff,error,5)) { st--; err_cnt++; } }  mess=0;} }break;
    case 2: { if (cell==0) sprintf(mass_master_number,n); end=1; }break; // если писали мастер номер - то скопируем его в массив и выйдем
    }
    if (!mess) Parser();
    if (timeout==0) return STATUS_TIMEOUT;
    if (err_cnt>10) return STATUS_ERROR;
  }
  return STATUS_OK;
} 
return STATUS_ERROR;
}
//------------------------------------------------------------------------------
void Check_Incoming_Call(void)
{
  char st=0, end=0;
  char attempt=3; // кол-во попыток ввода пароля
 
  
  overflow=15*100; // 15 сек
  dtmf_cnt=0;
  while(!end)
  {
    __watchdog_reset();
    switch(st)
    {
    case 0:{ put_message_modem (ata); st++; }break; 
    case 1:{ if (mess) {  if (!strncmp_P(rx_buff,ok,2)) st++; else {} mess=0; } }break;
    case 2:
      { 
        Delay_ms(300); 
         if ((ring_master && pa[9]==0x31) || pa[9]==0x30) { gudok_ok(); st=6; }  // если звоним с мастер-номера и не надо пароль или просто пароль выключен
         else  { gudok(); st++; } 
      }break;
    case 3:
      {
        if(attempt!=0) {dtmf_cnt=0; st++; } else st=10; // если не кончились попытки ввода, то ждем пароля, иначе повесим трубку и выйдем
      }break;
    case 4:
      { 
        if(dtmf_cnt==4) st++; // overflow обновляется при каждом нажатии кнопки (приходе +DTMF) в парсере
        if (mess) { if (!strncmp_P(rx_buff,no_carrier,strlen_P(no_carrier))) end=1; mess=0;}
      }break;// ждем 4-х символов dtmf
    case 5:
      { 
        if (!strncmp(dtmf_buff,p,4)) { st++; Delay_ms(500); password(); Delay_ms(200); prinyat(); } // пароль совпал
        else 
         { 
           st=3; attempt--;
           Delay_ms(300); say_error(); // говорим ошибка
         }
      }break;
    case 6:{ Check_DTMF_Cmd(); end=1;  }break;  
    
    case 10:{ put_message_modem(hold);  st++; timeout=200; }break;
    case 11:
      { 
        if (mess) {  if (!strncmp_P(rx_buff,ok,2)) end=1; else {st--;} mess=0;} 
        if (timeout==0) end=1;
      }break;
    }//switch

    if (overflow==0 && st<10) st=10; 
    if (!mess) Parser();
  }
}

//------------------------------------------------------------------------------
//------------ обработка ДТМФ команд -------------------------------------------
void Check_DTMF_Cmd(void)
{
  char st=0, end=0;
  char max_count=3; // если забыть это и max_count=0, то будет нормально подглючивать из-за строчки  if(dtmf_cnt==max_count) st++; 
  while(!end)
  {
   __watchdog_reset();
   switch (st)
   {
   case 0:{ dtmf_cnt=0; dtmf_buff[0]=0; dtmf_buff[1]=0; max_count=3; overflow=15*100; st++; }break; 
   case 1:
     { 
       if (dtmf_buff[0]=='#' || (dtmf_buff[0]=='*' && dtmf_buff[1]=='8' && dtmf_buff[2]=='2')) max_count=4; // # или *82...
         else if (dtmf_buff[0]=='*') max_count=3;
          else dtmf_cnt=0; // если не команда - то сбрасываем dtmf буфер на начало
       if(dtmf_cnt==max_count) st++; 
       if (mess) { if (!strncmp_P(rx_buff,no_carrier,strlen_P(no_carrier))) end=1; mess=0; }
     }break;
   case 2: { Execute_DTMF_Cmd(); st=0; }break;
   case 10:{ put_message_modem(hold);  st++;}break;
   case 11:{ if (mess) {  if (!strncmp_P(rx_buff,ok,2)) end=1; else {} mess=0;} }break;
   }
   if (!mess) Parser();
   if (overflow==0 && st<10) st=10;
  }  
}
//------------------------------------------------------------------------------
void Execute_DTMF_Cmd(void /*char max_count*/)
{
     if (!strncmp_P(dtmf_buff,cmd_11,3)) { rele1=1; putchar_debug(0x11); Delay_ms(300);say_relay(); say_one(); say_relay_on();relay1_status=1; }
     else if (!strncmp_P(dtmf_buff,cmd_10,3)) { rele1=0; putchar_debug(0x10); Delay_ms(100);say_relay(); say_one(); say_relay_off();relay1_status=0; }
     else if (!strncmp_P(dtmf_buff,cmd_21,3)) // ручное вмешательство в работу реле 2 отключает терморегулятор (если он работает)
        { 
          rele2=1; putchar_debug(0x21); Delay_ms(300); say_relay(); say_two(); say_relay_on();relay2_status=1; 
          if (pa[6]==0x31 && !vv && relay_zapret==0) relay_zapret=1; termoregulator(); Delay_ms(300); say_off(); // если в сим включен терморег., датчик не в аварии и разрешена работа терморег - отключим его.
        }
     else if (!strncmp_P(dtmf_buff,cmd_20,3)) 
        { 
          rele2=0; putchar_debug(0x20); Delay_ms(100);say_relay(); say_two(); say_relay_off();relay2_status=0; 
          if (pa[6]==0x31 && !vv && relay_zapret==0) relay_zapret=1; termoregulator(); Delay_ms(300); say_off(); // если в сим включен терморег., датчик не в аварии и разрешена работа терморег - отключим его.
        }
     else if (!strncmp_P(dtmf_buff,cmd_31,3)) { rele3=1; putchar_debug(0x31); Delay_ms(300);say_relay(); say_three(); say_relay_on();relay3_status=1; }
     else if (!strncmp_P(dtmf_buff,cmd_30,3)) { rele3=0; putchar_debug(0x30); Delay_ms(100);say_relay(); say_three(); say_relay_off();relay3_status=0; }
     else if (!strncmp_P(dtmf_buff,cmd_41,3)) { rele4=1;Delay_ms(300);say_relay(); say_four(); say_relay_on();relay4_status=1; }
     else if (!strncmp_P(dtmf_buff,cmd_40,3)) { rele4=0;Delay_ms(300);say_relay(); say_four(); say_relay_off();relay4_status=0; }
     else if (!strncmp_P(dtmf_buff,cmd_51,3)) { rele5=1;Delay_ms(300);say_relay(); say_five(); say_relay_on();relay5_status=1; }
     else if (!strncmp_P(dtmf_buff,cmd_50,3)) { rele5=0;Delay_ms(300);say_relay(); say_five(); say_relay_off();relay5_status=0; }
     else if (!strncmp_P(dtmf_buff,cmd_61,3)) { rele6=1;Delay_ms(300);say_relay(); say_six(); say_relay_on();relay6_status=1; }
     else if (!strncmp_P(dtmf_buff,cmd_60,3)) { rele6=0;Delay_ms(300);say_relay(); say_six(); say_relay_off();relay6_status=0; }
     else if (!strncmp_P(dtmf_buff,cmd_71,3)) { rele7=1;Delay_ms(300);say_relay(); say_seven(); say_relay_on();relay7_status=1; }
     else if (!strncmp_P(dtmf_buff,cmd_70,3)) { rele7=0;Delay_ms(300);say_relay(); say_seven(); say_relay_off();relay7_status=0; }
     
      // ключи
     else if (!strncmp_P(dtmf_buff,cmd_62,3)) { putchar_debug (0x60); gudok();} // 62 запись ключа 1
     else if (!strncmp_P(dtmf_buff,cmd_63,3)) { putchar_debug (0x61); gudok();} // 63 запись ключа 1
     else if (!strncmp_P(dtmf_buff,cmd_64,3)) { putchar_debug (0x62); gudok();} // 64 запись ключа 1
     else if (!strncmp_P(dtmf_buff,cmd_65,3)) { putchar_debug (0x63); gudok();} // 65 запись ключа 1
     else if (!strncmp_P(dtmf_buff,cmd_66,3)) { putchar_debug (0x64); gudok();} // 66 запись ключа 1
     else if (!strncmp_P(dtmf_buff,cmd_67,3)) { putchar_debug (0x65); gudok();} // 67 запись ключа 1


     
     else if (!strncmp_P(dtmf_buff,cmd_81,3)) { bit_temp_zapret=0; control_(); Delay_ms(300); say_on(); } // включили отзвон
     else if (!strncmp_P(dtmf_buff,cmd_80,3)) { bit_temp_zapret=1; control_(); Delay_ms(300); say_off(); } // выключили отзвон
     else if (!strncmp_P(dtmf_buff,cmd_821,4)) // можно вкл/выкл если только включено в симке
     { 
       if (pa[6]==0x31) { if(relay_zapret==1) relay_zapret=0; termoregulator(); Delay_ms(300); say_on(); }
       else zaprescheno();
     }
     else if (!strncmp_P(dtmf_buff,cmd_820,4)) // можно вкл/выкл если только включено в симке
     { 
       if (pa[6]==0x31) { if(relay_zapret==0) relay_zapret=1; termoregulator(); Delay_ms(300); say_off(); }
       else zaprescheno();
     }

     if (!strncmp_P(dtmf_buff,cmd_91,3))  //  постановка на охрану с телефона
      {
        putchar_debug(0x91);        
        sec_stat = 1;       //  это бит в один чтобы не звонить если стоит обзвон по установке с кнопки
        if (chan_stat1 ==1) {channel_();Delay_ms(200);say_one();Delay_ms(300);say_off();Delay_ms(400);}
        if (chan_stat2 ==1) {channel_();Delay_ms(200);say_two();Delay_ms(300);say_off();Delay_ms(400);}
        if (chan_stat3 ==1) {channel_();Delay_ms(200);say_three();Delay_ms(300);say_off();Delay_ms(400);}
        if (chan_stat4 ==1) {channel_();Delay_ms(200);say_four();Delay_ms(300);say_off();Delay_ms(400);}
        Delay_ms(400);
        say_regime();Delay_ms(300); say_on();
      } // включили охрану
     if (!strncmp_P(dtmf_buff,cmd_90,3))   //  снятие с  охраны с телефона
      {
        putchar_debug(0x90);
        sec_stat = 0; Delay_ms(400); say_regime(); Delay_ms(400); say_off();      
      } // выключили охрану   
     
         
     else if (!strncmp_P(dtmf_buff,cmd_00,3))  // проговариваем статус
        {         
          if (sec_stat==0)
           { say_regime();Delay_ms(400);say_off(); Delay_ms(400); }
          if (sec_stat==1)
           { say_regime();Delay_ms(400);say_on(); Delay_ms(400);
             if (chan_stat1 ==1) {channel_();Delay_ms(200);say_one();Delay_ms(300);razomknut();Delay_ms(400);}
             if (chan_stat2 ==1) {channel_();Delay_ms(200);say_two();Delay_ms(300);razomknut();Delay_ms(400);}
             if (chan_stat3 ==1) {channel_();Delay_ms(200);say_three();Delay_ms(300);razomknut();Delay_ms(400);}
             if (chan_stat4 ==1) {channel_();Delay_ms(200);say_four();Delay_ms(300);razomknut();Delay_ms(400);}
           }
          if (relay1_status==1) { say_relay(); say_one(); Delay_ms(200);say_relay_on(); Delay_ms(500); }
          if (relay2_status==1) { say_relay(); say_two(); Delay_ms(200);say_relay_on(); Delay_ms(500); }
          if (relay3_status==1) { say_relay(); say_three();Delay_ms(200); say_relay_on(); Delay_ms(500); }
          if (relay4_status==1) { say_relay(); say_four(); Delay_ms(200);say_relay_on(); Delay_ms(500); }
          if (relay5_status==1) { say_relay(); say_five(); Delay_ms(200);say_relay_on(); Delay_ms(500); }
          if (relay6_status==1) { say_relay(); say_six();Delay_ms(200); say_relay_on(); Delay_ms(500); }
          if (relay7_status==1) { say_relay(); say_seven();Delay_ms(200); say_relay_on(); Delay_ms(500); }
          if (pa[5]==0x31) // если в сим карте разрешено работать с датчиком температуры
           {
            if (!vv) // скажем текущую температуру
             { temperatura(); Delay_ms(400); say_numeric(temperature); Delay_ms(300); say_gradus(temperature); Delay_ms(300); }
            else { Delay_ms(200);kty_failed();Delay_ms(200); } 
           }
          if (pa[6]==0x31)  // если в симкарте включен терморегулятор
           {
            ustavka_();Delay_ms(400); say_numeric(ust_m); Delay_ms(300); say_gradus(ust_m); Delay_ms(300); // скажем уставку температуры
            gisterezis(); Delay_ms(400); say_numeric(hyst); Delay_ms(300); say_gradus(hyst); // скажем гистерезис
            Delay_ms(300); termoregulator(); Delay_ms(200);
            if (relay_zapret==1) say_off(); else say_on(); // терморегулятор on/off
            if (!bit_temp_zapret) {Delay_ms(200); control_(); Delay_ms(300); say_on();} else {control_(); Delay_ms(300); say_off();} //контроль on/off               
           }
          
          if (!zb) {power();norm();} else {power();power_off();}
          if (baterry_bit==0) { Delay_ms(200); baterry_low();}
          
          

          
        }// *00
     
     else if (dtmf_buff[0]=='#') // если 1-й символ #, то приняли уставку
     {
        char temp[3]; // int tmp_ust=0;
        char tmp_ust;
        temp[0]=dtmf_buff[1]; temp[1]=dtmf_buff[2]; temp[2]=dtmf_buff[3];
        tmp_ust=atoi(temp); // декодируем уставку в число
        if (tmp_ust>120) tmp_ust=120;
        ust_m=tmp_ust;
        ustavka_();Delay_ms(400); say_numeric(ust_m); Delay_ms(300); say_gradus(ust_m);        
     }


  
  overflow=15*100;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// message 0 - сработала сигналка, 1 - температура ниже уставки, 2 - температура выше уставки, 3 - авария датчика
// 4 - питание исчезло, 5 - питание появилось,
// 6 - режим охраны включен, 7 - режим охраны выключен
//                 кому звонить         что говорим            канал, если говорим что сработало
void call_master(unsigned char who, unsigned char message, unsigned char channel)
{
  unsigned char temp_retry; 
  char st=0, end=0;
  temp_retry=retry_count;
  if (temp_retry==0) temp_retry=1;
  if (temp_retry>9) temp_retry=9;
  
  while (!end)
  {
    __watchdog_reset();
    switch(st)
    {
    case 0:
      { 
        overflow=30*100; // 30 сек
        if (who==1) put_message_modem (dial_master);  // наберем мастеру
        if (who==2) put_message_modem (dial_number1); // наберем на следующий номер
        if (who==3) put_message_modem (dial_number2); // наберем на следующий номер
        if (who==4) put_message_modem (dial_number3); // наберем на следующий номер
        if (who==5) put_message_modem (dial_number4); // наберем на следующий номер
        if (who==6) put_message_modem (dial_number5); // наберем на следующий номер
        st++;
      }break; 
    case 1:
      { 
        if (mess)
        {
          if (!strncmp_P(rx_buff,collp,strlen_P(collp))) st++; // сняли трубку
          if (!strncmp_P(rx_buff,busy,strlen_P(busy)) || !strncmp_P(rx_buff,no_answer,strlen_P(no_answer))  // скинули звонок / нет ответа
              || !strncmp_P(rx_buff,no_carrier,strlen_P(no_carrier))) st=6; // еще no carrier может быть     
          mess=0;
        }      
      }break;
    case 2:
      { 
        if (mess) { if (!strncmp_P(rx_buff,ok,2)) st++; else { st=10; }  mess=0;} // после +COLP придет OK (если не ОК -> ATH)         
      }break;  
    case 3:
      {     // говорим  
        Delay_ms(300);
        ahtung(); Delay_ms(300);
        switch (message)
        {
        case 0:
          { 
            alarm_on(); Delay_ms(300); channel_(); Delay_ms(300); // внимание, сработала сигнализация, канал
            if (channel==1) say_one();
            else if (channel==2) say_two();
            else if (channel==3) say_three();
            else if (channel==4) say_four();
            Delay_s(2);
            ahtung();  Delay_ms(300); alarm_on(); Delay_ms(300); channel_(); Delay_ms(300); // внимание, сработала сигнализация, канал
            if (channel==1) say_one();
            else if (channel==2) say_two();
            else if (channel==3) say_three();
            else if (channel==4) say_four();
            Delay_ms(500);    
          }break;
        case 1:
          { 
            ustavka_();Delay_ms(400); say_numeric(ust_m); Delay_ms(300); say_gradus(ust_m); Delay_ms(300); 
            temperatura(); Delay_ms(400); say_numeric(temperature); Delay_ms(300); say_gradus(temperature);
          }break;// температура  ниже уставки
        case 2:{ temperatura(); Delay_ms(400); say_numeric(temperature); Delay_ms(300); say_gradus(temperature); Delay_ms(300);norm(); }break;//температура  выше уставки
        case 3:{ kty_failed(); }break;// скажем что датчик в аварии..
        case 4:{ power();Delay_ms(300);power_off(); }break;// питание исчезло
        case 5:{ power();Delay_ms(300);power_on(); }break;// питание появилось
        case 6:{ say_regime();Delay_ms(300);say_on(); }break; // режим охраны вкл
        case 7:{ say_regime();Delay_ms(300);say_off(); }break;// режим охраны выкл
        case 8:{ baterry_low(); }break;// батарея разряжена  
        }
        Delay_ms(500); 
        st++; 
        overflow=15*100;
      }break;
    case 4:{ if (mess) { if (!strncmp_P(rx_buff,no_carrier,strlen_P(no_carrier))) st=6;  mess=0; } else st++; }break; // если пока говорили повесили трубку
    case 5:{ Check_DTMF_Cmd();  st++;  }break;
    case 6:{ temp_retry--; if (temp_retry>0) st=0; else end=1; }break; // если есть еще попытки дозвона - повторяем или выходим
    
    case 10:{ put_message_modem(hold);  st++; timeout=200; }break; // сюда придем по таймауту (overflow)
    case 11:
      { 
        if (mess) {  if (!strncmp_P(rx_buff,ok,2)) end=1; else { if (!strncmp_P(rx_buff,error,5)) st--; } mess=0; } 
        if (timeout==0) end=1;// выход по таймауту
      }break;
    }//switch    
    
    if (!mess) Parser(); 
    if (overflow==0 && st<10) st=10;   // повесим трубку если 15 сек не нажимали на кнопки
  }// while
}

/*
void show_err(char n)
{
  while(1)
  {
    for(int i = 0; i < n; ++i)
    {
      putchar_debug(0x11);  putchar_debug(0x21); putchar_debug(0x31); 
      delay_ms(200);
      putchar_debug(0x10); putchar_debug(0x20); putchar_debug(0x30);
      delay_ms(200);
    }
    __watchdog_reset();
    delay_ms(1000);
  } 
}
*/
//------------------------------------------------------------------------------
int main( void )
{ 
  
  init_cpu();

// initI2C();
// lcd_init();
  
  ADMUX=ADC_VREF_TYPE & 0xff;
  ADCSRA=0x87;
  
  /* Отправим наверх, что реле у нас вырублены (нужно при пересбросе)*/ 
  putchar_debug(0x10); putchar_debug(0x20); putchar_debug(0x30);
/*  
  put_message_UCS2(aaa);
  while(1){__watchdog_reset();}
*/
  if (setup!=1)
  {
    ust_m=25; bit_temp_zapret=0; relay_zapret=0; hyst_m=2;
    if(SIM800_First_Setup()!=STATUS_OK) while(1);
  }  
  
  if (SIM800_PWR_ON()!=STATUS_OK) while(1); 
   
  if (SIM800_Init()!=STATUS_OK) while(1);
  
  if (SSB_Config()!=STATUS_OK) while(1);
  

  
  //------------------------------------------------------------------------------
 
  ust = ust_m;          // считаем уставку из еепром
  hyst=hyst_m;

  
  sec_stat=0;
  
  
  task=0;
  //--------------------================================Основной цикл======================================---------------------
  while(1)
  {
    
    __watchdog_reset();
    
    if (!mess) Parser();
    
    if (mess) //  Пришло что то с модема???
    {     
      if (!strncmp_P(rx_buff,ring,strlen_P(ring))) //  посмотрим а не пришел ли RING
      { 
        ring_cnt++;
        if (ring_cnt>1) { ring_cnt=0; Check_Incoming_Call(); } // трубку берем со 2-го гудка (2-го RING)
      }  
      mess=0;
    }
    
    if (sms_flag) { task=1; SIM800_SMS_Read(); sms_flag=0; task=0; }// если пришло СМС, читаем его и сбрасываем флаг => готовы читать следующее
    
    
    
    //-==================================подпрограмма мониторенья питания системы------------------------  
    // 180 V -330, 190 - 360, 200 - 380, 210 - 400, 220 - 430 (при выключенных реле)
    // при вкл всех реле напруга проседает: 220 -> 360
    // от аккума при всех вкл реле - 12В -> 270, выкл реле 12 V -> 270
    u=0;
    for(int i = 0; i < 5; ++i)
    {
      u+=read_adc(0);
      Delay_ms(5);
    }
    u/=5;


    if (!zb) // если питание в норме - будем смотреть просадку
    {  
      if (u<300)
      {
        Delay_ms(300);  // исключим ложные проседания
        u=read_adc(0);
        if (u<300)
        {
          //write_blog_eeprom(4); 
          if (pa[2]==0x31) call_master(1,4,0);  //  4 - питание исчезло
          zb=1;
        }
      }
    }
    
    if (zb) // если питание от акума - будем смотреть вдруг уже нормальное
    { 
      if (u>345)
      {
        //write_blog_eeprom(5);
        if (pa[2]==0x31) call_master(1,5,0);  // 5 - питание появилось
        zb=0;
        baterry_bit = 1; // при появлении питания скинем бит разряженного акума
      }
      
      
      if (u<260)
      {
        if (baterry_bit)
        {
          baterry_bit = 0;
          if (pa[2]==0x31) call_master(1,8,0); // батарея разряжена
        }
      }
    }
    
    
    
    //-=====================Подпрограмма считывания температуры======================------------------------
    reztemp = read_adc(5);
    reztemp += read_adc(5);
    reztemp += read_adc(5);
    reztemp += read_adc(5);
    reztemp += read_adc(5);
    reztemp += read_adc(5);
    reztemp += read_adc(5);
    reztemp += read_adc(5);
    reztemp >>= 3;
    if (reztemp<210 || reztemp>550 || vv)  // датчик неисправен или стоит бит аварии
    {
      if (pa[5]==0x31) //  если в сим карте включен режим опроса датчика температуры
      {
        if (!vv) // если датчик в норме
        {
          rele2 = 0; putchar_debug(0x20); //вырубим реле нагрева
          relay2_status=0;
          vv=1; //признак аварии датчика
          call_master(1,3,0); // звоним и говорим что с датчиком херня
        }
      }         
    }
    else  //-------=============== все пучком, какая то температура считана, щас посмотрим что там с ней ====------------
    {
      temperature = ADC2Temp(reztemp);
      ust = ust_m;          // считаем уставку из еепром  
      hyst=hyst_m; // и гистерезис
      t_on  = ust-(hyst-hyst/2);  // температура включения
      t_off = ust+(hyst/2); // температура выключения
      if (pa[6]==0x31 && relay_zapret==0) //  если в сим карте включен терморегулятор и не отключен командой *820 (или ручным упр. реле)
      { 
        if (temperature<t_on) // измеренная температура меньше температуры включения (меньше уставки)
        {
          if (PINF_Bit4==0) // реле выключено //(temperature_status==1)
          {
            temperature_status=0;
            rele2=1;putchar_debug(0x21); // включим реле терморегулятора (нагрев)
            relay2_status=1; // запомним что реле2 у нас работает
            if (!bit_temp_zapret) call_master(1,1,0); // звоним и говорим, температура ниже уставки (1 раз)
          }
        }
        if (temperature>=t_off) // измеренная температура больше температуры ВЫключения 
        {
          if (PINF_Bit4==1) // реле включено //(temperature_status==0)
          {
            temperature_status=1;
            rele2=0;putchar_debug(0x20); // вырубим нагрев
            relay2_status=0; // запомним что реле2 у нас отключено
            if (!bit_temp_zapret) call_master(1,2,0);  // звоним и говорим, температура в норме(при условии, что это позволено)
          }
        }
      }//termoregulator
    }
    //----============================================================================----------------------------    
    
    
    //-==============================================================================------------------------
    // Если надо включать реле, и какой-то канал сработал, и реле 3 выключено, то вначале включим его на 60 сек (если можно) (время зависит от длительности звонка, т.к. вырубится только после того как повесим трубку)
    if (pa[1]==0x31 && (sec_stat1==1 || sec_stat2==1 || sec_stat3==1 || sec_stat4==1) && PINA_Bit4==0) 
      { flag_rele3=1; rele3_count=60*100; rele3=1; putchar_debug(0x31); relay3_status=1; }
    if (flag_rele3 && rele3_count==0) 
      { flag_rele3=0; rele3=0; putchar_debug(0x30); relay3_status=0; } // реле3 включили по срабатыванию охраны и пора его вырубать
    //------------------------------------------------------------------------------------
    //-------------сработал канал 1-----------------------------
    if (sec_stat1 == 1) 
    { 
      sec_stat1 = 0;
      putchar_debug(0x56); // подтвердим верхней платке что мы приняли байт и отзвонились
      call_master(1,0,1); // позвоним мастеру (он должен быть)
      if (cm[0]==1) call_master(2,0,1); // если там не звездочка то наберем его
      if (cm[1]==1) call_master(3,0,1); // 
      if (cm[2]==1) call_master(4,0,1); // 
      if (cm[3]==1) call_master(5,0,1); // 
      if (cm[4]==1) call_master(6,0,1); // 
      if (pa[4]==0x31) { 
        putchar_debug(0x91); // отправим постановку на охрану
        sec_stat=1;
      }
      else {sec_stat=0; }  
    }
    
    //-------------Сработал канал2--------------------------
    if (sec_stat2 == 1) 
    { 
      sec_stat2 = 0;
      putchar_debug(0x56); // подтвердим верхней платке что мы приняли байт и отзвонились
      call_master(1,0,2); // 
      if (cm[0]==1) call_master(2,0,2); // звоним следующему абоненту
      if (cm[1]==1) call_master(3,0,2); // звоним следующему абоненту
      if (cm[2]==1) call_master(4,0,2); // звоним следующему абоненту
      if (cm[3]==1) call_master(5,0,2); // звоним следующему абоненту
      if (cm[4]==1) call_master(6,0,2); // звоним следующему абоненту
      //write_blog_eeprom(7);
      if (pa[4]==0x31)       //Автопостановка на охрану  
      {
        putchar_debug(0x91); // отправим постановку на охрану
        sec_stat=1;
      }
      else {sec_stat=0; }
    }
    
    //-------------Сработал канал3--------------------------
    if (sec_stat3 == 1) 
    { 
      sec_stat3 = 0; //позвоним 1 раз
      putchar_debug(0x56); // подтвердим верхней платке что мы приняли байт и отзвонились
      call_master(1,0,3); // 
      if (cm[0]==1) call_master(2,0,3); // звоним следующему абоненту
      if (cm[1]==1) call_master(3,0,3); // звоним следующему абоненту
      if (cm[2]==1) call_master(4,0,3); // звоним следующему абоненту
      if (cm[3]==1) call_master(5,0,3); // звоним следующему абоненту
      if (cm[4]==1) call_master(6,0,3); // звоним следующему абоненту 
      if (pa[4]==0x31) //Автопостановка на охрану  
      {
        putchar_debug(0x91); // отправим постановку на охрану
        sec_stat=1;
      }
      else {sec_stat=0; }
    }

    //-------------Сработал канал4--------------------------
    if (sec_stat4 == 1) 
    { 
      sec_stat4 = 0;
      putchar_debug(0x56); // подтвердим верхней платке что мы приняли байт и отзвонились
      call_master(1,0,4); // 
      if (cm[0]==1) call_master(2,0,4); // 
      if (cm[1]==1) call_master(3,0,4); // 
      if (cm[2]==1) call_master(4,0,4); // 
      if (cm[3]==1) call_master(5,0,4); // 
      if (cm[4]==1) call_master(6,0,4); // 
      if (pa[4]==0x31) //Автопостановка на охрану  
      {
        putchar_debug(0x91); // отправим постановку на охрану
        sec_stat=1;
      }
      else { sec_stat=0; }
    }
    
    //---------------------- Далее идет обмен данных с  сопроцессором --------
    
    recived_byte = 0x00; 
    
    //-------------------Узнаем состояние каналов ------------------------------------------------
    putchar_debug(0x86); // узнаем состояния каналов //через 5мс славик скажет о состоянии канала1
    Delay_ms(8);                                          
    if (recived_byte==0x81) {chan_stat1 = 1; Delay_ms(5); } // передали нам что канал1 разомкнут
    if (recived_byte==0x51) {chan_stat1 = 0; Delay_ms(5); } // передали нам что канал1 замкнут
    
    if (recived_byte==0x82) {chan_stat2 = 1; Delay_ms(5); } //    канал2 разомкнут
    if (recived_byte==0x52) {chan_stat2 = 0; Delay_ms(5); } // передали нам что канал2 замкнут  с
    
    if (recived_byte==0x83) {chan_stat3 = 1; Delay_ms(5); } //    канал3 разомкнут
    if (recived_byte==0x53) {chan_stat3 = 0; Delay_ms(5); } // передали нам что канал3 замкнут
    
    if (recived_byte==0x84) {chan_stat4 = 1; Delay_ms(5); } //    канал4 разомкнут
    if (recived_byte==0x54) {chan_stat4 = 0; Delay_ms(5); } // передали нам что канал4 замкнут
    
    if (recived_byte==0x87) {security_stand = 1; Delay_ms(5);} //    стоим на охране (все зашибись, канал не разомкнут и кнопкой не выключили)
    if (recived_byte==0x88) {security_stand = 0; Delay_ms(5);} //    охрана выключена (либо сработал канал, либо кнопкой выкл...разбираться надо)
//    else { if (recived_byte!=0x00) security_stand = 1; Delay_ms(5); }  // обманка, чтоб не глючило. Тогда стабильно звонит при вкл/выкл охраны. // глюк убрался задержкой в 10 мс в конце приема пакета
                                                                  
    
    if (recived_byte==0x61) {ignor1 = 1; Delay_ms(5);} // игнор канала 1
    if (recived_byte==0x71) {ignor1 = 0; Delay_ms(5);}
    
    if (recived_byte==0x62) {ignor2 = 1; Delay_ms(5);} // игнор канала 2
    if (recived_byte==0x72) {ignor2 = 0; Delay_ms(5);}
    
    if (recived_byte==0x63) {ignor3 = 1; Delay_ms(5);} // игнор канала 3  
    if (recived_byte==0x73) {ignor3 = 0; Delay_ms(5);}
    
    if (recived_byte==0x64) {ignor4 = 1; Delay_ms(5);} // игнор канала 4
    if (recived_byte==0x74) {ignor4 = 0; Delay_ms(5);}
    
    if (recived_byte==0x26) {sec_stat1=1; Delay_ms(5);}     // последний байт пакета - если 26-29 звоним на соотв. канал
    if (recived_byte==0x27) {sec_stat2=1; Delay_ms(5);}
    if (recived_byte==0x28) {sec_stat3=1; Delay_ms(5);}
    if (recived_byte==0x29) {sec_stat4=1; Delay_ms(5);}
    
    Delay_ms(10); // НЕ ТРОГАТЬ!!! без этой задержки не звонит о постановке/снятии с охраны

    
    if (security_stand == 0 && sec_stat == 1) // разбираемся дальше, вдруг с кнопки сняли
    { // до этого проверили состояния каналов. проверим, если какой то из них сработал - значит условие не пройдет
      if (sec_stat1==0)
      {
        if (sec_stat2==0)
        {
          if (sec_stat3==0)
          {
            if (sec_stat4==0)
              // все таки сняли с кнопки, выключим охрану на нижней платке, запишем в дневник // 7 - режим охраны выключен
            { if (pa[3]==0x31) call_master(1,7,0); sec_stat = 0; } //  если бит обзвона стоит - надо звонить
          }
        }
      }
    }
    
    // теперь смотрим, вдруг с кнопки на охрану поставили
    //    верх                 низ
    if (security_stand == 1 && sec_stat == 0)  // верхняя платка на охране, низ еще нет.....
    {
      if (pa[3]==0x31) call_master(1,6,0); // режим охраны вкл
      sec_stat = 1;  
    }
    // вроде как все
    
  }//while(1)
  
}//main
