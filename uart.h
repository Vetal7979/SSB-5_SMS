//#include "compiler.h"



char const __flash call_ready[] =       "Call Ready";
char const __flash sms_ready[] =        "SMS Ready";
char const __flash echo_off[] =         "ATE0\r"; // AT&W
char const __flash echo_on[] =          "ATE1\r"; // AT&W

char const __flash ring[]=              "RING";
char const __flash no_carrier[]=        "NO CARRIER";
char const __flash no_answer[]=         "NO ANSWER";
char const __flash busy[] =             "BUSY";
char const __flash ok[] =               "OK";
char const __flash error[] =            "ERROR";
char const __flash cops_p[] =           "+COPS:";
char const __flash cops_ans[] =         "+COPS: 0,0,\"";
char const __flash cops_no_ans[] =      "+COPS: 0";
char const __flash power_dn[] =         "NORMAL POWER DOWN";
char const __flash parol_ch[] =         "+CPBR: 2,\"*";
char const __flash cbpr[] =             "+CPBR:";
char const __flash collp[] =            "+COLP";
char const __flash dtmf[] =             "+DTMF:";
char const __flash cpms[] =             "+CPMS:";
char const __flash cmgr[] =             "+CMGR:";
char const __flash cmti[] =             "+CMTI:";
char const __flash clip_p[]=            "+CLIP:";
char const __flash at_p[] =             "AT+";
char const __flash at[] =               "AT\r";


char const __flash init_str[] = {0x49, 0x49, 0x49, 0x49, 0xFF, 0xFF, 0xFF, 0xFF};

char const __flash cpin[] =             "+CPIN:";
char const __flash cpin_ready[] =       "+CPIN: READY";
char const __flash cpin_not_ready[] =   "+CPIN: NOT READY";
char const __flash cpin_not_ins[] =     "+CPIN: NOT INSERTED";

char const __flash atw[] =              "AT&W\r";

#ifdef SIM800
 #ifndef SIM900
   char const __flash dtmf_en[]=           "AT+DDET=1,0,0\r"; // ��� SIM800
 #endif 
#else
 #ifdef SIM900
  #ifndef SIM800 
   char const __flash dtmf_en[]=           "AT+DDET=1\r"; // ��� SIM900, AT&W
  #endif 
 #endif 
#endif

char const __flash ipr_115200[] =       "AT+IPR=115200\r"; // AT&W
char const __flash cops[]  =            "AT+COPS?\r";

char const __flash _atl3[]=             "ATL3\r";                               //��������� ���������
char const __flash atm1[]=              "ATM1\r";                               //�������� �������
char const __flash _atclvl[]=           "AT+CLVL=80\r";                         //������� ��������� ��������� �� 80 ���������
char const __flash _atcpbs[]=           "AT+CPBS=\"SM\"\r" ;
char const __flash _sm[]=               "\r\n";
char const __flash ata[]=               "ATA\r\n";
char const __flash hold[] =             "ATH\r";

char const __flash mic_lvl[] =          "AT+CMIC=0,7\r";
char const __flash ciurc[] =            "AT+CIURC=1\r"; // AT&W


char const __flash dial_master[] =      "ATD>1;\r";
char const __flash dial_number1[] =     "ATD>4;\r";
char const __flash dial_number2[] =     "ATD>5;\r";
char const __flash dial_number3[] =     "ATD>6;\r";
char const __flash dial_number4[] =     "ATD>7;\r";
char const __flash dial_number5[] =     "ATD>8;\r";


char const __flash colp[]        =      "AT+COLP=1\r";                          // ��� ������ ������ ����� ����� �������� +COLP: ....  � OK
char const __flash clip[] =             "AT+CLIP=1\r";
char const __flash set_master[] =       "AT+CPBW=1,\"+79000000000\",129,\"MASTER\"\r";
char const __flash set_parol[]  =       "AT+CPBW=2,\"*1*1*1*1234\",129,\"PAROL\"\r";
char const __flash set_config[] =       "AT+CPBW=3,\"*000011****\",129,\"CONFIG\"\r";


char const __flash set_number1[] =      "AT+CPBW=4,\"*\",129,\"NUMBER1\"\r";
char const __flash set_number2[] =      "AT+CPBW=5,\"*\",129,\"NUMBER2\"\r";
char const __flash set_number3[] =      "AT+CPBW=6,\"*\",129,\"NUMBER3\"\r";
char const __flash set_number4[] =      "AT+CPBW=7,\"*\",129,\"NUMBER4\"\r";
char const __flash set_number5[] =      "AT+CPBW=8,\"*\",129,\"NUMBER5\"\r";

char const __flash write_number[] =     "AT+CPBW=%d,\"%s\",129,\"%s\"\r";       // AT+CPBW=%d,"%s",129,"%s"\r

char const __flash check_parol[] =      "AT+CPBR=2\r";
char const __flash check_config[] =     "AT+CPBR=3\r";
char const __flash check_xnumber[] =    "AT+CPBR=9\r";
char const __flash clear_numberx[] =    "AT+CPBW=9,\"89000000000\",129,\"FREE\"\r";
char const __flash check_master_num[] = "AT+CPBR=1\r";
char const __flash check_number1[] =    "AT+CPBR=4\r";
char const __flash check_number2[] =    "AT+CPBR=5\r";
char const __flash check_number3[] =    "AT+CPBR=6\r";
char const __flash check_number4[] =    "AT+CPBR=7\r";
char const __flash check_number5[] =    "AT+CPBR=8\r";

/*SMS*/
char const __flash sms_list_all[]=      "AT+CMGL=\"ALL\"\r";
char const __flash sms_read[]=          "AT+CMGR=";
char const __flash sms_del_all[]=       "AT+CMGD=1,4\r";
char const __flash sms_storage_sm[]=    "AT+CPMS=\"SM\"\r";

char const __flash sms_mess_ind[] =     "AT+CNMI=2,1\r"; // AT&W

char const __flash sms_format_text[] =  "AT+CMGF=1\r";  // AT&W
char const __flash set_unicod[] =       "AT+CSCS=\"UCS2\"\r";  // AT&W
char const __flash set_gsm[]=           "AT+CSCS=\"GSM\"\r"; // AT&W
char const __flash sms_parameters_soxr[] = "AT+CSMP=17,167,0,25\r"; // 25 - ��������� 24 - �����������

char const __flash sms_parameters[] =   "AT+CSMP=17,167,0,24\r";
char const __flash sms_number[] =       "AT+CMGS=\"";
char const __flash sms_end[] =          "\"";

//------------------------------------------
/* DTMF/SMS Command*/
char const __flash cmd_11[] = "*11";    char const __flash RELE1_ON[]=  "RELE1 ON";   char const __flash rele1_on[]=  "rele1 on"; 
char const __flash cmd_10[] = "*10";    char const __flash RELE1_OFF[]= "RELE1 OFF";  char const __flash rele1_off[]= "rele1 off";
char const __flash cmd_21[] = "*21";    char const __flash RELE2_ON[]=  "RELE2 ON";   char const __flash rele2_on[]=  "rele2 on";
char const __flash cmd_20[] = "*20";    char const __flash RELE2_OFF[]= "RELE2 OFF";  char const __flash rele2_off[]= "rele2 off";
char const __flash cmd_31[] = "*31";    char const __flash RELE3_ON[]=  "RELE3 ON";   char const __flash rele3_on[]=  "rele3 on";
char const __flash cmd_30[] = "*30";    char const __flash RELE3_OFF[]= "RELE3 OFF";  char const __flash rele3_off[]= "rele3 off";
char const __flash cmd_41[] = "*41";    char const __flash RELE4_ON[]=  "RELE4 ON";   char const __flash rele4_on[]=  "rele4 on";
char const __flash cmd_40[] = "*40";    char const __flash RELE4_OFF[]= "RELE4 OFF";  char const __flash rele4_off[]= "rele4 off";
char const __flash cmd_51[] = "*51";    char const __flash RELE5_ON[]=  "RELE5 ON";   char const __flash rele5_on[]=  "rele5 on";
char const __flash cmd_50[] = "*50";    char const __flash RELE5_OFF[]= "RELE5 OFF";  char const __flash rele5_off[]= "rele5 off";
char const __flash cmd_61[] = "*61";    char const __flash RELE6_ON[]=  "RELE6 ON";   char const __flash rele6_on[]=  "rele6 on";
char const __flash cmd_60[] = "*60";    char const __flash RELE6_OFF[]= "RELE6 OFF";  char const __flash rele6_off[]= "rele6 off";
char const __flash cmd_71[] = "*71";    char const __flash RELE7_ON[]=  "RELE7 ON";   char const __flash rele7_on[]=  "rele7 on";
char const __flash cmd_70[] = "*70";    char const __flash RELE7_OFF[]= "RELE7 OFF";  char const __flash rele7_off[]= "rele7 off";
char const __flash cmd_81[] = "*81";    char const __flash DIAL_ON[]=   "DIAL ON";    char const __flash dial_on[]=   "dial on";
char const __flash cmd_80[] = "*80";    char const __flash DIAL_OFF[]=  "DIAL OFF";   char const __flash dial_off[]=  "dial off";
char const __flash cmd_91[] = "*91";    char const __flash SEC_ON[]=    "SEC ON";     char const __flash sec_on[]=    "sec on";
char const __flash cmd_90[] = "*90";    char const __flash SEC_OFF[]=   "SEC OFF";    char const __flash sec_off[]=   "sec off";
char const __flash cmd_821[] = "*821";  char const __flash TERM_ON[]=   "TERM ON";    char const __flash term_on[]=   "term on";
char const __flash cmd_820[] = "*820";  char const __flash TERM_OFF[]=  "TERM OFF";   char const __flash term_off[]=  "term off";

char const __flash cmd_62[] = "*62";
char const __flash cmd_63[] = "*63";
char const __flash cmd_64[] = "*64";
char const __flash cmd_65[] = "*65";
char const __flash cmd_66[] = "*66";
char const __flash cmd_67[] = "*67";

char const __flash cmd_00[] = "*00";  char const __flash STATUS[]= "STATUS"; char const __flash status[]= "status";
char const __flash SET[] = "SET"; char const __flash set[] = "set";
char const __flash HYS[] = "HYS"; char const __flash hys[] = "hys";



char const __flash master[] = "MASTER";    char const __flash master_sm[] = "master";
char const __flash number1[] = "NUMBER1";  char const __flash number1_sm[] = "number1";
char const __flash number2[] = "NUMBER2";  char const __flash number2_sm[] = "number2";
char const __flash number3[] = "NUMBER3";  char const __flash number3_sm[] = "number3";
char const __flash number4[] = "NUMBER4";  char const __flash number4_sm[] = "number4";
char const __flash number5[] = "NUMBER5";  char const __flash number5_sm[] = "number5";
char const __flash config[] = "CONFIG";    char const __flash config_sm[] = "config";
char const __flash parol[] = "PAROL";      char const __flash parol_sm[] = "parol";

char const __flash rst[] = "RESET";
char const __flash restart[] = "RESTART";

/* SMS */ /* ����������� ��� 70 �������� */
char const __flash rele_on[] =          "���� %d ��������.";
char const __flash rele_off[] =         "���� %d ���������.";
char const __flash ctrl_on[] =          "�������� �������.";
char const __flash ctrl_off[] =         "�������� ��������.";
char const __flash t_reg_off[] =        "�������������� ��������.";
char const __flash t_reg_on[] =         "�������������� �������.";
char const __flash t_reg_off_sim[] =    "�������������� �������� � ���������� SIM";
char const __flash err_cmd[] =          "������ � �������."; 
char const __flash mas_ok[] =           "������ MASTER ������� ��������.";
char const __flash n_ok[] =             "������ NUMBER%d ������� ��������."; 
char const __flash cfg_ok[] =           "������ CONFIG ������� ��������.";
char const __flash pass_ok[] =          "������ PAROL ������� ��������.";  
char const __flash default_settings[] = "��������� ��������."; 
char const __flash security_on[] =      "����� ������ �������"; 
char const __flash security_off[] =     "����� ������ ��������";
char const __flash ustavka[] =          "������� %d ��."; 
char const __flash hysterezis[] =       "���������� %d �C.";
char const __flash all_temp[] =         "Temp=%d�C, Ust=%d�C, Hyst=%d�C. Rele1=%d, Rele2=%d, Rele3=%d"; 


/*
char const  __flash set_date_modem[] = "AT+CCLK=\"10/04/05,08:55:00+03\"\r";
char const __flash get_date_time_modem[] = "AT+CCLK?\r";
*/

