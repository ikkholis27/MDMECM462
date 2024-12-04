/*
********************************************************************
**  Copyright (C) 2022. All Rights Reserved.
**  Filename    : main.c
**  Author      : Ikhwannul Kholis
**  Date        : 2022.06.07
**  Description : Program of Mode A V1.2

** FMC : 
0 - 14 APN, 15 - 19 PORT, 20 - 34 WL1, 35 - 49 WL2, 50 - 64 WL3, 65 Autoreboot, 67: baudrate, 68-75: passcode, 76: ENWL
	mintotal_srst = fmc_data_backup[100];
	hour_srst = fmc_data_backup[101];
	min_srst = fmc_data_backup[102];
	hour_hrst = fmc_data_backup[103];
	min_hrst = fmc_data_backup[104];
	day_hrst = fmc_data_backup[105];
240 : Memory Location of Program (0/255: 0x8000000; 100:0x8010000
********************************************************************
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "gd32f30x.h"
#include "usart.h"
#include "gd32f30x_timer.h"
#include "gd32f30x_fmc.h"
#include "gd32f30x_fwdgt.h"
#include "common.h"
//#include "systick.h"
#define LED_GPIO_PIN (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define LEDG 0
#define LEDB 1
#define LEDY 2

//#define FMC_PAGE_SIZE           ((uint16_t)0x800U)
//#define FMC_WRITE_START_ADDR    ((uint32_t)0x08004000U)
//#define FMC_WRITE_END_ADDR      ((uint32_t)0x08004800U)

//#define FMC_PAGE_SIZE           ((uint16_t)0x400U)
//#define FMC_WRITE_START_ADDR    ((uint32_t)0x08007800U)
//#define FMC_WRITE_END_ADDR      ((uint32_t)0x08007C00U)

#define FMC_PAGE_SIZE           ((uint16_t)0x400U)
#define FMC_WRITE_START_ADDR    ((uint32_t)0x08020000U)
#define FMC_WRITE_END_ADDR      ((uint32_t)0x08020400U)

#define FOTA_FMC_PAGE_SIZE           ((uint16_t)0x800U)

pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t BlockNbr = 0, UserMemoryMask = 0;
__IO uint32_t FlashProtection = 0;
extern uint32_t FlashDestination;

static char fmc_buf[256] = {0};
uint32_t fmc_data_backup[256]= {0};
int fota_addr=0, fota_idx=0;
uint32_t fota_fw1[512]= {0};
uint32_t fota_write_start_addr=(uint32_t)0x08030000U, fota_write_end_addr=(uint32_t)0x0803F000U; // A1D
//uint32_t fota_write_start_addr=(uint32_t)0x08010000U, fota_write_end_addr=(uint32_t)0x0801F000U; // A1A
/* calculate the number of page to be programmed/erased */
uint32_t FotaPageNum;
/* calculate the number of page to be programmed/erased */
//uint32_t FotaWordNum;

static char MCU_SOFT_VERSION[]="MDA_V1.2E38A1U";
uint32_t *ptrd;
uint32_t address = 0x00000000U;
uint32_t data1   = 0x12345678U;
uint8_t autoreboot = 0;
// calculate the number of page to be programmed/erased 
uint32_t PageNum = (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) / FMC_PAGE_SIZE;
// calculate the number of page to be programmed/erased 
uint32_t WordNum = ((FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) >> 2);

int baudrateDefault=19200;
int counter_wait_sim;
int	counter_wait_mtr=0;
uint8_t max_counter_wait, index_content=0, idx_plus=0;
uint8_t outage=0, outage_old=0, fotaerror=0;
uint8_t TESTMODE=0, fotasmscheck=0, process_sms, tandapetik;
uint8_t modecfg=0;
uint8_t rebootcmd=0;
int timerYellowBlink=0, yellowOn=0, timerGBlink=0;
int LEDGStatus, LEDBStatus;
int network, starthour, startmin, hournow, minnow, startday, daynow, secnow, startday_srst, starthour_srst, startmin_srst, cntcclk;
int getnetwork=0;
int enablesrst=0, enablehrst=0, hour_srst=0, day_hrst=0, min_srst=0, mintotal_srst=0, mintotal_hrst=0, hour_hrst=0, min_hrst=0;
int MAXBUFFER=2100;
int cmp_config;
int enablehrst_default=1;
int enablesrst_default=1;
uint32_t tick=0;
uint32_t tick_max=250000; //5menit

long baud0;
int afterreset=0;
int cntSimflag0=0;
static char cmp_msg0[]="AT+GETIP?";
static char cmp_msg1[]="AT+MWL?";
static char cmp_msg2[]="AT+MCFG?";
static char cmp_msg3[]="AT+MREBOOT";
static char cmp_msg4[]="AT+MIPCALL?";
static char cmp_msg5[]="AT+MGNET?";
static char cmp_msg7[]="AT+MFW2";
static char cmp_msg8[]="AT+REOPS";
static char APN[15]={0};
static char PORT[5]={0};
static char uart0_rxbuf[2200], uart0_rxbuf1[2200] = {0};
static char white_list1[15]={0}, white_list2[15]={0}, white_list3[15]={0};
static volatile int rx_cnt = 0;
static volatile int len_rxmeter = 0;
static volatile char sim_flag = 0; // AT=0; READY=1; CSQ=2; MIPCALL M2MPNAMR=3; MIPCALL?=4; MIPOPEN=5
char sim_flag_old = 0, sim_flag_old1 = 0, sim_flag_old2 = 0; // AT=0; READY=1; CSQ=2; MIPCALL M2MPNAMR=3; MIPCALL?=4; MIPOPEN=5

static char ippush[50] = {0};
static char portpush[8] = {0};
static char ipftp[50] = {0};
static char userftp[20] = {0};
static char pwdftp[20] = {0};
static char fileftp[20] = {0};
static char temp_global[1200] = {0};
static volatile int len_meter_buf = 0;
static char uart1_tx[2100] = {0};
// Uart1 Vars
int uart1_index = 0, socket_id=0, cmpnum=0, checksocket=0;
int uart0_rxbuf_index = 0;
static char uart1_rxbuf[2100] = {0};
static char mipcall[100] = {0};
int numinwhitelist, cnt_data_meter;
uint8_t has_configured=0,cntmipcall=0;
int tempint, tempint2, tempint3;
//static char data_byte_meter;
char smsnum[15] = {0}, smsnum1[15] = {0}, callernum[15] = {0};
char passcode[8] = {0}, passcode_saved[8] = {0};
char smsmsg[255] = {0};
char smsmsg1[255] = {0};
char smsmsg2[200] = {0};
char smsmsg3[255] = {0};
int cmp_smsmsg=-1, cmp_caller=-1, cmp_passcode=-1, idx_smscontent, callingcnt=0;
int cntcclkerr;
void check_wl1_fmc();
void check_wl2_fmc();
void check_wl3_fmc();
void check_set_fmc();
void led_off(int led_num);
void led_on(int led_num);
void mfota_start(int ser);
//void check_baud_fmc();
void fmc_erase_pages();
void fmc_erase_pages_check();
void send_sms();
void fmc_program2();
void fmc_backup();
void check_set_fmc_byserial(int chosen);
void send_sms_withsmsnum();
void reboot_by_calling();
void wdt_on(int arg);
void wdt_off(int arg);

uint8_t hex2int(char hex);
void jump_to_address_fw2(){
		  JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
      /* Jump to user application */
      Jump_To_Application = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t*) ApplicationAddress);
      Jump_To_Application();	
}
/*!
    \brief      delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tiktok(void)
{
    if (0U != tick){
        tick--;
    }
    if (0U == tick){
			NVIC_SystemReset();
    }
}

void delay(uint32_t t)
{
    int i,j;
    for(i=0; i<t; i++) {
        for(j=0; j<=500000; j++){
					__NOP();
				}
    }
}

void delay_mid(uint32_t t)
{
    int i,j;
    for(i=0; i<t; i++) {
        for(j=0; j<=750000; j++){
					__NOP();
				}
    }
}

void delay_1ms(uint32_t t)
{
    int i,j;
    for(i=0; i<t; i++) {
        for(j=0; j<=2000; j++){
					__NOP();
				}
    }
}

int uart1_send_buffer(char *str, int len)
{
    char *p = str;
	int x=0;
    while(x<len) {
        usart_data_transmit(USART1, *p);
        p++;
			x++;
    }
    return 0;
}

int uart0_send_string(char *str)
{
    char *p = str;

    while(*p) {
        usart_data_transmit(USART0, *p);
        p++;
    }

    return 0;
}

int check_sms(int xx){
	int val=100;
//	if(TESTMODE) printf("\r\n %s ",smsmsg);
		switch(xx){
			case 0: cmpnum = strcmp(smsmsg, cmp_msg0);
							if(cmpnum==0) {
								val=0;
						 };
							break;
			case 1: cmpnum = strcmp(smsmsg, cmp_msg1);
							if(cmpnum==0) {
								val=1;
						 };
							break;
			case 2: cmpnum = strcmp(smsmsg, cmp_msg2);
							if(cmpnum==0) {
								val=2;
						 };
							break;

			case 3: cmpnum = strcmp(smsmsg, cmp_msg3);
							if(cmpnum==0) {
								val=3;
						 };
							break;
			case 4: cmpnum = strcmp(smsmsg, cmp_msg4);
							if(cmpnum==0) {
								val=4;
						 };
							break;							
			case 5: cmpnum = strcmp(smsmsg, cmp_msg5);
							if(cmpnum==0) {
								val=18;
						 };
							break;
						 
/*			case 6: cmpnum = strcmp(smsmsg, cmp_msg6);
							if(cmpnum==0) {
								val=20;
						 };
							break;
						 */
			case 7: cmpnum = strcmp(smsmsg, cmp_msg7);
							if(cmpnum==0) {
								val=21;
						 };
							break;
			case 8: cmpnum = strcmp(smsmsg, cmp_msg8);
							if(cmpnum==0) {
								val=25;
						 };
							break;
			default : break;
		}
	return val;
}

int strcmp_smsmsg(){
	int cmp_smsmsg1 = 100;
	int val1=-1;
	int i=0;
	for(int i=0;i<9;i++){
		val1= check_sms(i);
		if(cmp_smsmsg1==100) cmp_smsmsg1 = val1;
//		printf("  %d  ",cmp_smsmsg1);
	}
	if(cmp_smsmsg1 == 100){
		i=0;
//	while(1){
			// MWL
			if(smsmsg[6]=='=' && smsmsg[5]=='L' && smsmsg[4]=='W' && smsmsg[3]=='M'){
					val1 = 5;
				idx_smscontent = 7;
			}
			// MPC
			else if(smsmsg[6]=='=' && smsmsg[5]=='C' && smsmsg[4]=='P' && smsmsg[3]=='M'){
					val1 = 9;
				idx_smscontent = 7;
			}
			// MCFG=
			else if(smsmsg[7]=='=' && smsmsg[6]=='G' && smsmsg[5]=='F' && smsmsg[4]=='C' && smsmsg[3]=='M'){
					val1 = 8;
				idx_smscontent = 8;
			}
			// MCFG?
			else if(smsmsg[7]=='?' && smsmsg[6]=='G' && smsmsg[5]=='F' && smsmsg[4]=='C' && smsmsg[3]=='M'){
					val1 = 2;
			}
			// MWL?
			else if(smsmsg[6]=='?' && smsmsg[5]=='L' && smsmsg[4]=='W' && smsmsg[3]=='M'){
					val1 = 1;
			}
			// ENWL
			else if(smsmsg[6]=='L' && smsmsg[5]=='W' && smsmsg[4]=='N' && smsmsg[3]=='E'){
				if(smsmsg[7]=='=') {
					val1=24;
					idx_smscontent = 8;
				}
				else val1 = 23;
			}
			// MBAUD
			else if(smsmsg[7]=='D' && smsmsg[6]=='U'){
				if(smsmsg[8]=='=') {
					val1=10;
					idx_smscontent = 9;
				}
				else val1 = 11;				
			}				
			// WRST
			else if(smsmsg[6]=='T' && smsmsg[5]=='S' && smsmsg[4]=='R' && smsmsg[3]=='W'){
				if(smsmsg[7]=='=') {
					val1=14;
					idx_smscontent = 8;
				}
				else val1 = 16;
			}
			// WDTOFF
//			else if(smsmsg[5]=='T' && smsmsg[4]=='D' && smsmsg[3]=='W'){
//				if(smsmsg[6]=='F') {
//					val1=27;
//				}
//				else val1 = 28;
//			}
			// HRST
			else if(smsmsg[6]=='T' && smsmsg[5]=='S' && smsmsg[4]=='R' && smsmsg[3]=='H'){
				if(smsmsg[7]=='='){
					val1=15;
					idx_smscontent = 8;
				}
				else val1 = 17;
			}
			// SFLAG
			else if(smsmsg[6]=='G' && smsmsg[5]=='L' && smsmsg[4]=='F' && smsmsg[3]=='S'){
				val1 = 26;
			}
			// MVER
			else if(smsmsg[6]=='R' && smsmsg[5]=='E' && smsmsg[4]=='V' && smsmsg[3]=='M'){
					val1 = 12;
			}
			// FFAC
			else if(smsmsg[6]=='C' && smsmsg[5]=='A' && smsmsg[4]=='F' && smsmsg[3]=='F'){
				sprintf(smsmsg,"FACTORY RESET IS WORKING. CANNOT UNDO THIS. Please Wait 3 Minutes. Current MCU Ver %s",MCU_SOFT_VERSION);
//				send_sms();
				send_sms_withsmsnum();
				delay(40);
				fmc_erase_pages();
				fmc_erase_pages_check();
				cmp_config = 5;
				sim_flag=sim_flag_old;
			 delay(20);
			 NVIC_SystemReset();
			 delay(20);
			}
			/*
			else if(smsmsg[7]=='=' && smsmsg[6]=='2' && smsmsg[5]=='L' && smsmsg[4]=='W' && smsmsg[3]=='M'){
					val1 = 6;
			}
			
			else if(smsmsg[7]=='=' && smsmsg[6]=='3' && smsmsg[5]=='L' && smsmsg[4]=='W' && smsmsg[3]=='M'){
					val1 = 7;
			}
				*/
			// SOFTRESET
			else if(smsmsg[6]=='T' && smsmsg[5]=='F' && smsmsg[4]=='O' && smsmsg[3]=='S'){
					val1 = 13;
			}
			// RECO
			else if(smsmsg[6]=='O' && smsmsg[5]=='C' && smsmsg[4]=='E' && smsmsg[3]=='R'){
					val1 = 13;
			}
			// GTRAT
			else if(smsmsg[8]=='=' && smsmsg[7]=='T' && smsmsg[6]=='A' && smsmsg[5]=='R'){
					val1 = 19;
					idx_smscontent = 9;
			}
			// FOTA
			else if(smsmsg[9]=='"' && smsmsg[8]=='=' && smsmsg[7]=='A' && smsmsg[6]=='T' && smsmsg[5]=='O' && smsmsg[4]=='F'){
					val1 = 20;
					idx_smscontent = 10;
			}
			cmp_smsmsg1 = val1;
//		if(val1!=100) {
//			cmp_smsmsg1 = val1;
//			break;
//		}
//		i++;
//		if(i>=sizeof(smsmsg)) break;
//	}
	}
	if(cmp_smsmsg1 == -1)cmp_smsmsg1=100;	
	return cmp_smsmsg1;
}

void send_sms(){
	char temp[200];
	sim_flag=36;
//	delay(2);
	//printf("SENDSMS\r\n");
	memset(temp,0,sizeof(temp));
	if(numinwhitelist==2) sprintf(temp,"AT+CMGS=\"%s\"\r\n",white_list2);
	else if(numinwhitelist==3) sprintf(temp,"AT+CMGS=\"%s\"\r\n",white_list3);
	else sprintf(temp,"AT+CMGS=\"%s\"\r\n",white_list1);

	uart1_send_string(temp);
	delay(30);
	uart1_send_string(smsmsg);
	delay(2);
  usart_data_transmit(USART1, 0x1A);
	memset(smsmsg,0,sizeof(smsmsg));
	memset(smsnum,0,sizeof(smsnum));
	led_on(LEDG);
}

void send_sms_withsmsnum(){
	char temp[200];
	sim_flag=36;
//	delay(2);
//	printf("SMSWITHNUM\r\n");
	memset(temp,0,sizeof(temp));
	sprintf(temp,"AT+CMGS=\"%s\"\r\n",smsnum1);
//	printf("SMSWITHNUM %s\r\n", temp);
	uart1_send_string(temp);
	delay(30);
	uart1_send_string(smsmsg);
	delay(2);
  usart_data_transmit(USART1, 0x1A);
	memset(smsmsg,0,sizeof(smsmsg));
	memset(smsnum,0,sizeof(smsnum));
	memset(smsnum1,0,sizeof(smsnum1));
	led_on(LEDG);
}

void send_sms_withsmsnum1(){
	char temp[200];
	sim_flag=36;
//	delay(2);
//	printf("SMSWITHNUM\r\n");
	memset(temp,0,sizeof(temp));
	sprintf(temp,"AT+CMGS=\"%s\"\r\n",smsnum1);
//	printf("SMSWITHNUM1 %s\r\n", temp);
	uart1_send_string(temp);
	delay(30);
	uart1_send_string(smsmsg1);
	delay(2);
  usart_data_transmit(USART1, 0x1A);
	led_on(LEDG);
}

void send_smscmdmode(){
	memset(smsmsg1,0,sizeof(smsmsg1));
	sprintf(smsmsg1,"%s. PROCESSED\r\n",smsmsg2);
	send_sms_withsmsnum1();
	delay(30);
}

void filter_cmt(){
	int i=0, j=0, k=0, found=0, stno=0, enno=0, numdone=0;
	led_off(LEDG);
		memset(smsmsg,0,sizeof(smsmsg));
	//printf("FILTERCMT\r\n");
	while(1){
		
		if(uart1_rxbuf[i]=='C' && uart1_rxbuf[i+1]=='M' && uart1_rxbuf[i+2]=='T' && uart1_rxbuf[i+3]==':'){
			stno=i+6;
			i = i+6;
		}
		if(stno!=0){
			uart1_rxbuf[j]=uart1_rxbuf[i];
			j++;
		}
		i++;
		if(i>=MAXBUFFER) break;
	}
	i=0; 
	j=0; 
	found=0; 
	stno=0; 
	enno=0;
	fotasmscheck=0;
	if(sim_flag==46 || sim_flag==47){
		while(1){
		// get Whitelist Num
		if(uart1_rxbuf[i]=='A' && uart1_rxbuf[i+1]=='T' && uart1_rxbuf[i+2]=='+' && uart1_rxbuf[i+3]=='M' && uart1_rxbuf[i+4]=='F' && found ==0){
			stno=i;
			for(j=0;j<100;j++){
						smsmsg1[j]=uart1_rxbuf[i];						
						found=stno;
					if(uart1_rxbuf[i]==0x0A && uart1_rxbuf[i-1]==0x0D){
						smsmsg1[j]=uart1_rxbuf[i];
						numdone=i;
						j=j+100;
					}
					i++;
			}
			i=i+5;
		}
		// get Message
		stno=0;
		enno=0;
		k=0;
		i++;
		if(found!=0) break;
		if(i>=MAXBUFFER) break;
		}
	
//		printf("\r\nsmsmsg1\r\n%s\r\n", smsmsg1);

		memset(smsmsg,0,sizeof(smsmsg));											
		sprintf(smsmsg,"MFOTA BY TCP ECM SOfTWARE OK\r\nMFOTA Starting. PLEASE WAIT IN 5 MINUTES\r\n");
//				send_sms();
		send_sms_withsmsnum();
		delay(60);
		delay(30);
		memcpy(smsmsg, smsmsg1, sizeof(smsmsg1));
		delay(10);
		numinwhitelist=1;
		mfota_start(1);	// FOTA
		return;
	}
	else {
//		printf("in Else SF46-47\r\n");
		tandapetik=0;
		while(1){
			// get Whitelist Num
			if(uart1_rxbuf[i]=='+' && found ==0){
				stno=i;
				for(j=0;j<14;j++){
						if(uart1_rxbuf[i+2]!=',' && enno==0 && numdone==0){
							smsnum[j]=uart1_rxbuf[i];						
							found=stno;
						}
						else {
							enno=i;
							found=enno;
						}
						if(uart1_rxbuf[i+2]==',' && numdone==0){
							smsnum[j]=uart1_rxbuf[i];
							numdone=i;
						}
						i++;
				}
				i=i+5;
				while(1){
					i++;
					if(uart1_rxbuf[i]==0x0A) {
						i++;
						found=i;
						break;
					}
					if(i>=MAXBUFFER) break;				
				}
			}		
			// get Message
			stno=0;
			enno=0;
			k=0;
			cmp_passcode=10;
			memset(passcode,0,sizeof(passcode));
			if(found!=0){
				j=0;
				stno=i;
				while(1){
					if(uart1_rxbuf[i]==0x0D && uart1_rxbuf[i+1]==0x0A) break;
					if(uart1_rxbuf[i]==0x0A) break;
					if(uart1_rxbuf[i]=='M' || uart1_rxbuf[i]=='F' || uart1_rxbuf[i]=='O' || uart1_rxbuf[i]=='T' || uart1_rxbuf[i]=='A') fotasmscheck++;
					if(uart1_rxbuf[i]=='"') tandapetik++;
					if(uart1_rxbuf[i]=='!' && (tandapetik==0 || tandapetik==2)) k++;
					if(k==0) smsmsg[j]=uart1_rxbuf[i];
					else if(k>1 && k<10) {
						passcode[k-2] = uart1_rxbuf[i];
//						if(fotasmscheck>4) smsmsg[j]=uart1_rxbuf[i]; // Update FOTA
						k++;
					}
					else if(k==1){
//						if(fotasmscheck>4) smsmsg[j]=uart1_rxbuf[i]; // Update FOTA
						k++;
					}
					j++;
					i++;
					if(i>=MAXBUFFER-2) break;
				}
			}
			i++;
			if(found!=0) break;
			if(i>=MAXBUFFER) break;
			}				
	}
		fotasmscheck=0;
		memset(smsnum1,0,sizeof(smsnum1));
		memcpy(smsnum1, smsnum, sizeof(smsnum1));
//		printf("AT+CMGD=1,4\r\nsmsnum1 %s\r\n", smsnum1);
		uart1_send_string("AT+CMGD=1,4\r\n");
		delay(20);
		uart1_index=0;
		memset(uart1_rxbuf,0,500);
		cmp_smsmsg=-1;
		numinwhitelist=0;
		cmpnum = strcmp(smsnum, white_list1);
		if(cmpnum==0) {
			 numinwhitelist=1;
		 } else {
			 numinwhitelist=0;
		 }
		delay(2);
		 if(numinwhitelist==0){
				cmpnum = strcmp(smsnum, white_list2);
			if(cmpnum==0) {
				 numinwhitelist=2;
			 } else {
				 numinwhitelist=0;
			 }
		 }
		delay(2);
		 if(numinwhitelist==0){
				cmpnum = strcmp(smsnum, white_list3);
				if(cmpnum==0) {
					 numinwhitelist=3;
				 } else {
					 numinwhitelist=0;
				 }
		 }
		delay(2);
		 process_sms=0;
//		 if(numinwhitelist!=0 && fmc_buf[76]==1){ // default WL EN
		 if(numinwhitelist!=0 && (fmc_buf[76]==1 || fmc_buf[76]==255)){ // default WL DIS
			process_sms=1;			 
		 }		 
		 else if(fmc_buf[76]==0 || fmc_buf[76]==255){ // default WL DIS
			process_sms=1;			 			 
		 }
//		 if(numinwhitelist!=0){
		 if(process_sms!=0){
			tempint = strncmp(passcode, passcode_saved,8);
			memset(smsmsg2,0,sizeof(smsmsg2));
			memcpy(smsmsg2, smsmsg, sizeof(smsmsg2));			
//			printf("\r\nsmsnum\r\n%s\r\nsms\r\n%s %s %d %s %d\r\n", smsnum, smsmsg, passcode, sizeof(passcode), passcode_saved, tempint); // DEBUG
			 if(tempint==0){
				cmp_smsmsg = strcmp_smsmsg();
				 switch(cmp_smsmsg){
					 case 0	 : 	memset(smsmsg,0,sizeof(smsmsg));
											memcpy(smsmsg, mipcall, sizeof(smsmsg));
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
					 case 1  :	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+MWL=%s,%s,%s",white_list1,white_list2,white_list3);
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
					 case 2  : 	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+MCFG=%s,%s,%d",APN, PORT,fmc_buf[65]);
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
					 case 3  : 	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"REBOOTING");
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											cmp_smsmsg=13;
											sim_flag=99;
											delay(20);
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
					 case 4  :	memset(smsmsg,0,sizeof(smsmsg));
											memcpy(smsmsg, mipcall, sizeof(smsmsg));
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
											// MWL - MCFG
					 case 5	 :	send_smscmdmode();
											check_set_fmc();
											delay(10);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
					 case 8  : 	send_smscmdmode();
											check_set_fmc();
//											memset(smsmsg,0,sizeof(smsmsg));
//											sprintf(smsmsg,"AT+MCFG=%s,%s,%d",APN, PORT,fmc_buf[65]);
//											send_sms();
//											send_sms_withsmsnum();
//											delay(30);
											delay(10);
											sprintf(temp_global,"AT+CGDCONT=1,\"IP\",\"%s\",\"0.0.0.0\",0,0,0,0\r\n", APN);
											uart1_send_string(temp_global);
											delay(30);
											memset(temp_global,0,sizeof(temp_global));					
											memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));					
											memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));					
											memset(uart0_rxbuf1,0,sizeof(uart0_rxbuf1));
											uart1_index=0;
											uart0_rxbuf_index=0;
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
											
					 case 9 	: send_smscmdmode();
											check_set_fmc();
//											memset(smsmsg,0,sizeof(smsmsg));
//											sprintf(smsmsg,"AT+MPC=%s",passcode_saved);
//											send_sms();
//											send_sms_withsmsnum();
//											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
											
					case 10 	: send_smscmdmode();
											check_set_fmc();
//											memset(smsmsg,0,sizeof(smsmsg));
//											sprintf(smsmsg,"AT+MBAUD=%d. Baudrate Default[255]=%d. Changing Baudrate Will take effect after Reboot.",fmc_buf[67], baudrateDefault);
//											send_sms_withsmsnum();
//											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;

					case 11 	: memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+MBAUD=%d. Baudrate Default[255]=%d",fmc_buf[67], baudrateDefault);
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;

					case 12 	: memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+MVER=%s",MCU_SOFT_VERSION);
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
											
					 case 13 :	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+SOFTRESET OK");
//											send_sms();
											send_sms_withsmsnum();
											delay(20);
											delay(10);
											cmp_smsmsg=13;
											sim_flag=99;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
// WRST SRST
					 case 14 : 	memset(smsmsg3,0,sizeof(smsmsg3));
											memcpy(smsmsg3, smsmsg, sizeof(smsmsg3));
											send_smscmdmode();
											memcpy(smsmsg, smsmsg3, sizeof(smsmsg));
											check_set_fmc();
//											check_set_fmc_byserial(7);
//											memset(smsmsg,0,sizeof(smsmsg));
//											sprintf(smsmsg,"AT+WRST=%d,%d\r\nOK\r\n",enablesrst, mintotal_srst);
//											send_sms();
//											send_sms_withsmsnum();
//											delay(20);
											delay(10);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
// HRST
					 case 15 : 	memset(smsmsg3,0,sizeof(smsmsg3));
											memcpy(smsmsg3, smsmsg, sizeof(smsmsg3));
											send_smscmdmode();
											memcpy(smsmsg, smsmsg3, sizeof(smsmsg));
											check_set_fmc();
//											check_set_fmc_byserial(8);
//											memset(smsmsg,0,sizeof(smsmsg));
//											sprintf(smsmsg,"AT+HRST=%d,%d\r\nOK\r\n",enablehrst, mintotal_hrst);
//											send_sms();
//											send_sms_withsmsnum();
//											delay(20);
											delay(10);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
// WRST Read											
					 case 16 :	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+WRST=%d,%d\r\nOK\r\n",enablesrst, mintotal_srst);
//											send_sms();
											send_sms_withsmsnum();
											delay(20);
											delay(10);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
// HRST Read											
					 case 17 :	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+HRST=%d,%d\r\nOK\r\n",enablehrst, mintotal_hrst);
//											send_sms();
											send_sms_withsmsnum();
											delay(20);
											delay(10);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
						 
					 case 18 : 	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+MGNET=%d,%d",network, getnetwork);
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
					 case 19 : 	uart1_send_string(smsmsg); 
											uart1_send_string("\r\n");
											delay(30);
											sim_flag=41;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
											// MFOTA
					 case 20 : 	memcpy(smsmsg1,smsmsg, sizeof(smsmsg));
											memset(smsmsg,0,sizeof(smsmsg));											
											sprintf(smsmsg,"AT+MFOTA OK\r\nMFOTA Starting. Please Wait in 5 Minutes\r\n");
//											send_sms();
											send_sms_withsmsnum();
											delay(60);
											delay(40);
											memcpy(smsmsg,smsmsg1, sizeof(smsmsg));
											mfota_start(3);
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
											
					 case 21 :	memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+MFW2 OK\r\nREBOOTING NOW\r\nPlease Wait in 5 Minutes");
//											send_sms();
											send_sms_withsmsnum();
											delay(50);
											// Jump to user application 
											fmc_buf[255]=1;
											fmc_buf[254]=1;
											fmc_data_backup[255]=1;
											fmc_data_backup[254]=1;
											fmc_buf[240]=100;
											fmc_data_backup[240]=100;
											fmc_erase_pages();
											fmc_program2();
											fmc_backup();
											cmp_smsmsg=13;
											sim_flag=99;
											sim_flag_old=99;
//											jump_to_address_fw2();
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
// ENWL
					 case 23 	: memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+ENWL=%d",fmc_buf[76]);
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
// ENWL

					 case 24 	: memcpy(smsmsg1, smsmsg, sizeof(smsmsg));
											memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"%s. \r\nWORKING. Rebooting Now. Please Wait in 3 mins!",smsmsg1);
											if(numinwhitelist!=0)send_sms();
											else send_sms_withsmsnum();
											delay(30);
											memcpy(smsmsg, smsmsg1, sizeof(smsmsg1));
											check_set_fmc();
											memset(smsmsg,0,sizeof(smsmsg));
											memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
											uart1_index=0;
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;

// REOPS
					 case 25 	: memcpy(smsmsg1, smsmsg, sizeof(smsmsg));
											memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"%s. \r\nWORKING. Reset Modem Now. Please Wait in 3 mins!",smsmsg1);
											printf("%s",smsmsg);
											if(numinwhitelist!=0)send_sms();
											else send_sms_withsmsnum();
											delay(30);
											memcpy(smsmsg, smsmsg1, sizeof(smsmsg1));
											memset(smsmsg,0,sizeof(smsmsg));
											memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
											uart1_index=0;
											afterreset=100;
											uart1_send_string("AT+MIPCLOSE=1,0\r\n");
											delay(150);
											cmp_smsmsg=13;
											sim_flag=98;
											sim_flag_old=98;
//											has_configured=0;
//											sim_flag=1; // ori sim_flag=2
//											sim_flag_old=1; // ori sim_flag_old=2
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
											break;
// SFLAG
					 case 26 	: memset(smsmsg,0,sizeof(smsmsg));
											sprintf(smsmsg,"AT+SFLAG=%d, %d, %d, %d, %d",sim_flag, sim_flag_old, sim_flag_old1, sim_flag_old2, socket_id);
//											send_sms();
											send_sms_withsmsnum();
											delay(30);
											sim_flag=sim_flag_old;
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG

					 case 100:  uart1_send_string(smsmsg); 
											uart1_send_string("\r\n");
											sim_flag=26;					
//											printf("\r\ncase %d OK\r\n", cmp_smsmsg); // DEBUG
										break;
					 default : 	break;
				 }
/*				
				else if(cmp_smsmsg==10){
					check_set_fmc();
					memset(smsmsg,0,sizeof(smsmsg));
					sprintf(smsmsg,"AT+MBAUD=%d. Changing Baudrate Will take effect after Reboot.",fmc_buf[67]);
					send_sms();
					delay(30);
					sim_flag=sim_flag_old;
				}
				else if(cmp_smsmsg==11){
					memset(smsmsg,0,sizeof(smsmsg));
					sprintf(smsmsg,"AT+MBAUD=%d",fmc_buf[67]);
					send_sms();
					delay(30);
					sim_flag=sim_flag_old;
				}				
				else if(cmp_smsmsg==9){
				 
				}
				else if(cmp_smsmsg==0){
					memset(smsmsg,0,sizeof(smsmsg));
					memcpy(smsmsg, mipcall, sizeof(smsmsg));
					send_sms();
					delay(30);
					sim_flag=sim_flag_old;
				}
				else if(cmp_smsmsg==19){
				}				
				else if(cmp_smsmsg==100){
					uart1_send_string(smsmsg); 
					uart1_send_string("\r\n");
					sim_flag=26;					
				}*/
				if(cmp_smsmsg==3) {
					cmp_smsmsg=13;
					sim_flag=99;
				}
			 }
			 else {				 
					strcat(smsmsg,"\nERROR CODE!");
					send_sms_withsmsnum();
					delay(30);
					sim_flag=sim_flag_old;
			 }
		 }
		 else {
//					printf("in else processsms=0. smsnum\r\n%s %s\r\nsms\r\n%s \r\n", smsnum, smsnum1, smsmsg); // DEBUG
//					strcat(smsmsg,"\nERROR!");
//					send_sms_withsmsnum();
//					delay(30);
					sim_flag=sim_flag_old;
		 }
		 
}

void uart0_handle(void)
{
    static uint8_t at_start_flag = 0;

    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
			char ch = (char)usart_data_receive(USART0);
			if(LEDBStatus) led_off(LEDB);
			else led_on(LEDB);
//			if((sim_flag!=27 && sim_flag!=28 && sim_flag_old1!=27 && sim_flag_old1!=28) && (sim_flag==4 || sim_flag==14)){
			uart0_rxbuf1[rx_cnt] = ch;
			if(sim_flag!=27 && sim_flag!=28){
					rx_cnt=0;
					sim_flag_old1=sim_flag;
					sim_flag_old2=sim_flag_old;
					sim_flag=27;
			}
			if(rx_cnt>3){
				if(ch=='E' && uart0_rxbuf1[rx_cnt-1]=='I' && uart0_rxbuf1[rx_cnt-2]=='M' && uart0_rxbuf1[rx_cnt-3]=='D' && uart0_rxbuf1[rx_cnt-4]=='E'){
					modecfg=1;
					sim_flag=28;
				}
				if(ch=='L' && uart0_rxbuf1[rx_cnt-1]=='D' && uart0_rxbuf1[rx_cnt-2]=='+' && uart0_rxbuf1[rx_cnt-3]=='T' && uart0_rxbuf1[rx_cnt-4]=='A'){
					modecfg=1;
					sim_flag=28;
				}
//				else if(uart0_rxbuf1[rx_cnt-2]=='T' && uart0_rxbuf1[rx_cnt-3]=='A' && sim_flag!=28){
//					sim_flag=30;
//				}
			}
			rx_cnt++;
			counter_wait_mtr=0;
//				else sim_flag=27;
			if(rx_cnt>2){
				if(uart0_rxbuf1[rx_cnt-3]==0x2B && uart0_rxbuf1[rx_cnt-2]==0x2B && uart0_rxbuf1[rx_cnt-1]==0x2B) {
					idx_plus=3;
					sim_flag=98;
					cmp_smsmsg=13;
				}
			}
			if(rx_cnt>=MAXBUFFER){
				rx_cnt=0;
			}
			if(rx_cnt>=MAXBUFFER-1 && (sim_flag_old1!=14 || sim_flag_old1!=18 || sim_flag_old1!=20 || sim_flag_old1!=21 || sim_flag_old1!=22 || sim_flag_old1!=23 || sim_flag_old1!=25)) {
				rx_cnt=0;
			}				
    }
}

void uart1_handle(void){
	if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)){
			uint8_t ch = usart_data_receive(USART1);
//			rx_now= ch;
			uart1_rxbuf[uart1_index]= ch;
			uart1_index++;
			counter_wait_sim=0;
			if(sim_flag==14 ){
				cntcclk=0;
				counter_wait_sim=0;
				uart0_rxbuf_index=0;
				uart1_index=0;
				sim_flag=34;
				uart0_rxbuf[uart0_rxbuf_index] = ch;
				uart0_rxbuf_index++;
				uart1_index++;
				sim_flag=34;
				if(uart1_index>3){
					// MFOTA
					if(uart1_rxbuf[uart1_index-1]=='A' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='O' && uart1_rxbuf[uart1_index-4]=='F'){
						sim_flag_old=sim_flag;
						sim_flag=46;
					}
					if(uart1_rxbuf[uart1_index-1]==0x0A && uart1_rxbuf[uart1_index-2]==0x0D && uart1_rxbuf[uart1_index-3]=='K' && uart1_rxbuf[uart1_index-4]=='O'){
						if(TESTMODE==0){							
							memset(uart0_rxbuf,0,uart0_rxbuf_index);
							memset(uart1_rxbuf,0,uart1_index);
							uart1_index=0;
							uart0_rxbuf_index=0;
						}
						sim_flag=14; // ori sf14
					}				
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
//						sim_flag=21;
						sim_flag=97;
					}
					// CMT SMS
//					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='C'){
//						sim_flag_old=sim_flag;
//						sim_flag=16;
//					}
					// CLIP:
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='P' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='L'){
							sim_flag_old = sim_flag;
							sim_flag=15;
					}
					// RING
					else if(uart1_rxbuf[uart1_index-1]=='G' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='R'){
						sim_flag_old = sim_flag;
						sim_flag=15;
					}
				}
			}
			else if(sim_flag==34){
				counter_wait_sim=0;
				uart0_rxbuf[uart0_rxbuf_index] = ch;
				uart0_rxbuf_index++;
				sim_flag=34;
				if(uart1_index>3){
					// MFOTA
					if(uart1_rxbuf[uart1_index-1]=='A' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='O' && uart1_rxbuf[uart1_index-4]=='F'){
						sim_flag_old=sim_flag;
						sim_flag=46;
					}
					if(sim_flag!=42 && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='A' && uart1_rxbuf[uart1_index-4]=='T'){
						sim_flag=3;
					}				
					else if(uart1_rxbuf[uart1_index-1]=='P' && uart1_rxbuf[uart1_index-2]=='I' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='+'){
						sim_flag_old=sim_flag;
						sim_flag=24;
					}				
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
//						sim_flag=21;
						sim_flag=97;
					}
//					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
//						sim_flag_old=sim_flag;
//						sim_flag=21;					
//					}
					else if(sim_flag==24 && uart1_rxbuf[uart1_index-1]==0x0A && uart1_rxbuf[uart1_index-2]==0x0D && uart1_rxbuf[uart1_index-3]=='K' && uart1_rxbuf[uart1_index-4]=='O'){
						if(TESTMODE==0){							
							memset(uart0_rxbuf,0,uart0_rxbuf_index);
							memset(uart1_rxbuf,0,uart1_index);
							uart1_index=0;
							uart0_rxbuf_index=0;
						}
						sim_flag=sim_flag_old;
					}				
					// CLIP:
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='P' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='L'){
							sim_flag_old = sim_flag;
							sim_flag=15;
					}
					// RING
					else if(uart1_rxbuf[uart1_index-1]=='G' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='R'){
						sim_flag_old = sim_flag;
						sim_flag=15;
					}
					// CMT SMS
//					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='C'){
//						sim_flag_old=sim_flag;
//						sim_flag=16;
//					}
					else if(uart1_rxbuf[uart1_index-1]=='D' && uart1_rxbuf[uart1_index-2]=='A' && uart1_rxbuf[uart1_index-3]=='O' && uart1_rxbuf[uart1_index-4]=='L'){
						sim_flag=40;
					}
				}
			}
			else if(sim_flag==24){
				counter_wait_sim=0;
				uart0_rxbuf[uart0_rxbuf_index] = ch;
				uart0_rxbuf_index++;
//				sim_flag=24;
				if(uart1_index>3){
					// MFOTA
					if(uart1_rxbuf[uart1_index-1]=='A' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='O' && uart1_rxbuf[uart1_index-4]=='F'){
						sim_flag_old=sim_flag;
						sim_flag=46;
					}
					if(sim_flag!=42 && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='A' && uart1_rxbuf[uart1_index-4]=='T'){
//						socket_id=0;
//						socket_id=0;
						sim_flag=3;
					}				
					else if(uart1_rxbuf[uart1_index-1]=='P' && uart1_rxbuf[uart1_index-2]=='I' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='+'){
						sim_flag=24;
					}				
					else if(uart1_rxbuf[uart1_index-1]=='D' && uart1_rxbuf[uart1_index-2]=='A' && uart1_rxbuf[uart1_index-3]=='O' && uart1_rxbuf[uart1_index-4]=='L'){
						sim_flag=40;
					}
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
//						sim_flag=21;
						sim_flag=97;
					}
//					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
//						sim_flag_old=sim_flag;
//						sim_flag=21;					
//					}
					if(uart1_rxbuf[uart1_index-1]==0x0A && uart1_rxbuf[uart1_index-2]==0x0D && uart1_rxbuf[uart1_index-3]=='K' && uart1_rxbuf[uart1_index-4]=='O'){
						if(TESTMODE==0){							
							memset(uart0_rxbuf,0,uart0_rxbuf_index);
							memset(uart1_rxbuf,0,uart1_index);
							uart1_index=0;
							uart0_rxbuf_index=0;
						}
						sim_flag=sim_flag_old;
					}				
					// CLIP:
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='P' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='L'){
							sim_flag_old = sim_flag;
							sim_flag=15;
					}
					// RING
					else if(uart1_rxbuf[uart1_index-1]=='G' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='R'){
						sim_flag_old = sim_flag;
						sim_flag=15;
					}
					// CMT SMS
//					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='C'){
//						sim_flag_old=sim_flag;
//						sim_flag=16;
//					}
				}
			}
			else if(sim_flag==7){
				counter_wait_sim=0;
				uart0_rxbuf[uart0_rxbuf_index] = ch;
				uart0_rxbuf_index++;
				if(ch=='>'){
					uart0_rxbuf_index=0;
					memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));
					uart1_index=0;
					memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
				}
				else if(uart1_index>3){
					if(sim_flag!=42 && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='A' && uart1_rxbuf[uart1_index-4]=='T'){
						sim_flag=3;
					}				
					else if(uart1_rxbuf[uart1_index-1]=='P' && uart1_rxbuf[uart1_index-2]=='I' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='+'){
						sim_flag_old=sim_flag;
						sim_flag=24;
					}				
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
						sim_flag=21;					
					}
//					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
//						sim_flag_old=sim_flag;
//						sim_flag=21;					
//					}
					if(uart1_rxbuf[uart1_index-1]==0x0A && uart1_rxbuf[uart1_index-2]==0x0D && uart1_rxbuf[uart1_index-3]=='K' && uart1_rxbuf[uart1_index-4]=='O'){
						if(TESTMODE==0){							
							memset(uart0_rxbuf,0,uart0_rxbuf_index);
							memset(uart1_rxbuf,0,uart1_index);
							uart1_index=0;
							uart0_rxbuf_index=0;
						}
						sim_flag=sim_flag_old;
					}				
					// CMT SMS
//					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='C'){
//						sim_flag_old=sim_flag;
//						sim_flag=16;
//					}
					// CLIP:
					else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='P' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='L'){
							sim_flag_old = sim_flag;
							sim_flag=15;
					}
					// RING
					else if(uart1_rxbuf[uart1_index-1]=='G' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='R'){
						sim_flag_old = sim_flag;
						sim_flag=15;
					}
					else if(uart1_rxbuf[uart1_index-1]=='D' && uart1_rxbuf[uart1_index-2]=='A' && uart1_rxbuf[uart1_index-3]=='O' && uart1_rxbuf[uart1_index-4]=='L'){
						sim_flag=40;
					}
				}
//				sim_flag=14;
			}
			// FOTA // sim_flag44
			else if(sim_flag==44){
				counter_wait_sim=0;
				sim_flag=44;
			}
			else if(uart1_index>3){
				counter_wait_sim=0;
			if(uart1_rxbuf[uart1_index-1]=='T' && uart1_rxbuf[uart1_index-2]=='A' && sim_flag==0){
				sim_flag=1;
				uart1_index=0;
				memset(uart1_rxbuf,0,MAXBUFFER);
				delay(10);
				sim_flag=1;
			}
			if(uart1_rxbuf[uart1_index-1]=='Y' && uart1_rxbuf[uart1_index-2]=='D' && uart1_rxbuf[uart1_index-3]=='A'){
				sim_flag=1;
				uart1_index=0;
				memset(uart1_rxbuf,0,MAXBUFFER);
			}
			else if(uart1_rxbuf[uart1_index-1]=='T' && uart1_rxbuf[uart1_index-2]=='A' && sim_flag==0){
				uart1_index=0;
				memset(uart1_rxbuf,0,MAXBUFFER);
				delay(10);
				sim_flag=1;
			}
			// CMT
			else if(sim_flag!=16 && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='T' && uart1_rxbuf[uart1_index-3]=='M' && uart1_rxbuf[uart1_index-4]=='C'){
				if(sim_flag==3 || sim_flag==4 || sim_flag==14 || sim_flag==24 || sim_flag==34 || sim_flag==44 || sim_flag==7 || sim_flag==27) sim_flag_old = sim_flag;
				sim_flag=16;
			}
			// MIPCALL
			else if((sim_flag<3 || sim_flag==11) && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
				sim_flag=18;					
			}
			// MIPCALL:
			else if((sim_flag==3 || sim_flag==4 || sim_flag==14 || sim_flag==24 || sim_flag==34) && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
				sim_flag=21;					
			}
			// MIPCALL=
			else if(sim_flag<3 && uart1_rxbuf[uart1_index-1]=='=' && uart1_rxbuf[uart1_index-2]=='L' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='A'){
				sim_flag=11;
			}
			// COPS
			else if(sim_flag!=21 && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='S' && uart1_rxbuf[uart1_index-3]=='P' && uart1_rxbuf[uart1_index-4]=='O'){
				if(sim_flag_old<3) sim_flag=22;
				else sim_flag=25;
			}
			// CCLK
			else if(sim_flag!=21 && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='K' && uart1_rxbuf[uart1_index-3]=='L' && uart1_rxbuf[uart1_index-4]=='C'){
				sim_flag=19;
			}
			// MIPOPEN sim_flag16 26
			else if((sim_flag==16 || sim_flag==26 ) && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='E' && uart1_rxbuf[uart1_index-4]=='P'){
				
			}
			// MIPOPEN sim_flag32
			else if(sim_flag==32 && uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='E' && uart1_rxbuf[uart1_index-4]=='P'){
				
			}
			else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='E' && uart1_rxbuf[uart1_index-4]=='P'){
//				if(sim_flag!=16 ) sim_flag=13;					
//				if(sim_flag!=2) sim_flag=13;
					sim_flag=13;
			}
			// CLIP=
			else if(uart1_rxbuf[uart1_index-1]=='=' && uart1_rxbuf[uart1_index-2]=='P' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='L'){
				if(sim_flag!=1) sim_flag=12;
			}
			// CLIP:
			else if(uart1_rxbuf[uart1_index-1]==':' && uart1_rxbuf[uart1_index-2]=='P' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='L'){
				if(sim_flag==3 || sim_flag==4 || sim_flag==14 || sim_flag==24 || sim_flag==34 || sim_flag==44 || sim_flag==7 || sim_flag==27) sim_flag_old = sim_flag;
					sim_flag=15;
			}
			// RING
			else if(uart1_rxbuf[uart1_index-1]=='G' && uart1_rxbuf[uart1_index-2]=='N' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='R'){
				if(sim_flag==3 || sim_flag==4 || sim_flag==14 || sim_flag==24 || sim_flag==34 || sim_flag==44 || sim_flag==7 || sim_flag==27) sim_flag_old = sim_flag;
				sim_flag=15;
			}
			else if(uart1_rxbuf[uart1_index-1]=='D' && uart1_rxbuf[uart1_index-2]=='A' && uart1_rxbuf[uart1_index-3]=='O' && uart1_rxbuf[uart1_index-4]=='L'){
						sim_flag=40;
			}
			if(sim_flag==24 && uart1_rxbuf[uart1_index-1]==0x0A && uart1_rxbuf[uart1_index-2]==0x0D && uart1_rxbuf[uart1_index-3]=='K' && uart1_rxbuf[uart1_index-4]=='O'){
						if(TESTMODE==0){							
							memset(uart0_rxbuf,0,uart0_rxbuf_index);
							memset(uart1_rxbuf,0,uart1_index);
							uart1_index=0;
							uart0_rxbuf_index=0;
						}
						sim_flag=sim_flag_old;
					}				

			if(sim_flag==11 && uart1_rxbuf[uart1_index-1]=='K' && uart1_rxbuf[uart1_index-2]=='O'){
				uart1_index=0;
				cntmipcall=0;
				memset(uart1_rxbuf,0,MAXBUFFER);
				sim_flag=18;
				delay(10);
			}
			else if(sim_flag==11 && uart1_rxbuf[uart1_index-1]=='R' && uart1_rxbuf[uart1_index-2]=='O' && uart1_rxbuf[uart1_index-3]=='R'){
				cntmipcall++;
				if(mipcall[0]==0 && mipcall[1]==0 && mipcall[2]==0 && mipcall[3]==0 && mipcall[4]==0 && mipcall[5]==0 && mipcall[6]==0) sim_flag=1;
				else sim_flag=2;
				uart1_index=0;
				memset(uart1_rxbuf,0,MAXBUFFER);
				if(cntmipcall>=20) sim_flag=22;
			}
			if(sim_flag==18 && uart1_rxbuf[uart1_index-1]=='\n'){
					sim_flag=2;
					uart1_index=0;
					memset(mipcall,0,sizeof(mipcall));
					memcpy(mipcall, uart1_rxbuf, sizeof(mipcall));
					memset(uart1_rxbuf,0,MAXBUFFER);						
			}
			else if(sim_flag==21 && uart1_rxbuf[uart1_index-1]=='\n'){
					counter_wait_sim=10;
			}
			if(sim_flag==32 && uart1_rxbuf[uart1_index-1]==',' && uart1_rxbuf[uart1_index-4]==':'){ // -4 :, -3 < >, -2 angka, -1 ,
					tempint=hex2int(uart1_rxbuf[uart1_index-2]);
					checksocket = tempint;
			}
			else if(sim_flag==13 && uart1_rxbuf[uart1_index-1]==',' && uart1_rxbuf[uart1_index-4]==':'){ // -4 :, -3 < >, -2 angka, -1 ,
					tempint=hex2int(uart1_rxbuf[uart1_index-2]);
					socket_id = tempint;
					if(socket_id==1) {
						sim_flag=3;
					}
					else if(socket_id==0) {
						sim_flag=2;
					}
			}
			else if(sim_flag==13 && uart1_rxbuf[uart1_index-1]=='\n'){
				sim_flag=14; // ori sf14
				uart1_index=0;					
				memset(uart1_rxbuf,0,MAXBUFFER);
			}
			else if(sim_flag==15 && uart1_rxbuf[uart1_index-1]=='\n'){
					sim_flag=5;
			}
//			else if(sim_flag==23 && uart1_rxbuf[uart1_index-1]=='\n'){
//				sim_flag=3;
//				uart1_index=0;					
//				memset(uart1_rxbuf,0,MAXBUFFER);
//			}
			if(sim_flag==29){
				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, uart1_index);
			}
			if(sim_flag==29 && uart1_rxbuf[uart1_index-3]=='\n' && uart1_rxbuf[uart1_index-4]=='\r' && uart1_rxbuf[uart1_index-1]=='K' && uart1_rxbuf[uart1_index-2]=='O'){
//				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, sizeof(smsmsg));
				uart1_index=0;					
				memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
			}
			else if(sim_flag==29 && uart1_rxbuf[uart1_index-1]=='\n' && uart1_rxbuf[uart1_index-2]=='\r' && uart1_rxbuf[uart1_index-3]=='R' && uart1_rxbuf[uart1_index-4]=='O'){
//				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, sizeof(smsmsg));
				uart1_index=0;					
				memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
			}
			if(sim_flag==26){
				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, uart1_index);
			}
			// OK
			if(sim_flag==26 && uart1_rxbuf[uart1_index-3]=='\n' && uart1_rxbuf[uart1_index-4]=='\r' && uart1_rxbuf[uart1_index-1]=='K' && uart1_rxbuf[uart1_index-2]=='O'){
				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, sizeof(smsmsg));
				sim_flag=36;
				uart1_index=0;					
				memset(uart1_rxbuf,0,MAXBUFFER);
			}
			// ERROR
			else if(sim_flag==26 && uart1_rxbuf[uart1_index-4]=='R' && uart1_rxbuf[uart1_index-3]=='R' && uart1_rxbuf[uart1_index-2]=='O' && uart1_rxbuf[uart1_index-1]=='R'){
				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, sizeof(smsmsg));
				sim_flag=36;
				uart1_index=0;					
				memset(uart1_rxbuf,0,MAXBUFFER);
			}
			else if(sim_flag==43 && uart1_rxbuf[uart1_index-4]=='T' && uart1_rxbuf[uart1_index-3]==':'){
				// FTPGET:
				tempint2=-1;
				tempint=tempint+20;
					memcpy(smsmsg,uart1_rxbuf,sizeof(smsmsg));
/*				if(uart1_rxbuf[uart1_index-1]=='1'){
					tempint=tempint+40;
					tempint2=0;
					sim_flag=44;
				}*/
					uart1_index=0;
					memset(uart1_rxbuf,0,MAXBUFFER);
			}
			// FTPRECV: 0
//			else if(sim_flag==44 && uart1_rxbuf[uart1_index-4]=='V' && uart1_rxbuf[uart1_index-3]==':' && uart1_rxbuf[uart1_index-2]==' ' && uart1_rxbuf[uart1_index-1]=='0' ){
				// FTPRECV: 0
//				tempint2=2;
//				counter_wait_sim=0;
//				sim_flag=44;
//			}
//			else if(sim_flag==44){
//				counter_wait_sim=0;
//				sim_flag=44;
//			}
			else if(sim_flag==12 && uart1_rxbuf[uart1_index-3]=='\n' && uart1_rxbuf[uart1_index-4]=='\r' && uart1_rxbuf[uart1_index-1]=='K' && uart1_rxbuf[uart1_index-2]=='O'){
				if(sim_flag_old<3) sim_flag=22;
				else sim_flag=sim_flag_old;
				uart1_index=0;					
				memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
			}
			else if(sim_flag==12 && uart1_rxbuf[uart1_index-3]=='\n' && uart1_rxbuf[uart1_index-4]=='\r' && uart1_rxbuf[uart1_index-1]=='R' && uart1_rxbuf[uart1_index-2]=='O'){
				sim_flag=2;
				uart1_index=0;					
				memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
			}
			else if(sim_flag==22 && uart1_rxbuf[uart1_index-3]=='\n' && uart1_rxbuf[uart1_index-4]=='\r' && uart1_rxbuf[uart1_index-1]=='K' && uart1_rxbuf[uart1_index-2]=='O'){
				if(sim_flag_old<4) sim_flag=23;
//				else sim_flag=sim_flag_old;
				else sim_flag=25;
			}
			else if(sim_flag==22 && uart1_rxbuf[uart1_index-3]=='R' && uart1_rxbuf[uart1_index-4]=='R' && uart1_rxbuf[uart1_index-1]=='R' && uart1_rxbuf[uart1_index-2]=='O'){
				if(sim_flag_old<4) sim_flag=23;
//				else sim_flag=sim_flag_old;
				else sim_flag=25;
				uart1_index=0;					
			}
			else if(sim_flag==19 && uart1_rxbuf[uart1_index-3]=='\n' && uart1_rxbuf[uart1_index-4]=='\r' && uart1_rxbuf[uart1_index-1]=='K' && uart1_rxbuf[uart1_index-2]=='O'){
				sim_flag=20;
			}
			else if(sim_flag==19 && uart1_rxbuf[uart1_index-3]=='R' && uart1_rxbuf[uart1_index-4]=='R' && uart1_rxbuf[uart1_index-1]=='R' && uart1_rxbuf[uart1_index-2]=='O'){
				sim_flag=20;
				uart1_index=0;					
			}
			else if(uart1_rxbuf[uart1_index-1]=='R' && uart1_rxbuf[uart1_index-2]=='E' && uart1_rxbuf[uart1_index-3]=='I' && uart1_rxbuf[uart1_index-4]=='R'){
					if(sim_flag==3 || sim_flag==4 || sim_flag==14 || sim_flag==24 || sim_flag==34 || sim_flag==44) sim_flag_old = sim_flag;
					sim_flag=5;
//				sim_flag=45;
//				if(fmc_buf[76]==0) process_sms=1;
				uart1_index=0;
				memset(uart1_rxbuf,0,MAXBUFFER);
			}
			if(sim_flag==40 && uart1_rxbuf[uart1_index-1]=='T' && uart1_rxbuf[uart1_index-2]=='R' && uart1_rxbuf[uart1_index-3]=='A'){
//				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, uart1_index-1);
				sim_flag=37;
				uart1_index=0;					
				memset(uart1_rxbuf,0,MAXBUFFER);
			}
			else if(uart1_rxbuf[uart1_index-1]=='S' && uart1_rxbuf[uart1_index-2]=='S' && uart1_rxbuf[uart1_index-3]=='E' && uart1_rxbuf[uart1_index-4]=='C'){
//				memset(smsmsg,0,sizeof(smsmsg));
				memcpy(smsmsg, uart1_rxbuf, uart1_index-1);
				sim_flag=38;
				uart1_index=0;					
				memset(uart1_rxbuf,0,MAXBUFFER);
			}
//			if(sim_flag==40 && uart1_rxbuf[uart1_index-1]=='\n' && uart1_rxbuf[uart1_index-2]=='\r'){
//				memset(smsmsg,0,sizeof(smsmsg));
//				memcpy(smsmsg, uart1_rxbuf, uart1_index);
//			}

			if(uart1_index>=MAXBUFFER){
				uart1_index=0;
			}
		}			
		if(uart1_index>=MAXBUFFER){
				uart1_index=0;
				memset(uart1_rxbuf,0,MAXBUFFER);				
		}
    }
}

void extractnum_clip(){
	cmp_caller  = -1;
	int i=0, j=0, found=0, stno=0, enno=0, numdone=0;
	while(1){
		if(uart1_rxbuf[i]==':' && uart1_rxbuf[i-1]=='P' && uart1_rxbuf[i-2]=='I' && uart1_rxbuf[i-3]=='L'){
			i=i+3;
			// get the caller number
			for(j=0;j<14;j++){
					if(uart1_rxbuf[i+2]!=',' && enno==0 && numdone==0){
						callernum[j]=uart1_rxbuf[i];
//						if(TESTMODE)   usart_data_transmit(USART0, callernum[j]);
						found=stno;
					}
					else {
						enno=i;
						found=enno;
					}
					if(uart1_rxbuf[i+2]==',' && uart1_rxbuf[i+1]=='\"' && numdone==0){
						callernum[j]=uart1_rxbuf[i];
						numdone=i;
					}
					i++;
			}
		}
		i++;
		if(i>=MAXBUFFER) break;
	}
	uart1_index=0;
	memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
	delay(20);
		cmpnum = strncmp(callernum, white_list1,15);
		numinwhitelist=1;
		 if(cmpnum!=0){
				cmpnum = strncmp(callernum, white_list2,15);
				numinwhitelist=2;
		 }
		 if(cmpnum!=0){
				cmpnum = strncmp(callernum, white_list3, 15);
				numinwhitelist=3;
		 }
		 if(cmpnum==0){
				cmp_caller = cmpnum;
			 delay(10);
				sim_flag=35;
		 }
		 else {
			numinwhitelist=0;
			 cmp_caller = -1;
			 sim_flag=3;
			 delay(10);
			 sim_flag=sim_flag_old;
		 }
		 process_sms=0;
//		 if(cmpnum==0 && fmc_buf[76]==1){ // default WL EN
		 if(cmpnum==0 && (fmc_buf[76]==1 || fmc_buf[76]==255)){ // default WL DIS
			process_sms=1;			 
		 }
		 if(fmc_buf[76]==0 || fmc_buf[76]==255){ // default WL DIS
			process_sms=1;			 			 
		 }
		memset(smsnum1,0,sizeof(smsnum1));
		memcpy(smsnum1, callernum, sizeof(smsnum1));
		printf("extractnum Callernum %s %d %d\r\n",callernum, cmpnum, process_sms);
		 callingcnt++;
		if(callingcnt>3 && process_sms==1){
			if(sim_flag_old==14 || sim_flag_old==24 || sim_flag_old==34 || sim_flag_old==44 || sim_flag_old==7 || sim_flag_old==27) {
				rebootcmd++;
				if(rebootcmd>200) rebootcmd=200;
			}
			else reboot_by_calling();
		}
		else {
			numinwhitelist=0;
			 cmp_caller = -1;
			 sim_flag=3;
			 delay(10);
			 sim_flag=sim_flag_old;
		}
}

void reboot_by_calling(){
	memset(callernum,0,sizeof(callernum));
//		if(cmp_caller == 0){
		if(process_sms == 1){
			 delay(10);
				cmp_caller = -1;
			 //NVIC_SystemReset();
			sprintf(smsmsg,"REBOOTING BY CALLING OK");
//											send_sms();
			send_sms_withsmsnum();
			delay(100);
			cmp_smsmsg=3;
			 sim_flag=99;
			 delay(10);
		 }
		 else {
			 cmp_caller = -1;
			 sim_flag=sim_flag_old;
		 }
			wdt_off(1);
		uart1_index=0;					
		memset(uart1_rxbuf,0,MAXBUFFER);
}

void send_data_server(){
	memset(uart1_tx,0,MAXBUFFER);
	int i=0;
	for(i=0;i<len_rxmeter;i++){
		usart_data_transmit(USART1, uart0_rxbuf1[i]);
	}
}
void prepare_data_server(){
	memset(uart1_tx,0,MAXBUFFER);
	sprintf(uart1_tx,"AT+MIPSEND=%d,%d\r\n",socket_id,len_rxmeter);
	uart1_send_string(uart1_tx);
	delay(10);
	send_data_server();
//	sim_flag=17;
}

void send_to_meter(){
	rx_cnt=0;
	counter_wait_mtr=0;
	len_rxmeter=0;
	int i=0, j=0, len=0;
	for(i=0;i<len_meter_buf;i++){
	   usart_data_transmit(USART0, uart0_rxbuf[i]);		
	}
}

uint8_t hex2int(char hex) {
    uint32_t val = 0;
			// get current character then increment
			uint8_t byte = hex; 
			// transform hex character to the 4bit equivalent number, using the ascii table indexes
			if (byte >= '0' && byte <= '9') byte = byte - '0';
			else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
			else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
			// shift 4 to make space for new digit, and add the 4 bits of the new digit 
			val = (val << 4) | (byte & 0xF);
    return val;
}

void checking_mipcall(){
	int i=0;
//	printf("Checking_MIPCALL\r\n");
	while(1){
		if(i<999){
			if(temp_global[i]=='A' && temp_global[i+1]=='L' && temp_global[i+2]=='L' && temp_global[i+3]==':' && temp_global[i+5]=='0'){
				//printf("mipcall 0\r\n");
				i = MAXBUFFER+6;
				cmp_smsmsg = 13;
				sim_flag=98;
				sim_flag_old=98;
			}			
		}
		i++;
		if(i>=999) break;
	}
	memcpy(mipcall, temp_global, sizeof(mipcall));
//	printf("%.30s\r\n",temp_global);
	uart1_index=0;
	memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
	if(sim_flag!=98) sim_flag = sim_flag_old;
}

/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fota_fmc_erase_pages(void)
{
//		FLASH_DisableWriteProtectionPages();
    uint32_t EraseCounter;
		
//		FotaPageNum = (fota_write_end_addr - fota_write_start_addr) / FOTA_FMC_PAGE_SIZE;
	FotaPageNum = 1;

//		printf("FOTAFMCERASE  %u  %u  %u\r\n", fota_write_start_addr, FotaPageNum, fota_write_start_addr + FOTA_FMC_PAGE_SIZE);
//		delay(10);

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < FotaPageNum; EraseCounter++){
        fmc_page_erase(fota_write_start_addr + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fota_fmc_erase_allpages(void)
{
//		FLASH_DisableWriteProtectionPages();
    uint32_t EraseCounter;
		
		FotaPageNum = (fota_write_end_addr - fota_write_start_addr) / FOTA_FMC_PAGE_SIZE;

//		printf("FOTAFMCERASE  %u  %u  %u\r\n", fota_write_start_addr, FotaPageNum, fota_write_start_addr + FOTA_FMC_PAGE_SIZE);
//		delay(10);

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < FotaPageNum; EraseCounter++){
        fmc_page_erase(fota_write_start_addr + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

void fota_fmc_erase_1page(void)
{
    uint32_t EraseCounter;

		FotaPageNum = 1;
//		printf("FOTAFMCPROG  %08jx %d\r\n", (uintmax_t)fota_write_start_addr, FotaPageNum);
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < PageNum; EraseCounter++){
        fmc_page_erase(fota_write_start_addr + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/

void fota_fmc_program(void)
{
//	printf("FOTAFMCPROG %08jx %08jx %08jx %08jx\r\n", (uintmax_t)fota_write_start_addr, (uintmax_t)fota_fw1[0], (uintmax_t)fota_fw1[256], (uintmax_t)fota_fw1[500]);
//	if(numinwhitelist==0) printf("Mem %u %u %u\r\n", fota_write_start_addr, fota_write_end_addr, fota_fw1[0]);
//	delay(10);
	unsigned int x=0;
    /* unlock the flash program/erase controller */
    fmc_unlock();
	
		x=0;
    address = fota_write_start_addr;
		data1 = 33;
    /* program flash */
    while(address < fota_write_end_addr){
			data1 = fota_fw1[x];
        fmc_word_program(address, data1);
			x++;
        address += 4;
			if(x>=sizeof(fota_fw1)) break;
    }
		data1 = (uint32_t) 512*4;
		fota_write_start_addr= fota_write_start_addr + data1;
    /* lock the main FMC after the program operation */
    fmc_lock();
}

// fota to memory in reverse order

void fotabin_to_memory(int addr_src, int addr_dest) {
    uint32_t val = 0;
			// get current character then increment
			uint8_t byte = temp_global[addr_src+3];
			val = (byte & 0xFF);
			byte = temp_global[addr_src+2];
			// transform hex character to the 4bit equivalent number, using the ascii table indexes
			val = (val << 8) | (byte & 0xFF);
			byte = temp_global[addr_src+1];
			// transform hex character to the 4bit equivalent number, using the ascii table indexes
			val = (val << 8) | (byte & 0xFF);
			byte = temp_global[addr_src];
			// transform hex character to the 4bit equivalent number, using the ascii table indexes
			val = (val << 8) | (byte & 0xFF);			
		fota_fw1[addr_dest]=val;
}

void process_mfota(int modeprocess){
	int i=0, j=0, z=0, y=0;
	if(LEDBStatus) led_off(LEDB);
	else led_on(LEDB);
		memset(temp_global,0,sizeof(temp_global));
	if(modeprocess==1){
		for(int p=0;p<512;p++){
			fota_fw1[p]=0xFFFFFFFF;
		}
	}
	else if(modeprocess==2){
		while(1){
			if(i>4 && i<500 && j==0){
				if(uart1_rxbuf[i]=='R' && uart1_rxbuf[i+1]=='E' && uart1_rxbuf[i+2]=='C' && uart1_rxbuf[i+3]=='V' && uart1_rxbuf[i+4]==':'){
					j++;
					i=i+5;
					for(int p=0;p<14;p++){
							j++;
							i++;
							if(uart1_rxbuf[i-1]== 0x0D && uart1_rxbuf[i]== 0x0A) {
								j++;
								i=i+1;
								p=p+20;
							}
					}
					for(int k=0;k<1024;k++){
						temp_global[k]=uart1_rxbuf[i];
//						printf("%d %d\r\n",temp_global[k], uart1_rxbuf[i]);
						y++;
						i++;
					}
					for(int kk=0;kk<256;kk++){
						fotabin_to_memory(kk*4,fota_idx);
						fota_idx++;
					}
//				printf("FOTAProcess %08jx %08jx %08jx %08jx %d\r\n", (uintmax_t)fota_write_start_addr, (uintmax_t)fota_fw1[0], (uintmax_t)fota_fw1[256], (uintmax_t)fota_fw1[fota_idx-3], fota_idx);
					if(fota_idx>=511){
						fota_fmc_erase_1page();
						delay_1ms(10);
						fota_fmc_program();
	//					if(numinwhitelist==0)printf("%u %u  %d\r\n", fota_write_start_addr, fota_fw1[0], fota_idx);
						for(int p=0;p<512;p++){
							fota_fw1[p]=0;
						}
						fota_idx=0;
					}
					i=1801;
				}
			}
			else if(i>1600) i = 1801;		
			i++;
			if(i>=1801) {
				break;
			}
		}
	}
	else if(modeprocess==3){
		tempint=100;
		while(1){
			if(i>4 && i<500 && j==0){
				if(uart1_rxbuf[i]=='R' && uart1_rxbuf[i+1]=='E' && uart1_rxbuf[i+2]=='C' && uart1_rxbuf[i+3]=='V' && uart1_rxbuf[i+4]==':'){
					j++;
					i=i+6;
					for(int p=0;p<14;p++){
							j++;
							i++;
							if(uart1_rxbuf[i-1]== 0x0D && uart1_rxbuf[i]== 0x0A) {
								j++;
								i=i+1;
								p=p+20;
							}
					}
					for(int k=0;k<1024;k++){
						temp_global[k]=uart1_rxbuf[i];
//						printf("%d %d\r\n",temp_global[k], uart1_rxbuf[i]);
						y++;
						i++;
					}
					for(int kk=0;kk<256;kk++){
						fotabin_to_memory(kk*4,fota_idx);
						fota_idx++;
					}
//					printf("FOTAProcess3 %08jx %08jx %08jx %08jx %d\r\n", (uintmax_t)fota_write_start_addr, (uintmax_t)fota_fw1[0], (uintmax_t)fota_fw1[256], (uintmax_t)fota_fw1[fota_idx-10], fota_idx);
					fota_fmc_erase_1page();
					delay_1ms(10);
					fota_fmc_program();
					for(int p=0;p<512;p++){
						fota_fw1[p]=0;
					}
					fota_idx=0;
					i=1801;
				}
			}
			else if(i>1600) i = 1801;		
			i++;
			if(i>=1801) {
				break;
			}
		}
	}

	uart1_index=0;
	memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
}

void wdt_off(int arg)
{
    delay(50);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_bit_reset(GPIOA, GPIO_PIN_0);
    delay(50);
}

void wdt_on(int arg)
{
    delay(50);
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,200);				
}

void L716_reset(int arg)
{
    gpio_bit_set(GPIOB, GPIO_PIN_1);
    delay(50);
    gpio_bit_reset(GPIOB, GPIO_PIN_1);
}

void led_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);

    gpio_bit_reset(GPIOC, LED_GPIO_PIN); //all led off
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_GPIO_PIN);
}

void led_on(int led_num)
{
	if(led_num==LEDB) LEDBStatus=1;
    gpio_bit_set(GPIOC, BIT(GPIO_PIN_SOURCE_13+led_num));
}

void led_off(int led_num)
{
	if(led_num==LEDB) LEDBStatus=0;
    gpio_bit_reset(GPIOC, BIT(GPIO_PIN_SOURCE_13+led_num));
}
/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while(1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}
/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_config(void)
{

    /* TIMER0 configuration: generate PWM signals with different duty cycles:
       TIMER0CLK = SystemCoreClock / 120 = 1MHz */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER0);
    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 500;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER0,&timer_initpara);

     /* CH0 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,250);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    timer_primary_output_config(TIMER0,ENABLE);
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);
    timer_enable(TIMER0);
}

/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void timer1_config(void)
{
    /* ----------------------------------------------------------------------------
    TIMER1 Configuration:
    TIMER1 count with external clock, the prescaler is 0, the period is 1000.
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* TIMER1 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x01;
    timer_input_capture_config(TIMER1, TIMER_CH_0, &timer_icinitpara);

    /* slave mode selection : TIMER1 */
    /* TIMER1 input trigger : external trigger connected to CI0 */
    timer_input_trigger_source_select(TIMER1, TIMER_SMCFG_TRGSEL_CI0FE0);
    timer_slave_mode_select(TIMER1, TIMER_SLAVE_MODE_EXTERNAL0);

    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);

    timer_enable(TIMER1);
}

void config_modem(){
	uart1_send_string("AT+CMGF=1;+CNMI=2,2;+CLIP=1\r\n");						
	led_off(LEDY);
	delay(10);
	led_on(LEDY);
	delay(10);							
	uart1_send_string("AT+GTSYSCMD=\"nv set WANPingFilter=1\"\r\n");
	delay(20);
	uart1_send_string("AT+GTSYSCMD=\"nv save\"\r\n");
	delay(20);
	uart1_send_string("AT+GTLANENABLE=1\r\n");
	delay(20);		
	has_configured=1;
}

void reset_modem(){
//	printf("Softreset Modem\r\n");
	uart1_send_string("AT+CFUN=15\r\n");
	delay(50);		
}

void hardreset_modem(){
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_bit_set(GPIOB, GPIO_PIN_1);
//    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
//		gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    delay(100);
    gpio_bit_reset(GPIOB, GPIO_PIN_1);
    delay(100);		
}

// ------------------------ FMC funcs ----------------------------------------
/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_erase_pages(void)
{
    uint32_t EraseCounter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < PageNum; EraseCounter++){
        fmc_page_erase(FMC_WRITE_START_ADDR + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/

void fmc_program2(void)
{
	unsigned int x=0;
    /* unlock the flash program/erase controller */
    fmc_unlock();
	
	for(int i=0; i<100;i++){
		fmc_data_backup[i] = (uint32_t) fmc_buf[i];
	}

	x=0;
    address = FMC_WRITE_START_ADDR;
		data1 = 33;
    /* program flash */
    while(address < FMC_WRITE_END_ADDR){
//			data1 = (uint32_t)fmc_buf[x];
			data1 = fmc_data_backup[x];
      fmc_word_program(address, data1);
			x++;
			if(x>256) x = 255;
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
//			if(data1>124) data1=33;
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

void fmc_program3(void)
{
	unsigned int x=0;
    /* unlock the flash program/erase controller */
    fmc_unlock();
	
	for(int i=0; i<99;i++){
		fmc_data_backup[i] = (uint32_t) fmc_buf[i];
	}

	x=0;
    address = FMC_WRITE_START_ADDR;
		data1 = 33;
    /* program flash */
    while(address < FMC_WRITE_END_ADDR){
//			data1 = (uint32_t)fmc_buf[x];
			data1 = fmc_data_backup[x];
      fmc_word_program(address, data1);
			x++;
			if(x>256) x = 255;
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
//			if(data1>124) data1=33;
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

void fmc_write(uint32_t addr, uint32_t data1)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* program flash */
//    while(address < FMC_WRITE_END_ADDR){
        fmc_word_program(addr, data1);
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
  //  }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

/*!
    \brief      check fmc erase result
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_erase_pages_check(void)
{
    uint32_t i;

    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;

    /* check flash whether has been erased */
    for(i = 0; i < WordNum; i++){
        if(0xFFFFFFFF != (*ptrd)){
            led_on(LEDB);
            break;
        }else{
            ptrd++;
        }
    }
}

/*!
    \brief      check fmc program result
    \param[in]  none
    \param[out] none
    \retval     none
*/

void fmc_backup(void)
{
    uint32_t i;
		char c;

    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;

    /* check flash whether has been programmed */
    for(i = 0; i < WordNum; i++){
			fmc_data_backup[i]=*ptrd;
			if(fmc_data_backup[i]>255) fmc_buf[i] = 255;
			else fmc_buf[i] = (char) fmc_data_backup[i];
            ptrd++;
    }
}

void fmc_program_read(void)
{
    uint32_t i;

    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;

    /* check flash whether has been programmed */
    for(i = 0; i < WordNum; i++){
            ptrd++;
    }
}

void fmc_program_read1(void)
{
    uint32_t i;
		char c;

    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;

    /* check flash whether has been programmed */
    for(i = 0; i < WordNum; i++){
			c = *ptrd;
            ptrd++;
    }
}
void mfota_start(int byser){
	if(byser>2){
		memset(smsmsg,0,sizeof(smsmsg));
		uart1_index=0;
		memset(uart1_rxbuf,0, sizeof(uart1_rxbuf));
		memcpy(smsmsg, smsmsg1, sizeof(smsmsg));
	}
		if(smsmsg[9]=='\"'){
//			fota_write_start_addr=(uint32_t)0x08010000U;
			sim_flag=43;
			memset(ipftp,0,sizeof(ipftp));
			memset(userftp,0,sizeof(userftp));
			memset(pwdftp,0,sizeof(pwdftp));
			memset(fileftp,0,sizeof(fileftp));
			memset(temp_global,0,sizeof(temp_global));
			int i=10, j=0, k=0, l=0, m=0, n=0;
			while(1){
				if(smsmsg[i]==',') {
					j++;
					i++;
				}
				if(smsmsg[i]=='\"') {
					j=3;
					break;
				}
				if(j==0){
					ipftp[k]=smsmsg[i];
					k++;
				}
				else if(j==1){
					userftp[l]=smsmsg[i];
					l++;
				}
				else if(j==2){
					pwdftp[m]=smsmsg[i];
					m++;
				}
				else if(j==3){
					fileftp[n]=smsmsg[i];
					n++;
				}
				i++;
				if(i>100) break;
			};
			uart1_send_string("AT+MIPCLOSE=1,0\r\n");
			delay(150);
			sim_flag=43;
			memset(temp_global,0,sizeof(temp_global));
			sprintf(temp_global, "AT+FTPOPEN=\"%s\",\"%s\",\"%s\"\r\n",ipftp,userftp,pwdftp);
			if(byser!=0) printf("%s\r\n", temp_global);
			uart1_send_string(temp_global);
			sim_flag=43;
			tempint=0;
			delay(250);
			sim_flag=43;
			tempint2=0;
			if(byser!=0) {
				printf("MFOTA STARTING\r\n");
				delay(10);
			}
			if(byser>2){
				memset(smsmsg,0,sizeof(smsmsg));
				uart1_index=0;
				memset(uart1_rxbuf,0, sizeof(uart1_rxbuf));
				delay(10);
				printf("%s\r\n", temp_global);
			}
			tempint=0;
			sprintf(temp_global, "AT+FTPGET=\"%s\",0\r\n",fileftp);
			sim_flag=43;
			uart1_send_string(temp_global);
			delay(360);
			if(byser!=0) printf("%s\r\n", uart1_rxbuf);
			cmp_caller =0;
			cmp_config =0;
			cmp_smsmsg=8;			
			sim_flag=43;
			tempint3=0;
//			sim_flag_old=43;	
		}
		else {
			if(byser!=0) {
				printf("MFOTA FAILED. SYNTAX: AT+MFOTA=\"IPFTP,USERFTP,PASSFTP,FILEFTP\"!<Passcode>\r\n");
				delay(20);
				sim_flag = sim_flag_old;
			}
			else {
				sprintf(smsmsg, "MVER NOW %s. MFOTA FAILED. SYNTAX: AT+MFOTA=\"IPFTP,USERFTP,PASSFTP,FILEFTP\"!<Passcode>",MCU_SOFT_VERSION);
//				send_sms();
				send_sms_withsmsnum();
				delay(40);
				sim_flag = sim_flag_old;
			}
		}
}

void config_byserial(){
	memset(passcode,0,sizeof(passcode_saved));
	tandapetik=0;
	for(int i=0;i<254;i++){
		if(smsmsg[i]=='"') tandapetik++;
		if(smsmsg[i]=='!' && i<240 && tandapetik!=1) {
			passcode[0]=smsmsg[i+1];
			passcode[1]=smsmsg[i+2];
			passcode[2]=smsmsg[i+3];
			passcode[3]=smsmsg[i+4];
			passcode[4]=smsmsg[i+5];
			passcode[5]=smsmsg[i+6];
			passcode[6]=smsmsg[i+7];
			passcode[7]=smsmsg[i+8];
			i=255;
		}
	}
	tempint = strncmp(passcode, passcode_saved,8);
//	printf("PC: %s %d\r\n", passcode, tempint);
//	tempint=0;
	cmp_smsmsg=0;
	// checking passcode
//	if(tempint==0){
	cmp_config=100;
	tandapetik=0;
	for(int i=0;i<254;i++){
			if(smsmsg[i]=='"') tandapetik++;
	//		if(smsmsg[i]=='!' && i<240) {
			if(smsmsg[i]=='!' && i<240 && tandapetik!=1) {
				smsmsg[i]=0;
				smsmsg[i+1]=0;
				smsmsg[i+2]=0;
				smsmsg[i+3]=0;
				smsmsg[i+4]=0;
				smsmsg[i+5]=0;
				smsmsg[i+6]=0;
				smsmsg[i+7]=0;
				smsmsg[i+8]=0;
				smsmsg[i+9]=0;
				smsmsg[i+10]=0;
				i=255;
			}
	}
	if(smsmsg[9]=='T' && smsmsg[8]=='O' && smsmsg[7]=='O'){
		printf("REBOOTING\r\n");
		delay(10);
		cmp_caller =0;
		cmp_config = 3;		
		cmp_smsmsg=3;
				sim_flag=99;
			 delay(10);
	}
/*	if(smsmsg[6]=='=' && smsmsg[5]=='L' && smsmsg[4]=='W'){
		check_set_fmc();
		printf("AT+MWL=%s,%s,%s\r\n",white_list1,white_list2,white_list3);
		cmp_config = 5;
		sim_flag=sim_flag_old;
	}*/
	else if(smsmsg[5]=='L' && smsmsg[4]=='W' && smsmsg[3]=='M'){
		if(smsmsg[6]=='='){
			check_set_fmc();
			printf("AT+MWL=%s,%s,%s\r\n",white_list1,white_list2,white_list3);
			cmp_config = 5;			
		}
		else {
			printf("AT+MWL=%s,%s,%s\r\n",white_list1,white_list2,white_list3);
			cmp_config = 1;			
		}
		sim_flag=sim_flag_old;
	}
			// ENWL
	else if(smsmsg[6]=='L' && smsmsg[5]=='W' && smsmsg[4]=='N'){
		if(smsmsg[7]=='='){
			check_set_fmc();
		}
		printf("AT+ENWL=%d. Default[255] = Disable",fmc_buf[76]);
		delay(30);
		sim_flag=sim_flag_old;
	}	
//	else if(smsmsg[7]=='=' && smsmsg[6]=='G' && smsmsg[5]=='F'){
//		check_set_fmc();
//		printf("AT+MCFG=%s,%s,%d,%d\r\n",APN,PORT,autoreboot,fmc_buf[67]);
//		cmp_config = 4;
//		if(socket_id<=1) sim_flag=3;
//		else sim_flag=14;
//	}
	// MCFG
	else if(smsmsg[6]=='G' && smsmsg[5]=='F' && smsmsg[4]=='C'){
		if(smsmsg[7]=='='){
			check_set_fmc();
//			check_set_fmc_byserial(0);
			delay(10);
			sprintf(temp_global,"AT+CGDCONT=1,\"IP\",\"%s\",\"0.0.0.0\",0,0,0,0\r\n", APN);
			uart1_send_string(temp_global);
			delay(40);
//			printf("%s",temp_global);
		}
		memset(temp_global,0,sizeof(temp_global));					
		memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));					
		memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));					
		memset(uart0_rxbuf1,0,sizeof(uart0_rxbuf1));
		uart1_index=0;
		uart0_rxbuf_index=0;
		memset(smsmsg,0,sizeof(smsmsg));
		printf("AT+MCFG=%s,%s,%d\r\n",APN,PORT,autoreboot);
		cmp_config = 2;
		sim_flag=sim_flag_old;
	}
	else if(smsmsg[7]=='D' && smsmsg[6]=='U' && smsmsg[5]=='A'){
//		check_set_fmc();
		if(smsmsg[8]=='='){
			tempint=1;
			check_set_fmc();
			printf("AT+MBAUD=%d. Changing Baudrate Will take effect after Reboot.\r\n",fmc_buf[67]);			
		}
		else {
			printf("AT+MBAUD=%d. Baudrate Default[255] =%d\r\n",fmc_buf[67],baudrateDefault);
		}
		cmp_config = 4;
		sim_flag=sim_flag_old;
	}
	else if(smsmsg[6]=='T' && smsmsg[5]=='F' && smsmsg[4]=='O' && smsmsg[3]=='S'){
		printf("SOFTRESET OK\r\n");
		delay(10);
		cmp_caller =0;
		cmp_config = 3;		
		cmp_smsmsg=13;
				sim_flag=99;
			 delay(10);
	}
	else if(smsmsg[5]=='C' && smsmsg[4]=='P'){
		if(smsmsg[6]=='=') check_set_fmc();
		printf("AT+MPC=%s\r\n",passcode_saved);
		cmp_config = 5;
		sim_flag=sim_flag_old;
	}
	else if(smsmsg[6]=='R' && smsmsg[5]=='E' && smsmsg[4]=='V'){
		printf("AT+MVER=%s\r\n",MCU_SOFT_VERSION);
		cmp_config = 5;
		sim_flag=sim_flag_old;
	}
	else if(smsmsg[6]=='M' && smsmsg[5]=='I' && smsmsg[4]=='S'){
		// MSIMFLAG
		printf("AT+MSIMFLAG=%d %d\r\n",sim_flag_old1, sim_flag_old2);
		cmp_config = 5;
		sim_flag=sim_flag_old1;
		sim_flag_old=sim_flag_old2;
	}
	else if(smsmsg[6]=='C' && smsmsg[5]=='A' && smsmsg[4]=='F' && smsmsg[3]=='F'){
		printf("FACTORY RESET IS WORKING\r\n");
		fmc_erase_pages();
		fmc_erase_pages_check();
		cmp_config = 5;
		sim_flag=sim_flag_old;
	 delay(20);
	 NVIC_SystemReset();
	 delay(20);
	}
	else if(smsmsg[6]=='T' && smsmsg[5]=='S' && smsmsg[4]=='R' && smsmsg[3]=='H'){
		if(smsmsg[7]=='=' ){
			check_set_fmc_byserial(8);
		}
		printf("AT+HRST=%d,%d\r\nOK\r\n",enablehrst, mintotal_hrst);
		delay(10);
		sim_flag=sim_flag_old;
	}
	else if(smsmsg[6]=='T' && smsmsg[5]=='S' && smsmsg[4]=='R' && smsmsg[3]=='W'){
		if(smsmsg[7]=='=' ){
			check_set_fmc_byserial(7);
		}
		printf("AT+WRST=%d,%d\r\nOK\r\n",enablesrst, mintotal_srst);
		delay(10);
		sim_flag=sim_flag_old;
	}
	// WDTOFF
	else if(smsmsg[6]=='F' && smsmsg[5]=='T' && smsmsg[4]=='D' && smsmsg[3]=='W'){
		printf("AT+WDTF\r\nOK\r\n");
		delay(10);
		wdt_off(1);
		sim_flag=sim_flag_old;
	}
	// WDTON
	else if(smsmsg[6]=='T' && smsmsg[5]=='T' && smsmsg[4]=='D' && smsmsg[3]=='W'){
		printf("AT+WDTT\r\nOK\r\n");
		delay(10);
		wdt_on(1);
		sim_flag=sim_flag_old;
	}
	// SFLAG
	else if(smsmsg[6]=='G' && smsmsg[5]=='L' && smsmsg[4]=='F' && smsmsg[3]=='S'){
		printf("AT+SFLAG=%d,%d,%d,%d,%d\r\nOK\r\n",sim_flag, sim_flag_old, sim_flag_old1, sim_flag_old2, socket_id);
		delay(10);
		sim_flag=sim_flag_old;
	}
	// CSOCKET
	else if(smsmsg[6]=='C' && smsmsg[5]=='O' && smsmsg[4]=='S' && smsmsg[3]=='C'){
		printf("AT+CSOCK=%d\r\nOK\r\n",socket_id);
		delay(10);
		sim_flag=sim_flag_old;
	}
	else if(smsmsg[6]=='P' && smsmsg[5]=='O' && smsmsg[4]=='E' && smsmsg[3]=='R'){
		printf("AT+REOPS OK\r\n");
		delay(20);
		afterreset=100;
		uart1_send_string("AT+MIPCLOSE=1,0\r\n");
		delay(150);
		has_configured=0;
//		sim_flag=1; //  ori sim_flag=2
//		sim_flag_old=1;
		cmp_smsmsg=13;
		sim_flag=98;
		sim_flag_old=98;
	}
	// MFW2
	else if(smsmsg[6]=='2' && smsmsg[5]=='W' && smsmsg[4]=='F' && smsmsg[3]=='M'){
		printf("AT+MFW2 OK\r\n");
		delay(20);
		fmc_buf[255]=1;
		fmc_buf[254]=1;
		fmc_data_backup[255]=1;
		fmc_data_backup[254]=1;
		fmc_buf[240]=100;
		fmc_data_backup[240]=100;
		fmc_erase_pages();
		fmc_program2();
		fmc_backup();
		cmp_smsmsg=13;
		sim_flag=99;
		sim_flag_old=99;
//		jump_to_address_fw2();
	}
	else if(smsmsg[7]=='A' && smsmsg[6]=='T' && smsmsg[5]=='O' && smsmsg[4]=='F'){
			numinwhitelist=0;
			mfota_start(1);
	}
/*	else if(smsmsg[8]=='S' && smsmsg[7]=='M' && smsmsg[6]=='L'){
		if(smsmsg[10]=='\"'){
			sim_flag=42;
			memset(ippush,0,sizeof(ippush));
			memset(portpush,0,sizeof(portpush));
			memset(temp_global,0,sizeof(temp_global));
			int i=11, j=0, k=0, l=0;
			while(1){
				if(smsmsg[i]==',') {
					j=1;
					i++;
				}
				if(smsmsg[i]=='\"') {
					j=2;
					break;
				}
				if(j==0){
					ippush[k]=smsmsg[i];
					k++;
				}
				else if(j==1){
					portpush[l]=smsmsg[i];
					l++;
				}
				i++;
				if(i>100) break;
			};
			uart1_send_string("AT+MIPCLOSE=1,0\r\n");
			delay(30);
			sprintf(temp_global, "AT+MIPOPEN=1,%s,\"%s\",%s,0\r\n",portpush,ippush,portpush);
			uart1_send_string(temp_global);
			delay(60);
			printf("CONNECT\r\n");
			delay(10);
			delay(10);
			cmp_caller =0;
			cmp_config =0;
			cmp_smsmsg=8;
		}
		else {
			printf("ERROR. SYNTAX AT+DTDLMS=\"IP,PORT\"\r\n");
			delay(10);
			if(cmp_smsmsg!=8) sim_flag=sim_flag_old;			
		}
		delay(10);
		if(cmp_smsmsg!=8) sim_flag=sim_flag_old;			
	}*/
		else if(cmp_config==100) {
			cmp_config=100;
			strcat(smsmsg,"\r\n");
			uart1_send_string(smsmsg); 
			sim_flag=29;
		}
//	} // end if tempint==0 strncmp
	// dlms
	if(smsmsg[8]=='S' && smsmsg[7]=='M' && smsmsg[6]=='L'){
		if(smsmsg[10]=='\"'){
			sim_flag=42;
			memset(ippush,0,sizeof(ippush));
			memset(portpush,0,sizeof(portpush));
			memset(temp_global,0,sizeof(temp_global));
			int i=11, j=0, k=0, l=0;
			while(1){
				if(smsmsg[i]==',') {
					j=1;
					i++;
				}
				if(smsmsg[i]=='\"') {
					j=2;
					break;
				}
				if(j==0){
					ippush[k]=smsmsg[i];
					k++;
				}
				else if(j==1){
					portpush[l]=smsmsg[i];
					l++;
				}
				i++;
				if(i>100) break;
			};
			uart1_send_string("AT+MIPCLOSE=1,0\r\n");
			delay(30);
			sprintf(temp_global, "AT+MIPOPEN=1,%s,\"%s\",%s,0\r\n",portpush,ippush,portpush);
			uart1_send_string(temp_global);
			delay(60);
			printf("CONNECT\r\n");
			delay(10);
			delay(10);
			cmp_caller =0;
			cmp_config =0;
			cmp_smsmsg=8;
		}
		else {
			printf("ERROR. SYNTAX AT+DTDLMS=\"IP,PORT\"\r\n");
			delay(10);
			if(cmp_smsmsg!=8) sim_flag=sim_flag_old;			
		}
		delay(10);
		if(cmp_smsmsg!=8) sim_flag=sim_flag_old;			
	}	
//	else {
//		printf("ERROR PASSCODE!\r\n");
//		cmp_config = 11;
//		if(cmp_smsmsg!=8) sim_flag=sim_flag_old;			
//	}
		if(cmp_config==3) {
			cmp_smsmsg=13;
			sim_flag=99;
		}
}

void check_apn_fmc(){
	if(fmc_buf[0]==255 || fmc_buf[1]==255 || fmc_buf[2]==255 || fmc_buf[0]==0 || fmc_buf[1]==0 || fmc_buf[2]==0) {
		APN[0]='M';
		APN[1]='2';
		APN[2]='M';
		APN[3]='P';
		APN[4]='L';
		APN[5]='N';
		APN[6]='A';
		APN[7]='M';
		APN[8]='R';
		for(int i=9;i<15;i++){
			APN[i]=0;
		}
/*		
		APN[0]='T';
		APN[1]='P';
		APN[2]='G';
		for(int i=3;i<15;i++){
			APN[i]=0;
		}		
*/
	}
	else {
		for(int i=0;i<15;i++){
			if(fmc_buf[i]<123 && fmc_buf[i]>47) APN[i]=fmc_buf[i];
			else APN[i]=0;
		}
	}
//	printf("APN %s\r\n",APN);
}

void check_passcode_fmc(){
//	if(fmc_buf[15]==255 || fmc_buf[15]==0 || fmc_buf[16]==255 || fmc_buf[16]==0 || fmc_buf[17]==255 || fmc_buf[17]==0) {
	if(fmc_buf[68]==255 || fmc_buf[68]==0) {
		passcode_saved[0]='E';
		passcode_saved[1]='D';
		passcode_saved[2]='M';
		passcode_saved[3]='I';
		passcode_saved[4]='E';
		passcode_saved[5]='D';
		passcode_saved[6]='M';
		passcode_saved[7]='I';
	}
	else {
		for(int i=0;i<8;i++){
			passcode_saved[i]=fmc_buf[i+68];
		}
	}
//	printf("Port %s\r\n",PORT);
}

void check_port_fmc(){
//	if(fmc_buf[15]==255 || fmc_buf[15]==0 || fmc_buf[16]==255 || fmc_buf[16]==0 || fmc_buf[17]==255 || fmc_buf[17]==0) {
	if(fmc_buf[15]==255 || fmc_buf[15]==0) {
		PORT[0]='1';
		PORT[1]='0';
		PORT[2]='0';
		PORT[3]='0';
		PORT[4]=0;
//		printf("1Port %s %d\r\n",PORT, fmc_buf[15]);
	}
	else {
		for(int i=0;i<5;i++){
			PORT[i]=fmc_buf[i+15];
		}
//		printf("2Port %s %d\r\n",PORT, fmc_buf[15]);
	}
}

void check_autoreboot_fmc(){
	if(fmc_buf[65]==255 || fmc_buf[65]==0) {
		autoreboot = enablesrst_default;
		enablesrst = enablesrst_default;
		fmc_buf[65]=enablesrst_default;
	}
	else {
		autoreboot = fmc_buf[65];
		enablesrst = fmc_buf[65];
	}
}

void check_autohrst_fmc(){
	if(fmc_buf[66]==255 || fmc_buf[66]==0) {
		enablehrst = enablehrst_default;
		fmc_buf[66]=enablehrst_default;
	}
	else {
		enablehrst = fmc_buf[66];
	}
}


void check_wl1_fmc(){
	if(fmc_buf[20]==255 || fmc_buf[20]==0 || fmc_buf[25]==255 || fmc_buf[25]==0) {
//		strncpy(white_list1,"+6282112697152",14);
//		strncpy(white_list1,"+628568131842",13);
		strncpy(white_list1,"+6285283530980",14);
//		strncpy(white_list1,"+6580130206",13);
	}
	else {
		for(int i=0;i<15;i++){
			white_list1[i]=fmc_buf[i+20];
		}
	}
}

void check_wl2_fmc(){
	if(fmc_buf[35]==255 || fmc_buf[35]==0 || fmc_buf[40]==255 || fmc_buf[40]==0) {
		strncpy(white_list2,"+6282310293199",14);
	}
	else {
		for(int i=0;i<15;i++){
			white_list2[i]=fmc_buf[i+35];
		}
	}
}
void check_wl3_fmc(){
	if(fmc_buf[50]==255 || fmc_buf[50]==0 || fmc_buf[55]==255 || fmc_buf[55]==0) {
		strncpy(white_list3,"+628112343172",14);
	}
	else {
		for(int i=0;i<15;i++){
			white_list3[i]=fmc_buf[i+50];
		}
	}
}

/*
void check_baud_fmc(){
	if(tempint) printf("AT+MBAUD=%d. Changing Baudrate Will take effect after Reboot.\r\n",fmc_buf[67]);
//		if(fmc_buf[67]==0) printf("BAUDRATE=9600. Changing Baudrate Will take effect after Reboot.\r\n");
//		else if(fmc_buf[67]==1) printf("BAUDRATE=19200. Changing Baudrate Will take effect after Reboot.\r\n");
//		else if(fmc_buf[67]==2) printf("BAUDRATE=38400. Changing Baudrate Will take effect after Reboot.\r\n");
//		else if(fmc_buf[67]==3) printf("BAUDRATE=57600. Changing Baudrate Will take effect after Reboot.\r\n");
//		else if(fmc_buf[67]==4) printf("BAUDRATE=115200. Changing Baudrate Will take effect after Reboot.\r\n");
	delay(10);
}
*/

void check_baud_init(){
		if(fmc_buf[67]==0) baud0=9600;
		else if(fmc_buf[67]==255) baud0=baudrateDefault;
		else if(fmc_buf[67]==1) baud0=19200;
		else if(fmc_buf[67]==2) baud0=38400;
		else if(fmc_buf[67]==3) baud0=57600;
		else if(fmc_buf[67]==4) baud0=115200;
		else baud0=baudrateDefault;
	delay(10);
}

void check_shrst_fmc(){
	mintotal_srst = fmc_data_backup[100];
	hour_srst = fmc_data_backup[101];
	min_srst = fmc_data_backup[102];
	hour_hrst = fmc_data_backup[103];
	min_hrst = fmc_data_backup[104];
	day_hrst = fmc_data_backup[105];
	if(hour_srst==0 && min_srst==0) hour_srst=12;
	if(hour_srst>24) hour_srst=24;
	if(hour_srst<0) hour_srst=0;
	if(min_srst<0) min_srst=0;
	if(mintotal_srst>=1440 || mintotal_srst<=0) {
		mintotal_srst = 719;
		hour_srst=11;
		min_srst =59;
	}
//	printf("1WRST: %d %d %d\r\n", mintotal_srst, min_srst, hour_srst);
	mintotal_srst = hour_srst*60;
	mintotal_srst = mintotal_srst + min_srst;
	if(mintotal_srst>=1440 || mintotal_srst<=0) {
		mintotal_srst = 719;
		hour_srst=11;
		min_srst =59;
	}
//	printf("2WRST: %d %d %d\r\n", mintotal_srst, min_srst, hour_srst);
	if(fmc_data_backup[106]>=10080 || fmc_data_backup[106]<=0) {
		fmc_data_backup[106] = 2880;
		fmc_data_backup[105] = 1;
		day_hrst = 1;
		hour_hrst = 23;
		min_hrst = 59;
		fmc_data_backup[103]=hour_hrst;
		fmc_data_backup[104]=min_hrst;
/*		fmc_erase_pages();
		fmc_program3();
		fmc_backup(); */
	}
	else {
		mintotal_hrst = fmc_data_backup[106];
		fmc_data_backup[105] = floor((float)fmc_data_backup[106]/1440);
		day_hrst = fmc_data_backup[105];
//					tempfloat = min_hrst/60;
//					hour_hrst = ceil(tempfloat);
//					hour_hrst--;
		tempint = day_hrst*1440;
		min_hrst = min_hrst - tempint;
		hour_hrst = floor((float)min_hrst/60);
		tempint = min_hrst;
		min_hrst = tempint%60;		
	}
//	mintotal_hrst = day_hrst*1440;
//	mintotal_hrst = mintotal_hrst + (hour_hrst*60);
//	mintotal_hrst = mintotal_hrst + min_hrst;
	if(mintotal_hrst>=10079) mintotal_hrst = 10080;
/*	if((fmc_data_backup[106]==255 || fmc_data_backup[106]<=0) && (fmc_data_backup[105]==255 || fmc_data_backup[105]<=0) && (fmc_data_backup[104]==255 || fmc_data_backup[104]<=0) && (fmc_data_backup[103]==255 || fmc_data_backup[103]<=0)) {
		fmc_data_backup[106] = 10080;
		fmc_data_backup[105] = 6;
		day_hrst = 6;
		hour_hrst = 23;
		min_hrst = 59;
		fmc_data_backup[103]=hour_hrst;
		fmc_data_backup[104]=min_hrst;
	}*/
	mintotal_hrst = fmc_data_backup[106];
//	printf("2HRST: %d %d %d %d\r\n", fmc_data_backup[103], fmc_data_backup[104], fmc_data_backup[105], fmc_data_backup[106]);
}

void check_set_fmc(){
	int cnt_for=0, i=0, j=0, temp=0, end=0;
	
	while(1){
		if(i>3){
/*			if(smsmsg[i]=='=' && smsmsg[i-1]=='1' && smsmsg[i-2]=='L'&& smsmsg[i-3]=='W'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+20] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+20] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl1_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='2' && smsmsg[i-2]=='L'&& smsmsg[i-3]=='W'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+35] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+35] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl2_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='3' && smsmsg[i-2]=='L'&& smsmsg[i-3]=='W'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+50] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+50] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl3_fmc();
			}
			*/
			if(smsmsg[i]=='=' && smsmsg[i-1]=='C' && smsmsg[i-2]=='P'&& smsmsg[i-3]=='M'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+8;i++){
					fmc_buf[j+68] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+68] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_passcode_fmc();
			}
			// MBAUD
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='D' && smsmsg[i-2]=='U'){
				if(smsmsg[i]=='=') i++;
				if(smsmsg[i]=='=') i++;
				tempint=smsmsg[i]-48;
				fmc_buf[67]=tempint;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_passcode_fmc();
//				check_baud_fmc();
			}
			// MWL=
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='L' && smsmsg[i-2]=='W'&& smsmsg[i-3]=='M'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				end=0;
				j=0;
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+20] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A || smsmsg[i]==',') end = i;
					if(end!=0) fmc_buf[j+20] = 0;
					j++;
				}
				temp = end;
				if(i!=temp){
					i=temp;
				}
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				temp=i;
				end=0;
				j=0;
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+35] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A || smsmsg[i]==',') end = i;
					if(end!=0) fmc_buf[j+35] = 0;
					j++;
				}
				temp=end;
				if(i!=temp){
					i=temp;
				}
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				temp=i;
				end=0;
				j=0;
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+50] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+50] = 0;
					j++;
				}
				check_wl1_fmc();
				check_wl2_fmc();
				check_wl3_fmc();
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl1_fmc();
				check_wl2_fmc();
				check_wl3_fmc();
				check_passcode_fmc();
			}
			// ENWL
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='L' && smsmsg[i-2]=='W' && smsmsg[i-3]=='N' && smsmsg[i-4]=='E'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				end=0;
				j=0;
				tempint=smsmsg[i]-48;
				if(tempint>0) fmc_buf[76]=tempint;
				else fmc_buf[76]=0;
//				printf("in checkFMC ENWL=%d %d\r\n", tempint, fmc_buf[76]);
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_passcode_fmc();
			}
			// MCFG=
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='G' && smsmsg[i-2]=='F'&& smsmsg[i-3]=='C'){
//				printf("\r\ncheckfmc smsmsg %d %s\r\n", i, smsmsg);
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				end=0;
				for(j=0;j<15;j++){
					if(smsmsg[i]==',') {
						end=i;
						fmc_buf[j] = 0;
					}
					else if(end!=0) {
						fmc_buf[j] = 0;
					}
					else {
						fmc_buf[j] = smsmsg[i];
					}
					if(end==0) i++;
				}
				i=end;
				i++;
//				printf("\r\n2checkfmc smsmsg %d %s\r\n", i, smsmsg);
				int ia = i+1;
				char ai = smsmsg[i];
				char ai2 = smsmsg[ia];
				ia++;
				char ai3 = smsmsg[ia];
//				printf("\r\nbeffor %d %d %d %d %d %d %d\r\n", fmc_buf[15],i,end, smsmsg[i], ai, ai2, ai3);
				end=0;
				j=0;
				for(j=0;j<5;j++){
						if(smsmsg[i]==',') end = i;
						if(end!=0){
							fmc_buf[j+15] = 0;
						}
						else fmc_buf[j+15] = smsmsg[i];
						if(end==0) i++;
				}
				i=end;
				i++;
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				tempint=smsmsg[i]-48;
//				if(tempint>0) fmc_buf[65]=tempint;
//				else fmc_buf[65]=0;
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_passcode_fmc();
				check_apn_fmc();
				check_port_fmc();
				check_autoreboot_fmc();
			}
			// WRST
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='T' && smsmsg[i-2]=='S' && smsmsg[i-3]=='R' && smsmsg[i-4]=='W'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				tempint=smsmsg[i]-48;
				if(tempint>0) fmc_buf[65]=tempint;
				else fmc_buf[65]=0;
				i++;
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				end=0;
				memset(temp_global,0,sizeof(temp_global));
				for(j=0;j<5;j++){
					temp_global[j]=0;
						if((smsmsg[i]==0 || smsmsg[i]=='!') && end ==0) end = j;
						if(end!=0){
							temp_global[j] = 0;
						}
						else temp_global[j] = smsmsg[i];
						i++;
				}
				min_srst=0;
				if(end==4){
					// 4 digits
					tempint = temp_global[0]-48;
					if(tempint>0) min_srst = tempint*1000;
					tempint = temp_global[1]-48;
					tempint = tempint*100;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*10;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[3]-48;
					if(tempint>0)min_srst = min_srst+tempint;
				}
				else if(end==3){
					// 3 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_srst = tempint*100;
					tempint = temp_global[1]-48;
					tempint = tempint*10;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[2]-48;
					if(tempint>0)min_srst = min_srst+tempint;
				}
				else if(end==2){
					// 2 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_srst = tempint*10;
					tempint = temp_global[1]-48;
					if(tempint>0)min_srst = min_srst+tempint;
				}
				else if(end==1){
					// 2 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_srst = tempint;
				}
				else {
					// 4 digits
					tempint = temp_global[0]-48;
					min_srst = tempint*10000;
					tempint = temp_global[1]-48;
					tempint = tempint*1000;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*100;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[3]-48;
					tempint = tempint*10;
					if(tempint>0)min_srst = min_srst+tempint;					
					tempint = temp_global[4]-48;
					if(tempint>0)min_srst = min_srst+tempint;					
				}
				tempint = min_srst;
				if(min_srst>1440 || min_srst <=0) min_srst=1440;
				mintotal_srst = min_srst;
				if(min_srst>=60)hour_srst = floor((float) (min_srst/60)) ;
				else hour_srst = 0;
				min_srst = tempint%60;
				printf("20WRST m_h_mt_ensrst: %d %d %d %d\r\n", min_srst, hour_srst, mintotal_srst, enablesrst);
				fmc_data_backup[100]=mintotal_srst;
				fmc_data_backup[101]=hour_srst;
				fmc_data_backup[102]=min_srst;
				check_autoreboot_fmc();
				fmc_erase_pages();
				fmc_program3();
				fmc_backup();
				check_shrst_fmc();
				check_passcode_fmc();
//				printf("20WRST FMCBCKUP: %d %d %d %d\r\n", fmc_data_backup[100], fmc_data_backup[101], fmc_data_backup[102], enablesrst);
//				printf("fmc_buff[65] %d %d %d %d %d %d %s\r\n", fmc_buf[65], mintotal_srst, min_srst, hour_srst, tempint, end, smsmsg);
			}
			// HRST
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='T' && smsmsg[i-2]=='S' && smsmsg[i-3]=='R' && smsmsg[i-4]=='H'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				tempint=smsmsg[i]-48;
				if(tempint>0) fmc_buf[66]=tempint;
				else fmc_buf[66]=0;
				fmc_data_backup[66] = fmc_buf[66];
				enablehrst = fmc_buf[66];
				i++;
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				end=0;
				printf("\r\nHRST %s \r\n",smsmsg);
				memset(temp_global,0,sizeof(temp_global));
				for(j=0;j<5;j++){
					temp_global[j]=0;
						if(smsmsg[i]==0 && end ==0) end = j;
						if(end!=0){
							temp_global[j] = 0;
						}
						else temp_global[j] = smsmsg[i];
						if(end==0)i++;
				}
				min_hrst=0;
				if(end==4){
					// 4 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*1000;
					tempint = temp_global[1]-48;
					tempint = tempint*100;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*10;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[3]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;
				}
				else if(end==3){
					// 3 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*100;
					tempint = temp_global[1]-48;
					tempint = tempint*10;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[2]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;
				}
				else if(end==2){
					// 2 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*10;
					tempint = temp_global[1]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;
				}
				else if(end==1){
					// 2 digits
					tempint = temp_global[0]-48;
					min_hrst = tempint;
				}
				else {
					// 4 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*10000;
					tempint = temp_global[1]-48;
					tempint = tempint*1000;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*100;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[3]-48;
					tempint = tempint*10;
					if(tempint>0)min_hrst = min_hrst+tempint;					
					tempint = temp_global[4]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;					
				}
				tempint = min_hrst;
				mintotal_hrst = tempint;
				if(tempint>10080 || tempint<=0) tempint = 10080;
				if(tempint>=10080 || tempint<=0) {
					fmc_data_backup[106] = 2880;
					fmc_data_backup[105] = 1;
					day_hrst = 1;
					hour_hrst = 23;
					min_hrst = 59;
					mintotal_hrst = 2880;
				}
				else {
					fmc_data_backup[106] = tempint;
					fmc_data_backup[105] = floor((double)(fmc_data_backup[106]/1440));
					day_hrst = fmc_data_backup[105];
					hour_hrst = floor((double)(min_hrst/60));
					min_hrst = tempint%60;
				}
				fmc_data_backup[103]=hour_srst;
				fmc_data_backup[104]=min_hrst;
				check_autohrst_fmc();
				fmc_erase_pages();
				fmc_program3();
				fmc_backup();
				check_shrst_fmc();
				printf("20HRST FMCBCKUP: %d %d %d %d\r\n", fmc_data_backup[103], fmc_data_backup[104], fmc_data_backup[105], fmc_data_backup[106]);
//				printf("fmc_buff[66] %d %d %d %d %d %d %s\r\n", fmc_buf[66], enablehrst, min_hrst, hour_hrst, day_hrst, fmc_data_backup[106], smsmsg);
			}
		}
		i++;
		if(j!=0) break;
		if(i>= MAXBUFFER) break;
	}
//	fmc_erase_pages();
//	fmc_program2();
	memset(smsmsg,0,sizeof(smsmsg));
}


void check_set_fmc_byserial(int chosen){
	int cnt_for=0, i=0, j=0, temp=0, end=0;
	
	while(1){
		if(i>3){
/*			if(smsmsg[i]=='=' && smsmsg[i-1]=='1' && smsmsg[i-2]=='L'&& smsmsg[i-3]=='W'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+20] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+20] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl1_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='2' && smsmsg[i-2]=='L'&& smsmsg[i-3]=='W'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+35] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+35] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl2_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='3' && smsmsg[i-2]=='L'&& smsmsg[i-3]=='W'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+50] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+50] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl3_fmc();
			}
			*/
			if(smsmsg[i]=='=' && smsmsg[i-1]=='C' && smsmsg[i-2]=='P'&& smsmsg[i-3]=='M'){
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				for(i=temp;i<temp+8;i++){
					fmc_buf[j+68] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+68] = 0;
					j++;
				}
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
//				check_passcode_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='D' && smsmsg[i-2]=='U'){
				if(smsmsg[i]=='=') i++;
				if(smsmsg[i]=='=') i++;
				tempint=smsmsg[i]-48;
				fmc_buf[67]=tempint;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
//				check_passcode_fmc();
//				check_baud_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='L' && smsmsg[i-2]=='W'&& smsmsg[i-3]=='M'){
// MWL
						i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				end=0;
				j=0;
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+20] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A || smsmsg[i]==',') end = i;
					if(end!=0) fmc_buf[j+20] = 0;
					j++;
				}
				temp = end;
				if(i!=temp){
					i=temp;
				}
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				temp=i;
				end=0;
				j=0;
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+35] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A || smsmsg[i]==',') end = i;
					if(end!=0) fmc_buf[j+35] = 0;
					j++;
				}
				temp=end;
				if(i!=temp){
					i=temp;
				}
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				temp=i;
				end=0;
				j=0;
				for(i=temp;i<temp+15;i++){
					fmc_buf[j+50] = smsmsg[i];
					if(smsmsg[i]==0 || smsmsg[i]==0x0D || smsmsg[i]==0x0A) end = i;
					if(end!=0) fmc_buf[j+50] = 0;
					j++;
				}
				check_wl1_fmc();
				check_wl2_fmc();
				check_wl3_fmc();
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_wl1_fmc();
				check_wl2_fmc();
				check_wl3_fmc();
//				check_passcode_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='G' && smsmsg[i-2]=='F'&& smsmsg[i-3]=='C'){
				// MCFG
				i++;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				end=0;
				for(j=0;j<15;j++){
					if(smsmsg[i]==',') {
						end=i;
						fmc_buf[j] = 0;
					}
					else if(end!=0) {
						fmc_buf[j] = 0;
					}
					else {
						fmc_buf[j] = smsmsg[i];
					}
					i++;
				}
				i=end;
				i++;
//				printf("\r\nbeffor %d %d %d %d %d %d %d\r\n", fmc_buf[15],i,end, smsmsg[i], smsmsg[i+1], smsmsg[i+2], smsmsg[i+3]);
				end=0;
				j=0;
				for(j=0;j<5;j++){
						if(smsmsg[i]==',') end = i;
						if(end!=0){
							fmc_buf[j+15] = 0;
						}
						else fmc_buf[j+15] = smsmsg[i];
						printf("\r\n%d %d %d %d\r\n", i, fmc_buf[j+15], end, smsmsg[i]);
						delay(2);
						i++;
				}
				i=end;
				i++;
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				tempint=smsmsg[i]-48;
//				if(tempint>0) fmc_buf[65]=tempint;
//				else fmc_buf[65]=0;
//				printf("\r\nbefs %d %d %d %d\r\n", fmc_buf[15],fmc_buf[16],fmc_buf[17],fmc_buf[18]);
				fmc_erase_pages();
				fmc_program2();
				fmc_backup();
				check_apn_fmc();
				check_port_fmc();
				check_autoreboot_fmc();
//				check_passcode_fmc();
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='T' && smsmsg[i-2]=='S' && smsmsg[i-3]=='R' && smsmsg[i-4]=='W'){
				// WRST
				break;
			}
			else if(smsmsg[i]=='=' && smsmsg[i-1]=='T' && smsmsg[i-2]=='S' && smsmsg[i-3]=='R' && smsmsg[i-4]=='H'){
				// HRST
				break;
			}
		}
		i++;
		if(j!=0) break;
		if(i>= MAXBUFFER) break;
	}
	if(chosen==1){
		
			}
	else if(chosen==7){
				i=8;
				temp=i;
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				tempint=smsmsg[i]-48;
				if(tempint>0) fmc_buf[65]=tempint;
				else fmc_buf[65]=0;
				i++;
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				end=0;
		memset(temp_global,0,sizeof(temp_global));
				for(j=0;j<5;j++){
					temp_global[j]=0;
						if(smsmsg[i]==0 && end ==0) end = j;
						if(end!=0){
							temp_global[j] = 0;
						}
						else temp_global[j] = smsmsg[i];
						i++;
				}
//				printf("in chosen7 smsmsg %s %d %s\r\n", smsmsg, i, temp_global);	
				min_srst=0;
				if(end==4){
					// 4 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_srst = tempint*1000;
					tempint = temp_global[1]-48;
					tempint = tempint*100;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*10;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[3]-48;
					if(tempint>0)min_srst = min_srst+tempint;
				}
				else if(end==3){
					// 3 digits
					if(tempint>0)tempint = temp_global[0]-48;
					min_srst = tempint*100;
					if(tempint>0)tempint = temp_global[1]-48;
					tempint = tempint*10;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[2]-48;
					if(tempint>0)min_srst = min_srst+tempint;
				}
				else if(end==2){
					// 2 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_srst = tempint*10;
					tempint = temp_global[1]-48;
					if(tempint>0)min_srst = min_srst+tempint;
				}
				else if(end==1){
					// 2 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_srst = tempint;
				}
				else {
					// 4 digits
					tempint = temp_global[0]-48;
					min_srst = tempint*10000;
					tempint = temp_global[1]-48;
					tempint = tempint*1000;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*100;
					if(tempint>0)min_srst = min_srst+tempint;
					tempint = temp_global[3]-48;
					tempint = tempint*10;
					if(tempint>0)min_srst = min_srst+tempint;					
					tempint = temp_global[4]-48;
					if(tempint>0)min_srst = min_srst+tempint;					
				}
				tempint = min_srst;
//				printf("in chosen7 smsmsg %s %d min_srst %d\r\n", smsmsg, i, min_srst);	
				if(min_srst>1440) min_srst=1440;
				if(min_srst<=0) min_srst=1440;
				mintotal_srst = min_srst;
				if(min_srst>=60)hour_srst = min_srst/60;
				else hour_srst = 0;
				min_srst = tempint%60;
//				printf("in chosen7 smsmsg %s %d min_srst %d hour_srst %d mintotal_srst %d\r\n", smsmsg, i, min_srst, hour_srst, mintotal_srst);	
				fmc_data_backup[100]=mintotal_srst;
				fmc_data_backup[101]=hour_srst;
				fmc_data_backup[102]=min_srst;
//				printf("in chosen7 smsmsg %s %d fmc102 %d fmc101 %d fmc100 %d\r\n", smsmsg, i, fmc_data_backup[102], fmc_data_backup[101], fmc_data_backup[100]);	
				check_autoreboot_fmc();
				fmc_erase_pages();
//				fmc_program2();
				fmc_program3();
				fmc_backup();
				check_shrst_fmc();
//				check_passcode_fmc();
//				printf("smsmsg %s mintotal %d minsrst %d hoursrst %d %d %s\r\n", smsmsg, mintotal_srst, min_srst, hour_srst, end, smsmsg);
//				printf("fmc_buff[65] %d %d %d %d %d %d %s\r\n", fmc_buf[65], mintotal_srst, min_srst, hour_srst, tempint, end, smsmsg);
	}
	else if(chosen==8){
		/////// HRST SRST SHRST
				i=8;
				temp=i;
//				printf("in chosen smsmsg %s %d %d\r\n", smsmsg, i, temp);	
				fmc_buf[254]=1;
				fmc_data_backup[254]=fmc_buf[254];
				tempint=smsmsg[i]-48;
				if(tempint>0) fmc_buf[66]=tempint;
				else fmc_buf[66]=0;
				fmc_data_backup[66] = fmc_buf[66];
				enablehrst = fmc_buf[66];
				i++;
				if(smsmsg[i]==',') i++;
				if(smsmsg[i]==',') i++;
				end=0;
				memset(temp_global,0,sizeof(temp_global));
				for(j=0;j<5;j++){
					temp_global[j]=0;
						if((smsmsg[i]=='!' || smsmsg[i]==0) && end ==0) end = j;
						if(end!=0){
							temp_global[j] = 0;
						}
						else temp_global[j] = smsmsg[i];
						if(end==0)i++;
				}
				min_hrst=0;
				if(end==4){
					// 4 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*1000;
					tempint = temp_global[1]-48;
					tempint = tempint*100;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*10;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[3]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;
				}
				else if(end==3){
					// 3 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*100;
					tempint = temp_global[1]-48;
					tempint = tempint*10;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[2]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;
				}
				else if(end==2){
					// 2 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*10;
					tempint = temp_global[1]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;
				}
				else if(end==1){
					// 1 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint;
				}
				else {
					// 5 digits
					tempint = temp_global[0]-48;
					if(tempint>0)min_hrst = tempint*10000;
					tempint = temp_global[1]-48;
					tempint = tempint*1000;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[2]-48;
					tempint = tempint*100;
					if(tempint>0)min_hrst = min_hrst+tempint;
					tempint = temp_global[3]-48;
					tempint = tempint*10;
					if(tempint>0)min_hrst = min_hrst+tempint;					
					tempint = temp_global[4]-48;
					if(tempint>0)min_hrst = min_hrst+tempint;					
				}
				tempint = min_hrst;
				mintotal_hrst = tempint;
				if(tempint>10080 || tempint<=0) tempint = 10080;
				if(tempint>=10080 || tempint<=0) {
					fmc_data_backup[106] = 2880;
					fmc_data_backup[105] = 1;
					day_hrst = 6;
					hour_hrst = 23;
					min_hrst = 59;
				}
				else {
					fmc_data_backup[106] = tempint;
					fmc_data_backup[105] = fmc_data_backup[106]/1440;
					day_hrst = fmc_data_backup[105];
//					tempfloat = min_hrst/60;
//					hour_hrst = ceil(tempfloat);
//					hour_hrst--;
					tempint = day_hrst*1440;
					min_hrst = min_hrst - tempint;
					hour_hrst = floor((float)min_hrst/60);
					tempint = min_hrst;
					min_hrst = tempint%60;
				}
				fmc_data_backup[103]=hour_srst;
				fmc_data_backup[104]=min_hrst;
				mintotal_hrst = fmc_data_backup[106];
				if(fmc_data_backup[103]!=hour_srst)fmc_data_backup[103]=hour_srst;
				if(fmc_data_backup[104]!=min_hrst)fmc_data_backup[104]=min_hrst;
//				printf("1msg %s fmc_buff[66] %d enablehrst %d %d %d %d %d\r\n", smsmsg, fmc_buf[66], enablehrst, min_hrst, hour_hrst, fmc_data_backup[103], mintotal_hrst);
//				check_autohrst_fmc();
				fmc_erase_pages();
//				fmc_program2();
				fmc_program3();
				fmc_backup();
//				printf("2msg %s fmc_buff[66] %d enablehrst %d %d %d %d %d\r\n", smsmsg, fmc_buf[66], enablehrst, min_hrst, hour_hrst, fmc_data_backup[103], mintotal_hrst);
				check_shrst_fmc();
				printf("3msg %s fmc_buff[66] %d enablehrst %d %d %d %d %d\r\n", smsmsg, fmc_buf[66], enablehrst, min_hrst, hour_hrst, day_hrst, mintotal_hrst);
			}
//	fmc_erase_pages();
//	fmc_program2();
	memset(smsmsg,0,sizeof(smsmsg));
}


/*!
    \brief      return the selected key state
    \param[in]  key: specify the key to be checked
      \arg        KEY_TAMPER: tamper key
      \arg        KEY_WAKEUP: wakeup key
      \arg        KEY_USER1: user key
      \arg        KEY_USER2: user key2
    \param[out] none
    \retval     the key's GPIO pin value
*/
uint8_t gd_eval_key_state_get()
{
    return gpio_input_bit_get(GPIOB, GPIO_PIN_0);
}

void sim_flag1(){
			sim_flag_old=sim_flag;
		if(outage && LEDGStatus){
			LEDGStatus=0;
			led_off(LEDG);
		}
		else {
			LEDGStatus=1;
			led_on(LEDG);							
		}
		delay(5);
		led_on(LEDY);
		if(has_configured==0) {
			config_modem();
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
		}
		led_off(LEDY);
		delay(10);
		led_on(LEDY);
		delay(10);
		memset(temp_global,0,sizeof(temp_global));
//		if(afterreset!=100){
			sprintf(temp_global,"AT+MIPCALL=1,\"%s\"\r\n",APN);
			uart1_send_string(temp_global);		
//		}
		for(int i=0;i<10;i++){
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
		}
}
void sim_flag2(){
		sim_flag_old=sim_flag;
		led_off(LEDY);
		delay(10);
		memset(temp_global,0,sizeof(temp_global));
		sprintf(temp_global,"AT+MIPOPEN=1,%s,\"0.0.0.0\",0,0\r\n",PORT);
		uart1_send_string(temp_global);
		led_on(LEDY);
		delay(10);
		sim_flag=3;
		for(int i=0;i<4;i++){
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
		}
	if(afterreset==1){
		uart1_send_string("AT+CMGF=1;+CNMI=2,2;+CLIP=1\r\n");						
		delay(30);
		numinwhitelist=1;
		sprintf(smsmsg, "FOTA DOWNLOAD COMPLETED. REBOOTED DONE.");
		send_sms();
		delay(20);
		printf("FOTA DOWNLOAD COMPLETED. REBOOTED DONE. %d", afterreset);
	}
	else if(afterreset==2){
		uart1_send_string("AT+CMGF=1;+CNMI=2,2;+CLIP=1\r\n");						
		delay(40);
		numinwhitelist=1;
		sprintf(smsmsg, "AFTER GTRAT. REBOOTED DONE.");
		send_sms();
		delay(20);
		printf("AT+GTRAT= DONE. REBOOTED DONE. %d", afterreset);
	}
	else if(afterreset==3){
		uart1_send_string("AT+CMGF=1;+CNMI=2,2;+CLIP=1\r\n");						
		delay(40);
		numinwhitelist=1;
		sprintf(smsmsg, "AFTER CCLK ERROR. REBOOTED DONE. GOING TO SOFTRESET PLEASE WAIT.");
		send_sms();
		delay(20);
		printf("AFTER CCLK ERROR. REBOOTED DONE. %d. GOING TO SOFTRESET PLEASE WAIT.", afterreset);
		cmp_smsmsg=13;
		sim_flag=99;
		sim_flag_old=99;
	}
		afterreset=0;
		uart1_send_string("AT+GTSET=\"IPRFMT\",1;+CMGF=1\r\n");						
		for(int i=0;i<4;i++){
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
		}
		uart1_send_string("AT+GTSET=\"CALLBREAK\",0;+CNMI=2,2\r\n");
		for(int i=0;i<4;i++){
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
		}
		uart1_send_string("AT+CLIP=1\r\n");
		for(int i=0;i<4;i++){
			led_off(LEDY);
			delay(10);
			led_on(LEDY);
			delay(10);							
		}	
}
void sim_flag3(){
	tick=tick_max; // 5mnit
	idx_plus=0;
	sim_flag_old=sim_flag;
	cntcclk++;
	timerGBlink++;
	if(outage && timerGBlink>=6){
		LEDGStatus=1;
		led_on(LEDG);
		timerGBlink=0;
	}
	else if(outage){
		LEDGStatus=0;
		led_off(LEDG);							
	}
	else led_on(LEDG);
	timerYellowBlink++;
	if(network>6){
		if(yellowOn==1 && timerYellowBlink>=3) {
				//printf("network 4g %d\r\n", network);
			if(cntmipcall>=20) {
				led_on(LEDY);
				led_off(LEDB);
			}
			else {
				led_off(LEDY);	
				led_on(LEDB);
			}
			yellowOn=0;
			timerYellowBlink=0;
		}
		else if(yellowOn==0 && timerYellowBlink>=3){
			LEDBStatus=1;
			if(cntmipcall>=20) {
				led_off(LEDB);
			}
			else {
				led_on(LEDB);
			}
			led_on(LEDY);								
			yellowOn=1;
			timerYellowBlink=0;
		}							
	}
	else if(network>3 ){
		if(yellowOn==1 && timerYellowBlink>=2) {
			if(cntmipcall>=20) {
				led_on(LEDY);
				led_off(LEDB);
			}
			else {
				led_off(LEDY);	
				led_on(LEDB);
			}
			yellowOn=0;
			timerYellowBlink=0;
		}
		else if(yellowOn==0 && timerYellowBlink>=4){
			if(cntmipcall>=20) {
				led_off(LEDB);
			}
			else {
				LEDBStatus=1;
				led_on(LEDB);
			}
			led_on(LEDY);								
			yellowOn=1;
			timerYellowBlink=0;
		}							
	}
	else {
		if(yellowOn==1 && timerYellowBlink>=0) {
			if(cntmipcall>=20) {
				led_on(LEDY);
				led_off(LEDB);
			}
			else {
				led_off(LEDY);	
				led_on(LEDB);
			}
			yellowOn=0;
			timerYellowBlink=0;
		}
		else if(yellowOn==0 && timerYellowBlink>=8){
			if(cntmipcall>=20) {
				led_off(LEDB);
			}
			else {
				LEDBStatus=1;
				led_on(LEDB);
			}
			led_on(LEDY);								
			yellowOn=1;
			timerYellowBlink=0;
		}							
	}
	
	if(outage!=outage_old){
		numinwhitelist=1;
		led_off(LEDG);
		memset(smsmsg,0,sizeof(smsmsg));
		if(outage==1) sprintf(smsmsg,"Power Outage %d",outage);
		else sprintf(smsmsg,"Power Normal ! Outage %d",outage);
		send_sms();
		delay(40);
		uart1_send_string("AT+CNMI=2,2\r\n");
		delay(40);
		outage_old=outage;
		sim_flag=sim_flag_old;					
	}
//	if(getnetwork!=network) {
//		cmp_smsmsg=13;
//		sim_flag=98;		
//	}
	//					if(daynow==startday && hournow==starthour && minnow == startmin+1) {
	if(fmc_buf[65]==1 && mintotal_srst<=1439 && hournow==starthour_srst+hour_srst && minnow >= startmin_srst+min_srst) {						
		cmp_smsmsg=13;
		sim_flag=98;
	//						printf("SRST %d:%d:%d | %d %d %d %d | %d %d %d %d %d\r\n",hournow,minnow,secnow, day_hrst, fmc_buf[65], sim_flag, daynow, startday, starthour_srst, startmin_srst, hour_srst, min_srst);
		printf("SOFTRESET\r\n");
	}
	if(fmc_buf[66]==1 && daynow==startday+day_hrst && hournow==starthour+hour_hrst && minnow >= startmin+min_hrst) {
		cmp_smsmsg=3;
		sim_flag=99;
	//						printf("HRST %d %d:%d:%d | %d %d %d| %d %d %d %d %d\r\n",day_hrst, hournow,minnow,secnow, fmc_buf[65],fmc_buf[66], sim_flag, startday, starthour, startmin, hour_srst, min_hrst);
		printf("HARDRESET\r\n");
	}
	if(fmc_buf[65]==1 && mintotal_srst>1439 && daynow==startday_srst+1 && hournow==starthour_srst && minnow >= startmin_srst+min_srst) {
			cmp_smsmsg=13;
			sim_flag=98;
	//						printf("SRST D+1 %d:%d:%d | %d %d %d %d | %d %d %d %d %d\r\n",hournow,minnow,secnow, day_hrst, fmc_buf[65], sim_flag, daynow, startday, starthour_srst, startmin_srst, hour_srst, min_srst);
		printf("SOFTRESET D+1\r\n");
	}
	if(cntcclk>50 && cntcclk<53){
		if(rebootcmd>0){
			rebootcmd=0;
			reboot_by_calling();
		}
		cntcclk=cntcclk+5;
	}
	else if(cntcclk>=150 && cntcclk<153){
		if(sim_flag==3){
			sim_flag_old = sim_flag;
			sim_flag=32;
			uart1_send_string("AT+MIPOPEN?\r\n");
			delay(10);
//			printf("AT+MIPOPEN?\r\n");
		}
		cntcclk=cntcclk+5;
	}
	else if(cntcclk>=300){
		if(sim_flag==3)sim_flag=22;
		cntcclk=0;
	}
	delay(10);	
}
void sim_flag4(){
	tick=tick_max; // 5mnit
	idx_plus=0;
	sim_flag_old = sim_flag;
	timerGBlink++;
	cntcclk++;
	if(outage && timerGBlink>=6){
		LEDGStatus=1;
		led_on(LEDG);
		timerGBlink=0;
	}
	else if(outage){
		LEDGStatus=0;
		led_off(LEDG);							
	}
	else led_on(LEDG);
	timerYellowBlink++;
	if(network>6){
		if(yellowOn==1 && timerYellowBlink>=3) {
			if(cntmipcall>=20) {
				led_on(LEDY);
				led_off(LEDB);
			}
			else {
				led_off(LEDY);	
				led_on(LEDB);
			}
			yellowOn=0;
			timerYellowBlink=0;
		}
		else if(yellowOn==0 && timerYellowBlink>=3){
			LEDBStatus=1;
			if(cntmipcall>=20) {
				led_off(LEDB);
			}
			else {
				led_on(LEDB);
			}
			led_on(LEDY);								
			yellowOn=1;
			timerYellowBlink=0;
		}							
	}
	else if(network>3 ){
		if(yellowOn==1 && timerYellowBlink>=2) {
			if(cntmipcall>=20) {
				led_on(LEDY);
				led_off(LEDB);
			}
			else {
				led_off(LEDY);	
				led_on(LEDB);
			}
			yellowOn=0;
			timerYellowBlink=0;
		}
		else if(yellowOn==0 && timerYellowBlink>=4){
			if(cntmipcall>=20) {
				led_off(LEDB);
			}
			else {
				LEDBStatus=1;
				led_on(LEDB);
			}
			led_on(LEDY);								
			yellowOn=1;
			timerYellowBlink=0;
		}							
	}
	else {
		if(yellowOn==1 && timerYellowBlink>=0) {
			if(cntmipcall>=20) {
				led_on(LEDY);
				led_off(LEDB);
			}
			else {
				led_off(LEDY);	
				led_on(LEDB);
			}
			yellowOn=0;
			timerYellowBlink=0;
		}
		else if(yellowOn==0 && timerYellowBlink>=8){
			if(cntmipcall>=20) {
				led_off(LEDB);
			}
			else {
				LEDBStatus=1;
				led_on(LEDB);
			}
			led_on(LEDY);								
			yellowOn=1;
			timerYellowBlink=0;
		}							
	}
	if(outage!=outage_old){
		numinwhitelist=1;
		led_off(LEDG);
		memset(smsmsg,0,sizeof(smsmsg));
		if(outage==1) sprintf(smsmsg,"Power Outage %d",outage);
		else sprintf(smsmsg,"Power Normal ! Outage %d",outage);
		send_sms();
		delay(30);
		uart1_send_string("AT+CNMI=2,2\r\n");
		delay(20);
		outage_old=outage;
		sim_flag=sim_flag_old;					
	}
	if(cntcclk>50 && cntcclk<53){
		if(rebootcmd>0){
			rebootcmd=0;
			reboot_by_calling();
		}
		cntcclk=cntcclk+5;
	}
	else if(cntcclk>=150 && cntcclk<153){
		if(sim_flag==3){
			sim_flag_old = sim_flag;
			sim_flag=32;
			uart1_send_string("AT+MIPOPEN?\r\n");
			delay(10);
//			printf("AT+MIPOPEN?\r\n");
		}
		cntcclk=cntcclk+5;
	}
	else if(cntcclk>=300){
		if(sim_flag==3)sim_flag=22;
		cntcclk=0;
	}

	delay(10);	
}
void sim_flag5(){
		led_off(LEDB);
		led_off(LEDY);
		extractnum_clip();	
}
void sim_flag7(){
		led_on(LEDG);
		led_off(LEDB);
		sim_flag=24;
		prepare_data_server();
		memset(uart0_rxbuf1,0,sizeof(uart0_rxbuf1));
		rx_cnt=0;
		len_meter_buf=0;
		len_rxmeter=0;
		counter_wait_sim=0;	
}
void sim_flag14(){
		tick=tick_max; // 5mnit
		idx_plus=0;
		sim_flag_old = sim_flag;
		timerGBlink++;
	cntcclk++;
		if(outage && timerGBlink>=6){
			LEDGStatus=1;
			led_on(LEDG);
			timerGBlink=0;
		}
		else if(outage){
			LEDGStatus=0;
			led_off(LEDG);							
		}
		else led_on(LEDG);
		timerYellowBlink++;

		if(network>6){
			if(yellowOn==1 && timerYellowBlink>=3) {
				if(cntmipcall>=20) {
					led_on(LEDY);
					led_off(LEDB);
				}
				else {
					led_off(LEDY);	
					led_on(LEDB);
				}
				yellowOn=0;
				timerYellowBlink=0;
			}
			else if(yellowOn==0 && timerYellowBlink>=3){
				LEDBStatus=1;
				if(cntmipcall>=20) {
					led_off(LEDB);
				}
				else {
					led_on(LEDB);
				}
				led_on(LEDY);								
				yellowOn=1;
				timerYellowBlink=0;
			}							
		}
		else if(network>3 ){
			if(yellowOn==1 && timerYellowBlink>=2) {
				if(cntmipcall>=20) {
					led_on(LEDY);
					led_off(LEDB);
				}
				else {
					led_off(LEDY);	
					led_on(LEDB);
				}
				yellowOn=0;
				timerYellowBlink=0;
			}
			else if(yellowOn==0 && timerYellowBlink>=4){
				if(cntmipcall>=20) {
					led_off(LEDB);
				}
				else {
					LEDBStatus=1;
					led_on(LEDB);
				}
				led_on(LEDY);								
				yellowOn=1;
				timerYellowBlink=0;
			}							
		}
		else {
			if(yellowOn==1 && timerYellowBlink>=0) {
				if(cntmipcall>=20) {
					led_on(LEDY);
					led_off(LEDB);
				}
				else {
					led_off(LEDY);	
					led_on(LEDB);
				}
				yellowOn=0;
				timerYellowBlink=0;
			}
			else if(yellowOn==0 && timerYellowBlink>=8){
				if(cntmipcall>=20) {
					led_off(LEDB);
				}
				else {
					LEDBStatus=1;
					led_on(LEDB);
				}
				led_on(LEDY);								
				yellowOn=1;
				timerYellowBlink=0;
			}							
		}
		if(outage!=outage_old){
			numinwhitelist=1;
			led_off(LEDG);
			memset(smsmsg,0,sizeof(smsmsg));
			if(outage==1) sprintf(smsmsg,"Power Outage %d",outage);
			else sprintf(smsmsg,"Power Normal ! Outage %d",outage);
			send_sms();
			delay(40);
			outage_old=outage;
			sim_flag=sim_flag_old;					
		};
	if(cntcclk>50 && cntcclk<53){
		if(rebootcmd>0){
			rebootcmd=0;
			reboot_by_calling();
		}
		cntcclk=cntcclk+5;
	}
	else if(cntcclk>=150 && cntcclk<153){
		if(sim_flag==3){
			sim_flag_old = sim_flag;
			sim_flag=32;
			uart1_send_string("AT+MIPOPEN?\r\n");
			delay(10);
//			printf("AT+MIPOPEN?\r\n");
		}
		cntcclk=cntcclk+5;
	}
	else if(cntcclk>=300){
		if(sim_flag==3)sim_flag=22;
		cntcclk=0;
	}

		delay(10);	
}
void sim_flag18(){
		if(tempint<5) tempint++;
		else sim_flag=1;
		led_off(LEDY);
		delay(10);
		led_on(LEDY);
		delay(10);							
}
void sim_flag19(){
//						uart1_send_string("AT\r\n");
		led_off(LEDY);
		delay(10);
		led_on(LEDY);
		delay(10);									
		led_on(LEDB);
		delay(20);	
}

void sim_flag20(){
//						led_off(LEDB);
	tempint=0;
	cnt_data_meter=0;
	for(int p=3;p<sizeof(uart1_rxbuf);p++){
		if(uart1_rxbuf[p-3]=='C' && uart1_rxbuf[p-2]=='C' && uart1_rxbuf[p-1]=='L' && uart1_rxbuf[p]=='K') {
			tempint=0;
			p=sizeof(uart1_rxbuf);
		}
		else tempint++;
	}
	if(tempint>0) cntcclkerr++;
	tempint=0;
	if(sim_flag_old<3) {
		while(1){
			if(tempint>6){
				if(uart1_rxbuf[tempint]=='/') cnt_data_meter++;
				if(uart1_rxbuf[tempint]==',') cnt_data_meter++;
				if(cnt_data_meter>=3) {
					starthour = hex2int((char) uart1_rxbuf[tempint+1]);
					if(starthour>0) starthour=starthour*10;
					cntSimflag0 = hex2int((char) uart1_rxbuf[tempint+2]);
					starthour = starthour + cntSimflag0;
					startmin = hex2int((char) uart1_rxbuf[tempint+4]);
					if(startmin>0) startmin=startmin*10;
					cntSimflag0 = hex2int((char) uart1_rxbuf[tempint+5]);
					startmin = startmin + cntSimflag0;
					tempint=MAXBUFFER;
				}
			}
			tempint++;
			if(tempint>=MAXBUFFER) break;
		}
		sim_flag=3;
	}
	else {
		while(1){
			if(tempint>6){
				if(uart1_rxbuf[tempint]=='/') cnt_data_meter++;
				if(uart1_rxbuf[tempint]==',') cnt_data_meter++;
				if(cnt_data_meter>=3) {
					tempint2=tempint;
					daynow = hex2int((char) uart1_rxbuf[tempint-2]);
					if(daynow>0) daynow=daynow*10;
					cntSimflag0 = hex2int((char) uart1_rxbuf[tempint-1]);
					daynow = daynow + cntSimflag0;
					hournow = hex2int((char) uart1_rxbuf[tempint+1]);
					if(hournow>0) hournow=hournow*10;
					cntSimflag0 = hex2int((char) uart1_rxbuf[tempint+2]);
					hournow = hournow + cntSimflag0;
					minnow = uart1_rxbuf[tempint+4]-48;
					if(minnow>0) minnow=minnow*10;
					cntSimflag0 = uart1_rxbuf[tempint+5]-48;
					minnow = minnow + cntSimflag0;
					secnow = hex2int((char) uart1_rxbuf[tempint+7]);
					if(secnow>0) secnow=secnow*10;
					cntSimflag0 = hex2int((char) uart1_rxbuf[tempint+8]);
					secnow = secnow + cntSimflag0;
					tempint=MAXBUFFER;
				}
			}
			tempint++;
			if(tempint>=MAXBUFFER) break;
		}
		sim_flag=sim_flag_old;
	}
//						if(TESTMODE==1){
//							printf("%s\r\n", uart1_rxbuf);
//							printf("tempint2 %d %c %d %d\r\n",tempint2, uart1_rxbuf[tempint2-2], uart1_rxbuf[tempint2+5], minnow);
//							delay(10);
//						}
	tempint=0;
	uart1_index=0;
//						if(TESTMODE==1){
//							memset(uart0_rxbuf,0,uart0_rxbuf_index);
//							memset(uart1_rxbuf,0,uart1_index);
//							uart1_index=0;
//							uart0_rxbuf_index=0;
//							TESTMODE=0;
//						}

	if(cntcclkerr>10 && sim_flag_old>2){
		numinwhitelist=1;
		sprintf(smsmsg, "RESTARTING. ERROR CCLK COUNTER: %d", cntcclkerr);
		printf("%s",smsmsg);
		send_sms();
		delay(40);
		afterreset=3;
		cmp_smsmsg=13;
		sim_flag=98;
		sim_flag_old=98;
	}
	memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));	
}

void sim_flag21(){
	counter_wait_sim++;
	delay_1ms(10);
	if(counter_wait_sim>=2) {
		counter_wait_sim=100;
		sim_flag=sim_flag_old;
		memset(temp_global,0,sizeof(temp_global));
		memcpy(temp_global, uart1_rxbuf, uart1_index);
		memset(uart1_rxbuf,0,MAXBUFFER);
		checking_mipcall();
	}	
}
void sim_flag22(){
		if(sim_flag==22)uart1_send_string("AT+COPS?\r\n");
		delay(30);
		if(sim_flag_old<3 )sim_flag=23;
		else sim_flag=25;	
}

void sim_flag23(){
		tick=tick_max; // 5mnit
		tempint=0;
		cnt_data_meter=0;
		while(1){
			if(tempint>6){
				if(uart1_rxbuf[tempint]==',') cnt_data_meter++;
				if(cnt_data_meter>=3) {
					network = hex2int((char) uart1_rxbuf[tempint+1]);
					getnetwork = network;
					tempint=MAXBUFFER;
				}
			}
			tempint++;
			if(tempint>=MAXBUFFER) break;
		}
		tempint=0;
		uart1_index=0;
		memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
		startday = 1;
		startday_srst = 1;
		starthour = 8;
		startmin=0;
		delay(20);
		memset(temp_global,0,sizeof(temp_global));
//						uart1_send_string("AT+CCLK=\"23/01/01,00:01:00+28\"\r\n");
		if(hournow==-1) {
			startmin_srst = minnow;
			minnow=0;
			secnow = 0;
			starthour_srst = 8;
			startmin_srst = 0;
			sprintf(temp_global, "AT+CCLK=\"70/01/01,08:00:00+28\"\r\n");
		}
		else {
			startmin_srst = minnow;
			minnow=minnow+1;
			secnow = 0;
			startday_srst = daynow;
			starthour_srst = hournow;
			if(minnow>=60){
				hournow++;
				minnow = minnow-60;
			}
			if(hournow>23){
				hournow=0;
				daynow++;
			}
			if(startmin_srst>=60){
				startmin_srst = startmin_srst-60;
				starthour_srst = starthour_srst+1;
			}
			if(starthour_srst>23){
				startday_srst = startday_srst+1;
				starthour_srst = starthour_srst-24;
			}
//							sprintf(temp_global, "AT+CCLK=\"70/01/01,08:00:00+28\"\r\n");
//							sprintf(temp_global, "AT+CCLK=\"70/01/0%d,",daynow);
			if(hournow>=10 && minnow>=10) sprintf(temp_global, "AT+CCLK=\"70/01/0%d,%d:%d:00+28\"\r\n", daynow, hournow, minnow);
			else if(hournow>=10 && minnow<10) sprintf(temp_global, "AT+CCLK=\"70/01/0%d,%d:0%d:00+28\"\r\n", daynow, hournow, minnow);
			else if(hournow<10 && minnow>=10) sprintf(temp_global, "AT+CCLK=\"70/01/0%d,0%d:%d:00+28\"\r\n", daynow, hournow, minnow);
			else sprintf(temp_global, "AT+CCLK=\"70/01/0%d,0%d:0%d:00+28\"\r\n", daynow, hournow, minnow);
		}
//						TESTMODE=1;
		uart1_send_string(temp_global);
		sim_flag=3;
		sim_flag_old=3;
		led_off(LEDY);
		delay(10);
		led_on(LEDY);
		delay(10);
		led_off(LEDY);
		delay(10);
		led_on(LEDY);
		delay(10);
		sim_flag=20;	
}
void sim_flag24(){
		counter_wait_sim++;
		if(LEDBStatus) {
			LEDBStatus=0;
			led_off(LEDB);
		}
		else {
			LEDBStatus=1;
			led_on(LEDB);
		}
//		delay_1ms(10);
		delay(2);
		if(counter_wait_sim>=max_counter_wait) {
				cntcclk=0;
				counter_wait_sim=100;
				sim_flag=14;
		}	
}
void sim_flag25(){
	if(sim_flag_old!=98){
		tempint=0;
		cnt_data_meter=0;
		while(1){
			if(tempint>6){
				if(uart1_rxbuf[tempint]==',') cnt_data_meter++;
				if(cnt_data_meter>=3) {
					getnetwork = hex2int((char) uart1_rxbuf[tempint+1]);
					tempint=MAXBUFFER;
				}
			}
			tempint++;
			if(tempint>=MAXBUFFER) break;
		}
		tempint=0;
		cnt_data_meter=0;
		uart1_index=0;
		memset(uart1_rxbuf,0,sizeof(uart1_rxbuf));
//							TESTMODE=1;
		if(sim_flag==25) uart1_send_string("AT+CCLK?\r\n");
		delay(20);
		sim_flag=20;
	}	
}
void sim_flag26(){
		delay(50);
//											send_sms();
		send_sms_withsmsnum();
		delay(30);
		sim_flag=sim_flag_old;	
}

void sim_flag27(){
		counter_wait_mtr++;
		if(LEDBStatus) {
			LEDBStatus=0;
			led_off(LEDB);
		}
		else {
			LEDBStatus=1;
			led_on(LEDB);
		}
//		delay_1ms(10);
		delay_mid(1); // ori delay(2)
		if(idx_plus>=3) {
			cmp_smsmsg=13;
			sim_flag=98;
			sim_flag_old=98;
		}
		else idx_plus=0;
		if(counter_wait_mtr>=max_counter_wait) {
			cntcclk=0;
			counter_wait_mtr=100;
			len_rxmeter=rx_cnt;
			memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));
			memset(uart1_tx,0,sizeof(uart1_tx));
			if(sim_flag!=98 && sim_flag_old1==14) {
				idx_plus=0;
//				printf("sim_flag %d %d %d %d\r\n", sim_flag, sim_flag_old, sim_flag_old1, sim_flag_old2);
				sim_flag=sim_flag-20;
			}
			else sim_flag = sim_flag_old1;
		}
}

void sim_flag28(){
		counter_wait_mtr++;
		if(LEDBStatus) {
			LEDBStatus=0;
			led_off(LEDB);
		}
		else {
			LEDBStatus=1;
			led_on(LEDB);
		}
//		delay_1ms(10);
		delay(2);
		if(idx_plus>=3) {
			cmp_smsmsg=13;
			sim_flag=98;
			sim_flag_old=98;
		}
		else idx_plus=0;
		if(counter_wait_mtr>=max_counter_wait) {
			cntcclk=0;
			counter_wait_mtr=100;
			len_rxmeter=rx_cnt;
			memcpy(smsmsg,uart0_rxbuf1,sizeof(smsmsg));
			memset(uart0_rxbuf1,0,sizeof(uart0_rxbuf1));
			memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));
			memset(uart1_tx,0,sizeof(uart1_tx));
			if(sim_flag!=98) {
				idx_plus=0;
//				printf("sim_flag %d %d %d %d\r\n", sim_flag, sim_flag_old, sim_flag_old1, sim_flag_old2);
				if(modecfg) sim_flag=8;
				else {
					memcpy(smsmsg,uart0_rxbuf1,sizeof(smsmsg));
					memset(uart0_rxbuf1,0,sizeof(uart0_rxbuf1));
					memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));
					memset(uart1_tx,0,sizeof(uart1_tx));
					rx_cnt=0;
					sim_flag = sim_flag_old1;
				}
				modecfg=0;
			}
			else sim_flag = sim_flag_old1;
		}
}

void sim_flag29(){
		delay(30);
		printf("%s\r\n",smsmsg);
		memset(smsmsg,0,sizeof(smsmsg));
		delay(10);
		sim_flag=sim_flag_old;	
}

void sim_flag30(){
		delay(50);
	uart1_send_string(uart1_rxbuf);
	delay(10);
		sim_flag=31;	
}
void sim_flag31(){
		counter_wait_mtr++;
		if(LEDBStatus) {
			LEDBStatus=0;
			led_off(LEDB);
		}
		else {
			LEDBStatus=1;
			led_on(LEDB);
		}
//		delay_1ms(10);
		delay(2);
		if(idx_plus>=3) {
			cmp_smsmsg=13;
			sim_flag=98;
			sim_flag_old=98;
		}
		else idx_plus=0;
		if(counter_wait_mtr>=max_counter_wait) {
			counter_wait_mtr=100;
			len_rxmeter=rx_cnt;
			memcpy(smsmsg,uart0_rxbuf1,sizeof(smsmsg));
			memset(uart0_rxbuf1,0,sizeof(uart0_rxbuf1));
			memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));
			memset(uart1_tx,0,sizeof(uart1_tx));
			if(sim_flag!=98) {
				idx_plus=0;
//				printf("sim_flag %d %d %d %d\r\n", sim_flag, sim_flag_old, sim_flag_old1, sim_flag_old2);
				if(modecfg) sim_flag=8;
				else {
					memcpy(smsmsg,uart0_rxbuf1,sizeof(smsmsg));
					memset(uart0_rxbuf1,0,sizeof(uart0_rxbuf1));
					memset(uart0_rxbuf,0,sizeof(uart0_rxbuf));
					memset(uart1_tx,0,sizeof(uart1_tx));
					rx_cnt=0;
					sim_flag = sim_flag_old1;
				}
				modecfg=0;
			}
			else sim_flag = sim_flag_old1;
		}
}
void sim_flag32(){
	if(sim_flag>2){
		delay(30);
		if(checksocket==1) {
			sprintf(smsmsg,"RESTART Because of Socket closed. SOCKET: %d\r\n",checksocket);
//			printf("SF32A %s %d %d %d\r\n", smsmsg, checksocket, sim_flag, sim_flag_old);
			send_sms();
			delay(100);
			sim_flag=98;
			cmp_smsmsg=13;
		}
		else sim_flag=sim_flag_old;
	}
	else sim_flag=sim_flag_old;
//	printf("SF32 %s %d %d %d\r\n", smsmsg, checksocket, sim_flag, sim_flag_old);
}

void sim_flag34(){
		counter_wait_sim++;
	led_off(LEDG);
		if(LEDBStatus) {
			LEDBStatus=0;
			led_off(LEDB);
		}
		else {
			LEDBStatus=1;
			led_on(LEDB);
		}
		delay_1ms(10);
		if(counter_wait_sim>=2) {
			cntcclk=0;
			counter_wait_sim=100;
			len_meter_buf=uart0_rxbuf_index;
			send_to_meter();
			rx_cnt=0;
			sim_flag=14;
		}	
}
void sim_flag37(){
		uart1_send_string("AT+CMGF=1;+CNMI=2,2;+CLIP=1\r\n");						
		delay(30);
		numinwhitelist=1;
		sprintf(smsmsg, "FOTA DOWNLOAD STARTED.");
		send_sms();
		delay(40);
		sim_flag=sim_flag_old;	
}
void sim_flag38(){
		uart1_send_string("AT+CMGF=1;+CNMI=2,2;+CLIP=1\r\n");						
		delay(30);
		numinwhitelist=1;
		sprintf(smsmsg, "FOTA DOWNLOAD COMPLETED. REBOOTING NOW.");
		send_sms();
		delay(20);
		afterreset=1;
		cmp_smsmsg=13;
		sim_flag=98;	
}
void sim_flag41(){
		afterreset=2;
		delay(30);
		afterreset=2;
		cmp_smsmsg=13;
		sim_flag=98;
//		cmp_smsmsg=5;	
}
void sim_flag42(){
		sim_flag_old=sim_flag;
		delay_1ms(30);
		if(idx_plus>=3) {
			cmp_smsmsg=13;
			sim_flag=98;
			sim_flag_old=98;
		}
		else idx_plus=0;	
}
// MFOTA STARTING FTPGET="filename",1

void sim_flag43(){
	tempint++;
	if(tempint>0){
		printf("in sf %d %d %d\r\n",tempint, tempint2, sim_flag);
		if(tempint<40)delay(20);
		if(tempint>=33 && tempint<40) {
			if(numinwhitelist==0) printf("FOTA FAILED timeout: %d\r\n",tempint);
			else {
				sprintf(smsmsg,"FOTA FAILED timeout: %d\r\n",tempint);
				send_sms();
				delay(60);
			}

			tempint2=0;
			cmp_smsmsg=13;
			sim_flag=99; // ori sim_flag=2
			sim_flag_old=99;			// ori sim_flag_old=2	
		}
		else if(tempint>=40) {
			tempint2=0;
			sim_flag=44;
//			fota_fmc_erase_pages();
		}
		fota_idx=0;
		printf("sf43 bottom sf %d %d %d\r\n",tempint, tempint2, sim_flag);
	}
	delay(30);				
}

// MFOTA PROCESS FTPRECV=1440
void sim_flag44(){
//		printf(" in sf44 %d %d %d %d\r\n",sim_flag, sim_flag_old, tempint, tempint2);
	if(tempint>=40 && tempint2<0) {
			tempint2=0;
	}
	if(tempint2==0){
		sprintf(temp_global,"AT+FTPRECV=1024\r\n");
		uart1_send_string(temp_global);
		printf("%s\r\n",temp_global);
		counter_wait_sim=0;
		tempint2=3;
		tempint=-100;
		delay(10);
	}
	else if(tempint2==1){
		tempint=-100;
		counter_wait_sim++;
		delay_1ms(5);
		if(counter_wait_sim>=2) {
			counter_wait_sim=100;
			process_mfota(2);
			sprintf(temp_global,"AT+FTPRECV=1024\r\n");
		  printf("%s\r\n",temp_global);
			uart1_send_string(temp_global);
			tempint2=3;
			delay(10);
			sim_flag=44;
		}
	}
	else if(tempint2==3){
		tempint=-100;
		counter_wait_sim++;
		delay(2);
		if(counter_wait_sim>=2) {
			counter_wait_sim=100;
			printf("\r\nMFOTA uart_index: %d %d sf %d\r\n", uart1_index, tempint2, sim_flag);
//			if(uart1_index>1038 || tempint2!=2) { // Ori
			if(uart1_index>1038) {
				process_mfota(2); // MFOTA
				memset(uart1_tx,0,sizeof(uart1_tx));
				sprintf(uart1_tx,"AT+FTPRECV=1024\r\n");
				uart1_send_string(uart1_tx);
				tempint2=3;
				delay(5);
				sim_flag=44;				
			}
			else {
//				printf("\r\nMFOTA uart_index: %d %d\r\n", uart1_index, tempint2);
				process_mfota(3); // MFOTA
				if(tempint2==2){
					tempint=-100;
				}
				tempint2=1;
				sim_flag=3;
				if(numinwhitelist==0 && tempint==100 && fotaerror>=100) printf("\r\nMFOTA FAILED WHEN DOWNLOADING\r\n");
				else if( numinwhitelist==0 && tempint!=100 && fotaerror<100) printf("\r\nMFOTA DONE. REBOOTING NOW\r\n");
				else if(numinwhitelist>0 && tempint!=100 && fotaerror<100){
					sprintf(smsmsg,"MFOTA DONE SUCCESS. REBOOTING. PLEASE WAIT IN 3 MINUTES\r\n");
//				send_sms();
					send_sms_withsmsnum();
					delay(60);
				}
				else if(fotaerror>=100){
					sprintf(smsmsg,"MFOTA FAILED\r\n");
//				send_sms();
					send_sms_withsmsnum();
					delay(60);
				}
		//				delay(10);
					fmc_buf[255]=1;
					fmc_buf[254]=1;
					fmc_data_backup[255]=1;
					fmc_data_backup[254]=1;
				if(tempint!=100 && fotaerror<100){
					fmc_data_backup[240]=100;
					fmc_buf[240]=100;
					fmc_erase_pages();
					fmc_program2();
					fmc_backup();
					cmp_smsmsg=13;
					sim_flag=98;
					sim_flag_old=98;
				}
				else {
					fmc_data_backup[240]=0;
					fmc_buf[240]=0;
					fmc_erase_pages();
					fmc_program2();
					fmc_backup();
					cmp_smsmsg=13;
					sim_flag=98;
					sim_flag_old=98;
				}
			}
		}
	}
	else {
		fmc_buf[255]=1;
		fmc_buf[254]=1;
		fmc_data_backup[255]=1;
		fmc_data_backup[254]=1;
		if(tempint!=100 && fotaerror<100){
			fmc_data_backup[240]=100;
			fmc_buf[240]=100;
			fmc_erase_pages();
			fmc_program2();
			fmc_backup();
			cmp_smsmsg=13;
			sim_flag=98;
			sim_flag_old=98;
		}
		else {
			fmc_data_backup[240]=0;
			fmc_buf[240]=0;
			fmc_erase_pages();
			fmc_program2();
			fmc_backup();
			cmp_smsmsg=13;
			sim_flag=98;
			sim_flag_old=98;
		}
	}
}

int main(void)
{
	nvic_vector_table_set(NVIC_VECTTAB_FLASH,0x10000); // A1D
	
	rebootcmd=0;
	callingcnt=0;
	max_counter_wait=2;
	daynow=0;
	minnow=0;
	hournow=-1;
	secnow=0;
	afterreset=0;
    led_init();
	while(1){
    gpio_bit_set(GPIOC, LED_GPIO_PIN); //all led on
    //enable gpio clk
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

		rebootcmd=0;
		idx_plus=0;
		cntSimflag0=0;
		has_configured=0;
		LEDGStatus=0;
		outage=0;
		outage_old=0;		
		LEDBStatus=0;
		network=0;
		cntcclk=0;
		cntcclkerr=0;
		rx_cnt=0;
		sim_flag=0;
		sim_flag_old=0;
		cmp_smsmsg=0;
		uart1_index=0;
		    /* TESTING FOR FMC */
		fmc_backup();
		if(fmc_data_backup[254]!=1){
			fmc_erase_pages();
			fmc_erase_pages_check();			
			fmc_backup();
		}
//		if(fmc_buf[240]==100) jump_to_address_fw2(); // A1A
//		else { // A1A
			check_apn_fmc();
			check_port_fmc();
			check_wl1_fmc();
			check_wl2_fmc();
			check_wl3_fmc();
			check_baud_init();
			check_passcode_fmc();
//			check_shrst_fmc();
//	} // A1A
		
			uart_init_live(baud0);
			delay(15);
//			printf("MCU_%s\r\n",MCU_SOFT_VERSION);
//			printf("fmc_backup[254] %d %d\r\n", fmc_data_backup[254], (int)fmc_buf[254]);
//			printf("APN %.15s PORT %.5s Passcode %.8s auto %d baud %d\r\n",APN,PORT, passcode_saved, autoreboot, (int)fmc_buf[67]);
//			delay(5);
//			printf("WL1 %.15s WL2 %.15s WL3 %.15s\r\n",white_list1,white_list2, white_list3);
//			delay(5);
//		printf("fmc_data_backup[100] %d %d %d %d \r\n", mintotal_srst, fmc_buf[65], hour_srst, min_srst);

    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    //start: reset L716 module
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_bit_set(GPIOB, GPIO_PIN_1);
    delay(100);
    gpio_bit_reset(GPIOB, GPIO_PIN_1);
    //end: reset L716 module


	  // configure the TIMER peripheral
    timer_config();
		wdt_on(0);
		timer1_config();
		sim_flag=0;
		led_off(0);
		led_off(1);
		led_off(2);
		check_autoreboot_fmc();
		check_autohrst_fmc();
		check_shrst_fmc();
//		printf("%d %d %d %d", enablesrst, mintotal_srst, enablehrst, mintotal_hrst);
    printf("GO %s\r\n", MCU_SOFT_VERSION);
//		printf("APN %.15s PORT %.5s baud %d\r\n",APN,PORT, fmc_buf[67]);
//		printf("WL1 %.15s WL2 %.15s WL3 %.15s\r\n",white_list1,white_list2, white_list3);
//		printf("MINWRST %d %d %d MINHRST %d %d %d\r\n", hour_srst, min_srst, mintotal_srst, hour_hrst, min_hrst, fmc_data_backup[106]);

		delay(400);
		cntmipcall=0;
		memset(mipcall,0,sizeof(mipcall));
		LEDBStatus=1;
		led_on(LEDG);
		/* configure systick */
//		tick=10000; // 12s
//		tick=250000; // 5mnit
		tick=tick_max; // 5mnit
    systick_config();
		//printf("tick0 %d\r\n", tick);		
//		delay(10);
		tempint=0;
    while (1)
    {
			if(gd_eval_key_state_get()) outage=1;
			else outage=0;
			
			if(sim_flag!=0){
				switch(sim_flag){
					case 1	: sim_flag1();
									  break;
					case 18	: sim_flag18();
										break;
					case 2	: sim_flag2();
										break;
					case 22	: sim_flag22();
										break;
					case 23 : sim_flag23();
										break;
					case 19	: sim_flag19();
										break;
					case 20 : sim_flag20();
										break;
					case 3	: sim_flag3();
										break;
					case 25 : sim_flag25();
										break;
					case 4 	: sim_flag4();
										break;
					case 14	: sim_flag14();
										break;
					case 34	: sim_flag34();
										break;
					case 35	: delay(20);
										break;
					case 21	: sim_flag21();
										break;
					case 5	: sim_flag5();
										break;
					case 45	: reboot_by_calling();
										delay(10);
										break;
					case 16	: delay(40);
										filter_cmt();
										break;
					case 26	: sim_flag26();
										break;
					case 30	: sim_flag30();
										break;
					case 31	: sim_flag31();
										break;
					case 32	: sim_flag32();
										break;
					case 36	: delay(50);
										sim_flag=sim_flag_old;
										break;
					case 37 : sim_flag37();
										break;
					case 27 : sim_flag27();
										break;
					case 7 	: sim_flag7();
										break;
					case 38 : sim_flag38();
										break;
					case 24 : sim_flag24();
										break;
					case 28 : sim_flag28();
										break;
					case 8	: delay(10);
										config_byserial();
										break;
					case 29 : sim_flag29();
										break;
					case 41 : sim_flag41();
										break;
					case 42 : sim_flag42();
										break;
										// FOTA COMMISSIONING
					case 44 : sim_flag44();
										break;
					case 46 : printf("RECEIVE FOTA FROM ECM SOFTWARE\r\n");
										delay(240);
										filter_cmt();
//										sim_flag46();
										break;
					default	: break;
				} // end of switch
				if(sim_flag==43){
						sim_flag43();
				}
				if(sim_flag==98){
					break;
				}
				if(sim_flag==97){
					break;
				}
				else if(sim_flag==99){
					break;
				}
			} // end of sf!=0
			else {
				sim_flag_old = sim_flag;
					cntSimflag0++;
					if(cntSimflag0>=55){
						uart1_send_string("AT\r\n");
						delay(20);
						cntSimflag0=0;
						led_off(LEDB);
						led_off(LEDY);
					}
					delay(10);
			}
		} // end of while1 1
//		printf("OUT WHILE SF: %d %d, CMP: %d %d\r\n",sim_flag, sim_flag_old, cmp_smsmsg, checksocket);
		if(sim_flag==98){
			if(idx_plus>=3){
				printf("DISCONNECT");
				delay(10);
			}
		}
    gpio_bit_reset(GPIOC, LED_GPIO_PIN); //all led off
		delay(40);
		if(cmp_smsmsg==3){
			uart1_send_string("AT+CFUN=15\r\n");
			delay(2840);
		}
		else if(cmp_smsmsg==13){
				sprintf(smsmsg,"RESTART BY SOFTRESET CONFIG. Current MCU Ver %s",MCU_SOFT_VERSION);
				send_sms();
//				send_sms_withsmsnum();
				delay(100);
			uart1_send_string("AT+CFUN=15\r\n");			
			delay(20);
		}
		delay(20);
		if(cmp_smsmsg==3)		break;
		if(sim_flag==97){
			delay(1000);
			break;
		}
		if(sim_flag==99) {
//			uart1_send_string("AT+CFUN=15\r\n");
			delay(20);
			break;
		}
	}
	NVIC_SystemReset();
}