#ifndef __ENERGYCON_H__
#define __ENERGYCON_H__

#include <string.h>
#include <stdio.h>
#include "global.h"
#include "chargepile.h"

extern ChargPilePara_TypeDef ChargePilePara_Set;
extern ChargPilePara_TypeDef ChargePilePara_Get;


extern struct rt_thread energycon;
extern struct rt_semaphore rx_sem_setpower;
extern struct rt_semaphore rx_sem_adjpower;


/******************************* ������ *************************************/
typedef struct
{
	char OrderSn[17];			//������  octet-string��SIZE(16)��
	char cAssetNO[23];			//·�����ʲ����  visible-string��SIZE(22)��
	unsigned char GunNum;		//ǹ���	{Aǹ��1����Bǹ��2��}
	unsigned long SetPower;		//�趨��繦�ʣ���λ��W�����㣺-1��
	unsigned char cSucIdle;		//�ɹ���ʧ��ԭ��:{0���ɹ� 1��ʧ�� 255������}
}CTL_CHARGE;/*������������*/

typedef struct
{
	unsigned long A;
	unsigned long B;
	unsigned long C;
}PHASE_LIST;

typedef struct
{
	char cAssetNO[23];		//·�����ʲ����  visible-string��SIZE(22)��
	unsigned char GunNum;	//ǹ���	{Aǹ��1����Bǹ��2��}
}CHARGE_EXE_STATE_ASK;/*·��������״̬��ѯ*/

typedef struct
{
	char cRequestNO[17];			//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[23];				//·�����ʲ����  visible-string��SIZE(22)��
	unsigned char GunNum;			//ǹ���	{Aǹ��1����Bǹ��2��}
	unsigned char exeState;			//ִ��״̬ {1������ִ�� 2��ִ�н��� 3��ִ��ʧ��}
	unsigned char ucTimeSlotNum;	//ʱ�������
	unsigned long ulEleBottomValue[5]; 	//����ʾֵ��ֵ������״�ִ��ʱʾֵ������λ��kWh�����㣺-2��
	unsigned long ulEleActualValue[5]; 	//��ǰ����ʾֵ����λ��kWh�����㣺-2��
	unsigned long ucChargeEle[5];		//�ѳ��������λ��kWh�����㣺-2��
	unsigned long ucChargeTime;		//�ѳ�ʱ�䣨��λ��s��
	unsigned long ucPlanPower;		//�ƻ���繦�ʣ���λ��W�����㣺-1��
	unsigned long ucActualPower;	//��ǰ��繦�ʣ���λ��W�����㣺-1��
	PHASE_LIST ucVoltage;			//��ǰ����ѹ����λ��V�����㣺-1��
	PHASE_LIST ucCurrent;			//��ǰ����������λ��A�����㣺-3��
	unsigned char ChgPileState;		//���׮״̬��1������ 2������ 3�����ϣ�
}CHARGE_EXE_STATE;/*·��������״̬  �� ���ƻ���ִ��״̬*/
CCMRAM extern CHARGE_EXE_STATE Chg_ExeState;

/********************************** ���ü�¼��Ԫ *************************************/
typedef struct
{
	char OrderSn[17];			//������  octet-string��SIZE(16)��
	char cAssetNO[23];			//·�����ʲ����  visible-string��SIZE(22)��
	unsigned char GunNum;		//ǹ���	{Aǹ��1����Bǹ��2��}
	unsigned char CtrlType;		//��������{1������  2��ֹͣ  3����������}
	unsigned char StartType;	//��������{1��4G����  2:��������}
	unsigned char StopType;		//ͣ������{1��4Gͣ��  2:����ͣ��}
	unsigned long SetPower;		//�趨��繦�ʣ���λ��W�����㣺-1��
	unsigned char cSucIdle;		//�ɹ���ʧ��ԭ��:{0���ɹ� 1��ʧ�� 255������}
}CTL_CHARGE_EVENT;/*�����Ƽ�¼��Ԫ*/

/********************************** �¼���¼��Ԫ *************************************/
typedef struct
{
	unsigned long OrderNum;				//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;		//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;	//  �¼�����ʱ��  
	unsigned char OccurSource;			//	�¼�����Դ    NULL 		
	unsigned char ChannelState;			//  �¼��ϱ�״̬ = ͨ���ϱ�״̬
	CHARGE_EXE_STATE Chg_ExeState; 		//  ��ǰ���ִ��״̬ structure
}CHARGE_EXE_EVENT;/*���ִ���¼���¼��Ԫ*/

typedef struct
{
	unsigned long OrderNum;				//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;		//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;	//  �¼�����ʱ��  
	unsigned char OccurSource;			//	�¼�����Դ    NULL     
	unsigned char ChannelState;			//  �¼��ϱ�״̬ = ͨ���ϱ�״̬
	
	char cUserID[65];   				//	�û�id  visible-string��SIZE(64)��
	
	char RequestNO[17];					//	������뵥��   ��SIZE(16)��
	char AssetNO[23];					//	·�����ʲ���� visible-string��SIZE(22)��
	unsigned char GunNum;				//	ǹ���	{Aǹ��1����Bǹ��2��}
	unsigned long ChargeReqEle;			//	��������������λ��kWh�����㣺-2��
	STR_SYSTEM_TIME RequestTimeStamp;	//	�������ʱ��
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	�ƻ��ó�ʱ��
	unsigned char ChargeMode;			//	���ģʽ {������0��������1��}
	unsigned long StartMeterValue[5];	//	����ʱ�����ֵ
	unsigned long StopMeterValue[5];	//	ֹͣʱ�����ֵ
	STR_SYSTEM_TIME	ChgStartTime;		//	�������ʱ��
	STR_SYSTEM_TIME ChgStopTime;		//	���ֹͣʱ��
	unsigned long ucChargeEle[5];		//	�ѳ��������λ��kWh�����㣺-2��
	unsigned long ucChargeTime;			//	�ѳ�ʱ�䣨��λ��s��
}CHG_ORDER_EVENT;/*��綩���¼���¼��Ԫ*/

#endif

