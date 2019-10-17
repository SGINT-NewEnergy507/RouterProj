#ifndef __STRATEGY_H__
#define __STRATEGY_H__

#include <string.h>
#include <stdio.h>
#include "global.h"
#include "chargepile.h"


extern struct rt_semaphore rt_sem_bluetoothfau;

/******************************** �������������Ϣ ***********************************/
typedef enum
{
	DISORDER=0,
	ORDER,
}CHARGE_MODE;/*���ģʽ {������0��������1��}*/

typedef enum
{
	EXE_NULL=0,
	EXE_ING,
	EXE_END,
	EXE_FAILED,
}EXESTATE;/*ִ��״̬ {0��δִ��  1������ִ�� 2��ִ�н��� 3��ִ��ʧ��}*/

typedef enum
{
	CONNECT=0,
	DISCONNECT,
}PILE_COM_STATE;/*��׮ͨ��״̬ {������0�����쳣��1��}*/

typedef enum
{
	CTRL_NULL=0,
	CTRL_START,
	CTRL_STOP,
	CTRL_ADJPOW,
}CTRL_TYPE;/*{0��δ���� 1������  2��ֹͣ  3����������}*/


/******************************* ���ƻ� *************************************/
typedef struct
{
	STR_SYSTEM_TIME strDecStartTime;//��ʼʱ��
	STR_SYSTEM_TIME strDecStopTime;	//����ʱ��
	unsigned long ulChargePow;		//��繦���趨ֵ����λ��kW�����㣺-4��
}CHARGE_TIMESOLT;/*ʱ�θ��ɵ�Ԫ*/

typedef struct
{
	char cRequestNO[17];			//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[23];				//·�����ʲ����  visible-string��SIZE(22)��
	unsigned char GunNum;			//ǹ���	{Aǹ��1����Bǹ��2��}
	char cUserID[65];   			//�û�id  visible-string��SIZE(64)��	
	unsigned char ucDecMaker;		//������  {��վ��1������������2��}
	unsigned char ucDecType; 		//��������{���ɣ�1�� ��������2��}
	STR_SYSTEM_TIME strDecTime;		//����ʱ��	
	
	unsigned long ulChargeReqEle;	//��������������λ��kWh�����㣺-2��
	unsigned long ulChargeRatePow;	//������� ����λ��kW�����㣺-4��
	unsigned char ucChargeMode;		//���ģʽ {������0��������1��}
	unsigned char ucTimeSlotNum;	//ʱ�������
	CHARGE_TIMESOLT strChargeTimeSolts[50];//ʱ������ݣ����50��

}CHARGE_STRATEGY;/*���ƻ���*/
extern CHARGE_STRATEGY Chg_Strategy;//�·��ƻ���
extern CHARGE_STRATEGY Adj_Chg_Strategy;//����ƻ���

typedef struct
{
	char cRequestNO[17];	//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[23];		//·�����ʲ����  visible-string��SIZE(22)��
	unsigned char cSucIdle;	//�ɹ���ʧ��ԭ��:{0���ɹ� 1��ʧ�� 255������}
}CHARGE_STRATEGY_RSP;/*���ƻ�����Ӧ*/


/****************************************** ������� **********************************************/
typedef struct
{
	char cRequestNO[17];				//	���뵥��  octet-string��SIZE(16)��
	char cAssetNO[23];					//	·�����ʲ����  visible-string��SIZE(22)�� 
	unsigned char GunNum;				//	ǹ���	{Aǹ��1����Bǹ��2��}
	char cUserID[65];   				//	�û�id  visible-string��SIZE(64)��
	unsigned long ulChargeReqEle;		//	��������������λ��kWh�����㣺-2��
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	�ƻ��ó�ʱ��
	unsigned char ChargeMode;			//	���ģʽ {������0��������1��}
	char Token[33];   					//	�û���¼����  visible-string��SIZE(32)��
}CHARGE_APPLY;/*������뵥(BLE)*/

typedef struct
{
	char cRequestNO[17];	//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[23];		//·�����ʲ����  visible-string��SIZE(22)��
	unsigned char cSucIdle;	//�ɹ���ʧ��ԭ��:{0���ɹ� 1��ʧ�� 255������}
}CHARGE_APPLY_RSP;/*������뵥��Ӧ*/

/******************************** �¼���Ϣ��¼ ***********************************/
typedef struct
{
	unsigned long OfflinePeriod;		//��������ʱ������λ���룩
	unsigned char OfflineReason;		//����ԭ�� {δ֪��0����ͣ�磨1�����ŵ��仯��2��}
}OFFLINE_IFO;/*������Ϣ*/

typedef struct
{
	unsigned long OrderNum;					//��¼���
	STR_SYSTEM_TIME OnlineTimestamp;		//����ʱ��
	STR_SYSTEM_TIME OfflineTimestamp;		//����ʱ��
	unsigned char OccurSource;				//�¼�����Դ    NULL 
	unsigned char ChannelState;				//ͨ��״̬
	unsigned char AutualState;				//״̬�仯 {���ߣ�0���� ���ߣ�1��}
	OFFLINE_IFO OfflineIfo;					//������Ϣ
}ONLINE_STATE;/*�������״̬*/

typedef struct
{
	unsigned long OrderNum;			//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;	//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;//  �¼�����ʱ�� 
	unsigned char OccurSource;		//	�¼�����Դ    NULL 	
//	unsigned char Reason;			//  �¼�����ԭ��     
	unsigned char ChannelState;		//  �¼��ϱ�״̬ = ͨ���ϱ�״̬
	char RequestNO[17];				//	������뵥��   ��SIZE(16)��
	char AssetNO[23];				//	·�����ʲ���� visible-string��SIZE(22)��
	unsigned char GunNum;			//	ǹ���	{Aǹ��1����Bǹ��2��}
}PLAN_FAIL_EVENT;/*���ƻ�����ʧ�ܼ�¼��Ԫ*/

typedef struct
{
	unsigned long OrderNum;				//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;		//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;	//  �¼�����ʱ��  
//	unsigned char Reason;				//  �¼�����ԭ��
	unsigned char OccurSource;			//	�¼�����Դ    NULL 		
	unsigned char ChannelState;			//  �¼��ϱ�״̬ = ͨ���ϱ�״̬	
	CHARGE_STRATEGY Chg_Strategy; 
}PLAN_OFFER_EVENT;/*���ƻ��ϱ���¼��Ԫ*/

typedef struct
{
	unsigned long OrderNum;				//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;		//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;	//  �¼�����ʱ��  
	unsigned char OccurSource;			//	�¼�����Դ    NULL 
	unsigned char ChannelState;			//  �¼��ϱ�״̬ = ͨ���ϱ�״̬
	
	char RequestNO[17];					//	������뵥��   ��SIZE(16)�� 
	char AssetNO[23];					//	·�����ʲ���� visible-string��SIZE(22)��
	unsigned char GunNum;				//	ǹ���	{Aǹ��1����Bǹ��2��}
	
	STR_SYSTEM_TIME RequestTimeStamp;	//	�������ʱ��
	unsigned long actSOC;				//	��ǰSOC����λ��%�����㣺-2��
	unsigned long aimSOC;				//  Ŀ��SOC����λ��%�����㣺-2��
	unsigned long CellCapacity;			//	�����������λ��kWh�����㣺-2��
	unsigned long ChargeReqEle;			//	��������������λ��kWh�����㣺-2��
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	�ƻ��ó�ʱ��
	unsigned char ChargeMode;			//	���ģʽ {������0��������1��}
	char Token[39];   					//	�û���¼����  visible-string��SIZE(38)��
	char UserAccount[10];				//  ����û��˺�  visible-string��SIZE(9)��
}CHARGE_APPLY_EVENT;/*��������¼���¼��Ԫ*/



typedef struct
{
	unsigned long OrderNum;				//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;		//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;	//  �¼�����ʱ��  
	unsigned char OccurSource;			//	�¼�����Դ    NULL    
	unsigned char ChannelState;			//  �¼��ϱ�״̬ = ͨ���ϱ�״̬
	rt_err_t TotalFault;				//	����״̬
	char Fau[32];						//	�������״̬
}ORDER_CHG_EVENT;/*�������¼���¼��Ԫ*/






#endif

