#ifndef __STRATEGY_H__
#define __STRATEGY_H__

#include <rtthread.h>
#include <rtconfig.h>
#include <string.h>
#include <stdio.h>
#include "global.h"
#include "chargepile.h"
#include "strategy.h"


//extern struct rt_semaphore rt_sem_bluetoothfau;
extern ChargPilePara_TypeDef ChargePilePara_Set;
extern ChargPilePara_TypeDef ChargePilePara_Get;



/******************************* ���ƻ� *************************************/
typedef struct
{
	STR_SYSTEM_TIME strDecStartTime;//��ʼʱ��
	STR_SYSTEM_TIME strDecStopTime;	//����ʱ��
	unsigned long ulChargePow;		//��繦���趨ֵ����λ��kW�����㣺-4��
}CHARGE_TIMESOLT;/*ʱ�θ��ɵ�Ԫ*/

typedef struct
{
	char cRequestNO[18];			//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[24];				//·�����ʲ����  visible-string��SIZE(22)��
	GUN_NUM GunNum;					//ǹ���	{Aǹ��1����Bǹ��2��}
	char cUserID[66];   			//�û�id  visible-string��SIZE(64)��	
	unsigned char ucDecMaker;		//������  {��վ��1������������2��}
	unsigned char ucDecType; 		//��������{���ɣ�1�� ��������2��}
	STR_SYSTEM_TIME strDecTime;		//����ʱ��	
	
	unsigned long ulChargeReqEle;	//��������������λ��kWh�����㣺-2��
	unsigned long ulChargeRatePow;	//������� ����λ��kW�����㣺-4��
	unsigned char ucChargeMode;		//���ģʽ {������0��������1��}
	unsigned char ucTimeSlotNum;	//ʱ�������
	CHARGE_TIMESOLT strChargeTimeSolts[50];//ʱ������ݣ����50��

}CHARGE_STRATEGY;/*���ƻ���*/
//CCMRAM extern CHARGE_STRATEGY Chg_Strategy;//�·��ƻ���
//CCMRAM extern CHARGE_STRATEGY Adj_Chg_Strategy;//����ƻ���

typedef struct
{
	char cRequestNO[18];	//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[24];		//·�����ʲ����  visible-string��SIZE(22)��
	GUN_NUM GunNum;			//ǹ���	{Aǹ��1����Bǹ��2��}
	unsigned char cSucIdle;	//�ɹ���ʧ��ԭ��:{0���ɹ� 1��ʧ�� 255������}
}CHARGE_STRATEGY_RSP;/*���ƻ�����Ӧ*/


/****************************************** ������� **********************************************/
typedef struct
{
	char cRequestNO[18];				//	���뵥��  octet-string��SIZE(16)��
	char cAssetNO[24];					//	·�����ʲ����  visible-string��SIZE(22)�� 
	GUN_NUM GunNum;						//	ǹ���	{Aǹ��1����Bǹ��2��}
	char cUserID[66];   				//	�û�id  visible-string��SIZE(64)��
	unsigned long ulChargeReqEle;		//	��������������λ��kWh�����㣺-2��
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	�ƻ��ó�ʱ��
	CHARGE_MODE ChargeMode;				//	���ģʽ {������0��������1��}
	char Token[40];   					//	�û���¼����  visible-string��SIZE(32)��
}CHARGE_APPLY;/*������뵥(BLE)*/

typedef struct
{
	char cRequestNO[18];	//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[24];		//·�����ʲ����  visible-string��SIZE(22)��
	GUN_NUM GunNum;			//ǹ���	{Aǹ��1����Bǹ��2��}
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
}ONLINE_STATE;/*�������״̬�¼�*/

typedef struct
{
	unsigned long OrderNum;			//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;	//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;//  �¼�����ʱ�� 
	unsigned char OccurSource;		//	�¼�����Դ    NULL 	
//	unsigned char Reason;			//  �¼�����ԭ��     
	unsigned char ChannelState;		//  �¼��ϱ�״̬ = ͨ���ϱ�״̬
	char RequestNO[18];				//	������뵥��   ��SIZE(16)��
	char AssetNO[24];				//	·�����ʲ���� visible-string��SIZE(22)��
	GUN_NUM GunNum;					//	ǹ���	{Aǹ��1����Bǹ��2��}
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
	
	char RequestNO[18];					//	������뵥��   ��SIZE(16)�� 
	char AssetNO[24];					//	·�����ʲ���� visible-string��SIZE(22)��
	GUN_NUM GunNum;						//	ǹ���	{Aǹ��1����Bǹ��2��}
	
	STR_SYSTEM_TIME RequestTimeStamp;	//	�������ʱ��
	unsigned long actSOC;				//	��ǰSOC����λ��%�����㣺-2��
	unsigned long aimSOC;				//  Ŀ��SOC����λ��%�����㣺-2��
	unsigned long CellCapacity;			//	�����������λ��kWh�����㣺-2��
	unsigned long ChargeReqEle;			//	��������������λ��kWh�����㣺-2��
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	�ƻ��ó�ʱ��
	CHARGE_MODE ChargeMode;				//	���ģʽ {������0��������1��}
	char Token[40];   					//	�û���¼����  visible-string��SIZE(38)��
	char UserAccount[66];				//  ����û��˺�  visible-string��SIZE(9)��
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


/******************************* ������ *************************************/
typedef enum
{
	CTRL_NULL=0,
	CTRL_START,
	CTRL_STOP,
	CTRL_ADJPOW,
}CTRL_TYPE;/*{0��δ���� 1������  2��ֹͣ  3����������}*/

typedef enum 
{
	CTRL_UNIT=1,		//�������ٿ�
	BLE_UNIT,			//�����ٿ�
}CTRL_CMD_SOURCE;/*����������Դ*/

typedef enum
{
	EXE_NULL=0,
	EXE_ING,
	EXE_END,
	EXE_FAILED,
}EXESTATE;/*ִ��״̬ {0��δִ��  1������ִ�� 2��ִ�н��� 3��ִ��ʧ��}*/

typedef struct
{
	char OrderSn[18];			//������  octet-string��SIZE(16)��
	char cAssetNO[24];			//·�����ʲ����  visible-string��SIZE(22)��
	GUN_NUM GunNum;				//ǹ���	{Aǹ��1����Bǹ��2��}
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
	char cAssetNO[24];		//·�����ʲ����  visible-string��SIZE(22)��
	GUN_NUM GunNum;	//ǹ���	{Aǹ��1����Bǹ��2��}
}CHARGE_EXE_STATE_ASK;/*·��������״̬��ѯ*/

typedef struct
{
	char cRequestNO[18];			//���뵥��  octet-string��SIZE(16)��
	char cAssetNO[24];				//·�����ʲ����  visible-string��SIZE(22)��
	GUN_NUM GunNum;					//ǹ���	{Aǹ��1����Bǹ��2��}
	EXESTATE exeState;				//ִ��״̬ {1������ִ�� 2��ִ�н��� 3��ִ��ʧ��}
	unsigned char ucTimeSlotNum;	//ʱ�������
	unsigned long ulEleBottomValue[5]; 	//����ʾֵ��ֵ������״�ִ��ʱʾֵ������λ��kWh�����㣺-2��
	unsigned long ulEleActualValue[5]; 	//��ǰ����ʾֵ����λ��kWh�����㣺-2��
	unsigned long ucChargeEle[5];		//�ѳ��������λ��kWh�����㣺-2��
	unsigned long ucChargeTime;		//�ѳ�ʱ�䣨��λ��s��
	unsigned long ucPlanPower;		//�ƻ���繦�ʣ���λ��W�����㣺-1��
	unsigned long ucActualPower;	//��ǰ��繦�ʣ���λ��W�����㣺-1��
	PHASE_LIST ucVoltage;			//��ǰ����ѹ����λ��V�����㣺-1��
	PHASE_LIST ucCurrent;			//��ǰ����������λ��A�����㣺-3��
	PILE_WORKSTATE ChgPileState;//���׮״̬��1������ 2������ 3�����ϣ�
	char cUserID[66];   			//�û�id  visible-string��SIZE(64)��
}CHARGE_EXE_STATE;/*·��������״̬  �� ���ƻ���ִ��״̬*/
//CCMRAM extern CHARGE_EXE_STATE Chg_ExeState;

/********************************** ���ü�¼��Ԫ *************************************/

typedef union 
{
	rt_uint32_t Info;
	struct
	{
		rt_uint32_t Charge_Apply:1;		//	�������
		rt_uint32_t Charge_Apply_Ack:1;		//  �������Ӧ��
		rt_uint32_t Charge_Apply_Event:1;		//  ��������¼��ϱ�
		rt_uint32_t Charge_Apply_Event_Ack:1;	//  ��������¼��ϱ�Ӧ��
		rt_uint32_t Charge_Plan:1;		//  ���ƻ��·�
		rt_uint32_t Charge_Plan_Ack:1;		//  ���ƻ��·�Ӧ��
		rt_uint32_t Charge_Plan_Event:1;	//	���ƻ��ϱ��¼�
		rt_uint32_t Charge_Plan_Event_Ack:1;		//  ���ƻ��ϱ��¼�Ӧ��
		rt_uint32_t Charge_Plan_Adj:1;			//	���ƻ�����
		rt_uint32_t Charge_Plan_Adj_Ack:1;			//	���ƻ�����Ӧ��
		rt_uint32_t Charge_Plan_Adj_Event:1;		// ���ƻ������¼��ϱ�
		rt_uint32_t Charge_Plan_Adj_Event_Ack:1;		//  ���ƻ������¼�Ӧ��
		rt_uint32_t Charge_Record_Event:1;		//  ��綩���¼��ϱ�
		rt_uint32_t Charge_Record_Event_Ack:1; 	//	��綩���¼��ϱ�Ӧ��
		rt_uint32_t Router_Svc_Start:1;         //	·������������
		rt_uint32_t Router_Svc_Start_Ack:1;         //	·������������Ӧ��
		rt_uint32_t Router_Svc_Stop:1;         //	·��������ֹͣ
		rt_uint32_t Router_Svc_Stop_Ack:1;         //	·��������ֹͣӦ��
		rt_uint32_t Charge_Power_Adj:1;					//��繦�ʵ���
		rt_uint32_t Charge_Power_Adj_Ack:1;			//��繦�ʵ���Ӧ��
		
		rt_uint32_t Router_Fault_Event:1;
		rt_uint32_t Router_Fault_Event_Ack:1;
		rt_uint32_t Pile_Fault_Event:1;
		rt_uint32_t Pile_Fault_Event_Ack:1;
	}
	Bit;
}CTRL_CHARGE_INFO;/*·��������*/


typedef struct
{
	CTRL_CHARGE_INFO Ctrl_Chg_Info;
	CTL_CHARGE Ctrl_ChgData;
	CTRL_TYPE CtrlType;			//��������{1������  2��ֹͣ  3����������}
	CTRL_CMD_SOURCE StartSource;//����Դ{1��4G����  2:��������}
	CTRL_CMD_SOURCE StopSource;	//ͣ��Դ{1��4Gͣ��  2:����ͣ��}
}CTRL_CHARGE_EVENT;/*�����Ƽ�¼��Ԫ*/
CCMRAM extern CTRL_CHARGE_EVENT CtrlCharge_Event;

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
	
	char cUserID[66];   				//	�û�id  visible-string��SIZE(64)��
	
	char RequestNO[18];					//	������뵥��   ��SIZE(16)��
	char AssetNO[24];					//	·�����ʲ���� visible-string��SIZE(22)��
	GUN_NUM GunNum;						//	ǹ���	{Aǹ��1����Bǹ��2��}
	unsigned long ChargeReqEle;			//	��������������λ��kWh�����㣺-2��
	STR_SYSTEM_TIME RequestTimeStamp;	//	�������ʱ��
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	�ƻ��ó�ʱ��
	CHARGE_MODE ChargeMode;				//	���ģʽ {������0��������1��}
	unsigned long StartMeterValue[5];	//	����ʱ�����ֵ
	unsigned long StopMeterValue[5];	//	ֹͣʱ�����ֵ
	STR_SYSTEM_TIME	ChgStartTime;		//	�������ʱ��
	STR_SYSTEM_TIME ChgStopTime;		//	���ֹͣʱ��
	unsigned long ucChargeEle[5];		//	�ѳ��������λ��kWh�����㣺-2��
	unsigned long ucChargeTime;			//	�ѳ�ʱ�䣨��λ��s��
}CHG_ORDER_EVENT;/*��綩���¼���¼��Ԫ*/


typedef struct
{
	unsigned long OrderNum;				//	�¼���¼��� 
	STR_SYSTEM_TIME StartTimestamp;		//  �¼�����ʱ��  
	STR_SYSTEM_TIME FinishTimestamp;	//  �¼�����ʱ��  
	unsigned char OccurSource;			//	�¼�����Դ    NULL     
	unsigned char ChannelState;			//  �¼��ϱ�״̬ = ͨ���ϱ�״̬
	
	ROUTER_FAULT Router_Fault;//·��������״̬
	CHARGE_PILE_FAULT Pile_Fault;//���׮����״̬
	
}ROUTER_FAULT_EVENT;/*��綩���¼���¼��Ԫ*/

#endif

