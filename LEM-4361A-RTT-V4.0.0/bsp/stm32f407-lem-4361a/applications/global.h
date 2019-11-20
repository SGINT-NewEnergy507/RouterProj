#ifndef __GLOBAL_H
#define __GLOBAL_H	

#include <rtthread.h>
#include  <rtconfig.h>
#include  <string.h>
#include  <stdarg.h>

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//LEM_4242A Board
//ϵͳʱ�ӳ�ʼ��	
//³������@LNINT
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У����ؾ���
//Copyright(C) ɽ��³�����ܼ������޹�˾ 2017-2099
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
//////////////////////////////////////////////////////////////////////////////////


#define CCMRAM __attribute__((section("ccmram")))

#define MY_HEX	1
#define MY_CHAR 2

extern unsigned char DEBUG_MSH;

typedef struct{
	rt_uint16_t DataRx_len;
	rt_uint8_t Rx_data[1024];
	rt_uint16_t DataTx_len;
	rt_uint8_t Tx_data[1024];

}ScmUart_Comm;

//////////////////////////////////////////////////////////////////////////////////
typedef struct 				
{
	unsigned char  Second;        // ��
	unsigned char  Minute;        // ��
	unsigned char  Hour;          // ʱ
	unsigned char  Day;           // ��
	
	unsigned char  Week;          //����
	
	unsigned char  Month;         // ��
	unsigned char  Year;          // �� ����λ
}STR_SYSTEM_TIME;
extern STR_SYSTEM_TIME System_Time_STR;

/******************************** ·���������Ϣ ***********************************/	//zcx190710
//typedef enum {
//	RtSt_Starting=0,			// ������
//	RtSt_StandbyOK,          	// ��������
//	RtSt_CtrlPower,				// ������������У�����ִ�мƻ��ͳ����ƣ�
//	RtSt_Fault,           		// ����
//	RtSt_Update,				// ������
//}ROUTER_WORKSTATE;/*·����״̬*/


typedef struct
{
	char AssetNum[24];					//·�����ʲ���� �ַ��� maxlen=22
	char Addr[14];					//·����ͨѶ��ַ �ַ��� maxlen=13
//	ROUTER_WORKSTATE WorkState;			//·��������״̬
}ROUTER_INFO_UNIT;/*·������Ϣ��Ԫ*/
extern ROUTER_INFO_UNIT RouterInfo;

typedef struct
{
	unsigned char PileNum[18];			//���׮���         visible-string��SIZE(16)����
	rt_uint8_t PileIdent;       		//���ӿڱ�ʶ(A/B)
	unsigned char PileInstallAddr[50];	//���׮�İ�װ��ַ   visible-string��
	unsigned long minChargePow;			//���׮��ͳ�繦�� double-long����λ��W�����㣺-1����
	unsigned long ulChargeRatePow;		//������ʶ���� double-long����λ��W�����㣺-1��
	rt_uint8_t AwakeSupport;			//�Ƿ�֧�ֻ���    {0:��֧�� 1��֧��}
//	PILE_WORKSTATE WorkState;			//����״̬
}PILE_INFO_UNIT;/*���׮��Ϣ��Ԫ*/
extern PILE_INFO_UNIT PileInfo;

typedef enum 
{
	ChgState_Standby=0,            //��������
	ChgState_InCharging,           //�����
	ChgState_DisCharging,          //�ŵ���
	ChgState_Finished,				//��ŵ���ɣ�3��ش�����
	ChgState_Fault,            	//����
	ChgState_Update,
}ROUTER_WORKSTATE;/*���׮״̬��·�������ã�*/

typedef enum 
{
	PILE_STANDBY=1,         //��������
	PILE_WORKING,           //������
	PILE_FAU,            	//����
}PILE_WORKSTATE;/*���׮״̬���������ͣ�*/

typedef union 
{
	rt_uint32_t Total_Fau;
	struct
	{
		rt_uint32_t Memory_Fau:1;		//	�ն������ڴ���ϣ�0��
		rt_uint32_t RTC_Fau:1;		//  ʱ�ӹ���        ��1��
		rt_uint32_t Board_Fau:1;		//  ����ͨ�Ź���    ��2��
		rt_uint32_t CallMeter_Fau:1;	//  485�������     ��3��
		rt_uint32_t Screen_Fau:1;		//  ��ʾ�����      ��4��
		rt_uint32_t Hplc_Fau:1;		//  �ز�ͨ���쳣    ��5��
		rt_uint32_t RAM_Fau:1;	//	�ڴ��ʼ������  ��6��
		rt_uint32_t ESAM_Fau:1;		//  ESAM����        ��7��
		rt_uint32_t Ble_Fau:1;			//	����ģ�����     ��8��
		rt_uint32_t MeterCom_Fau:1;		//	������ԪͨѶ���� 	��9��
		rt_uint32_t CanCom_Fau:1;		//  ���׮ͨ�Ź��� ��10��
		rt_uint32_t ChgPile_Fau:1;		//  ���׮�豸���� ��11��
		rt_uint32_t OrdRecord_Fau:1; 	//	���ض�����¼�� ��12��
		rt_uint32_t PowOver_Fau:1;         //	���������Ƴ�����  (13)
	}
	Bit;
}ROUTER_FAULT;/*·��������*/

typedef union 
{
	rt_uint32_t Total_Fau;
	struct
	{
		rt_uint32_t Memory_Fau:1;		//	�ն������ڴ���ϣ�0��
		rt_uint32_t RTC_Fau:1;		//  ʱ�ӹ���        ��1��
		rt_uint32_t Board_Fau:1;		//  ����ͨ�Ź���    ��2��
		rt_uint32_t CallMeter_Fau:1;	//  485�������     ��3��
		rt_uint32_t Screen_Fau:1;		//  ��ʾ�����      ��4��
		rt_uint32_t Hplc_Fau:1;		//  �ز�ͨ���쳣    ��5��
		rt_uint32_t RAM_Fau:1;	//	�ڴ��ʼ������  ��6��
		rt_uint32_t ESAM_Fau:1;		//  ESAM����        ��7��
		rt_uint32_t Ble_Fau:1;			//	����ģ�����     ��8��
		rt_uint32_t MeterCom_Fau:1;		//	������ԪͨѶ���� 	��9��
		
		rt_uint32_t StopEct_Fau:1;        //��ͣ��������  ��10��
		rt_uint32_t	Arrester_Fau:1;       //����������	    ��11��
    rt_uint32_t GunOut_Fau:1;            //���ǹδ��λ		��12��
  	rt_uint32_t TemptOV_Fau:1;     //���׮���¹���		��13��
		rt_uint32_t VOVWarn:1;           //�����ѹ��ѹ 			��14��
		rt_uint32_t VLVWarn:1;           //�����ѹǷѹ					��15��
		rt_uint32_t Pilot_Fau:1;		   //����г������Ƶ�������     ��16��   
		rt_uint32_t ACCon_Fau:1;		   //�����Ӵ�������	    ��17��
		rt_uint32_t OutCurr_Fau:1;		   //��������澯		   ��18��         
		rt_uint32_t CurrAct_Fau:1;	   //���������������	    ��19��					
		rt_uint32_t ACCir_Fau:1;		   //������·�����ϣ�20��
		rt_uint32_t Lock_Fau:1;	   //���ӿڵ���������״̬��21��
		rt_uint32_t GunTempt_Fau:1;     //���ӿڹ��¹���				��22��
		rt_uint32_t CC_Fau:1;				   //�������״̬CC����4    ��23��
		rt_uint32_t CP_Fau:1;				   //������״̬CP����1��24��
		rt_uint32_t PE_Fau:1;			   //PE���߹��ϣ�25��
		rt_uint32_t OTHERS_Fau:1;			   //PE���߹��ϣ�25��
	}
	Bit;
}CHARGE_PILE_FAULT;/*���׮����*/


typedef struct
{
	ROUTER_WORKSTATE Router_State;
	PILE_WORKSTATE	Pile_State;
	
	ROUTER_FAULT Router_Fault;//·����������Ϣ
	CHARGE_PILE_FAULT Pile_Fault;//���׮������Ϣ
	
}ROUTER_WORK_STATE;//·��������״̬

extern ROUTER_WORK_STATE Router_WorkState;








	
//typedef struct
//{
//	char UserID[66];   			//�û�id  visible-string��SIZE(64)��
//	char Token[40];   			//�û���¼����  visible-string��SIZE(38)��
//	unsigned char AccountState;	//�˻�״̬ {0��������1��Ƿ��}
//}WHITE_LIST;/*·����������*/

//typedef struct
//{
//	char kAddress[17];   		//ͨѶ��ַ	visible-string��SIZE(16)��
//	char MeterNum[9];   		//���  visible-string��SIZE(8)��
//	char KeyNum[9];				//��Կ��Ϣ	visible-string��SIZE(8)��	
//}KEY_INFO_UNIT;/*·������Կ��Ϣ��Ԫ*/


/******************************** �������������Ϣ ***********************************/
typedef enum
{
	DISORDER=0,
	ORDER,
}CHARGE_MODE;/*���ģʽ {������0��������1��}*/

typedef enum
{
	CONNECT=0,
	DISCONNECT,
}PILE_COM_STATE;/*��׮ͨ��״̬ {������0�����쳣��1��}*/

typedef enum
{
	PLAN_CREATE=1,
	PLAN_ADJ,
}PILE_TYPE;/*�������� {���ɣ�1�� ��������2��*/

/******************************** ���׮�����Ϣ ***********************************/	//zcx190807


typedef enum
{
	GUN_SINGLE,
	GUN_A,
	GUN_B,
}GUN_NUM;/*ǹ��� {Aǹ��1����Bǹ��2��}*/

typedef enum
{
	SEV_ENABLE=0,
	SEV_DISABLE,
}PILE_SERVICE;/*׮������ {���ã�0����ͣ�ã�1��}*/

//typedef enum 
//{
//	PILE_NOFAULT        =0x00000000,
//	PILE_Memory_FAULT	=0x00000001,	//	�ն������ڴ���ϣ�0��
//	PILE_Clock_FAULT	=0x00000002,	//  ʱ�ӹ���        ��1��
//	PILE_Board_FAULT	=0x00000004,	//  ����ͨ�Ź���    ��2��
//	PILE_MeterCom_FAULT	=0x00000008,	//  485�������    ��3��
//	PILE_Screen_FAULT	=0x00000010,	//  ��ʾ�����      ��4��
//	CardOffLine_FAULT	=0x00000020,	//	������ͨѶ�ж�  ��5��
//	PILE_ESAM_FAULT		=0x00000040,	//  ESAM����        ��6��
//	StopEct_FAULT		=0x00000080,	//  ��ͣ��ť�������ϣ�7��
//	Arrester_FAULT		=0x00000100,	//	����������		��8��
//	GunHoming_FAULT		=0x00000200,	//	���ǹδ��λ		��9��
//	OverV_FAULT			=0x00000400,	//	�����ѹ�澯		��10��
//	UnderV_FAULT		=0x00000800,	//	����Ƿѹ�澯		��11��
//	Pilot_FAULT			=0x00001000,	//	����г������Ƶ����澯��12��
//	Connect_FAULT		=0x00002000,	//	�����Ӵ�������	��13��
//	OverI_Warning		=0x00004000,	//	��������澯		��14��
//	OverI_FAULT			=0x00008000,	//	���������������	��15��
//	ACCir_FAULT			=0x00010000,	//	������·������	��16��
//	GunLock_FAULT		=0x00020000,	//	���ӿڵ��������ϣ�17��
//	GunOverTemp_FAULT	=0x00040000,	//	���ӿڹ��¹���	��18��
//	CC_FAULT			=0x00080000,	//	�������״̬CC�쳣��19��
//	CP_FAULT			=0x00100000,	//	������״̬CP�쳣��20��
//	PE_FAULT			=0x00200000,	//	PE���߹���		��21��
//	Dooropen_FAULT		=0x00400000,	//	���Ŵ򿪹���		(22)
//	Other_FAULT			=0x00800000,	//	������������	��23��
//}PILE_FAULT;/*���׮��������*/


/******************************** ������Ϣ ***********************************/		//zcx190710
//typedef enum 
//{
//	NO_FAU=0,
//	MEMORY_FAU,		//	�ն������ڴ���ϣ�0��
//	CLOCK_FAU,		//  ʱ�ӹ���        ��1��
//	BOARD_FAU,		//  ����ͨ�Ź���    ��2��
//	METER_FAU,		//  485�������     ��3��
//	SCREEN_FAU,		//  ��ʾ�����      ��4��
//	HPLC_FAU,		//  �ز�ͨ���쳣    ��5��
//	NANDFLSH_FAU,	//	NandFLASH��ʼ������  ��6��
//	ESAM_FAU,		//  ESAM����        ��7��
//	BLE_FAU,		//	����ģ�����     ��8��
//	BATTERY_FAU,	//	��Դģ����� 	��9��
//	CANCOM_FAU,		//  ���׮ͨ�Ź��� ��10��
//	CHGPILE_FAU,	//  ���׮�豸���� ��11��
//	ORDRECOED_FAU, 	//	���ض�����¼�� ��12��
//	RTC_FAU,		//	RTCͨ�Ź��� ��13��
//}ROUTER_FAU;/*·������������*/

/************************************** ������ҵ�� *******************************************/
typedef enum {
	Cmd_Null=0,								//δ�յ�ָ��
	
	Cmd_ChgRequest,							//�����������
	Cmd_ChgRequestAck,						//�����������Ӧ��
	
	Cmd_ChgPlanIssue, 						//���ƻ��·�
	Cmd_ChgPlanIssueAck,                 	//���ƻ��·�Ӧ��
	Cmd_ChgPlanOffer, 						//���ƻ��¼��ϱ�
	Cmd_ChgPlanOfferAck,                 	//���ƻ��ϱ��¼�Ӧ��
	
	Cmd_ChgPlanAdjust,                 		//���ƻ�����
	Cmd_ChgPlanAdjustAck,                 	//���ƻ�����Ӧ��

	Cmd_ChgRequestReport,					//��������¼�����
	Cmd_ChgRequestReportAck,				//��������¼�����Ӧ��

	Cmd_ChgPlanExeState,                    //���ƻ�ִ��״̬�¼��ϱ�
	Cmd_ChgPlanExeStateAck,                 //���ƻ�ִ��״̬�¼��ϱ�Ӧ��
//	Cmd_ChgRequestConfirm,					//�������ȷ�ϣ�֪ͨ������
	
	Cmd_StartChg,							//�����������·�
	Cmd_StartChgAck,						//�������Ӧ��
	Cmd_StopChg,							//ֹͣ�������·�
	Cmd_StopChgAck,							//ֹͣ���Ӧ��
	Cmd_PowerAdj,							//���ʵ��ڲ����·�
	Cmd_PowerAdjAck,						//���ʵ���Ӧ��

	Cmd_ChgRecord,							//���ͳ�綩���¼�
	Cmd_ChgRecordAck,						//���ͳ�綩���¼�ȷ��
	Cmd_DeviceFault,                      	//����·�����쳣״̬
	Cmd_DeviceFaultAck,                      	//����·�����쳣״̬Ӧ��
	Cmd_PileFault,                 			//���ͳ��׮�쳣״̬
	Cmd_PileFaultAck,                 			//���ͳ��׮�쳣״̬Ӧ��
	Cmd_ChgPlanIssueGetAck,
	
	Cmd_RouterExeState,                    	//·����ִ��״̬��ѯ��
	Cmd_RouterExeStateAck,                 	//·����ִ��״̬Ӧ���
	
	Cmd_STAOnlineState,						//STA�������·��������״̬��
	Cmd_STAOnlineStateAck,					//STA�������·��������״̬ȷ�ϡ�
}COMM_CMD_C;//ҵ�������������
#define COMM_CMD_C rt_uint32_t

//�����¼�����
typedef enum {
	CTRL_NO_EVENT             	=0x00000000,
	ChgPlanIssue_EVENT       	=0x00000001,        	//���ƻ��·��¼�
	ChgPlanIssueGet_EVENT     	=0x00000002,      		//���ƻ��ٲ��¼�	
	ChgPlanAdjust_EVENT    		=0x00000004,        	//���ƻ������¼�
	StartChg_EVENT				=0x00000008,          	//��������¼�
	StopChg_EVENT				=0x00000010,          	//ֹͣ����¼�
	PowerAdj_EVENT				=0x00000020,          	//���������¼�
	AskState_EVENT				=0x00000040,          	//��ѯ����״̬�¼�
	ChgRequest_EVENT			=0x00000080,          	//�����������¼�
	ChgReqReportConfirm_EVENT	=0x00000100,          	//�������ȷ���¼�����������·������
}CTRL_EVENT_TYPE;//ҵ�����¼�

//////////////////////////////////////////////////////////////////////////////////


extern unsigned long timebin2long(unsigned char *buf);
CCMRAM extern char Printf_Buffer[1024];
CCMRAM extern char Sprintf_Buffer[1024];


extern const char ProgramVersion[8]; // �汾��
extern void Kick_Dog(void);

extern unsigned char str2bcd(char*src,unsigned char *dest);
extern void BCD_toInt(unsigned char *data1,unsigned char *data2,unsigned char len);
extern void Int_toBCD(unsigned char *data1,unsigned char *data2,unsigned char len);
extern unsigned char bcd2str(unsigned char *src,char *dest,unsigned char count);
extern unsigned char CRC7(unsigned char *ptr,unsigned int cnt);
//extern unsigned short CRC16_CCITT_ISO(unsigned char *ptr, unsigned int count);
extern unsigned int CRC_16(unsigned char *ptr, unsigned int nComDataBufSize);
extern unsigned char CRC7(unsigned char *ptr,unsigned int count);
extern unsigned char XOR_Check(unsigned char *pData, unsigned int Len);
extern char *itoa(int num,char *str,int radix);

extern void my_printf(char* buf,rt_uint32_t datalenth,rt_uint8_t type,rt_uint8_t cmd,char* function);
////////////////////////////////////////////////////////////////////////////////// 

#endif
