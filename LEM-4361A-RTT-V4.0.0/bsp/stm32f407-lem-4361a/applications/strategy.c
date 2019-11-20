#include <rtthread.h>
#include <rtdevice.h>
#include "strategy.h"
#include "chargepile.h"
#include "698.h"
#include "meter.h"
#include "storage.h"
#include "bluetooth.h"
#include <board.h>

#ifndef FALSE
#define FALSE         0
#endif
#ifndef TRUE
#define TRUE          1
#endif
#ifndef SUCCESSFUL
#define SUCCESSFUL    0
#endif
#ifndef FAILED
#define FAILED        1
#endif
#ifndef ORTHERS
#define ORTHERS       255
#endif

#define THREAD_STRATEGY_PRIORITY     8
#define THREAD_STRATEGY_STACK_SIZE   1024*4
#define THREAD_STRATEGY_TIMESLICE    5

static rt_uint8_t strategy_stack[THREAD_STRATEGY_STACK_SIZE];//�̶߳�ջ
static struct rt_thread strategy;

//static rt_mutex_t strategy_mutex = RT_NULL;

#define RELAYA_PIN    GET_PIN(F, 2)
#define RELAYB_PIN    GET_PIN(F, 3)

#define KC1_PIN     GET_PIN(F, 4)
#define KC2_PIN     GET_PIN(F, 5)
#define KC3_PIN     GET_PIN(F, 6)

extern rt_uint32_t Strategy_get_BLE_event(void);
extern rt_uint8_t BLE_CtrlUnit_RecResp(COMM_CMD_C cmd,void *STR_SetPara,int count);
////////////////////////////// ���������� ////////////////////////////////
//����ԭ��
static char *err_strfault[] = 
{
           "                       ",            /* ERR_OK          0  */
           "�ն������ڴ���ϣ�   ",               /* ERR             1  */
           "ʱ�ӹ��ϣ�         ",                 /* ERR             2  */
           "����ͨ�Ź��ϣ�	       ",            /* ERR             3  */
           "485������ϣ�         ",              /* ERR             4  */
           "��ʾ����ϣ�         ",               /* ERR             5  */
           "�ز�ͨ���쳣��       ",               /* ERR             6  */
           "NandFLASH��ʼ������       ",        /* ERR             7  */
           "ESAM����         ",                 /* ERR             8  */
           "����ģ����ϣ�         ",             /* ERR             9  */
           "��Դģ����ϣ�         ",             /* ERR             10 */
           "���׮ͨ�Ź��ϣ�        ",            /* ERR             11 */
           "���׮�豸���ϣ�         ",           /* ERR             12 */
	       "���ض�����¼����       ",             /* ERR             13 */
		   "RTCͨ�Ź��ϣ�       ",             	 /* ERR             14 */
};

const char* _hplc_event_char[]={//698 �¼�����   ��ӡ��־��
	"null",
	"Charge_Request",							//�����������
	"Charge_Request_Ack",						//�����������Ӧ��
	
	"Charge_PlanIssue", 					//���ƻ��·�
	"Charge_PlanIssue_Ack",                 	//���ƻ��·�Ӧ��
	"Charge_Plan_Offer", 						//���ƻ��¼��ϱ�
	"Charge_Plan_Offer_Ack",                 	//���ƻ��ϱ��¼�Ӧ��
	
	"Charge_Plan_Adjust",                 		//���ƻ�����
	"Charge_Plan_Adjust_Ack",                 	//���ƻ�����Ӧ��


	"Charge_Request_Report",					//��������¼�����
	"Charge_Request_Report_Ack",				//��������¼�����Ӧ��
//	"Charge_Request_Report_APP",				//��������¼���֪APP
//	"Charge_Request_Confirm",					//�������ȷ�ϣ�֪ͨ������
	
	"Charge_Plan_Exe_State",                    //���ƻ�ִ��״̬�¼��ϱ�
	"Charge_Plan_Exe_State_Ack",                 //���ƻ�ִ��״̬�¼��ϱ�Ӧ��
	
	"Start_Charge",							//�����������·�
	"Start_Charge_Ack",						//�������Ӧ��
	"Stop_Charge",							//ֹͣ�������·�
	"Stop_Charge_Ack",							//ֹͣ���Ӧ��
	"Power_Adj",							//���ʵ��ڲ����·�
	"Power_Adj_Ack",						//���ʵ���Ӧ��

	"Charge_Record",							//���ͳ�綩��
	"Charge_RecordAck",						//���ͳ�綩���¼�ȷ��
	"Device_Fault",                      	//����·�����쳣״̬
	"Device_FaultAck",                      	//����·�����쳣״̬
	"Pile_Fault",                 			//���ͳ��׮�쳣״̬
	"Pile_FaultAck",                 			//���ͳ��׮�쳣״̬
	"Charge_Plan_Issue_Get_Ack",
	
	"Read_Router_State",                    	//·����ִ��״̬��ѯ
	"Read_Router_State_Ack",                 	//·����ִ��״̬Ӧ��
	
	"STAOnlineState",						//STA�������·��������״̬��
	"STAOnlineStateAck",					//STA�������·��������״̬ȷ�ϡ�
};//ҵ�������������


CCMRAM ChargPilePara_TypeDef ChargePilePara_Set;
CCMRAM ChargPilePara_TypeDef ChargePilePara_Get;

CCMRAM static CHARGE_EXE_STATE_ASK ExeState_CtrlAsk;//������
CCMRAM static CHARGE_EXE_STATE_ASK ExeState_BleAsk;//����

CCMRAM static CHARGE_STRATEGY Chg_Strategy;
CCMRAM static CHARGE_STRATEGY_RSP Chg_StrategyRsp;
CCMRAM static CHARGE_STRATEGY Chg_Strategy_Adj;
CCMRAM static CHARGE_STRATEGY_RSP Chg_StrategyRsp_Adj;

CCMRAM static CHARGE_APPLY Chg_Apply;
CCMRAM static CHARGE_APPLY_RSP Chg_Apply_Rsp;

//�����¼�
CCMRAM static PLAN_OFFER_EVENT Plan_Offer_Event;
CCMRAM static CHARGE_APPLY_EVENT Chg_Apply_Event;
//CCMRAM static CHARGE_EXE_EVENT BLE_ChgExe_Event;
//�������״̬�仯�¼�
CCMRAM static ONLINE_STATE OnlineState_Event;	

//��ʱ����
CCMRAM static rt_timer_t StartChgResp_Timer;
CCMRAM static rt_timer_t StopChgResp_Timer;
CCMRAM static rt_timer_t PowerAdjResp_Timer;
CCMRAM static rt_timer_t ChgReqReportRsp_Timer;
//��ʱ��ѵ
CCMRAM static rt_timer_t ChgPileStateGet_Timer;
//�ȴ��ϱ��¼�ȷ��
CCMRAM static rt_timer_t ChgPlanOfferAck_Timer;
CCMRAM static rt_timer_t ChgPlanExeStateAck_Timer;
CCMRAM static rt_timer_t ChgRecordAck_Timer;


//ָ���־
CCMRAM static rt_bool_t startchg_flag;
CCMRAM static rt_bool_t stopchg_flag;
CCMRAM static rt_bool_t adjpower_flag;

CCMRAM static CTL_CHARGE Ctrl_Start;//�������ز���
CCMRAM static CTL_CHARGE Ctrl_Stop;
CCMRAM static CTL_CHARGE BLE_Stop;
CCMRAM static CTL_CHARGE Ctrl_PowerAdj;

//���ִ��״̬
CCMRAM static CHARGE_EXE_STATE Chg_ExeState;

CCMRAM static CTRL_CHARGE_EVENT CtrlCharge_Event;
CCMRAM static CHARGE_EXE_EVENT Ctrl_ChgExe_Event;
CCMRAM static CHG_ORDER_EVENT ChgOrder_Event;//��������һ������

//CCMRAM static unsigned char PlanSlotCount;
//CCMRAM static unsigned char SetPowerFinishFlag[50];
//CCMRAM static char cRequestNO_Old[17];
//CCMRAM static char cRequestNO_New[17];


CCMRAM ScmMeter_HisData stMeter_HisData_Strategy;


//CCMRAM rt_uint32_t* Charge_Event_Ptr[] = {
//	0,
//	(rt_uint32_t*)&Chg_Apply,//�������
//	(rt_uint32_t*)&Chg_Apply,//���������Ӧ
//	(rt_uint32_t*)&Chg_Strategy,//���ƻ��·�
//	(rt_uint32_t*)&Chg_StrategyRsp,//���ƻ��·�Ӧ��
//	(rt_uint32_t*)&Plan_Offer_Event,//���ƻ��¼��ϱ�
//	(rt_uint32_t*)&Plan_Offer_Event,//���ƻ��ϱ��¼�Ӧ��
//	(rt_uint32_t*)&Chg_Strategy_Adj,//���ƻ�����
//	(rt_uint32_t*)&Chg_StrategyRsp_Adj,//���ƻ�����Ӧ��
//	(rt_uint32_t*)&Chg_Apply_Event,//��������¼�����
//	(rt_uint32_t*)&Chg_Apply_Event,//��������¼�����Ӧ��
//	(rt_uint32_t*)&Chg_Apply_Event,//��������¼�����
//	
//	(rt_uint32_t*)&Plan_Offer_Event,//���ƻ�ִ��״̬�¼��ϱ�
//	(rt_uint32_t*)&Plan_Offer_Event,//���ƻ�ִ��״̬�¼��ϱ�Ӧ��
//	(rt_uint32_t*)&Ctrl_Start,//��������
//	(rt_uint32_t*)&Ctrl_Start,//��������Ӧ��
//	(rt_uint32_t*)&Ctrl_Stop,//ֹͣ����
//	(rt_uint32_t*)&Ctrl_Stop,//ֹͣ����Ӧ��
//	(rt_uint32_t*)&Ctrl_PowerAdj,//���ʵ��ڲ����·�
//	(rt_uint32_t*)&Ctrl_PowerAdj,//���ʵ���Ӧ��
//	(rt_uint32_t*)&ChgOrder_Event,//���ͳ�綩���¼�
//	(rt_uint32_t*)&ChgOrder_Event,//���ͳ�綩���¼�ȷ��
//	0,														//����·�����쳣״̬
//	0,														//���ͳ��׮�쳣״̬
//	0, 														//Cmd_ChgPlanIssueGetAck
//	(rt_uint32_t*)&Chg_ExeState,	//·����ִ��״̬��ѯ��
//	(rt_uint32_t*)&Chg_ExeState,//·����ִ��״̬Ӧ���
//	(rt_uint32_t*)&OnlineState_Event,//STA�������·��������״̬��
//	(rt_uint32_t*)&OnlineState_Event,//STA�������·��������״̬ȷ�ϡ�
//	(rt_uint32_t*)&OnlineState_Event,//����״̬֪ͨAPP	
//};




/********************************************************************  
*	�� �� ��: TimeCalculation
*	����˵��: ʱ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static time_t TimeCalculation(STR_SYSTEM_TIME* TimeStamp)
{
	time_t time_s = 0;
	rt_uint8_t year,month,day,hour,minute,second;
	struct tm timep;
	
//	rt_mutex_take(strategy_mutex, RT_WAITING_FOREVER);
	
	BCD_toInt(&second,&TimeStamp->Second,1);
	BCD_toInt(&minute,&TimeStamp->Minute,1);
	BCD_toInt(&hour,&TimeStamp->Hour,1);
	BCD_toInt(&day,&TimeStamp->Day,1);	
//	BCD_toInt((unsigned char*)&timep.tm_wday,&TimeStamp.Week,1);
	BCD_toInt(&month,&TimeStamp->Month,1);
	BCD_toInt(&year,&TimeStamp->Year,1);
	
	year += 100;
	month -=1;
	
	timep.tm_sec = second;
	timep.tm_min = minute;
	timep.tm_hour = hour;
	timep.tm_mday = day;
	timep.tm_mon = month;
	timep.tm_year = year;
	

	time_s = mktime(&timep);
	
//	rt_mutex_release(strategy_mutex);
	
	return time_s;
}

/**************************************************************
 * ��������: RELAY_ON 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ���ϼ̵���
 **************************************************************/
 void RELAY_ON(void)
{
	rt_pin_write(RELAYB_PIN, PIN_LOW);
	rt_thread_mdelay(200);
	rt_pin_write(RELAYA_PIN, PIN_HIGH);
}
/**************************************************************
 * ��������: RELAY_OFF 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �Ͽ��̵���
 **************************************************************/
void RELAY_OFF(void)
{
	rt_pin_write(RELAYA_PIN, PIN_LOW);
	rt_thread_mdelay(200);
	rt_pin_write(RELAYB_PIN, PIN_HIGH);
}
/**************************************************************
 * ��������: StartChgResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ������糬ʱ����
 **************************************************************/
static void StartChgResp_Timeout(void *parameter)
{
	rt_int8_t p_rst;
	rt_lprintf("[strategy]: StartChgResp event is timeout!\n");
	p_rst = ChargepileDataGetSet(Cmd_ChargeStartResp,0);
	
//	if(p_rst != SUCCESSFUL)
	
}
/**************************************************************
 * ��������: StopChgResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ֹͣ��糬ʱ����
 **************************************************************/
static void StopChgResp_Timeout(void *parameter)
{
    rt_uint8_t p_rst;
	rt_lprintf("[strategy]: StopChgResp event is timeout!\n");
	p_rst = ChargepileDataGetSet(Cmd_ChargeStopResp,0);
	
//	if(p_rst == SUCCESSFUL)
//		PileIfo.WorkState = ChgSt_Finished;
}
/**************************************************************
 * ��������: PowAdjResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �������ʳ�ʱ����
 **************************************************************/
static void PowAdjResp_Timeout(void *parameter)
{
    rt_uint8_t p_rst;
	rt_lprintf("[strategy]: PowerAdjResp event is timeout!\n");
	p_rst = ChargepileDataGetSet(Cmd_SetPowerResp,0);
}
/**************************************************************
 * ��������: ChgReqReportResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ���ͳ�����볬ʱ����
 **************************************************************/
static void ChgReqReportResp_Timeout(void *parameter)
{
    rt_lprintf("[strategy]: ChgReqReportResp event is timeout!\n");
	
}
/**************************************************************
 * ��������: ChgReqReportResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ��ѵ���׮״̬����
 **************************************************************/
static void ChgPileStateGet_Timeout(void *parameter)
{
    rt_lprintf("[strategy]: ChgPileStateGet event is timeout!\n");
	//��ѯ���׮״̬
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
}
/**************************************************************
 * ��������: ChgPlanOfferAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ����ƻ�ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t ChgPlanOfferAck_count;
static void ChgPlanOfferAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;
	rt_kprintf("[strategy]: ChgPlanOfferAck event is timeout!\n");
	
	c_rst = CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		ChgPlanOfferAck_count++;
	}
		
	if(ChgPlanOfferAck_count>=3)
		rt_timer_stop(ChgPlanOfferAck_Timer);
}

/**************************************************************
 * ��������: ChgPlanExeStateAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ����ƻ�ִ��״̬ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t ChgPlanExeStateAck_count;
static void ChgPlanExeStateAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;
	rt_kprintf("[strategy]: ChgPlanExeStateAck event is timeout!\n");
	
	c_rst = CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//�ϱ����ƻ�ִ���¼�
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		ChgPlanExeStateAck_count++;
	}
		
	if(ChgPlanExeStateAck_count>=3)
		rt_timer_stop(ChgPlanExeStateAck_Timer);
}
/**************************************************************
 * ��������: ChgRecordAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ���綩��ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t ChgRecordAck_count;
static void ChgRecordAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;
	rt_kprintf("[strategy]: ChgRecordAck event is timeout!\n");
	
	c_rst = CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//�ϱ���綩��
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		ChgRecordAck_count++;
	}
		
	if(ChgRecordAck_count>=3)
		rt_timer_stop(ChgRecordAck_Timer);
}

/**************************************************************
 * ��������: timer_create_init 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ��ʱ��
 **************************************************************/
static void timer_create_init()
{
     /* ���������ظ���ʱ�� */
	 StartChgResp_Timer = rt_timer_create("StartChgResp",  /* ��ʱ�������� StartChgResp */
									StartChgResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* һ���Զ�ʱ�� */
	/* ����ͣ���ظ���ʱ�� */
	 StopChgResp_Timer = rt_timer_create("StopChgResp",  /* ��ʱ�������� StopChgResp */
									StopChgResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* һ���Զ�ʱ�� */
	/* �����������ʻظ���ʱ�� */
	 PowerAdjResp_Timer = rt_timer_create("PowerAdjResp",  /* ��ʱ�������� PowerAdjResp */
									PowAdjResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* һ���Զ�ʱ�� */
	
	/* ���� ����������ͻظ� ��ʱ�� */
	 ChgReqReportRsp_Timer = rt_timer_create("ChgReqReportRsp",  /* ��ʱ�������� ChgReqReportRsp */
									ChgReqReportResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* һ���Զ�ʱ�� */
	/* ���� ���׮״̬��ѯ ��ʱ�� */
	ChgPileStateGet_Timer = rt_timer_create("ChgPileStateGet",  /* ��ʱ�������� ChgPileStateGet */
									ChgPileStateGet_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									1000, /* ��ʱ���ȣ���OS TickΪ��λ����1000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
	/* ���� ���ƻ�ȷ�� ��ʱ�� */
	ChgPlanOfferAck_Timer = rt_timer_create("ChgPlanOfferAck",  /* ��ʱ�������� ChgPlanOfferAck */
									ChgPlanOfferAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
	/* ���� ���ƻ�ִ��״̬ȷ�� ��ʱ�� */
	ChgPlanExeStateAck_Timer = rt_timer_create("ChgPlanExeStateAck",  /* ��ʱ�������� ChgPlanExeStateAck */
									ChgPlanExeStateAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
	/* ���� ��綩��ȷ�� ��ʱ�� */
	ChgRecordAck_Timer = rt_timer_create("ChgRecordAck",  /* ��ʱ�������� ChgRecordAck */
									ChgRecordAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
	/* ������ʱ�� */
	if (ChgPileStateGet_Timer != RT_NULL)
		rt_timer_start(ChgPileStateGet_Timer);
}
/********************************************************************  
*	�� �� ��: ExeState_Update()
*	����˵��: ·�������ִ��״̬����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void ExeState_Update(void)
{
	
	memcpy(Chg_ExeState.cRequestNO,Plan_Offer_Event.Chg_Strategy.cRequestNO,sizeof(Chg_ExeState.cRequestNO));
	memcpy(Chg_ExeState.cUserID,Plan_Offer_Event.Chg_Strategy.cUserID,sizeof(Chg_ExeState.cUserID));
	
	memcpy(Chg_ExeState.cAssetNO,RouterInfo.AssetNum,sizeof(Chg_ExeState.cAssetNO));
	Chg_ExeState.GunNum = Plan_Offer_Event.Chg_Strategy.GunNum;
	
	if(Chg_ExeState.exeState != EXE_ING)//��ִ�й����У����ƻ��ж���ʴ�
		Chg_ExeState.ucPlanPower = Chg_Strategy.ulChargeRatePow;
	
	ScmMeter_HisData stgMeter_HisData;
	cmMeter_get_data(EMMETER_HISDATA,&stgMeter_HisData);//��ȡ����������
	memcpy(&Chg_ExeState.ulEleBottomValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
	memcpy(&Chg_ExeState.ulEleActualValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
	
	ScmMeter_Analog stgMeter_Analog;
	cmMeter_get_data(EMMETER_ANALOG,&stgMeter_Analog);
	Chg_ExeState.ucActualPower = stgMeter_Analog.ulAcPwr/100;
	Chg_ExeState.ucVoltage.A = stgMeter_Analog.ulVol;
	Chg_ExeState.ucCurrent.A = stgMeter_Analog.ulCur;
	
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
	Router_WorkState.Pile_State = ChargePilePara_Get.ChgPileState;
	Chg_ExeState.ChgPileState = Router_WorkState.Pile_State;//ֱ��ȡ�ó��׮����̬	
}

/********************************************************************  
*	�� �� ��: Charge_PlanIssue_RSP()
*	����˵��: ���ƻ���Ӧ
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Charge_Apply_RSP(CHARGE_APPLY* chg_apply,CHARGE_APPLY_RSP* chg_apply_rsp)//���������Ӧ
{
	memcpy(&chg_apply_rsp->cRequestNO,&chg_apply->cRequestNO,sizeof(chg_apply_rsp->cRequestNO));
	memcpy(&chg_apply_rsp->cAssetNO,&chg_apply->cAssetNO,sizeof(chg_apply_rsp->cAssetNO));
	chg_apply_rsp->GunNum = chg_apply->GunNum;

	if(memcmp(&RouterInfo.AssetNum,&chg_apply->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
	{
		chg_apply_rsp->cSucIdle = SUCCESSFUL;
	}
	else
		chg_apply_rsp->cSucIdle = ORTHERS;
	
	rt_kprintf("[strategy]: (%s)  Charge Apply response sucidle = %d \n",__func__,chg_apply_rsp->cSucIdle);
	
	BLE_CtrlUnit_RecResp(Cmd_ChgRequestAck,&Chg_Apply_Rsp,0);	//�������ȷ����Ӧ
}

/********************************************************************  
*	�� �� ��: Charge_Apply_Event_Create()
*	����˵��: ���ɳ�������¼���¼
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

static rt_int8_t Charge_Apply_Event_Create(CHARGE_APPLY* chg_apply,CHARGE_APPLY_EVENT* chg_apply_event)
{
	rt_int8_t res;
	static rt_uint32_t OrderNum = 0;
	
	memset(chg_apply_event,0,sizeof(CHARGE_APPLY_EVENT));

	rt_kprintf("[strategy]:  (%s) Create charge apply event record...!\n",__func__);

	if(memcmp(&RouterInfo.AssetNum,&chg_apply->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
	{
		OrderNum++;

		chg_apply_event->OrderNum = OrderNum;

		memcpy(&chg_apply_event->RequestTimeStamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�������ʱ��
		memcpy(&chg_apply_event->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
		memcpy(&chg_apply_event->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��		

		memcpy(&chg_apply_event->RequestNO,&chg_apply->cRequestNO,sizeof(chg_apply_event->RequestNO));
		memcpy(&chg_apply_event->AssetNO,&chg_apply->cAssetNO,sizeof(chg_apply_event->AssetNO));
		chg_apply_event->GunNum = chg_apply->GunNum;
		
		chg_apply_event->ChargeReqEle = chg_apply->ulChargeReqEle;			
		memcpy(&chg_apply_event->PlanUnChg_TimeStamp,&chg_apply->PlanUnChg_TimeStamp,sizeof(STR_SYSTEM_TIME));
		chg_apply_event->ChargeMode = chg_apply->ChargeMode;
		
		memcpy(&chg_apply_event->UserAccount,&chg_apply->cUserID,sizeof(chg_apply_event->UserAccount));							
		memcpy(&chg_apply_event->Token,&chg_apply->Token,sizeof(chg_apply_event->Token));		//��32��Ч��ȡֵ
		
		rt_kprintf("[strategy]:  (%s) Create charge apply event record sucess!\n",__func__);
		
		
		//�����¼�����
		res = SetStorageData(Cmd_ChgRequestWr,chg_apply_event,sizeof(CHARGE_APPLY_EVENT));
		if(res == SUCCESSFUL)
		{
			rt_kprintf("[strategy]:  (%s) Storage Chg_Apply_Event, Successful!\n",__func__);
		}
		else
		{
			rt_kprintf("[strategy]:  (%s) Storage Chg_Apply_Event, fail!\n",__func__);
		}
		return 1;
	}
	else
	{
		rt_kprintf("[strategy]:  (%s) Create charge apply event record fail!\n",__func__);
		return -1;
	}
	return -1;
}




/********************************************************************  
*	�� �� ��: Charge_PlanIssue_RSP()
*	����˵��: ���ƻ���Ӧ
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Charge_PlanIssue_RSP(CHARGE_STRATEGY* charge_plan,CHARGE_STRATEGY_RSP* charge_plan_rsp)
{
	memcpy(&charge_plan_rsp->cRequestNO,&charge_plan->cRequestNO,sizeof(charge_plan_rsp->cRequestNO));
	memcpy(&charge_plan_rsp->cAssetNO,&charge_plan->cAssetNO,sizeof(charge_plan_rsp->cAssetNO));
	charge_plan_rsp->GunNum = charge_plan->GunNum;
	
	if(memcmp(&RouterInfo.AssetNum,&charge_plan->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
	{
//		memset(&SetPowerFinishFlag,0,50);//��ձ�־λ
//		PlanSlotCount = 0;
		charge_plan_rsp->cSucIdle = SUCCESSFUL;
	}
	else
		charge_plan_rsp->cSucIdle = ORTHERS;
	
	rt_kprintf("[strategy]: (%s)  Charge Plan response sucidle = %d \n",__func__,charge_plan_rsp->cSucIdle);	
	
	CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,charge_plan_rsp,0);//���ƻ���Ӧ
	
	
	CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Ack = RT_TRUE;//���ƻ�����Ӧ
		
}


/********************************************************************  
*	�� �� ��: Charge_Plan_Event_Create()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

static rt_int8_t Charge_Plan_Event_Create(CHARGE_STRATEGY* charge_plan,PLAN_OFFER_EVENT* charge_plan_event)
{
	rt_int8_t res;
	static rt_uint32_t OrderNum = 0;
	
	memset(charge_plan_event,0,sizeof(PLAN_OFFER_EVENT));
	
	rt_kprintf("[strategy]:  (%s) Create charge plan event record...!\n",__func__);

	if(memcmp(&RouterInfo.AssetNum,&charge_plan->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
	{	
		OrderNum++;
		charge_plan_event->OrderNum = OrderNum;		
		charge_plan_event->ChannelState = 0;//ͨ��״̬
		charge_plan_event->OccurSource = 0;//�¼�����Դ
		
		memcpy(&charge_plan_event->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//��¼�ϱ�ʱ��
		memcpy(&charge_plan_event->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
		
		memcpy(&charge_plan_event->Chg_Strategy,charge_plan,sizeof(CHARGE_STRATEGY));
			
		rt_kprintf("[strategy]:  (%s) Create charge plan event record sucess!\n",__func__);
		
		res = SetStorageData(Cmd_PlanOfferWr,charge_plan_event,sizeof(PLAN_OFFER_EVENT));//�洢���ƻ��ϱ��¼���¼

		if(res == SUCCESSFUL)
		{
			rt_kprintf("[strategy]:  (%s) Storage Plan_Offer_Event, Successful!\n",__func__);
		}
		else
		{
			rt_kprintf("[strategy]:  (%s) Storage Plan_Offer_Event, fail!\n",__func__);
		}
		return 1;
	}
	else
	{
		rt_kprintf("[strategy]:  (%s) Create charge plan event record fail!\n",__func__);
		return -1;
	}
	return -1;
}

/********************************************************************  
*	�� �� ��: Charge_PlanAdj_RSP()
*	����˵��: ���ƻ�������Ӧ
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Charge_PlanAdj_RSP(CHARGE_STRATEGY* charge_plan_adj,CHARGE_STRATEGY_RSP* charge_plan_adj_rsp)
{
	if(memcmp(&RouterInfo.AssetNum,&charge_plan_adj->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
	{
		memcpy(&charge_plan_adj_rsp->cRequestNO,&Chg_Strategy_Adj.cRequestNO,sizeof(Chg_Strategy_Adj.cRequestNO));
		memcpy(&charge_plan_adj_rsp->cAssetNO,&Chg_Strategy_Adj.cAssetNO,sizeof(Chg_Strategy_Adj.cAssetNO));
		charge_plan_adj_rsp->cSucIdle = SUCCESSFUL;
	}
	else
	{
		charge_plan_adj_rsp->cSucIdle = ORTHERS;
	}
	
	CtrlUnit_RecResp(Cmd_ChgPlanAdjustAck,&charge_plan_adj_rsp,0);//�ظ�
}

/********************************************************************  
*	�� �� ��: Charge_PlanAdj_RSP()
*	����˵��: ���ƻ�������Ӧ
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 


static void ChargePile_Start_Charge(ChargPilePara_TypeDef* PilePara)
{
	rt_int8_t res=0;
	
	startchg_flag = TRUE;
	
	CtrlCharge_Event.CtrlType = CTRL_START;
	
	ChargepileDataGetSet(Cmd_SetPower,&PilePara);//�·����׮�趨����
	
	rt_kprintf("[strategy]:  (%s) set charge duty = %d !\n",__func__,PilePara->PWM_Duty);

	res = ChargepileDataGetSet(Cmd_ChargeStart,0);//������������
	
	if(res == SUCCESSFUL)
	{
		if (StartChgResp_Timer != RT_NULL)
			rt_timer_start(StartChgResp_Timer);

		Chg_ExeState.exeState = EXE_ING;				
	}
	else
	{
		Chg_ExeState.exeState = EXE_FAILED;
	}
	rt_kprintf("[strategy]:  (%s) Charge Start Exe state = %d \n",__func__,Chg_ExeState.exeState);
}

static void ChargePile_Stop_Charge(void)
{
	rt_int8_t res=0;
	
	stopchg_flag = TRUE;
	
	res = ChargepileDataGetSet(Cmd_ChargeStop,0);
	
	if(res == SUCCESSFUL)
	{
		if(StopChgResp_Timer != RT_NULL)
			rt_timer_start(StopChgResp_Timer);
	}
	
	rt_kprintf("[strategy]:  (%s) stop charge cmd  res = %d !\n",__func__,res);
}


/********************************************************************  
*	�� �� ��: Charge_Plan_Event_Create()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

static rt_int8_t Charge_Plan_Exe_Event_Create(CHARGE_EXE_STATE* charge_state,CHARGE_EXE_EVENT* charge_plan_exe_event)
{
	rt_int8_t res;
	static rt_uint32_t OrderNum = 0;
	
	memset(charge_plan_exe_event,0,sizeof(CHARGE_EXE_EVENT));
	
	rt_kprintf("[strategy]:  (%s) Create charge plan exe event record...!\n",__func__);

	OrderNum++;
	charge_plan_exe_event->OrderNum = OrderNum;
	charge_plan_exe_event->OccurSource = 0;
	charge_plan_exe_event->ChannelState = 0;

	memcpy(&charge_plan_exe_event->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//��¼�ϱ�ʱ��
	memcpy(&charge_plan_exe_event->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));

	memcpy(&charge_plan_exe_event->Chg_ExeState,charge_state,sizeof(CHARGE_EXE_STATE));
					
	res = SetStorageData(Cmd_ChgExecuteWr,charge_plan_exe_event,sizeof(CHARGE_EXE_EVENT));

	if(res == SUCCESSFUL)
	{
		rt_kprintf("[strategy]:  (%s) Storage Plan_exe_Event, Successful!\n",__func__);
		return 1;
	}
	else
	{
		rt_kprintf("[strategy]:  (%s) Storage Plan_exe_Event, fail!\n",__func__);
	}
	return -1;
}

/********************************************************************  
*	�� �� ��: Charge_Record_Create()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

static rt_int8_t Charge_Record_Create(CHARGE_APPLY_EVENT* chg_apply_event,PLAN_OFFER_EVENT* charge_plan_event,CHG_ORDER_EVENT* charge_record)
{
	rt_uint8_t i;
	static rt_uint32_t OrderNum = 0;
	
	OrderNum++;
	charge_record->OrderNum = OrderNum;
	charge_record->ChannelState = 0;
	charge_record->OccurSource = 0;
	
	memcpy(&charge_record->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
	memcpy(&charge_record->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
	
	memcpy(&charge_record->RequestNO,&charge_plan_event->Chg_Strategy.cRequestNO,sizeof(charge_record->RequestNO));
	memcpy(&charge_record->AssetNO,&charge_plan_event->Chg_Strategy.cAssetNO,sizeof(charge_record->AssetNO));
	memcpy(&charge_record->cUserID,&charge_plan_event->Chg_Strategy.cUserID,sizeof(charge_record->cUserID));
	
	memcpy(&charge_record->RequestTimeStamp,&chg_apply_event->RequestTimeStamp,sizeof(STR_SYSTEM_TIME));//
	memcpy(&charge_record->PlanUnChg_TimeStamp,&chg_apply_event->PlanUnChg_TimeStamp,sizeof(STR_SYSTEM_TIME));//
	
	memcpy(&charge_record->ChgStartTime,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
	memcpy(&charge_record->ChgStopTime,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
			
	cmMeter_get_data(EMMETER_HISDATA,&stMeter_HisData_Strategy);
	memcpy(&charge_record->StartMeterValue[0],&stMeter_HisData_Strategy.ulMeter_Total,5*sizeof(long));
	memcpy(&charge_record->StopMeterValue[0],&stMeter_HisData_Strategy.ulMeter_Total,5*sizeof(long));

	charge_record->ChargeReqEle = charge_plan_event->Chg_Strategy.ulChargeReqEle;
	charge_record->ChargeMode = charge_plan_event->Chg_Strategy.ucChargeMode;
	
	charge_record->GunNum = charge_plan_event->Chg_Strategy.GunNum;
	
	memset(&charge_record->ucChargeEle,0,5*sizeof(long));
	charge_record->ucChargeTime = 0;
	
//			s_rst = SetStorageData(Cmd_HistoryRecordWr,&ChgOrder_Event,sizeof(CHG_ORDER_EVENT));
//		if(s_rst == SUCCESSFUL)
//		{
//			rt_kprintf("[strategy]:  (%s) Storage ChgOrder_Event, Successful!\n",__func__);
//		}
//		else
//		{
//			rt_kprintf("[strategy]:  (%s) �����綩���¼���ʧ�ܣ�\n",__func__);
//			SetStorageData(Cmd_HistoryRecordWr,&ChgOrder_Event,sizeof(CHG_ORDER_EVENT));//�ٴ�һ��
//		}
}

/********************************************************************  
*	�� �� ��: Charge_Record_Update()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

static rt_int8_t Charge_Record_Update(CHG_ORDER_EVENT* charge_record)
{
	rt_uint8_t i;
	time_t start_time,stop_time;
	
	memcpy(&charge_record->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
	
	memcpy(&charge_record->ChgStopTime,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
	
	cmMeter_get_data(EMMETER_HISDATA,&stMeter_HisData_Strategy);
	memcpy(&charge_record->StopMeterValue[0],&stMeter_HisData_Strategy.ulMeter_Total,5*sizeof(long));
	
	
	
	for(i=0;i<5;i++)
		charge_record->ucChargeEle[i] = charge_record->StopMeterValue[i] - charge_record->StartMeterValue[i]; 
	
	start_time = TimeCalculation(&charge_record->ChgStartTime);
	stop_time = TimeCalculation(&charge_record->ChgStopTime);
	charge_record->ucChargeTime = stop_time - start_time;
	
}

static void Charge_Event_Data_Clear(void)
{
	memset(&Chg_Apply,0,sizeof(CHARGE_APPLY_EVENT));
	memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
	memset(&Plan_Offer_Event,0,sizeof(PLAN_OFFER_EVENT));
	
	
	memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
}

/********************************************************************  
*	�� �� ��: get_event_cmd()
*	����˵��: ��ѯ��ǰָ��
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static rt_uint8_t get_event_cmd(rt_uint32_t event)
{
	rt_uint8_t i;
	for(i = 0; i <sizeof(rt_uint32_t)*8;i++)
	{
		if(event&(0x00000001<<i))
		{
			if(i != 0)
			{
				rt_kprintf("[strategy]:  (%s)  recv event is [%s]  \n",__func__,_hplc_event_char[i]);
				return i;
			}
		}
	}
	return Cmd_Null;
}	



/********************************************************************  
*	�� �� ��: HPLC_Data_RecProcess()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void HPLC_Data_RecProcess(void)
{
	rt_int8_t i,res,cmd;
	rt_uint32_t EventCmd=Cmd_Null;
	
	/***************************** ���տ��������� *******************************/
	EventCmd = strategy_event_get();
	cmd = get_event_cmd(EventCmd);
	
	switch(cmd)
	{
		case Cmd_ChgPlanIssue://�յ����ƻ�
		{
			if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Apply == RT_FALSE)
			{
				memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
			}
			
			if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan == RT_TRUE)
			{
				CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,-1);//���ƻ�ȡֵ
				Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//���ƻ���Ӧ
				break;
			}
			
			memset(&Chg_Strategy,0,sizeof(Chg_Strategy));
			CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//���ƻ�ȡֵ
			
			Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//���ƻ���Ӧ
			
			if(Chg_StrategyRsp.cSucIdle == SUCCESSFUL)//��ȷ  ���ɳ��ƻ��¼�
			{
				if(Charge_Plan_Event_Create(&Chg_Strategy,&Plan_Offer_Event) > 0 )
				{
					CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
					if (ChgPlanOfferAck_Timer != RT_NULL)
					{
						rt_timer_start(ChgPlanOfferAck_Timer);
						ChgPlanOfferAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer start!\n",__func__);	
					}					
				}
				
				CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Event = RT_TRUE; //���ƻ��ϱ�
			}
					
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan = RT_TRUE;//�ѽ��յ����ƻ�

			break;
		}
		
		case Cmd_ChgPlanOfferAck:////�յ����ƻ��¼�ȷ��
		{	
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);

			CtrlUnit_RecResp(Cmd_ChgPlanOfferAck,0,0);//�ظ�
			rt_timer_stop(ChgPlanOfferAck_Timer);//�յ�ȷ����Ӧ ֹͣ�ط�
					
			if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Adj_Event == RT_TRUE)
				CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Adj_Event_Ack = RT_TRUE;
			else	
				CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Event_Ack = RT_TRUE; //���ƻ��ϱ�Ӧ��
			break;
		}
		case Cmd_ChgPlanExeStateAck://�յ����ƻ�ִ���¼�ȷ��
		{
			CtrlUnit_RecResp(Cmd_ChgPlanExeStateAck,0,0);//ȡֵ	
			rt_timer_stop(ChgPlanExeStateAck_Timer);		
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);
			break;
		}
		case Cmd_ChgRequestReportAck://��������¼�Ӧ��
		{
			CtrlUnit_RecResp(Cmd_ChgRequestReportAck,0,0);//�����־
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRequestReportAck clear!\n",__func__);
			break;
		}
		
		//�յ����ƻ�����
		case Cmd_ChgPlanAdjust:
		{
			res = CtrlUnit_RecResp(Cmd_ChgPlanAdjust,&Chg_Strategy_Adj,0);//ȡֵ	
			
			Charge_PlanAdj_RSP(&Chg_Strategy_Adj,&Chg_StrategyRsp_Adj);//���ƻ�������Ӧ
			
			if(memcmp(&Chg_Strategy_Adj.strChargeTimeSolts[0].strDecStartTime,\
				&Chg_Strategy_Adj.strChargeTimeSolts[0].strDecStopTime,sizeof(STR_SYSTEM_TIME)) == 0)
				{
					if(memcmp(&RouterInfo.AssetNum,&Chg_Strategy_Adj.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����		
					{
						ChargePile_Stop_Charge();
					}
					break;
				}
			
			if(Chg_StrategyRsp_Adj.cSucIdle == SUCCESSFUL)//��ȷ  ���³��ƻ�
			{
				memcpy(&Chg_Strategy,&Chg_Strategy_Adj,sizeof(Chg_Strategy));//��ȷ  ���³��ƻ�
				
				if(Charge_Plan_Event_Create(&Chg_Strategy,&Plan_Offer_Event) > 0 )//���ɳ��ƻ��¼�
				{
					CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
					
					if (ChgPlanOfferAck_Timer != RT_NULL)
					{
						rt_timer_start(ChgPlanOfferAck_Timer);
						ChgPlanOfferAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer start!\n",__func__);	
					}
					CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Adj_Event = RT_TRUE;
				}
			}
			
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Adj = RT_TRUE;
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Adj_Ack = RT_TRUE;
			
			break;
		}
		//�յ������������
		case Cmd_StartChg:
		{		
			res = CtrlUnit_RecResp(Cmd_StartChg,&Ctrl_Start,0);//ȡ����������

			if(memcmp(&RouterInfo.AssetNum,&Ctrl_Start.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
			{
				if(Router_WorkState.Router_State == ChgState_Standby)
				{
					ChargePile_Start_Charge(&ChargePilePara_Set);
					Ctrl_Start.cSucIdle = SUCCESSFUL;
				}
				else
				{
					Chg_ExeState.exeState = EXE_FAILED;
					rt_kprintf("[strategy]:  (%s) Charge Started, fail! Router_WorkState = %d\n",__func__,Router_WorkState.Router_State);
				}						
			}
			else
			{
				Ctrl_Start.cSucIdle = ORTHERS;
			}			
			res = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//�ظ�
			
			////////////////////////Ԥ�� ׼����¼�����¼�//////////////////////////
			
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Router_Svc_Start = RT_TRUE;
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Router_Svc_Start_Ack = RT_TRUE;
			
//			memcpy(&CtrlCharge_Event.Ctrl_ChgData,&Ctrl_Start,sizeof(Ctrl_Start));
//			CtrlCharge_Event.CtrlType = CTRL_START;
//			CtrlCharge_Event.StartSource = CTRL_UNIT;		
			//////////////////////////////////////////////////////////////////////
			break;
		}
		//�յ�ֹͣ�������
		case Cmd_StopChg:
		{	
			res = CtrlUnit_RecResp(Cmd_StopChg,&Ctrl_Stop,0);//ȡֵ			 

			if((memcmp(&RouterInfo.AssetNum,&Ctrl_Stop.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
				||(memcmp(&Ctrl_Start.OrderSn,&Ctrl_Stop.OrderSn,sizeof(Ctrl_Start.OrderSn)) == 0))//У����ͣ����һ����		
			{
				ChargePile_Stop_Charge();
			}
			else
			{
				Ctrl_Stop.cSucIdle = ORTHERS;				
			}
			res = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
			
			////////////////////////Ԥ�� ׼����¼�����¼�//////////////////////////
			
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Router_Svc_Stop = RT_TRUE;
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Router_Svc_Stop_Ack = RT_TRUE;
			
			
//			memcpy(&CtrlCharge_Event.Ctrl_ChgData,&Ctrl_Stop,sizeof(Ctrl_Stop));			
//			CtrlCharge_Event.CtrlType = CTRL_STOP;
//			CtrlCharge_Event.StopSource = CTRL_UNIT;
			//////////////////////////////////////////////////////////////////////
			break;
		}
		//�յ�������������
		case Cmd_PowerAdj:
		{
			adjpower_flag = TRUE;
			res = CtrlUnit_RecResp(Cmd_PowerAdj,&Ctrl_PowerAdj,0);//ȡֵ	
 	
			if(memcmp(&RouterInfo.AssetNum,&Ctrl_PowerAdj.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//У���ʲ�һ����
			{
				ChargePilePara_Set.PWM_Duty = Ctrl_PowerAdj.SetPower*10/132;//���ʻ���: D(��һλС��)=I/60*1000=P/(60*220)*1000
				ChargepileDataGetSet(Cmd_SetPower,&ChargePilePara_Set);	
			}
			else
			{
				Ctrl_PowerAdj.cSucIdle = ORTHERS;
			}
			res = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);//�ظ�
			
			////////////////////////Ԥ�� ׼����¼�����¼�//////////////////////////
			memcpy(&CtrlCharge_Event.Ctrl_ChgData,&Ctrl_PowerAdj,sizeof(Ctrl_PowerAdj));
			CtrlCharge_Event.CtrlType = CTRL_ADJPOW;
			//////////////////////////////////////////////////////////////////////
			break;
		}
		//�յ�����������·��������״̬
		case Cmd_RouterExeState:
		{
			CtrlUnit_RecResp(Cmd_RouterExeState,&Chg_ExeState,0);//ȡֵ	
			
			ExeState_Update();	
		
			CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,1);//�ظ�	 wyg191105
			
			break;
		}
		//�յ���綩���¼�ȷ��
		case Cmd_ChgRecordAck:
		{
			CtrlUnit_RecResp(Cmd_ChgRecordAck,0,0);//�����־
			rt_timer_stop(ChgRecordAck_Timer);
			
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
					
			Charge_Event_Data_Clear();
			
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRequestReportAck clear!\n",__func__);
			break;
		}
	
		//�յ�STA�������·��������״̬
		case Cmd_STAOnlineState:
		{			 
			res = CtrlUnit_RecResp(Cmd_STAOnlineState,&OnlineState_Event,0);//ȡֵ
			if(res == SUCCESSFUL)
			{
				rt_kprintf("[strategy]:  (%s) OnlineState_Event,Cmd_STAOnlineState,Successful!\n",__func__);
				CtrlUnit_RecResp(Cmd_STAOnlineStateAck,&OnlineState_Event,0);//�ظ�ȷ��
			}
		
			//�洢����״̬�仯�¼�
//			res = SetStorageData(Cmd_OnlineStateWr,&OnlineState_Event,sizeof(ONLINE_STATE));
//			if(res == SUCCESSFUL)
//			{
//				rt_kprintf("[strategy]:  (%s) Storage BLE_ChgExe_Event, Successful!\n",__func__);
//			}
//			else
//			{
//				rt_kprintf("[strategy]:  (%s) ��������״̬��ʧ�ܣ�\n",__func__);
//				SetStorageData(Cmd_OnlineStateWr,&OnlineState_Event,sizeof(ONLINE_STATE));//�ٴ�һ��
//			}
					
//			res = BLE_CtrlUnit_RecResp(Cmd_STAOnlineStateAPP,&OnlineState_Event,0);//����������
//			if(res == SUCCESSFUL)
//				rt_kprintf("[strategy]:  (%s) OnlineState_Event Apply to BLE, Successful!\n",__func__);				
//			else
//				rt_kprintf("[strategy]:  (%s) �ظ�BLE �������״̬�¼���ʧ�ܣ�\n",__func__);		
			break;
		}			
		default:
			break;
	}
	EventCmd = 0;//��λ
}

/********************************************************************  
*	�� �� ��: BLE_Data_RecProcess()
*	����˵��: ��������������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void BLE_Data_RecProcess(void)
{
	rt_int8_t i,res,cmd;
	rt_uint32_t EventCmd=Cmd_Null;
	
	/***************************** ���տ��������� *******************************/
	EventCmd = Strategy_get_BLE_event();
	cmd = get_event_cmd(EventCmd);
	
	if(cmd > Cmd_STAOnlineStateAck)
		return;

	switch(cmd)
	{	
		//�������    Ȼ��ֱ��������磬��������������ִ���¼�
		case Cmd_ChgRequest:
		{	
			memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));///////////////////////////////////////##############
			
			res = BLE_CtrlUnit_RecResp(Cmd_ChgRequest,&Chg_Apply,0);//ȡֵ
			
			Charge_Apply_RSP(&Chg_Apply,&Chg_Apply_Rsp);

			if(Chg_Apply_Rsp.cSucIdle == SUCCESSFUL)//���������Ӧ�ɹ� ���ɳ�������¼���¼ ����ֱ������
			{
				res = Charge_Apply_Event_Create(&Chg_Apply,&Chg_Apply_Event);//���ɳ�������¼���¼
				if(res > 0)
				{					
					res = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//֪ͨ������
					res = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//ͬʱ���¼��ش�APP
					
					CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Apply_Event = RT_TRUE;
				}
			}
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Apply = RT_TRUE;
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Apply_Ack = RT_TRUE;
			break;
		}
		
		case Cmd_ChgPlanIssue://�յ����ƻ�
		{
			if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Apply == RT_FALSE)
			{
				memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
			}
			
			if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan == RT_TRUE)
			{
				BLE_CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,-1);//���ƻ�ȡֵ
				Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//���ƻ���Ӧ
				break;
			}
			
			memset(&Chg_Strategy,0,sizeof(Chg_Strategy));
			BLE_CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//���ƻ�ȡֵ
			
			Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//���ƻ���Ӧ
			
			if(Chg_StrategyRsp.cSucIdle == SUCCESSFUL)//��ȷ  ���ɳ��ƻ��¼�
			{
				if(Charge_Plan_Event_Create(&Chg_Strategy,&Plan_Offer_Event) > 0 )
				{
					BLE_CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
//					if (ChgPlanOfferAck_Timer != RT_NULL)
//					{
//						rt_timer_start(ChgPlanOfferAck_Timer);
//						ChgPlanOfferAck_count = 0;
//						rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer start!\n",__func__);	
//					}					
				}
				
				CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Event = RT_TRUE; //���ƻ��ϱ�
			}
					
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan = RT_TRUE;//�ѽ��յ����ƻ�

			break;
		}

		//�յ���������·��������״̬
		case Cmd_RouterExeState:
		{			
			res = BLE_CtrlUnit_RecResp(Cmd_RouterExeState,0,0);//ȡֵ
			
			ExeState_Update();				
			
			res = BLE_CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,0);//�ظ�

			if(res == SUCCESSFUL)
			{
				rt_kprintf("[strategy]: (%s) Chg_ExeState Response to BLE, Successful!\n",__func__);
			}
			else
			{
				rt_kprintf("[strategy]: (%s) Chg_ExeState Response to BLE, fail!\n",__func__);
			}
			break;
		}
		//�յ�����ͣ������
		case Cmd_StopChg:
		{	
			res = BLE_CtrlUnit_RecResp(Cmd_StopChg,&Ctrl_Stop,0);//ȡֵ	

			ChargePile_Stop_Charge();
			
//			Chg_ExeState.exeState = EXE_END;
			
			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Router_Svc_Stop = RT_TRUE;
			
//			res = ChargepileDataGetSet(Cmd_ChargeStop,0);

//			if(res == SUCCESSFUL)
//			{
////				PileIfo.WorkState = ChgSt_Finished;	
//				Chg_ExeState.exeState = EXE_END;
//				rt_kprintf("[strategy]: BLE Stop Charge sucess\n");
//			}

//			memcpy(&CtrlCharge_Event.Ctrl_ChgData,&Ctrl_Stop,sizeof(Ctrl_Stop));
//			CtrlCharge_Event.CtrlType = CTRL_STOP;
//			CtrlCharge_Event.StopSource = BLE_UNIT;
		}
		default:
			break;
	}
}

/********************************************************************  
*	�� �� ��: CtrlData_RecProcess()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void CtrlData_RecProcess(void)
{
	BLE_Data_RecProcess();//BLE �¼�����
	HPLC_Data_RecProcess();//HPLC�¼�����
}


/********************************************************************  
*	�� �� ��: PileData_RecProcess()
*	����˵��: ��׮���ݽ��մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void PileData_RecProcess(void)
{
	rt_int8_t res;
	rt_uint32_t start_result,stop_result,adjpow_result;
	
	start_result = 0;
	stop_result = 0;
	adjpow_result = 0;
	
	if(startchg_flag == TRUE)
	{
		if(rt_event_recv(&ChargePileEvent, ChargeStartOK_EVENT|ChargeStartER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &start_result) == RT_EOK)	
		{
			startchg_flag = FALSE;//��λ
			
			rt_timer_stop(StartChgResp_Timer);
			
			if(start_result|ChargeStartOK_EVENT)
			{
				Ctrl_Start.cSucIdle = SUCCESSFUL;
				
				Charge_Record_Create(&Chg_Apply_Event,&Plan_Offer_Event,&ChgOrder_Event);//���ɳ�綩����¼
				
				Router_WorkState.Router_State = ChgState_InCharging; //��״̬ �����
			}
			else if(start_result|ChargeStartER_EVENT)
			{
				ExeState_Update();//·����״̬����
				Ctrl_Start.cSucIdle = FAILED;
				Chg_ExeState.exeState = EXE_FAILED;
				
				ChargepileDataGetSet(Cmd_ChargeStartResp,&ChargePilePara_Get);//��ȡʧ��ԭ��
			}
			
			CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);
			
			res = Charge_Plan_Exe_Event_Create(&Chg_ExeState,&Ctrl_ChgExe_Event);//���ɳ��ִ���¼���¼
			
			if(res > 0)
			{
				CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//�ظ�	wyg191105
				if (ChgPlanExeStateAck_Timer != RT_NULL)
				{
					rt_timer_start(ChgPlanExeStateAck_Timer);
					ChgPlanExeStateAck_count = 0;
				}
				BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);	
			}

			rt_kprintf("[strategy]: (%s) start chargepile event, sucidle = %d  .......!\n",__func__,Ctrl_Start.cSucIdle);
		}
	}
	
	if(stopchg_flag == TRUE)
	{
		//ͣ���ɹ�
		if(rt_event_recv(&ChargePileEvent, ChargeStopOK_EVENT|ChargeStopER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &stop_result) == RT_EOK)		
		{
			stopchg_flag = FALSE;//��λ
			rt_timer_stop(StopChgResp_Timer);
			
			if(stop_result|ChargeStopOK_EVENT)
			{
				Ctrl_Stop.cSucIdle = SUCCESSFUL;
				
				Charge_Record_Update(&ChgOrder_Event);//���³���¼
				
				Router_WorkState.Router_State = ChgState_Finished;//״̬���
			}
			else if(stop_result|ChargeStopER_EVENT)
			{
				Ctrl_Stop.cSucIdle = FAILED;
				ChargepileDataGetSet(Cmd_ChargeStopResp,&ChargePilePara_Get);//��ȡʧ��ԭ��			
			}
			
			CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);
			
			rt_kprintf("[strategy]: (%s) stop chargepile event, sucidle = %d  .......!\n",__func__,Ctrl_Stop.cSucIdle);
		}
	}
	
	if(adjpower_flag == TRUE)
	{
		//�������ʳɹ�
		if(rt_event_recv(&ChargePileEvent, SetPowerOK_EVENT|SetPowerER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &adjpow_result) == RT_EOK)	
		{
			adjpower_flag = FALSE;//��λ
			rt_timer_stop(PowerAdjResp_Timer);
			
			if(adjpow_result|SetPowerOK_EVENT)
			{
				Ctrl_PowerAdj.cSucIdle = SUCCESSFUL;
			}
			else if(adjpow_result|SetPowerER_EVENT)
			{
				Ctrl_PowerAdj.cSucIdle = FAILED;
				ChargepileDataGetSet(Cmd_SetPowerResp,&ChargePilePara_Get);//��ȡʧ��ԭ��
			}
		}
		rt_kprintf("[strategy]: (%s) Adjust chargepile power, sucidle = %d  .......!\n",__func__,Ctrl_PowerAdj.cSucIdle);
	}
}



/********************************************************************  
*	�� �� ��: TimeSolt_PilePowerCtrl()
*	����˵��: ��ʱ�ν��е�׮���ʿ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void TimeSolt_PilePowerCtrl(void)
{
	rt_uint8_t i;
	rt_int8_t res=0;
	STR_SYSTEM_TIME sys_time;
	time_t NowTime_s;
	time_t PlanStartTime_start[50];
	time_t PlanStartTime_stop[50];
	static rt_uint8_t PlanSlotCount,start_flag,stop_flag,start_cmd_send,stop_cmd_send;
	
	
	if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Event_Ack == RT_FALSE)//�޳��ƻ�Ӧ��
	{
		PlanSlotCount = 0;
		start_flag = 0;
		stop_flag = 0;
		start_cmd_send = 0;
		stop_cmd_send=0;
		return;
	}

	memcpy(&sys_time,&System_Time_STR,sizeof(STR_SYSTEM_TIME)); //�����������ڴ� �����ͻ
	NowTime_s = TimeCalculation(&sys_time);//��ȡ��ǰʱ������
	//��λ��ǰ�����ƻ�����ʼʱ���
	for(i=PlanSlotCount;i<Chg_Strategy.ucTimeSlotNum;i++)
	{
		if((start_flag)||(stop_flag))
			break;
		if(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Day> 0)//У��ʱ��ĺϷ���
		{
			PlanStartTime_start[i] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[i].strDecStartTime);
			PlanStartTime_stop[i] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[i].strDecStopTime);
			
			if(NowTime_s>= PlanStartTime_stop[i])//���ƻ���ʼ��ֹͣʱ��һ�� ֹͣ���
			{
				if(stop_cmd_send)
					break;
//				if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Adj == RT_TRUE)
//				{
//					if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Adj_Event_Ack == RT_TRUE)
//					{
//						stop_flag = 1;
//					}					
//				}
//				else
//				{
					stop_flag = 1;
//				}
				rt_kprintf("[strategy]:  (%s)  stop_flag = %d ��\n",__func__,stop_flag);
				rt_kprintf("[strategy]:  (%s) NowTime_s time = %d��\n",__func__,NowTime_s);
				rt_kprintf("[strategy]:  (%s) PlanStartTime_start time = %d��\n",__func__,PlanStartTime_start[i]);
				rt_kprintf("[strategy]:  (%s) PlanStartTime_stop time = %d��\n",__func__,PlanStartTime_stop[i]);
				rt_kprintf("[strategy]:  (%s) Systerm time is %02X-%02X-%02X-%02X-%02X-%02X!\r\n",__func__,System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day\
								,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
				break;
			}
			else if((NowTime_s >= PlanStartTime_start[i])&&(NowTime_s < PlanStartTime_stop[i]))//��⵽ִ�е���ǰʱ���	
			{
				if((start_cmd_send)||(Router_WorkState.Router_State > ChgState_Standby))
					break;
				
				Chg_ExeState.ucPlanPower = Chg_Strategy.strChargeTimeSolts[i].ulChargePow;		
				ChargePilePara_Set.PWM_Duty = Chg_Strategy.strChargeTimeSolts[i].ulChargePow*10/132;//���ʻ���
				PlanSlotCount = i;
				
				if(CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Plan_Event_Ack == RT_TRUE)
					start_flag = 1;	

				rt_kprintf("[strategy]:  (%s) start_flag = %d ��\n",__func__,start_flag);
				rt_kprintf("[strategy]:  (%s) NowTime_s time = %d��\n",__func__,NowTime_s);
				rt_kprintf("[strategy]:  (%s) PlanStartTime_start time = %d��\n",__func__,PlanStartTime_start[i]);
				rt_kprintf("[strategy]:  (%s) PlanStartTime_stop time = %d��\n",__func__,PlanStartTime_stop[i]);
				rt_kprintf("[strategy]:  (%s) Systerm time is %02X-%02X-%02X-%02X-%02X-%02X!\r\n",__func__,System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day\
				,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
				break;
			}
		}
		else
		{
			rt_kprintf("[strategy]:  (%s) Charge Strategy  time error��\n",__func__);		
		}
	}
	
	if(start_flag)//�������
	{
		start_flag = 0;
		start_cmd_send = 1;
		ChargePile_Start_Charge(&ChargePilePara_Set);
	}
	
	if(stop_flag)//ֹͣ���
	{
		stop_flag = 0;
		stop_cmd_send =1;
		ChargePile_Stop_Charge();
	}
}
/********************************************************************  
*	�� �� ��: ChgOrder_Apply()
*	����˵��: �γɳ�綩���ϱ�
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void ChgOrder_Apply(void)
{
	rt_int8_t c_rst=0,b_rst=0;
	rt_uint8_t i;
	time_t start_time = 0;
	time_t stop_time = 0;
	
//	if(Router_WorkState.Router_State == ChgState_Finished)//��⵽������
//	{
//		
		Charge_Record_Update(&ChgOrder_Event);//���³���¼

//		CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//�ϱ���綩��

		if (ChgRecordAck_Timer != RT_NULL)
		{
			rt_timer_start(ChgRecordAck_Timer);
			ChgRecordAck_count = 0;	
			rt_kprintf("[strategy]:  (%s) ChgOrder_Event Apply to Contrllor, Successful!\n",__func__);
		}
		BLE_CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//ͬʱ���¼��ش�APP
		Router_WorkState.Router_State = ChgState_Standby;//״̬���
//	}
}
/********************************************************************  
*	�� �� ��: RtState_Judge()
*	����˵��: ·�����ͳ��׮״̬�ж�
*	��    ��: ��
*	�� �� ֵ: ���ϴ���
********************************************************************/ 
static void RtState_Judge(void)
{
//	if(PileIfo.WorkState != ChgSt_Finished)
//	{
//		if(ChargePilePara_Get.ChgPileState == PILE_STANDBY)
//			PileIfo.WorkState = ChgSt_Standby;
//		else if(ChargePilePara_Get.ChgPileState == PILE_WORKING)
//			PileIfo.WorkState = ChgSt_InCharging;
//		else if(ChargePilePara_Get.ChgPileState == PILE_FAU)
//			PileIfo.WorkState = ChgSt_Fault;
//	}
	
	if((Router_WorkState.Router_Fault.Total_Fau != RT_FALSE)||(Router_WorkState.Pile_State == PILE_FAU))
		Router_WorkState.Router_State = ChgState_Fault;
	
	switch(Router_WorkState.Router_State)
	{
		case ChgState_Standby:
			break;
		case ChgState_InCharging:           //�����
			Charge_Record_Update(&ChgOrder_Event);//���³���¼
			break;
		case ChgState_DisCharging:          //�ŵ���
			break;
		case ChgState_Finished:				//��ŵ���ɣ�3��ش�����
			ChgOrder_Apply();
			break;
		case ChgState_Fault:            	//����

			break;
		case ChgState_Update:
			break;
		default:
			break;
	}
			
		
//		case ChgSt_Fault:
//		{
//			Router_Fault.Bit.ChgPile_Fau = TRUE;
//			break;
//		}
//		case ChgSt_InCharging:
//		{
//			Charge_Record_Update(&ChgOrder_Event);//���³���¼
//			break;
//		}
//		case ChgSt_DisCharging:
//		{
//			RouterIfo.WorkState = RtSt_CtrlPower;
//			break;
//		}
//		case ChgSt_Finished:
//		{
//			RouterIfo.WorkState = RtSt_StandbyOK;//�������ش���
//			break;
//		}
//		default:
//			break;
//	}

//	
//	if(Router_Fault.Total_Fau != 0)
//		RouterInfo.WorkState = RtSt_Fault;	
}

static void strategy_thread_entry(void *parameter)
{
	rt_err_t res,p_rst;

	/*��ʼ������*/
	
			/* ���������� */
//	strategy_mutex = rt_mutex_create("strategy_mutex", RT_IPC_FLAG_FIFO);
//	if (strategy_mutex == RT_NULL)
//	{
//		rt_kprintf("[strategy]:  (%s) rt_mutex_create make fail\n");
//	}

	Router_WorkState.Router_State = ChgState_Standby;
	Router_WorkState.Router_Fault.Total_Fau = RT_FALSE;

	Chg_ExeState.exeState = EXE_NULL;
	Charge_Event_Data_Clear();
	CtrlCharge_Event.CtrlType = CTRL_NULL;

	rt_thread_mdelay(3000);
	
	SetStorageData(Cmd_ChgRequestWr,&Chg_Apply_Event,sizeof(CHARGE_APPLY_EVENT));
	
//	GetStorageData(Cmd_MeterNumRd,&RouterIfo,sizeof(RouterIfo));
//	rt_thread_mdelay(100);
	
	rt_pin_mode(RELAYA_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RELAYB_PIN, PIN_MODE_OUTPUT);
	RELAY_ON();//�ϵ����ϼ̵���
	while (1)
	{
		ExeState_Update();//·����״̬����
		RtState_Judge();
//		ChgOrder_Apply();
		CtrlData_RecProcess();
		PileData_RecProcess();
			
		TimeSolt_PilePowerCtrl();
						
		rt_thread_mdelay(500);
	}
}

int strategy_thread_init(void)
{
	rt_err_t res;
	
	/* ��ʼ����ʱ�� */
    timer_create_init();
	
	res=rt_thread_init(&strategy,
									"strategy",
									strategy_thread_entry,
									RT_NULL,
									strategy_stack,
									THREAD_STRATEGY_STACK_SIZE,
									THREAD_STRATEGY_PRIORITY,
									THREAD_STRATEGY_TIMESLICE);
	if (res == RT_EOK) 
	{	
		rt_kprintf("[strategy]: Strategy Initialized!\n");
		rt_thread_startup(&strategy);
	}
	return res;
}


#if defined (RT_STRATEGY_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
	INIT_APP_EXPORT(strategy_thread_init);
#endif
MSH_CMD_EXPORT(strategy_thread_init, strategy thread run);

void REALY_ON(int argc, char**argv)
{
	RELAY_ON();
}
MSH_CMD_EXPORT(REALY_ON, AC out CMD);

void REALY_OFF(int argc, char**argv)
{
	RELAY_OFF();
}
MSH_CMD_EXPORT(REALY_OFF, AC out  CMD);

