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

//const char* _hplc_event_char[]={//698 �¼�����   ��ӡ��־��
//	"null",
//	"Charge_Request",							//�����������
//	"Charge_Request_Ack",						//�����������Ӧ��
//	
//	"Charge_PlanIssue", 					//���ƻ��·�
//	"Charge_PlanIssue_Ack",                 	//���ƻ��·�Ӧ��
//	"Charge_Plan_Offer", 						//���ƻ��¼��ϱ�
//	"Charge_Plan_Offer_Ack",                 	//���ƻ��ϱ��¼�Ӧ��
//	
//	"Charge_Plan_Adjust",                 		//���ƻ�����
//	"Charge_Plan_Adjust_Ack",                 	//���ƻ�����Ӧ��


//	"Charge_Request_Report",					//��������¼�����
//	"Charge_Request_Report_Ack",				//��������¼�����Ӧ��
////	"Charge_Request_Report_APP",				//��������¼���֪APP
////	"Charge_Request_Confirm",					//�������ȷ�ϣ�֪ͨ������
//	
//	"Charge_Plan_Exe_State",                    //���ƻ�ִ��״̬�¼��ϱ�
//	"Charge_Plan_Exe_State_Ack",                 //���ƻ�ִ��״̬�¼��ϱ�Ӧ��
//	
//	"Start_Charge",							//�����������·�
//	"Start_Charge_Ack",						//�������Ӧ��
//	"Stop_Charge",							//ֹͣ�������·�
//	"Stop_Charge_Ack",							//ֹͣ���Ӧ��
//	"Power_Adj",							//���ʵ��ڲ����·�
//	"Power_Adj_Ack",						//���ʵ���Ӧ��

//	"Charge_Record",							//���ͳ�綩��
//	"Charge_RecordAck",						//���ͳ�綩���¼�ȷ��
//	"Device_Fault",                      	//����·�����쳣״̬
//	"Device_FaultAck",                      	//����·�����쳣״̬
//	"Pile_Fault",                 			//���ͳ��׮�쳣״̬
//	"Pile_FaultAck",                 			//���ͳ��׮�쳣״̬
//	"Charge_Plan_Issue_Get_Ack",
//	
//	"Read_Router_State",                    	//·����ִ��״̬��ѯ
//	"Read_Router_State_Ack",                 	//·����ִ��״̬Ӧ��
//	
//	"STAOnlineState",						//STA�������·��������״̬��
//	"STAOnlineStateAck",					//STA�������·��������״̬ȷ�ϡ�
//};//ҵ�������������


CCMRAM ChargPilePara_TypeDef ChargePilePara_Set;
CCMRAM ChargPilePara_TypeDef ChargePilePara_Get;

CCMRAM static CHARGE_EXE_STATE_ASK ExeState_CtrlAsk;//������
//CCMRAM static CHARGE_EXE_STATE_ASK ExeState_BleAsk;//����

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

CCMRAM static ROUTER_FAULT_EVENT Router_Pile_Fault_Event;

//��ʱ����
CCMRAM static rt_timer_t StartChgResp_Timer;
CCMRAM static rt_timer_t StopChgResp_Timer;
CCMRAM static rt_timer_t PowerAdjResp_Timer;

//��ʱ��ѵ
CCMRAM static rt_timer_t ChgPileStateGet_Timer;
//�ȴ��ϱ��¼�ȷ��
CCMRAM static rt_timer_t HPLC_ChgReqReportAck_Timer;
CCMRAM static rt_timer_t HPLC_ChgPlanOfferAck_Timer;
CCMRAM static rt_timer_t HPLC_ChgPlanExeStateAck_Timer;
CCMRAM static rt_timer_t HPLC_ChgRecordAck_Timer;

//�ȴ��ϱ��¼�ȷ��
CCMRAM static rt_timer_t BLE_ChgReqReportAck_Timer;
CCMRAM static rt_timer_t BLE_ChgPlanOfferAck_Timer;
CCMRAM static rt_timer_t BLE_ChgPlanExeStateAck_Timer;
CCMRAM static rt_timer_t BLE_ChgRecordAck_Timer;


CCMRAM static CTL_CHARGE Ctrl_Start;//�������ز���
CCMRAM static CTL_CHARGE Ctrl_Stop;
CCMRAM static CTL_CHARGE BLE_Stop;
CCMRAM static CTL_CHARGE Ctrl_PowerAdj;

//���ִ��״̬
CCMRAM static CHARGE_EXE_STATE Chg_ExeState;

CCMRAM static CHARGE_EXE_EVENT Ctrl_ChgExe_Event;
CCMRAM static CHG_ORDER_EVENT ChgOrder_Event;//��������һ������

CCMRAM static unsigned char PlanSlotCount;
CCMRAM static unsigned char PlanExeNum;
CCMRAM static unsigned char PlanExeNum_Old;
CCMRAM static rt_uint32_t PlanSolts_ChargeTime[50];//ʱ�γ��ʱ��
CCMRAM static rt_uint32_t PlanSolts_ChargeEle[50][5];//ʱ�γ�����

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
////static time_t TimeCalculation(STR_SYSTEM_TIME* TimeStamp)
//static rt_uint32_t TimeCalculation(STR_SYSTEM_TIME* TimeStamp)
//{
//	rt_uint8_t buf[6];
//	rt_uint32_t time;
//	
//	buf[0] = TimeStamp->Second;
//	buf[1] = TimeStamp->Minute;
//	buf[2] = TimeStamp->Hour;
//	buf[3] = TimeStamp->Day;
//	buf[4] = TimeStamp->Month;
//	buf[5] = TimeStamp->Year;
//	
//	time = timebin2long(buf);
//	
//	return time;
//	
//	
//	
//	
///*	time_t time_s = 0;
//	rt_uint8_t year,month,day,hour,minute,second;
//	struct tm timep;
//	
////	rt_mutex_take(strategy_mutex, RT_WAITING_FOREVER);
//	
//	BCD_toInt(&second,&TimeStamp->Second,1);
//	BCD_toInt(&minute,&TimeStamp->Minute,1);
//	BCD_toInt(&hour,&TimeStamp->Hour,1);
//	BCD_toInt(&day,&TimeStamp->Day,1);	
////	BCD_toInt((unsigned char*)&timep.tm_wday,&TimeStamp.Week,1);
//	BCD_toInt(&month,&TimeStamp->Month,1);
//	BCD_toInt(&year,&TimeStamp->Year,1);
//	
//	year += 100;
//	month -=1;
//	
//	timep.tm_sec = second;
//	timep.tm_min = minute;
//	timep.tm_hour = hour;
//	timep.tm_mday = day;
//	timep.tm_mon = month;
//	timep.tm_year = year;
//	

//	time_s = mktime(&timep);
//	
////	rt_mutex_release(strategy_mutex);
//	
//	return time_s;*/
//}



static time_t TimeCalculation(STR_SYSTEM_TIME* TimeStamp)
{
	time_t time_s = 0;
	rt_uint8_t year,month,day,hour,minute,second;
	static STR_SYSTEM_TIME time;
	struct tm timep;
	
//	rt_mutex_take(strategy_mutex, RT_WAITING_FOREVER);
	memcpy(&time,TimeStamp,sizeof(STR_SYSTEM_TIME));

	BCD_toInt(&second,&time.Second,1);
//	rt_kprintf("[strategy]: (%s) Second = %d!\n",__func__,second);
//	rt_kprintf("[strategy]: (%s) Second = %02X!\n",__func__,time.Second);
	
	BCD_toInt(&minute,&time.Minute,1);
//	rt_kprintf("[strategy]: (%s) Minute = %d!\n",__func__,minute);
//	rt_kprintf("[strategy]: (%s) Minute = %02X!\n",__func__,time.Minute);
	BCD_toInt(&hour,&time.Hour,1);
//	rt_kprintf("[strategy]: (%s) Hour = %d!\n",__func__,hour);
//	rt_kprintf("[strategy]: (%s) Hour = %02X!\n",__func__,time.Hour);
	BCD_toInt(&day,&time.Day,1);	
//	BCD_toInt((unsigned char*)&timep.tm_wday,&TimeStamp.Week,1);
	BCD_toInt(&month,&time.Month,1);
	BCD_toInt(&year,&time.Year,1);
	
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
 * ��    ��: ��ѵ���׮״̬����
 **************************************************************/
static void ChgPileStateGet_Timeout(void *parameter)
{
    rt_lprintf("[strategy]: ChgPileStateGet event is timeout!\n");
	//��ѯ���׮״̬
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
}
/**************************************************************
 * ��������: ChgReqReportResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ���ͳ�����볬ʱ����
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgReqReportAck_count;
static void HPLC_ChgReqReportAck_Timeout(void *parameter)
{
	rt_int8_t res;

	res = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//֪ͨ������
	
	//�ȴ�ȷ��
	if(res == SUCCESSFUL)
	{	
		HPLC_ChgReqReportAck_count++;
	}
		
	if(HPLC_ChgReqReportAck_count>=3)
		rt_timer_stop(HPLC_ChgReqReportAck_Timer);
	
	rt_kprintf("[strategy]: HPLC_ChgReqReport event is timeout! count = %d\n",HPLC_ChgReqReportAck_count);
}
/**************************************************************
 * ��������: ChgPlanOfferAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ����ƻ�ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgPlanOfferAck_count;
static void HPLC_ChgPlanOfferAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;
	
	c_rst = CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		HPLC_ChgPlanOfferAck_count++;
	}
		
	if(HPLC_ChgPlanOfferAck_count>=3)
		rt_timer_stop(HPLC_ChgPlanOfferAck_Timer);
	
	rt_kprintf("[strategy]: HPLC_ChgPlanOffer event is timeout! count = %d\n",HPLC_ChgPlanOfferAck_count);
}

/**************************************************************
 * ��������: ChgPlanExeStateAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ����ƻ�ִ��״̬ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgPlanExeStateAck_count;
static void HPLC_ChgPlanExeStateAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//�ϱ����ƻ�ִ���¼�
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		HPLC_ChgPlanExeStateAck_count++;
	}
		
	if(HPLC_ChgPlanExeStateAck_count>=3)
		rt_timer_stop(HPLC_ChgPlanExeStateAck_Timer);
	
	rt_kprintf("[strategy]: HPLC_ChgPlanExeState event is timeout! count = %d\n",HPLC_ChgPlanExeStateAck_count);
}
/**************************************************************
 * ��������: ChgRecordAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ���綩��ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgRecordAck_count;
static void HPLC_ChgRecordAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//�ϱ���綩��
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		HPLC_ChgRecordAck_count++;
	}
		
	if(HPLC_ChgRecordAck_count>=3)
		rt_timer_stop(HPLC_ChgRecordAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgRecord event is timeout! count = %d\n",HPLC_ChgRecordAck_count);
}

/**************************************************************
 * ��������: ChgReqReportResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ���ͳ�����볬ʱ����
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgReqReportAck_count;
static void BLE_ChgReqReportAck_Timeout(void *parameter)
{
	rt_int8_t res;

	res = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//ͬʱ���¼��ش�APP
	
	//�ȴ�ȷ��
	if(res == SUCCESSFUL)
	{	
		BLE_ChgReqReportAck_count++;
	}
		
	if(BLE_ChgReqReportAck_count>=3)
		rt_timer_stop(BLE_ChgReqReportAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgReqReport event is timeout! count = %d\n",BLE_ChgReqReportAck_count);
}

/**************************************************************
 * ��������: ChgPlanOfferAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ����ƻ�ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgPlanOfferAck_count;
static void BLE_ChgPlanOfferAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = BLE_CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		BLE_ChgPlanOfferAck_count++;
	}
		
	if(BLE_ChgPlanOfferAck_count>=3)
		rt_timer_stop(BLE_ChgPlanOfferAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgPlanOffer event is timeout! count = %d\n",BLE_ChgPlanOfferAck_count);
}

/**************************************************************
 * ��������: ChgPlanExeStateAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ����ƻ�ִ��״̬ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgPlanExeStateAck_count;
static void BLE_ChgPlanExeStateAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//ͬʱ���¼��ش�APP
	
	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		BLE_ChgPlanExeStateAck_count++;
	}
		
	if(BLE_ChgPlanExeStateAck_count>=3)
		rt_timer_stop(BLE_ChgPlanExeStateAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgPlanExeState event is timeout! count = %d\n",BLE_ChgPlanExeStateAck_count);
}
/**************************************************************
 * ��������: ChgRecordAck_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: �ȴ���綩��ȷ�Ϻ���
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgRecordAck_count;
static void BLE_ChgRecordAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;
	
	c_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//ͬʱ���¼��ش�APP

	//�ȴ�ȷ��
	if(c_rst == SUCCESSFUL)
	{	
		BLE_ChgRecordAck_count++;
	}
		
	if(BLE_ChgRecordAck_count>=3)
		rt_timer_stop(BLE_ChgRecordAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgRecord event is timeout! count = %d\n",BLE_ChgRecordAck_count);
}

/**************************************************************
 * ��������: timer_create_init 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ��ʱ��
 **************************************************************/
static void timer_create_init(void)
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
							
	/* ���� ���׮״̬��ѯ ��ʱ�� */
		ChgPileStateGet_Timer = rt_timer_create("ChgPileStateGet",  /* ��ʱ�������� ChgPileStateGet */
									ChgPileStateGet_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									1000, /* ��ʱ���ȣ���OS TickΪ��λ����1000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
									
									
		/* ���� ����������ͻظ� ��ʱ�� */
		HPLC_ChgReqReportAck_Timer = rt_timer_create("HPLC_ChgReqReportAck",  /* ��ʱ�������� ChgReqReportRsp */
									HPLC_ChgReqReportAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* һ���Զ�ʱ�� */
									
									
	/* ���� ���ƻ�ȷ�� ��ʱ�� */
		HPLC_ChgPlanOfferAck_Timer = rt_timer_create("HPLC_ChgPlanOfferAck",  /* ��ʱ�������� ChgPlanOfferAck */
									HPLC_ChgPlanOfferAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
	/* ���� ���ƻ�ִ��״̬ȷ�� ��ʱ�� */
		HPLC_ChgPlanExeStateAck_Timer = rt_timer_create("HPLC_ChgPlanExeStateAck",  /* ��ʱ�������� ChgPlanExeStateAck */
									HPLC_ChgPlanExeStateAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
									
	/* ���� ��綩��ȷ�� ��ʱ�� */
		HPLC_ChgRecordAck_Timer = rt_timer_create("HPLC_ChgRecordAck",  /* ��ʱ�������� ChgRecordAck */
									HPLC_ChgRecordAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									20000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
									
									
		/* ���� ����������ͻظ� ��ʱ�� */
		BLE_ChgReqReportAck_Timer = rt_timer_create("BLE_ChgReqReportAck",  /* ��ʱ�������� ChgReqReportRsp */
									BLE_ChgReqReportAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* һ���Զ�ʱ�� */
									
									
	/* ���� ���ƻ�ȷ�� ��ʱ�� */
		BLE_ChgPlanOfferAck_Timer = rt_timer_create("BLE_ChgPlanOfferAck",  /* ��ʱ�������� ChgPlanOfferAck */
									BLE_ChgPlanOfferAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
	/* ���� ���ƻ�ִ��״̬ȷ�� ��ʱ�� */
		BLE_ChgPlanExeStateAck_Timer = rt_timer_create("BLE_ChgPlanExeStateAck",  /* ��ʱ�������� ChgPlanExeStateAck */
									BLE_ChgPlanExeStateAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									10000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* �����Զ�ʱ�� */
	/* ���� ��綩��ȷ�� ��ʱ�� */
		BLE_ChgRecordAck_Timer = rt_timer_create("BLE_ChgRecordAck",  /* ��ʱ�������� ChgRecordAck */
									BLE_ChgRecordAck_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									20000, /* ��ʱ���ȣ���OS TickΪ��λ����3000��OS Tick */
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

	if((memcmp(&RouterInfo.AssetNum,&chg_apply->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)
		&&(Router_WorkState.Router_Fault.Total_Fau == RT_FALSE))//У���ʲ�һ���� �����޹���
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
	
	if((memcmp(&RouterInfo.AssetNum,&charge_plan->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)
		&&(Router_WorkState.Router_Fault.Total_Fau == RT_FALSE))//У���ʲ�һ���� �����޹���
	{
		charge_plan_rsp->cSucIdle = SUCCESSFUL;
	}
	else
		charge_plan_rsp->cSucIdle = ORTHERS;
	
	rt_kprintf("[strategy]: (%s)  Charge Plan response sucidle = %d \n",__func__,charge_plan_rsp->cSucIdle);	
	
	CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,charge_plan_rsp,0);//���ƻ���Ӧ
	
	
	CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Ack = RT_TRUE;//���ƻ�����Ӧ
		
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
	
	ChargepileDataGetSet(Cmd_SetPower,&PilePara);//�·����׮�趨����
	
	rt_kprintf("[strategy]:  (%s) set charge duty = %d !\n",__func__,PilePara->PWM_Duty);

	res = ChargepileDataGetSet(Cmd_ChargeStart,0);//������������
	
	if(res == SUCCESSFUL)
	{
		if (StartChgResp_Timer != RT_NULL)
			rt_timer_start(StartChgResp_Timer);

//		Chg_ExeState.exeState = EXE_ING;				
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
	
	rt_uint32_t ChgStart_EleTotal[5];
	rt_uint32_t Charge_TimeTotal;
	
	static rt_uint32_t ChgStart_Ele[5];
	static STR_SYSTEM_TIME ChgStart_Time;
	
//	if(Router_WorkState.Router_State == ChgState_PlanSoltsStart)
//

	Charge_TimeTotal = 0;
	memset(ChgStart_EleTotal,0,sizeof(rt_uint32_t)*5);
			
	cmMeter_get_data(EMMETER_HISDATA,&stMeter_HisData_Strategy);
	memcpy(&charge_record->StopMeterValue[0],&stMeter_HisData_Strategy.ulMeter_Total,5*sizeof(long));
	
	if(PlanExeNum_Old^PlanExeNum)
	{
		PlanExeNum_Old = PlanExeNum;
		memset(ChgStart_Ele,0,sizeof(rt_uint32_t)*5);		
		memcpy(&ChgStart_Time,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
		memcpy(ChgStart_Ele,&stMeter_HisData_Strategy.ulMeter_Total,5*sizeof(long));
	}
	
	memcpy(&charge_record->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
	
	memcpy(&charge_record->ChgStopTime,&System_Time_STR,sizeof(STR_SYSTEM_TIME));


	start_time = TimeCalculation(&ChgStart_Time);
	stop_time = TimeCalculation(&charge_record->ChgStopTime);
	PlanSolts_ChargeTime[PlanExeNum] = stop_time - start_time;
	
	for(i = 0; i< PlanSlotCount;i++)
		Charge_TimeTotal += PlanSolts_ChargeTime[i];
	
	charge_record->ucChargeTime = Charge_TimeTotal;
	
	
	for(i=0;i<5;i++)
		PlanSolts_ChargeEle[PlanExeNum][i] = charge_record->StopMeterValue[i] - ChgStart_Ele[i];
	
	for(i=0;i<5;i++)
	{
		ChgStart_EleTotal[i] += PlanSolts_ChargeEle[PlanExeNum][i];
		charge_record->ucChargeEle[i] = ChgStart_EleTotal[i];
	}
}

static void Charge_Event_Data_Clear(void)
{
	memset(&Chg_Apply,0,sizeof(CHARGE_APPLY_EVENT));
	memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
	memset(&Plan_Offer_Event,0,sizeof(PLAN_OFFER_EVENT));
	
	memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
}


/********************************************************************  
*	�� �� ��: Charge_Plan_Event_Create()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

static rt_int8_t Router_Pile_Alarm_Event_Create(ROUTER_FAULT_EVENT* RouterFault,ROUTER_WORK_STATE* Router_state)
{
	rt_int8_t res;
	static rt_uint32_t OrderNum = 0;
	
	memset(RouterFault,0,sizeof(ROUTER_FAULT_EVENT));
	
	rt_kprintf("[strategy]:  (%s) Create Router Fault event record...!\n",__func__);

	OrderNum++;
	RouterFault->OrderNum = OrderNum;
	RouterFault->OccurSource = 0;
	RouterFault->ChannelState = 0;

	memcpy(&RouterFault->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//��¼�ϱ�ʱ��
	memcpy(&RouterFault->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
	
	RouterFault->Router_Fault.Total_Fau = Router_state->Router_Fault.Total_Fau;
	RouterFault->Pile_Fault.Total_Fau = Router_state->Pile_Fault.Total_Fau;
					
//	res = SetStorageData(Cmd_ChgExecuteWr,charge_plan_exe_event,sizeof(CHARGE_EXE_EVENT));

//	if(res == SUCCESSFUL)
//	{
//		rt_kprintf("[strategy]:  (%s) Storage Plan_exe_Event, Successful!\n",__func__);
//		return 1;
//	}
//	else
//	{
//		rt_kprintf("[strategy]:  (%s) Storage Plan_exe_Event, fail!\n",__func__);
//	}
//	return -1;
	return 1;
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
				rt_kprintf("[strategy]: (%s) get cmd: %s\n",__func__,comm_cmdtype_to_string(i));
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
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Apply == RT_FALSE)
			{
				memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
			}
			
			if(CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan == RT_TRUE)
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
					PlanSlotCount = Plan_Offer_Event.Chg_Strategy.ucTimeSlotNum;
					PlanExeNum = 0;
					PlanExeNum_Old = 0xff;					
					
					CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
					if (HPLC_ChgPlanOfferAck_Timer != RT_NULL)
					{
						rt_timer_start(HPLC_ChgPlanOfferAck_Timer);
						HPLC_ChgPlanOfferAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  HPLC_ChgPlanOfferAck_Timer start!\n",__func__);	
					}					
				}				
				CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Event = RT_TRUE; //���ƻ��ϱ�
			}
					
			CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan = RT_TRUE;//�ѽ��յ����ƻ�

			break;
		}
		
		case Cmd_ChgPlanOfferAck:////�յ����ƻ��¼�ȷ��
		{	
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);

			CtrlUnit_RecResp(Cmd_ChgPlanOfferAck,0,0);//�ظ�
			rt_timer_stop(HPLC_ChgPlanOfferAck_Timer);//�յ�ȷ����Ӧ ֹͣ�ط�
			
			Router_WorkState.Router_State = ChgState_PlanSoltsStart;//���յ����ƻ���Ӧ ��ʼ�жϳ��ƻ� ����ִ��״̬
					
			if(CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Adj_Event == RT_TRUE)
				CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Adj_Event_Ack = RT_TRUE;
			else	
				CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Event_Ack = RT_TRUE; //���ƻ��ϱ�Ӧ��
			break;
		}
		case Cmd_ChgPlanExeStateAck://�յ����ƻ�ִ���¼�ȷ��
		{
			CtrlUnit_RecResp(Cmd_ChgPlanExeStateAck,0,0);//ȡֵ	
			rt_timer_stop(HPLC_ChgPlanExeStateAck_Timer);		
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
						Router_WorkState.Router_State = ChgState_PlanExeEnd;//�ƻ�ִ�н���		
						Chg_ExeState.exeState = EXE_END;
					}
					break;
				}
			
			if(Chg_StrategyRsp_Adj.cSucIdle == SUCCESSFUL)//��ȷ  ���³��ƻ�
			{
				memcpy(&Chg_Strategy,&Chg_Strategy_Adj,sizeof(Chg_Strategy));//��ȷ  ���³��ƻ�
				
				if(Charge_Plan_Event_Create(&Chg_Strategy,&Plan_Offer_Event) > 0 )//���ɳ��ƻ��¼�
				{
					PlanSlotCount = Plan_Offer_Event.Chg_Strategy.ucTimeSlotNum;
					PlanExeNum = 0;
					PlanExeNum_Old = 0xff;
					
					CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
					
					if(HPLC_ChgPlanOfferAck_Timer != RT_NULL)
					{
						rt_timer_start(HPLC_ChgPlanOfferAck_Timer);
						HPLC_ChgPlanOfferAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer start!\n",__func__);	
					}
					CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Adj_Event = RT_TRUE;
				}
			}
			
			CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Adj = RT_TRUE;
			CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Adj_Ack = RT_TRUE;
			
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
//			res = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//�ظ�
			
			////////////////////////Ԥ�� ׼����¼�����¼�//////////////////////////
			
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Start = RT_TRUE;
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Start_Ack = RT_TRUE;
			
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
				Router_WorkState.Router_State = ChgState_PlanExeEnd;//�ƻ�ִ�н���		
				Chg_ExeState.exeState = EXE_END;
			}
			else
			{
				Ctrl_Stop.cSucIdle = ORTHERS;				
			}
			res = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
			
			////////////////////////Ԥ�� ׼����¼�����¼�//////////////////////////
			
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Stop = RT_TRUE;
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Stop_Ack = RT_TRUE;
			
			
//			memcpy(&CtrlCharge_Event.Ctrl_ChgData,&Ctrl_Stop,sizeof(Ctrl_Stop));			
//			CtrlCharge_Event.CtrlType = CTRL_STOP;
//			CtrlCharge_Event.StopSource = CTRL_UNIT;
			//////////////////////////////////////////////////////////////////////
			break;
		}
		//�յ�������������
		case Cmd_PowerAdj:
		{
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
			rt_timer_stop(HPLC_ChgRecordAck_Timer);
			
			CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
					
			Charge_Event_Data_Clear();
			
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRecordAck clear!\n",__func__);
			break;
		}
		//·�����쳣�¼�ȷ��
		case Cmd_DeviceFaultAck:
		{
			CtrlUnit_RecResp(Cmd_DeviceFaultAck,0,0);//�����־
//			rt_timer_stop(ChgRecordAck_Timer);
			
//			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
							
			rt_kprintf("[strategy]:  (%s)  Cmd_DeviceFaultAck clear!\n",__func__);
			break;
		}
				//���׮�쳣�¼�ȷ��
		case Cmd_PileFaultAck:
		{
			CtrlUnit_RecResp(Cmd_PileFaultAck,0,0);//�����־
//			rt_timer_stop(ChgRecordAck_Timer);
			
//			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
			
			rt_kprintf("[strategy]:  (%s)  Cmd_PileFaultAck clear!\n",__func__);
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
					CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//֪ͨ������
					
					if (HPLC_ChgReqReportAck_Timer != RT_NULL)
					{
						rt_timer_start(HPLC_ChgReqReportAck_Timer);
						HPLC_ChgReqReportAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  HPLC_ChgReqReportAck_Timer start!\n",__func__);	
					}
					CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Apply_Event = RT_TRUE;
					
					
					BLE_CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//ͬʱ���¼��ش�APP
					
					if (BLE_ChgReqReportAck_Timer != RT_NULL)
					{
						rt_timer_start(BLE_ChgReqReportAck_Timer);
						BLE_ChgReqReportAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  BLE_ChgReqReportAck_Timer start!\n",__func__);	
					}
					CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Apply_Event = RT_TRUE;
				}
			}
			CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Apply = RT_TRUE;
			CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Apply_Ack = RT_TRUE;
			break;
		}
		case Cmd_ChgRequestReportAck://��������¼�Ӧ��
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgRequestReportAck,0,0);//�����־
			rt_timer_stop(BLE_ChgReqReportAck_Timer);
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRequestReportAck clear!\n",__func__);
			break;
		}

		case Cmd_ChgPlanIssue://�յ����ƻ�
		{
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Apply == RT_FALSE)
			{
				memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
			}
			
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan == RT_TRUE)
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
					
					PlanSlotCount = Plan_Offer_Event.Chg_Strategy.ucTimeSlotNum;
					PlanExeNum = 0;
					PlanExeNum_Old = 0xff;
					
					BLE_CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
					if(BLE_ChgPlanOfferAck_Timer != RT_NULL)
					{
						rt_timer_start(BLE_ChgPlanOfferAck_Timer);
						BLE_ChgPlanOfferAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer start!\n",__func__);	
					}					
				}
				
				CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Event = RT_TRUE; //���ƻ��ϱ�
			}
			CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan = RT_TRUE;//�ѽ��յ����ƻ�
			break;
		}
		
		case Cmd_ChgPlanOfferAck:////�յ����ƻ��¼�ȷ��
		{	
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);

			BLE_CtrlUnit_RecResp(Cmd_ChgPlanOfferAck,0,0);//�ظ�
			rt_timer_stop(BLE_ChgPlanOfferAck_Timer);//�յ�ȷ����Ӧ ֹͣ�ط�
			
			Router_WorkState.Router_State = ChgState_PlanSoltsStart;//���յ����ƻ���Ӧ ��ʼ�жϳ��ƻ� ����ִ��״̬
					
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Adj_Event == RT_TRUE)
				CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Adj_Event_Ack = RT_TRUE;
			else	
				CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Event_Ack = RT_TRUE; //���ƻ��ϱ�Ӧ��
			break;
		}
		
		case Cmd_ChgPlanExeStateAck://�յ����ƻ�ִ���¼�ȷ��
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeStateAck,0,0);//ȡֵ	
			rt_timer_stop(BLE_ChgPlanExeStateAck_Timer);		
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);
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
			
			Router_WorkState.Router_State = ChgState_PlanExeEnd;//�ƻ�ִ�н���
			Chg_ExeState.exeState = EXE_END;		
			CtrlCharge_Event.Router_BLE_Info.Bit.Router_Svc_Stop = RT_TRUE;
			break;
		}
				//�յ���綩���¼�ȷ��
		case Cmd_ChgRecordAck:
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgRecordAck,0,0);//�����־
			rt_timer_stop(BLE_ChgRecordAck_Timer);
			
			CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
					
//			Charge_Event_Data_Clear();
			
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRecordAck clear!\n",__func__);
			break;
		}
				//·�����쳣�¼�ȷ��
		case Cmd_DeviceFaultAck:
		{
			BLE_CtrlUnit_RecResp(Cmd_DeviceFaultAck,0,0);//�����־
//			rt_timer_stop(ChgRecordAck_Timer);
			
//			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
							
			rt_kprintf("[strategy]:  (%s)  Cmd_DeviceFaultAck clear!\n",__func__);
			break;
		}
				//���׮�쳣�¼�ȷ��
		case Cmd_PileFaultAck:
		{
			BLE_CtrlUnit_RecResp(Cmd_PileFaultAck,0,0);//�����־
//			rt_timer_stop(ChgRecordAck_Timer);
			
//			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
			
			rt_kprintf("[strategy]:  (%s)  Cmd_PileFaultAck clear!\n",__func__);
			break;
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
*	�� �� ��: Start_ChargePile_Process()
*	����˵��: �������׮������
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Start_ChargePile_Process(void)
{
	rt_int8_t res;
	rt_uint32_t start_result;
	
	start_result = 0;
	
	rt_kprintf("[strategy]: (%s) waiting for start charge event!\n",__func__);

	if(rt_event_recv(&ChargePileEvent, ChargeStartOK_EVENT|ChargeStartER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &start_result) == RT_EOK)	
	{		
		rt_timer_stop(StartChgResp_Timer);
		
		if(start_result|ChargeStartOK_EVENT)//�������ɹ�
		{
			if(PlanExeNum	== 0)	//���ƻ���ʼ������綩��
				Charge_Record_Create(&Chg_Apply_Event,&Plan_Offer_Event,&ChgOrder_Event);//���ɳ�綩����¼
			
			Router_WorkState.Router_State = ChgState_InCharging; //��״̬ �����
			
			Ctrl_Start.cSucIdle = SUCCESSFUL;//�����ɹ�
			Chg_ExeState.exeState = EXE_ING;//�ƻ�ִ����
		}
		else if(start_result|ChargeStartER_EVENT)//�������ʧ��
		{
			ExeState_Update();//·����״̬����
			Ctrl_Start.cSucIdle = FAILED;//����ʧ��
			Chg_ExeState.exeState = EXE_FAILED;//ִ��ʧ��
			
			ChargepileDataGetSet(Cmd_ChargeStartResp,&ChargePilePara_Get);//��ȡʧ��ԭ��
		}
		
		if(CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Start == RT_TRUE)//�������վ���� �����������
			CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//�ظ��������
		
		res = Charge_Plan_Exe_Event_Create(&Chg_ExeState,&Ctrl_ChgExe_Event);//���ɳ��ִ���¼���¼
		
		if(res > 0)
		{
//			CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//��������ϱ����ִ���¼���¼
			
			if (HPLC_ChgPlanExeStateAck_Timer != RT_NULL)
			{
				rt_timer_start(HPLC_ChgPlanExeStateAck_Timer);
				HPLC_ChgPlanExeStateAck_count = 0;
			}
			if(CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT == RT_TRUE)//������������� ���������ϱ����ִ���¼���¼
			{
				BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);	//��APP�ϱ����ִ���¼���¼
				if (BLE_ChgPlanExeStateAck_Timer != RT_NULL)
				{
					rt_timer_start(BLE_ChgPlanExeStateAck_Timer);
					BLE_ChgPlanExeStateAck_count = 0;
				}
			}
		}

		rt_kprintf("[strategy]: (%s) start chargepile event, sucidle = %d  .......!\n",__func__,Ctrl_Start.cSucIdle);
	}
}

/********************************************************************  
*	�� �� ��: Stop_ChargePile_Process()
*	����˵��: ֹͣ���׮������
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Stop_ChargePile_Process(void)
{
	rt_int8_t res;
	rt_uint32_t stop_result;
	
	stop_result = 0;
	
	rt_kprintf("[strategy]: (%s) waiting for stop charge event!\n",__func__);
		//����ͣ���¼�
	if(rt_event_recv(&ChargePileEvent, ChargeStopOK_EVENT|ChargeStopER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &stop_result) == RT_EOK)		
	{
		rt_timer_stop(StopChgResp_Timer);
		
		if(stop_result|ChargeStopOK_EVENT)
		{
			Ctrl_Stop.cSucIdle = SUCCESSFUL;
			
			Charge_Record_Update(&ChgOrder_Event);//���³���¼
			if(Router_WorkState.Router_State == ChgState_PlanExeEnd)//ִ����� ת��Ϊ������״̬ �ȴ��ϴ�����
				Router_WorkState.Router_State = ChgState_Finished;//״̬���
			else
			{
				PlanExeNum++;
				Router_WorkState.Router_State = ChgState_PlanSoltsStart;//�ƻ�δִ���꣬�ȴ��¸�ʱ���
			}
		}
		else if(stop_result|ChargeStopER_EVENT)
		{
			Ctrl_Stop.cSucIdle = FAILED;
			ChargepileDataGetSet(Cmd_ChargeStopResp,&ChargePilePara_Get);//��ȡʧ��ԭ��			
		}
		
		if(CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Stop == RT_TRUE)//�������վֹͣ �����������
			CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);
		
		rt_kprintf("[strategy]: (%s) stop chargepile event, sucidle = %d  .......!\n",__func__,Ctrl_Stop.cSucIdle);
	}
}


/********************************************************************  
*	�� �� ��: Stop_ChargePile_Process()
*	����˵��: ֹͣ���׮������
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Adjust_ChargePile_Power(void)
{
	rt_int8_t res;
	rt_uint32_t adjpow_result;
	
	adjpow_result = 0;
	
	rt_kprintf("[strategy]: (%s) waiting for adjust power event!\n",__func__);
		//�������ʳɹ�
	if(rt_event_recv(&ChargePileEvent, SetPowerOK_EVENT|SetPowerER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &adjpow_result) == RT_EOK)	
	{
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


/********************************************************************  
*	�� �� ��: TimeSolt_PilePowerCtrl()
*	����˵��: ��ʱ�ν��е�׮���ʿ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Charge_Plan_Exe_Start(void)
{
	rt_uint8_t i;
	rt_uint32_t Time_s;
	rt_uint32_t PlanStartTime_start[50];
	rt_uint32_t PlanStartTime_stop[50];

	
	Time_s = TimeCalculation(&System_Time_STR);//��ȡ��ǰʱ������
	
	for(i = PlanExeNum; i<PlanSlotCount;i++)
	{
		PlanStartTime_start[i] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[i].strDecStartTime);
		PlanStartTime_stop[i] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[i].strDecStopTime);
		
		if((Time_s > PlanStartTime_start[i])&&(Time_s < PlanStartTime_stop[i]))
		{
			Chg_ExeState.ucPlanPower = Chg_Strategy.strChargeTimeSolts[i].ulChargePow;		
			ChargePilePara_Set.PWM_Duty = Chg_Strategy.strChargeTimeSolts[i].ulChargePow*10/132;//���ʻ���
			ChargePile_Start_Charge(&ChargePilePara_Set);			
			Router_WorkState.Router_State = ChgState_PlanSoltsStarting;//��ʼ�������׮
			
			break;
		}
		rt_kprintf("[strategy]: (%s) Time_s = %d!\n",__func__,Time_s);
		rt_kprintf("[strategy]: (%s) PlanStartTime_start[%d] = %d!\n",__func__,i,PlanStartTime_start[i]);
		rt_kprintf("[strategy]: (%s) PlanStartTime_stop[%d] = %d!\n",__func__,i,PlanStartTime_stop[i]);
		rt_kprintf("[strategy]: (%s) Sys time: %02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day\
								,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
	}
}

/********************************************************************  
*	�� �� ��: TimeSolt_PilePowerCtrl()
*	����˵��: ��ʱ�ν��е�׮���ʿ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 

static void Charge_Plan_Running(void)
{
	rt_uint32_t Time_s;
	rt_uint32_t PlanStartTime_stop[50];
	
	Time_s = TimeCalculation(&System_Time_STR);//��ȡ��ǰʱ������
	
	PlanStartTime_stop[PlanExeNum] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[PlanExeNum].strDecStopTime);
	
	if(Time_s > PlanStartTime_stop[PlanExeNum])
	{
		rt_kprintf("[strategy]: (%s) ChargePile_Stop_Charge!\n",__func__,Time_s);
		ChargePile_Stop_Charge();
		if(PlanExeNum == (Chg_Strategy.ucTimeSlotNum-1))
			Router_WorkState.Router_State = ChgState_PlanExeEnd;//�ƻ�ִ�н���
		else
			Router_WorkState.Router_State = ChgState_PlanSoltsStoping;//�ƻ�ִ��δ���� ��ʱͣ�� ���ϴ���綩��
	}
	
	rt_kprintf("[strategy]: (%s) Time_s = %d!\n",__func__,Time_s);
	rt_kprintf("[strategy]: (%s) Sys time: %02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day\
							,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
}
		
/********************************************************************  
*	�� �� ��: ChgOrder_Apply()
*	����˵��: �γɳ�綩���ϱ�
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void ChgOrder_Apply(void)
{	
		Charge_Record_Update(&ChgOrder_Event);//���³���¼

//		CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//�ϱ���綩��

		if (HPLC_ChgRecordAck_Timer != RT_NULL)
		{
			rt_timer_start(HPLC_ChgRecordAck_Timer);
			HPLC_ChgRecordAck_count = 0;	
			rt_kprintf("[strategy]:  (%s) HPLC_ChgOrder_Event Apply to Contrllor, Successful!\n",__func__);
		}
		
		if(CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT == RT_TRUE)//����������
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//ͬʱ���¼��ش�APP
			
			if (BLE_ChgRecordAck_Timer != RT_NULL)
			{
				rt_timer_start(BLE_ChgRecordAck_Timer);
				BLE_ChgRecordAck_count = 0;	
				rt_kprintf("[strategy]:  (%s) BLE_ChgOrder_Event Apply to Contrllor, Successful!\n",__func__);
			}
		}
		Chg_ExeState.exeState = EXE_END;
		Router_WorkState.Router_State = ChgState_Standby;//״̬���
}


/********************************************************************  
*	�� �� ��: ChgOrder_Apply()
*	����˵��: �γɳ�綩���ϱ�
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void Router_Alarm_Process(void)
{
	static rt_uint32_t l_RouterAlm,l_PileAlm;
	
	if(l_RouterAlm ^ Router_WorkState.Router_Fault.Total_Fau)//·��������
	{
		l_RouterAlm = Router_WorkState.Router_Fault.Total_Fau; 
		Router_Pile_Alarm_Event_Create(&Router_Pile_Fault_Event,&Router_WorkState);//�����쳣��Ϣ�¼���¼
		
		
		CtrlUnit_RecResp(Cmd_DeviceFault,&Router_Pile_Fault_Event,0);//�ϱ�·����������Ϣ	
			
		if(CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT == RT_TRUE)//����������
		{
			BLE_CtrlUnit_RecResp(Cmd_DeviceFault,&Router_Pile_Fault_Event,0);//�ϱ�·����������Ϣ
		}
	}
	
	if(l_PileAlm ^ Router_WorkState.Pile_Fault.Total_Fau)//���׮����
	{
		l_PileAlm = Router_WorkState.Pile_Fault.Total_Fau;
	}
	
	if(Router_WorkState.Router_Fault.Total_Fau == RT_FALSE)
		Router_WorkState.Router_State = ChgState_Standby;
}



/********************************************************************  
*	�� �� ��: RtState_Judge()
*	����˵��: ·�����ͳ��׮״̬�ж�
*	��    ��: ��
*	�� �� ֵ: ���ϴ���
********************************************************************/ 
static void RtState_Judge(void)
{
	
	ExeState_Update();//·����״̬����
//	
	if((Router_WorkState.Router_Fault.Total_Fau != RT_FALSE)||(Router_WorkState.Pile_State == PILE_FAU))
		Router_WorkState.Router_State = ChgState_Fault;
	
	switch(Router_WorkState.Router_State)
	{
		case ChgState_Standby://����
			break;
		
		case ChgState_PlanSoltsStart:          //���յ����ƻ� �жϳ��ƻ�
			Charge_Plan_Exe_Start();
			break;
		
		case ChgState_PlanSoltsStarting:    //�ѷ����������� �ȴ�����������Ӧ
			Start_ChargePile_Process();
		break;
		
		case ChgState_InCharging:           //�����		
			Charge_Record_Update(&ChgOrder_Event);//���³���¼
			Charge_Plan_Running();//�ж��Ƿ�����ֹͣ����
			break;
		
		case ChgState_PlanExeEnd:
		case ChgState_PlanSoltsStoping://�ѷ���ֹͣ���� �ȴ�����������Ӧ
			Stop_ChargePile_Process();
		break;
		
		case ChgState_Finished:				//���ƻ�ִ�����  �ϴ���綩�� ���ش���״̬
			ChgOrder_Apply();
			break;
		
		case ChgState_Fault:            	//·���������׮����
			Router_Alarm_Process();
			break;
			
		case ChgState_Update:
			break;
		
		default:
			break;
	}
}

static void strategy_thread_entry(void *parameter)
{
	rt_err_t res,p_rst;
	rt_uint32_t time;

	/*��ʼ������*/
	
			/* ���������� */
//	strategy_mutex = rt_mutex_create("strategy_mutex", RT_IPC_FLAG_FIFO);
//	if (strategy_mutex == RT_NULL)
//	{
//		rt_kprintf("[strategy]:  (%s) rt_mutex_create make fail\n");
//	}

	Router_WorkState.Router_State = ChgState_Standby;
	Router_WorkState.Router_Fault.Total_Fau = RT_FALSE;
	
	Router_WorkState.Pile_State = PILE_STANDBY;
	Router_WorkState.Pile_Fault.Total_Fau = RT_FALSE;
	
	Chg_ExeState.exeState = EXE_NULL;
	Charge_Event_Data_Clear();

	rt_thread_mdelay(3000);
	
	rt_pin_mode(RELAYA_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RELAYB_PIN, PIN_MODE_OUTPUT);
	RELAY_ON();//�ϵ����ϼ̵���
	
//	PlanExeNum_Old = 0xff;
//	PlanExeNum = 0;
//	PlanSlotCount = 1;
	while (1)
	{
		RtState_Judge();
		
		CtrlData_RecProcess();
		
//		Charge_Record_Update(&ChgOrder_Event);
		
//		time = TimeCalculation(&System_Time_STR);//��ȡ��ǰʱ������
//		rt_kprintf("[strategy]: (%s) time = %d  \n",__func__,time);
//		rt_kprintf("[strategy]: (%s) Sys time:%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day\
//								,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
		
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


void alarm(int argc, char**argv)
{
	rt_uint8_t tmp;
	
	tmp = strtol(argv[1],NULL,10);
	
	if(tmp == 1)
	{
		Router_WorkState.Router_Fault.Bit.Memory_Fau =1;
		Router_WorkState.Router_Fault.Bit.ESAM_Fau = 1;
	}
	else
	{
		Router_WorkState.Router_Fault.Bit.Memory_Fau =0;
		Router_WorkState.Router_Fault.Bit.ESAM_Fau = 0;
	}
}
MSH_CMD_EXPORT(alarm, AC out  CMD);



