#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdio.h>
#include <global.h>
#include "energycon.h"
#include "chargepile.h"
#include "strategy.h"
#include "698.h"
#include "meter.h"
#include "storage.h"
#include "time.h"
#include "bluetooth.h"
#include <board.h>

#define KC1_PIN     GET_PIN(F, 4)
#define KC2_PIN     GET_PIN(F, 5)
#define KC3_PIN     GET_PIN(F, 6)

#define THREAD_ENERGYCON_PRIORITY     18
#define THREAD_ENERGYCON_STACK_SIZE   1024
#define THREAD_ENERGYCON_TIMESLICE    20

#define RELAYA_PIN    GET_PIN(F, 2)
#define RELAYB_PIN    GET_PIN(F, 3)

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
extern rt_uint8_t BLE_CtrlUnit_RecResp(COMM_CMD_C cmd,void *STR_SetPara,int count);



static rt_uint8_t energycon_stack[THREAD_ENERGYCON_STACK_SIZE];//�̶߳�ջ

struct rt_thread energycon;
struct rt_semaphore rx_sem_setpower;     //���ڽ������ݵ��ź���
struct rt_semaphore rx_sem_adjpower;

CCMRAM CTL_CHARGE Ctrl_Start;
CCMRAM CTL_CHARGE Ctrl_Stop;
CCMRAM CTL_CHARGE Ctrl_PowerAdj;

CCMRAM CHARGE_EXE_STATE Chg_ExeState;

CCMRAM CTL_CHARGE_EVENT CtrlCharge_Event;
CCMRAM CHARGE_EXE_EVENT ChgExe_Event;
CCMRAM CHG_ORDER_EVENT ChgOrder_Event;

//ָ���־
CCMRAM static rt_bool_t startchg_flag;
CCMRAM static rt_bool_t stopchg_flag;
CCMRAM static rt_bool_t adjpower_flag;

//��ʱ���
CCMRAM static rt_timer_t StartChgResp;
CCMRAM static rt_timer_t StopChgResp;
CCMRAM static rt_timer_t PowerAdjResp;

CCMRAM static unsigned char count;
CCMRAM static unsigned char SetPowerFinishFlag[50];
CCMRAM static char cRequestNO_Old[17];
CCMRAM static char cRequestNO_New[17];



void RELAY_ON(void)//���ϼ̵���
{
	rt_pin_write(RELAYA_PIN, PIN_LOW);
	rt_pin_write(RELAYB_PIN, PIN_HIGH);
}

void RELAY_OFF(void)//�Ͽ��̵���
{
	rt_pin_write(RELAYB_PIN, PIN_LOW);
	rt_pin_write(RELAYA_PIN, PIN_HIGH);
}
/**************************************************************
 * ��������: StartChgResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ������糬ʱ����
 **************************************************************/
static void StartChgResp_Timeout(void *parameter)
{
	rt_uint8_t p_rst;
	rt_lprintf("[energycon] : StartChgResp event is timeout!\n");
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
	rt_lprintf("[energycon] : StopChgResp event is timeout!\n");
	p_rst = ChargepileDataGetSet(Cmd_ChargeStopResp,0);
	
	if(p_rst == SUCCESSFUL)
		PileIfo.WorkState = ChgSt_Finished;
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
	rt_lprintf("[strategy] : PowerAdjResp event is timeout!\n");
	p_rst = ChargepileDataGetSet(Cmd_SetPowerResp,0);
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
	 StartChgResp = rt_timer_create("StartChgResp",  /* ��ʱ�������� StartChgResp */
									StartChgResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* һ���Զ�ʱ�� */
	/* ����ͣ���ظ���ʱ�� */
	 StopChgResp = rt_timer_create("StopChgResp",  /* ��ʱ�������� StopChgResp */
									StopChgResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* һ���Զ�ʱ�� */
	/* �����������ʻظ���ʱ�� */
	 PowerAdjResp = rt_timer_create("PowerAdjResp",  /* ��ʱ�������� PowerAdjResp */
									PowAdjResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* һ���Զ�ʱ�� */
}
/*  */

/********************************************************************  
*	�� �� ��: CtrlData_RecProcess()
*	����˵��: ���������ݽ��մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void CtrlData_RecProcess(void)
{
	rt_uint8_t c_rst,p_rst;
	rt_uint32_t chgplanIssue,chgplanIssueAdj,startchg,stopchg;
	rt_uint32_t EventCmd,BLE_EventCmd;
	EventCmd = strategy_event_get();
	BLE_EventCmd = Strategy_get_BLE_event();
	
	switch(EventCmd)
	{	
		//�յ������������
		case StartChg_EVENT:
		{
			startchg_flag = TRUE;
			CtrlCharge_Event.StartSource = CONTRL_UNIT;
			
			c_rst = CtrlUnit_RecResp(Cmd_StartChg,&Ctrl_Start,0);//ȡֵ
			rt_lprintf("[energycon]  (%s)  �յ�@������@�����������  \n",__func__);
			memcpy(&CtrlCharge_Event,&Ctrl_Start,41);
			CtrlCharge_Event.CtrlType = CTRL_START;
			
			if(Fault.Total != TRUE)
			{
				if(memcmp(&RouterIfo.AssetNum,&Ctrl_Start.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//У���ʲ�һ����
				{
					p_rst = ChargepileDataGetSet(Cmd_ChargeStart,0);	
				
					/* ��ʼ�����ظ���ʱ */
					if (StartChgResp != RT_NULL)
						rt_timer_start(StartChgResp);
					else
						rt_lprintf("StartChgResp timer create error\n");							
				}
				else
				{
					Ctrl_Start.cSucIdle = ORTHERS;
					c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//�ظ�
				}			
			}
			else
			{
				Ctrl_Start.cSucIdle = FAILED;	
				c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//�ظ�			
			}		
			CtrlCharge_Event.cSucIdle = Ctrl_Start.cSucIdle;			
			break;
		}
		//�յ�ֹͣ�������
		case StopChg_EVENT:
		{
			stopchg_flag = TRUE;	
			CtrlCharge_Event.StopSource = CONTRL_UNIT;
			
			c_rst = CtrlUnit_RecResp(Cmd_StopChg,&Ctrl_Stop,0);//ȡֵ			
			rt_lprintf("[energycon]  (%s)  �յ�@������@ֹͣ�������  \n",__func__); 
			memcpy(&CtrlCharge_Event,&Ctrl_Stop,41);			
			CtrlCharge_Event.CtrlType = CTRL_STOP;
			
			if(Fault.Total != TRUE)
			{
				if((memcmp(&RouterIfo.AssetNum,&Ctrl_Stop.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//У���ʲ�һ����
					||(memcmp(&Ctrl_Start.OrderSn,&Ctrl_Stop.OrderSn,sizeof(Ctrl_Start.OrderSn)) == 0))//У����ͣ����һ����		
				{
					p_rst = ChargepileDataGetSet(Cmd_ChargeStop,0);	
				
					/* ��ʼͣ���ظ���ʱ */
					if (StopChgResp != RT_NULL)
						rt_timer_start(StopChgResp);
					else
						rt_lprintf("StopChgResp timer create error\n");
				}
				else
				{
					Ctrl_Stop.cSucIdle = ORTHERS;
					c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
				}
			}
			else
			{
				Ctrl_Stop.cSucIdle = FAILED;
				c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
			}
			CtrlCharge_Event.cSucIdle = Ctrl_Stop.cSucIdle;
			break;
		}
		//�յ�������������
		case PowerAdj_EVENT:
		{
			adjpower_flag = TRUE;
			c_rst = CtrlUnit_RecResp(Cmd_PowerAdj,&Ctrl_PowerAdj,0);//ȡֵ	
			rt_lprintf("[energycon]  (%s)  �յ�@������@������������  \n",__func__);  
			memcpy(&CtrlCharge_Event,&Ctrl_PowerAdj,41);
			CtrlCharge_Event.CtrlType = CTRL_ADJPOW;
			
			if(Fault.Total != TRUE)
			{
				if(memcmp(&RouterIfo.AssetNum,&Ctrl_PowerAdj.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//У���ʲ�һ����
				{
					ChargePilePara_Set.PWM_Duty = Ctrl_PowerAdj.SetPower*10/132;//���ʻ���: D(��һλС��)=I/60*1000=P/(60*220)*1000
					p_rst = ChargepileDataGetSet(Cmd_SetPower,&ChargePilePara_Set);	
					
					/* ��ʼ���ʵ����ظ���ʱ */
					if (PowerAdjResp != RT_NULL)
						rt_timer_start(PowerAdjResp);
					else
						rt_lprintf("[strategy] : PowerAdjResp timer create error\n");
				}
				else
				{
					Ctrl_PowerAdj.cSucIdle = ORTHERS;
					c_rst = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);//�ظ�
				}
			}
			else
			{
				Ctrl_PowerAdj.cSucIdle = FAILED;
				c_rst = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);//�ظ�
			}
			Ctrl_Stop.cSucIdle = Ctrl_PowerAdj.cSucIdle;
			break;
		}

		default:
			break;
	}
	
	//����ͣ��
	if(BLE_EventCmd == StopChg_EVENT)
	{
		CtrlCharge_Event.StopSource = BLE_UNIT;
			
		c_rst = BLE_CtrlUnit_RecResp(Cmd_StopChg,&Ctrl_Stop,0);//ȡֵ			
		rt_lprintf("[energycon]  (%s)  �յ�@����@ֹͣ�������  \n",__func__); 
		memcpy(&CtrlCharge_Event,&Ctrl_Stop,41);			
		CtrlCharge_Event.CtrlType = CTRL_STOP;
		
		if(Fault.Total != TRUE)
		{
			if(Ctrl_Stop.GunNum == GUN_A)		
			{
				p_rst = ChargepileDataGetSet(Cmd_ChargeStop,0);

				if(p_rst == SUCCESSFUL)
				{
					PileIfo.WorkState = ChgSt_Finished;	
					rt_lprintf("[energycon] : APPͣ�����յ�Chargepile����\n");
				}					
			}
//			else
//			{
//				Ctrl_Stop.cSucIdle = ORTHERS;
//				c_rst = BLE_CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
//			}
		}
//		else
//		{
//			Ctrl_Stop.cSucIdle = FAILED;
//			c_rst = BLE_CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
//		}
		
	}
}
/********************************************************************  
*	�� �� ��: PileData_RecProcess()
*	����˵��: ��׮���ݽ��մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void PileData_RecProcess(void)
{
	rt_uint8_t c_rst,p_rst;
	rt_uint32_t start_result,stop_result,adjpow_result;
	
	if(startchg_flag == TRUE)
	{
		//�����ɹ�
		if(rt_event_recv(&ChargePileEvent, ChargeStartOK_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &start_result) == RT_EOK)	
		{
			rt_timer_stop(StartChgResp);
			
			Ctrl_Start.cSucIdle = SUCCESSFUL;
			c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);
			
			//��¼���ֵ			
			ScmMeter_HisData engMeter_HisData;
			cmMeter_get_data(EMMETER_HISDATA,&engMeter_HisData);
			memcpy(&ChgOrder_Event.StartMeterValue[0],&engMeter_HisData.ulMeter_Day,5*sizeof(long));
			//��¼ʱ��
			memcpy(&ChgOrder_Event.ChgStartTime,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
			
			if(c_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				startchg_flag = FALSE;//��λ
				rt_lprintf("[energycon] : start charge successful!\n");
			}
		}
		//����ʧ��
		else if(rt_event_recv(&ChargePileEvent, ChargeStartER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &start_result) == RT_EOK)	
		{
			rt_timer_stop(StartChgResp);
			
			Ctrl_Start.cSucIdle = FAILED;
			p_rst = ChargepileDataGetSet(Cmd_ChargeStartResp,&ChargePilePara_Get);//��ȡʧ��ԭ��
			
			if(p_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				startchg_flag = FALSE;//��λ
				rt_lprintf("[energycon] : start charge failed,reason:%d!\n",ChargePilePara_Get.StartReson);
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);		
		}
		rt_lprintf("[energycon] : ChargePileEvent 0x%02X\n", start_result);
	}
	
	if(stopchg_flag == TRUE)
	{
		//ͣ���ɹ�
		if(rt_event_recv(&ChargePileEvent, ChargeStopOK_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &stop_result) == RT_EOK)		
		{
			rt_timer_stop(StopChgResp);
			
			Ctrl_Stop.cSucIdle = SUCCESSFUL;
			c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);
			
			//��¼���ֵ			
			ScmMeter_HisData engMeter_HisData;
			cmMeter_get_data(EMMETER_HISDATA,&engMeter_HisData);
			memcpy(&ChgOrder_Event.StopMeterValue[0],&engMeter_HisData.ulMeter_Day,5*sizeof(long));
			//��¼ʱ��
			memcpy(&ChgOrder_Event.ChgStopTime,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
						
			if(c_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				stopchg_flag = FALSE;//��λ
				rt_lprintf("[energycon] : stop charge successful!\n");
			}
		}
		//ͣ��ʧ��
		else if(rt_event_recv(&ChargePileEvent, ChargeStopER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &stop_result) == RT_EOK)		
		{
			rt_timer_stop(StopChgResp);
			
			Ctrl_Start.cSucIdle = FAILED;
			p_rst = ChargepileDataGetSet(Cmd_ChargeStartResp,&ChargePilePara_Get);//��ȡʧ��ԭ��
			
			if(p_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				stopchg_flag = FALSE;//��λ
				rt_lprintf("[energycon] : stop charge failed,reason:%d!\n",ChargePilePara_Get.StopReson);
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);			
		}
		rt_lprintf("[energycon] : ChargePileEvent 0x%02X\n", stop_result);
	}
	
	if(adjpower_flag == TRUE)
	{
		//�������ʳɹ�
		if(rt_event_recv(&ChargePileEvent, SetPowerOK_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &adjpow_result) == RT_EOK)	
		{
			rt_timer_stop(PowerAdjResp);
			
			Ctrl_PowerAdj.cSucIdle = SUCCESSFUL;
			c_rst = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);
			
			if(c_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				adjpower_flag = FALSE;//��λ
				rt_lprintf("[energycon] : start charge successful!\n");
			}
		}
		//��������ʧ��
		else if(rt_event_recv(&ChargePileEvent, SetPowerER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &adjpow_result) == RT_EOK)	
		{
			rt_timer_stop(PowerAdjResp);
			
			Ctrl_PowerAdj.cSucIdle = FAILED;
			p_rst = ChargepileDataGetSet(Cmd_SetPowerResp,&ChargePilePara_Get);//��ȡʧ��ԭ��
			
			if(p_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				adjpower_flag = FALSE;//��λ
				rt_lprintf("[energycon] : adjust power failed,reason:%d!\n",ChargePilePara_Get.AdjPowerReson);
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);		
		}
		rt_lprintf("[energycon] : ChargePileEvent 0x%02X\n", adjpow_result);
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
	rt_uint8_t c_rst,p_rst;
	
	//��λ��ǰ�����ƻ�����ʼʱ���
	for(i=count;i<Chg_Strategy.ucTimeSlotNum;i++)
	{
		if((Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Year <= System_Time_STR.Year)&& 
		(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Month <= System_Time_STR.Month)&& 
		(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Day <= System_Time_STR.Day)&& 
		(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Hour <= System_Time_STR.Hour)&& 
		(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Minute <= System_Time_STR.Minute)&& 
		(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Second <= System_Time_STR.Second))	
		{
			Chg_ExeState.ucPlanPower = Chg_Strategy.strChargeTimeSolts[i].ulChargePow;		
			ChargePilePara_Set.PWM_Duty = Chg_ExeState.ucPlanPower*10/132;//���ʻ���
			
			if(SetPowerFinishFlag[i] == FALSE)//���Ʒ���һ��
			{
				if(count == 0)
					memcpy(&ChgExe_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
				p_rst = ChargepileDataGetSet(Cmd_SetPower,&ChargePilePara_Set);
				
				ChgExe_Event.OrderNum++;
				memcpy(&ChgExe_Event.Chg_ExeState,&Chg_ExeState,sizeof(CHARGE_EXE_STATE));
				memcpy(&ChgExe_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
				c_rst = CtrlUnit_RecResp(Cmd_ChgPlanExeState,&ChgExe_Event,0);//�ϱ����ƻ�ִ���¼�
				
				Chg_ExeState.exeState = EXE_ING;
				SetPowerFinishFlag[i] = TRUE;
				count = i;
				//count++;
			}
			break;
		}
	}
	if(c_rst !=0 )
		CtrlUnit_RecResp(Cmd_ChgPlanExeState,&ChgExe_Event,0);//��ʧ�����ϱ�һ��
		
	if(count == Chg_Strategy.ucTimeSlotNum)//��⵽ִ����ƻ�
	{
		memcpy(cRequestNO_Old,cRequestNO_New,sizeof(cRequestNO_Old));//���ٸ�׮���͹����趨֡
	}
	
	if(PileIfo.WorkState == ChgSt_Finished)//��⵽������
	{
		memcpy(&ChgOrder_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
		
		ChgOrder_Event.OrderNum++;	
		
		if(CtrlCharge_Event.StartSource == CONTRL_UNIT)//�������ٿ�
		{
			memcpy(&ChgOrder_Event.cUserID,&Plan_Offer_Event.Chg_Strategy.cUserID,sizeof(Chg_Strategy.cUserID));
			memcpy(&ChgOrder_Event.RequestNO,&Plan_Offer_Event.Chg_Strategy.cRequestNO,sizeof(ChgOrder_Event.RequestNO));
			memcpy(&ChgOrder_Event.AssetNO,&Plan_Offer_Event.Chg_Strategy.cAssetNO,sizeof(ChgOrder_Event.AssetNO));
			ChgOrder_Event.GunNum = Plan_Offer_Event.Chg_Strategy.GunNum;
			
			ChgOrder_Event.ChargeReqEle = Plan_Offer_Event.Chg_Strategy.ulChargeReqEle;
//			memcpy(&ChgOrder_Event.RequestTimeStamp,&Chg_Apply_Event.RequestTimeStamp,sizeof(STR_SYSTEM_TIME));
//			memcpy(&ChgOrder_Event.FinishTimestamp,&Chg_Apply_Event.FinishTimestamp,sizeof(STR_SYSTEM_TIME));
			ChgOrder_Event.ChargeMode = Plan_Offer_Event.Chg_Strategy.ucChargeMode;	
		}
		else//�����ٿ�
		{
			memcpy(&ChgOrder_Event.cUserID,&Chg_Apply_Event.UserAccount,sizeof(Chg_Strategy.cUserID));
			memcpy(&ChgOrder_Event.RequestNO,&Chg_Apply_Event.RequestNO,sizeof(ChgOrder_Event.RequestNO));
			memcpy(&ChgOrder_Event.AssetNO,&Chg_Apply_Event.AssetNO,sizeof(ChgOrder_Event.AssetNO));
			ChgOrder_Event.GunNum = Chg_Apply_Event.GunNum;
			
			ChgOrder_Event.ChargeReqEle = Chg_Apply_Event.ChargeReqEle;
			memcpy(&ChgOrder_Event.RequestTimeStamp,&Chg_Apply_Event.RequestTimeStamp,sizeof(STR_SYSTEM_TIME));
			memcpy(&ChgOrder_Event.FinishTimestamp,&Chg_Apply_Event.FinishTimestamp,sizeof(STR_SYSTEM_TIME));
			ChgOrder_Event.ChargeMode = Chg_Apply_Event.ChargeMode;
		}
		
		//�����ѳ����
		for(i=0;i<5;i++)
			ChgOrder_Event.ucChargeEle[i] = ChgOrder_Event.StopMeterValue[i] - ChgOrder_Event.StartMeterValue[i]; 
		//�����ѳ�ʱ��
		time_t start_time = 0;
		struct tm* timep;
		timep->tm_sec = ChgOrder_Event.ChgStartTime.Second;
		timep->tm_min = ChgOrder_Event.ChgStartTime.Minute;
		timep->tm_hour = ChgOrder_Event.ChgStartTime.Hour;
		timep->tm_mday = ChgOrder_Event.ChgStartTime.Day;
		timep->tm_year = ChgOrder_Event.ChgStartTime.Year;
		start_time = mktime(timep);
		time_t stop_time = 0;
		timep->tm_sec = ChgOrder_Event.ChgStopTime.Second;
		timep->tm_min = ChgOrder_Event.ChgStopTime.Minute;
		timep->tm_hour = ChgOrder_Event.ChgStopTime.Hour;
		timep->tm_mday = ChgOrder_Event.ChgStopTime.Day;
		timep->tm_year = ChgOrder_Event.ChgStopTime.Year;
		stop_time = mktime(timep);
		ChgOrder_Event.ucChargeTime = stop_time - start_time;
		
		memcpy(&ChgOrder_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
		CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//�ϱ���綩��
	}
}



static void energycon_thread_entry(void *parameter)
{
	rt_err_t res,p_rst;
	rt_err_t ret = RT_EOK;
	rt_uint32_t* r_str;
	
	
	rt_pin_mode(RELAYA_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RELAYB_PIN, PIN_MODE_OUTPUT);
	RELAY_ON();//�ϵ����ϼ̵���
	
	rt_thread_mdelay(100);
	
	while (1)
	{
		if((res = rt_sem_take(&rx_sem_setpower, 1000)) == RT_EOK)
		{	
			memcpy(cRequestNO_New,Chg_Strategy.cRequestNO,sizeof(cRequestNO_New));
			memset(&SetPowerFinishFlag,0,50);//��ձ�־λ
			count = 0;						 
		}	
		
		if((res = rt_sem_take(&rx_sem_adjpower, 1000)) == RT_EOK)
		{
			memcpy(cRequestNO_New,Chg_Strategy.cRequestNO,sizeof(cRequestNO_New));
			
		}			
		
		PileData_RecProcess();	
		CtrlData_RecProcess();//
		
		if(memcmp(cRequestNO_Old,cRequestNO_New,sizeof(cRequestNO_Old)) != 0)
		{
			TimeSolt_PilePowerCtrl(); 
		}
		
		if(rt_pin_read(KC3_PIN) == PIN_LOW)
			p_rst = ChargepileDataGetSet(Cmd_ChargeStart,0);
//			strategy_event_send(StartChg_EVENT);
		
		if(rt_pin_read(KC2_PIN) == PIN_LOW)
			p_rst = ChargepileDataGetSet(Cmd_ChargeStop,0);
		rt_thread_mdelay(1000);
	}
}

int energycon_thread_init(void)
{
	rt_err_t res;
	/*��ʼ������*/
	CtrlCharge_Event.CtrlType = CTRL_NULL;
	
	/* ��ʼ����ʱ�� */
    timer_create_init();
	
	/*��ʼ���ź���*/
	rt_sem_init(&rx_sem_setpower, "rx_sem_setpower", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_sem_adjpower, "rx_sem_adjpower", 0, RT_IPC_FLAG_FIFO);
	
	res=rt_thread_init(&energycon,
											"energycon",
											energycon_thread_entry,
											RT_NULL,
											energycon_stack,
											THREAD_ENERGYCON_STACK_SIZE,
											THREAD_ENERGYCON_PRIORITY,
											THREAD_ENERGYCON_TIMESLICE);
	
	rt_lprintf("[energycon] : EnergyControl Initialized!\n");
	if (res == RT_EOK) 
	{
		rt_thread_startup(&energycon);
	}
	return res;
}


#if defined (RT_ENERGYCON_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
	INIT_APP_EXPORT(energycon_thread_init);
#endif
MSH_CMD_EXPORT(energycon_thread_init, energycon thread run);





