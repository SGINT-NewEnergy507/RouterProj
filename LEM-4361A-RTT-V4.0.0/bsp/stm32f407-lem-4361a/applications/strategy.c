#include <rtthread.h>
#include <rtdevice.h>
#include "strategy.h"
#include "chargepile.h"
#include "698.h"
#include "meter.h"
#include "energycon.h"
#include "storage.h"
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
#define THREAD_STRATEGY_STACK_SIZE   1024
#define THREAD_STRATEGY_TIMESLICE    5



extern rt_uint32_t Strategy_get_BLE_event(void);
extern rt_uint8_t BLE_CtrlUnit_RecResp(COMM_CMD_C cmd,void *STR_SetPara,int count);
///////////////////////////////////////////////////////////////////
//�������ԭ��
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

	
static rt_uint8_t strategy_stack[THREAD_STRATEGY_STACK_SIZE];//�̶߳�ջ
static struct rt_thread strategy;

CCMRAM ChargPilePara_TypeDef ChargePilePara_Set;
CCMRAM ChargPilePara_TypeDef ChargePilePara_Get;

CCMRAM CHARGE_EXE_STATE_ASK ChgExeStateAsk;//������
CCMRAM CHARGE_EXE_STATE_ASK RouterExeState;//����

CCMRAM CHARGE_STRATEGY Chg_Strategy;
CCMRAM CHARGE_STRATEGY_RSP Chg_StrategyRsp;
CCMRAM CHARGE_STRATEGY Adj_Chg_Strategy;
CCMRAM CHARGE_STRATEGY_RSP Adj_Chg_StrategyRsp;

CCMRAM CHARGE_APPLY Chg_Apply;
CCMRAM CHARGE_APPLY_RSP Chg_Apply_Rsp;

//�����¼�
CCMRAM PLAN_OFFER_EVENT Plan_Offer_Event;
CCMRAM CHARGE_APPLY_EVENT Chg_Apply_Event;





//��ʱ���
CCMRAM static rt_timer_t ChgReqReportRsp;
//��ʱ��ѵ
CCMRAM static rt_timer_t ChgPileStateGet;
/**************************************************************
 * ��������: ChgReqReportResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ���ͳ�����볬ʱ����
 **************************************************************/
static void ChgReqReportResp_Timeout(void *parameter)
{
    rt_lprintf("[strategy] : ChgReqReportResp event is timeout!\n");
	//���ش洢
	SetStorageData(Cmd_ChgRequestWr,&Chg_Apply_Event,sizeof(CHARGE_APPLY_EVENT));
	rt_timer_stop(ChgReqReportRsp);
}
/**************************************************************
 * ��������: ChgReqReportResp_Timeout 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ��ѵ���׮״̬����
 **************************************************************/
static void ChgPileStateGet_Timeout(void *parameter)
{
    rt_lprintf("[strategy] : ChgPileStateGet event is timeout!\n");
	//��ѯ���׮״̬
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
}
/**************************************************************
 * ��������: timer_create_init 
 * ��    ��: 
 * �� �� ֵ: 
 * ��    ��: ��ʱ��
 **************************************************************/
static void timer_create_init()
{
    /* ���� ����������ͻظ� ��ʱ�� */
	 ChgReqReportRsp = rt_timer_create("ChgReqReportRsp",  /* ��ʱ�������� ChgReqReportRsp */
									ChgReqReportResp_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									5000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* һ���Զ�ʱ�� */
	/* ���� ���׮״̬��ѯ ��ʱ�� */
	ChgPileStateGet = rt_timer_create("ChgPileStateGet",  /* ��ʱ�������� ChgPileStateGet */
									ChgPileStateGet_Timeout, /* ��ʱʱ�ص��Ĵ����� */
									RT_NULL, /* ��ʱ��������ڲ��� */
									1000, /* ��ʱ���ȣ���OS TickΪ��λ����5000��OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* һ���Զ�ʱ�� */
	/* ������ʱ�� */
	if (ChgPileStateGet != RT_NULL)
		rt_timer_start(ChgPileStateGet);
}
/********************************************************************  
*	�� �� ��: ChgPlan_RecProcess()
*	����˵��: ���ƻ�������_���մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void ChgPlan_RecProcess(void)
{
	rt_uint8_t c_rst,b_rst;
	rt_uint32_t chgplanIssue,chgplanIssueAdj,startchg,stopchg;
	rt_uint32_t Ctrl_EventCmd,BLE_EventCmd;
	Ctrl_EventCmd = strategy_event_get();
	BLE_EventCmd = Strategy_get_BLE_event();
	
	switch(Ctrl_EventCmd)
	{
		//�յ����ƻ�
		case ChgPlanIssue_EVENT:
		{
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//ȡֵ
			
			if((Chg_Strategy.ucChargeMode == 1)&&(Chg_Strategy.ucDecType == 1))
				rt_sem_release(&rx_sem_setpower);
			
			memcpy(&Chg_StrategyRsp,&Chg_Strategy,40);
			Chg_StrategyRsp.cSucIdle = 0;
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,&Chg_StrategyRsp,0);//�ظ�	
			
			if(c_rst == 0)
			{
				memcpy(&Plan_Offer_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//��¼�ϱ�ʱ��
				SetStorageData(Cmd_PlanOfferWr,&Plan_Offer_Event,sizeof(PLAN_OFFER_EVENT));
				//�ظ��ɹ������¼�
				Plan_Offer_Event.OrderNum++;		
				Plan_Offer_Event.ChannelState = 0;//ͨ��״̬
				memcpy(&Plan_Offer_Event.Chg_Strategy,&Chg_Strategy,sizeof(CHARGE_STRATEGY));
				memcpy(&Plan_Offer_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
				c_rst = CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ�ʱ��
				
				//ȱ�ȴ�ȷ��		
			}
			break;
		}
		//�յ����ƻ�����
		case ChgPlanAdjust_EVENT:
		{
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanAdjust,&Adj_Chg_Strategy,0);//ȡֵ	
			if((Chg_Strategy.ucChargeMode == 1)&&(Chg_Strategy.ucDecType == 2))
				rt_sem_release(&rx_sem_adjpower);
			
			memcpy(&Adj_Chg_StrategyRsp,&Adj_Chg_Strategy,40);
			Chg_StrategyRsp.cSucIdle = 0;
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanAdjustAck,&Adj_Chg_StrategyRsp,0);//�ظ�	
			break;
		}
		//�յ�����������·��������״̬
		case AskState_EVENT:
		{
			rt_lprintf("[strategy]  (%s)  �յ�@������@��ѯ����״̬������  \n",__func__);  
				
			if(memcmp(ChgExeStateAsk.cAssetNO,RouterIfo.AssetNum,sizeof(ChgExeStateAsk.cAssetNO)) == 0)
			{
				memcpy(Chg_ExeState.cAssetNO,RouterIfo.AssetNum,sizeof(Chg_ExeState.cRequestNO));
				
				if(Chg_ExeState.exeState != EXE_ING)//��ִ�й����У����ƻ��ж���ʴ�
					Chg_ExeState.ucPlanPower = Chg_Strategy.ulChargeRatePow;
				
				ScmMeter_HisData stgMeter_HisData;
				cmMeter_get_data(EMMETER_HISDATA,&stgMeter_HisData);//��ȡ����������
				memcpy(&Chg_ExeState.ulEleBottomValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
				memcpy(&Chg_ExeState.ulEleActualValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
				
				ScmMeter_Analog stgMeter_Analog;
				cmMeter_get_data(EMMETER_ANALOG,&stgMeter_Analog);
				Chg_ExeState.ucActualPower = stgMeter_Analog.ulAcPwr;
				Chg_ExeState.ucVoltage.A = stgMeter_Analog.ulVol;
				Chg_ExeState.ucCurrent.A = stgMeter_Analog.ulCur;
				
				ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
				Chg_ExeState.ChgPileState = ChargePilePara_Get.ChgPileState;
			}
			else
			{
				Chg_ExeState.exeState = EXE_FAILED;//���뵥�Ų�ƥ�䣬��Ϊ��ִ��ʧ��"
			}
			
			
			c_rst = CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,0);//�ظ�			   	
			break;
		}
		//�յ��������ȷ��
		case ChgReqReportConfirm_EVENT:
		{	
				
			break;
		}		
		default:
			break;
	}
	Ctrl_EventCmd = 0;//��λ
	
	//�յ��������
	switch(BLE_EventCmd)
	{
		case ChgRequest_EVENT:
		{
			memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
			memset(&Chg_Apply_Rsp,0,sizeof(CHARGE_APPLY_RSP));
			memcpy(&Chg_Apply_Event.RequestTimeStamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequest,&Chg_Apply,0);//ȡֵ
			if(b_rst == 0)
			{
				if(memcmp(&RouterIfo.AssetNum,&Chg_Apply.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//У���ʲ�һ����
				{
					Chg_Apply_Event.OrderNum++;
					memcpy(&Chg_Apply_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
					memcpy(&Chg_Apply_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
					memcpy(&Chg_Apply_Event.RequestNO,&Chg_Apply.cRequestNO,51);//���������ֶ�			
					Chg_Apply_Event.ChargeReqEle = Chg_Apply.ulChargeReqEle;			
					memcpy(&Chg_Apply_Event.PlanUnChg_TimeStamp,&Chg_Apply.PlanUnChg_TimeStamp,sizeof(STR_SYSTEM_TIME));
					Chg_Apply_Event.ChargeMode = Chg_Apply.ChargeMode;
					
					if(Chg_Apply.cUserID[0]<=9)//��ֹ���
					{
						memcpy(&Chg_Apply_Event.UserAccount[1],&Chg_Apply.cUserID[1],Chg_Apply.cUserID[0]);		
						Chg_Apply_Event.UserAccount[0] = Chg_Apply.cUserID[0];
					}
					
					c_rst = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//�����¼�
					b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReportAPP,&Chg_Apply_Event,0);//ͬʱ���¼��ش�APP

					/* ����������ͻظ���ʱ */
					if (ChgReqReportRsp != RT_NULL)
						rt_timer_start(ChgReqReportRsp);
					else
						rt_lprintf("ChgReqReportResp timer create error\n");

					Chg_Apply_Rsp.cSucIdle = SUCCESSFUL;
				}
				else
				{				
					Chg_Apply_Rsp.cSucIdle = FAILED;
				}
				memcpy(&Chg_Apply_Rsp.cRequestNO,&Chg_Apply.cRequestNO,sizeof(Chg_Apply.cRequestNO));
				memcpy(&Chg_Apply_Rsp.cAssetNO,&Chg_Apply.cAssetNO,sizeof(Chg_Apply.cAssetNO));
				Chg_Apply_Rsp.GunNum = Chg_Apply.GunNum;
			}
		
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestAck,&Chg_Apply_Rsp,0);//�ظ�	
			
			if(b_rst == SUCCESSFUL)
				ChargepileDataGetSet(Cmd_ChargeStart,0);
			break;
		}
		//�յ���������·��������״̬
		case AskState_EVENT:
		{
			rt_lprintf("[strategy]  (%s)  �յ�@������ѯ@����״̬������  \n",__func__);  
				
			if(memcmp(RouterExeState.cAssetNO,RouterIfo.AssetNum,sizeof(RouterExeState.cAssetNO)) == 0)//У���ʲ���
			{
				memcpy(Chg_ExeState.cRequestNO,RouterIfo.AssetNum,sizeof(Chg_ExeState.cRequestNO));
				
				if(Chg_ExeState.exeState != EXE_ING)//��ִ�й����У����ƻ��ж���ʴ�
					Chg_ExeState.ucPlanPower = Chg_Strategy.ulChargeRatePow;
				
				ScmMeter_HisData stgMeter_HisData;
				cmMeter_get_data(EMMETER_HISDATA,&stgMeter_HisData);//��ȡ����������
				memcpy(&Chg_ExeState.ulEleBottomValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
				memcpy(&Chg_ExeState.ulEleActualValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
				
				ScmMeter_Analog stgMeter_Analog;
				cmMeter_get_data(EMMETER_ANALOG,&stgMeter_Analog);
				Chg_ExeState.ucActualPower = stgMeter_Analog.ulAcPwr;
				Chg_ExeState.ucVoltage.A = stgMeter_Analog.ulVol;
				Chg_ExeState.ucCurrent.A = stgMeter_Analog.ulCur;
				
				ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
				Chg_ExeState.ChgPileState = ChargePilePara_Get.ChgPileState;
			}
			else
			{
				Chg_ExeState.exeState = EXE_FAILED;//���뵥�Ų�ƥ�䣬��Ϊ��ִ��ʧ��"
			}		
			
			b_rst = BLE_CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,0);//�ظ�			   	
			break;
		}
		default:
			break;
	}
	BLE_EventCmd = 0;//��λ
}

/********************************************************************  
*	�� �� ��: RtState_Judge()
*	����˵��: ·����״̬�ж�
*	��    ��: ��
*	�� �� ֵ: ���ϴ���
********************************************************************/ 
static void RtState_Judge(void)
{
	if(ChargePilePara_Get.ChgPileState == ChgSt_Fault)
	{
		Fault.Bit.ChgPile_Fau = TRUE;	
	}	
	else if(ChargePilePara_Get.ChgPileState == ChgSt_InCharging)
	{
		PileIfo.WorkState = RtSt_InCharging;
	}
	else if(ChargePilePara_Get.ChgPileState == ChgSt_Finished)
	{
		PileIfo.WorkState = RtSt_Finished;
	}
	
	if(Fault.Total == TRUE)
		PileIfo.WorkState = RtSt_Fault;	
}

static void strategy_thread_entry(void *parameter)
{
	rt_err_t res;
	
//	Fault.Total = FALSE;
	PileIfo.WorkState = RtSt_StandbyOK;//��������
	rt_thread_mdelay(100);
	
	while (1)
	{
		RtState_Judge();
		ChgPlan_RecProcess();
						
		rt_thread_mdelay(1000);
	}
}

int strategy_thread_init(void)
{
	rt_err_t res;
	
	PileIfo.WorkState = RtSt_Starting;//������
	Chg_ExeState.exeState = EXE_NULL;
	memset(&Plan_Offer_Event,0,sizeof(PLAN_OFFER_EVENT));
	memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
	
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
		rt_thread_startup(&strategy);
	}
	return res;
}


#if defined (RT_STRATEGY_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
	INIT_APP_EXPORT(strategy_thread_init);
#endif
MSH_CMD_EXPORT(strategy_thread_init, strategy thread run);



