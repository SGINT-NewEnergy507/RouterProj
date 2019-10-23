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
#define THREAD_STRATEGY_STACK_SIZE   2048
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

CCMRAM static CHARGE_EXE_STATE_ASK ExeState_CtrlAsk;//������
CCMRAM static CHARGE_EXE_STATE_ASK ExeState_BleAsk;//����

CCMRAM CHARGE_STRATEGY Chg_Strategy;
CCMRAM static CHARGE_STRATEGY_RSP Chg_StrategyRsp;
CCMRAM CHARGE_STRATEGY Adj_Chg_Strategy;
CCMRAM static CHARGE_STRATEGY_RSP Adj_Chg_StrategyRsp;

CCMRAM static CHARGE_APPLY Chg_Apply;
CCMRAM static CHARGE_APPLY_RSP Chg_Apply_Rsp;

//�����¼�
CCMRAM PLAN_OFFER_EVENT Plan_Offer_Event;
CCMRAM CHARGE_APPLY_EVENT Chg_Apply_Event;
CCMRAM static CHARGE_EXE_EVENT BLE_ChgExe_Event;
	


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
	
//	if(ChargePilePara_Get.ChgPileState == )
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
*	�� �� ��: ExeState_Update()
*	����˵��: ·�������ִ��״̬����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void ExeState_Update(void)
{	
	memcpy(Chg_ExeState.cRequestNO,Chg_Apply_Event.RequestNO,sizeof(Chg_ExeState.cRequestNO));
	memcpy(Chg_ExeState.cAssetNO,RouterIfo.AssetNum,sizeof(Chg_ExeState.cAssetNO));
	Chg_ExeState.GunNum = ExeState_BleAsk.GunNum;
	
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
	memcpy(&Chg_ExeState.cUserID,&Chg_Apply_Event.UserAccount,sizeof(Chg_ExeState.cUserID));	
}
/********************************************************************  
*	�� �� ��: ChgPlan_RecProcess()
*	����˵��: ���ƻ�������_���մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void ChgPlan_RecProcess(void)
{
	rt_uint8_t c_rst,b_rst,p_rst,s_rst;
	rt_uint32_t chgplanIssue,chgplanIssueAdj,startchg,stopchg;
	rt_uint32_t Ctrl_EventCmd,BLE_EventCmd;
	
	/***************************** ���տ��������� *******************************/
	Ctrl_EventCmd = strategy_event_get();
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
				
			if(memcmp(ExeState_CtrlAsk.cAssetNO,RouterIfo.AssetNum,sizeof(ExeState_CtrlAsk.cAssetNO)) == 0)
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
			
			c_rst = CtrlUnit_RecResp(Cmd_RouterExeState,&Chg_ExeState,0);//�ظ�			   	
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
	
	/***************************** ����APP���� *******************************/
	BLE_EventCmd = Strategy_get_BLE_event();
	switch(BLE_EventCmd)
	{
		//�������    Ȼ��ֱ��������磬��������������ִ���¼�
		case ChgRequest_EVENT:
		{	
			rt_lprintf("[strategy]  (%s) <ChgRequest_EVENT> �յ�@����@ ������� ������\n",__func__); 
			memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
			memset(&Chg_Apply_Rsp,0,sizeof(CHARGE_APPLY_RSP));
			memset(&BLE_ChgExe_Event,0,sizeof(CHARGE_EXE_EVENT));
			
			memcpy(&Chg_Apply_Event.RequestTimeStamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�������ʱ��
			memcpy(&Chg_Apply_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequest,&Chg_Apply,0);//ȡֵ
			
			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Receive the data of Chg_Apply from BLE, Successful!\n",__func__);
				if(memcmp(&RouterIfo.AssetNum,&Chg_Apply.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//У���ʲ�һ����
				{
					Chg_Apply_Event.OrderNum++;
					rt_lprintf("[strategy]  (%s) Chg_Apply_Event.OrderNum = %d\n",__func__,Chg_Apply_Event.OrderNum);	

					memcpy(&Chg_Apply_Event.RequestNO,&Chg_Apply.cRequestNO,sizeof(Chg_Apply_Event.RequestNO));
					memcpy(&Chg_Apply_Event.AssetNO,&Chg_Apply.cAssetNO,sizeof(Chg_Apply_Event.AssetNO));
					Chg_Apply_Event.GunNum = Chg_Apply.GunNum;
					
					Chg_Apply_Event.ChargeReqEle = Chg_Apply.ulChargeReqEle;			
					memcpy(&Chg_Apply_Event.PlanUnChg_TimeStamp,&Chg_Apply.PlanUnChg_TimeStamp,sizeof(STR_SYSTEM_TIME));
					Chg_Apply_Event.ChargeMode = Chg_Apply.ChargeMode;
					
					memcpy(&Chg_Apply_Event.UserAccount,&Chg_Apply.cUserID,sizeof(Chg_Apply_Event.UserAccount));							
					memcpy(&Chg_Apply_Event.Token,&Chg_Apply.Token,sizeof(Chg_Apply.Token));		//��32��Ч��ȡֵ

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
					rt_lprintf("[strategy]  (%s) У���ʲ�һ���ԣ�ʧ�ܣ�\n",__func__);
				}
				memcpy(&Chg_Apply_Rsp.cRequestNO,&Chg_Apply.cRequestNO,sizeof(Chg_Apply.cRequestNO));
				memcpy(&Chg_Apply_Rsp.cAssetNO,&Chg_Apply.cAssetNO,sizeof(Chg_Apply.cAssetNO));
				Chg_Apply_Rsp.GunNum = Chg_Apply.GunNum;
			}
			else
			{
				rt_lprintf("[strategy]  (%s) ��ȡBLE��������ݣ�ʧ�ܣ�\n",__func__);
			}
		
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestAck,&Chg_Apply_Rsp,0);//�ظ�	
			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Chg_Apply Response, Successful!\n",__func__);				
			}
			else
			{
				rt_lprintf("[strategy]  (%s) �ظ�BLE�������ȷ����Ӧ��ʧ�ܣ�\n",__func__);
			}
			
			
			//�����¼�����
			s_rst = SetStorageData(Cmd_ChgRequestWr,&Chg_Apply_Event,sizeof(CHARGE_APPLY_EVENT));
			if(s_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Storage Chg_Apply_Event, Successful!\n",__func__);
			}
			else
			{
				rt_lprintf("[strategy]  (%s) ���� BLE�����¼���ʧ�ܣ�\n",__func__);
				SetStorageData(Cmd_ChgRequestWr,&Chg_Apply_Event,sizeof(CHARGE_APPLY_EVENT));//�ٴ�һ��
			}
			memcpy(&Chg_Apply_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
//			c_rst = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//�����¼�
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReportAPP,&Chg_Apply_Event,0);//ͬʱ���¼��ش�APP
			
			//�ݶ������������
			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Charge_Apply Response to BLE, Successful!\n",__func__);
				CtrlCharge_Event.CtrlType = CTRL_START;
				CtrlCharge_Event.StartSource = BLE_UNIT;
				p_rst = ChargepileDataGetSet(Cmd_ChargeStart,0);
				memcpy(&BLE_ChgExe_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
				if(p_rst == SUCCESSFUL)
				{
					PileIfo.WorkState = ChgSt_InCharging;//�˴�������λ���·������ɹ���Ϊ���׮������
					rt_lprintf("[strategy]  (%s) Charge Started, Successful!\n",__func__);				
				}
				else
				{
					rt_lprintf("[strategy]  (%s) BLE���������������磬ʧ�ܣ�\n",__func__);
				}
				
				//���ͳ��ִ���¼�
				BLE_ChgExe_Event.OrderNum++;
				BLE_ChgExe_Event.OccurSource = 0;
				ExeState_Update();
				memcpy(&BLE_ChgExe_Event.Chg_ExeState,&Chg_ExeState,sizeof(CHARGE_EXE_STATE));
				BLE_ChgExe_Event.Chg_ExeState.GunNum = Chg_Apply.GunNum;//ע�����ȡֵ��ͬ
			
				memcpy(&BLE_ChgExe_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
				//�洢���ִ���¼�
				s_rst = SetStorageData(Cmd_ChgExecuteWr,&BLE_ChgExe_Event,sizeof(CHARGE_EXE_EVENT));
				if(s_rst == SUCCESSFUL)
				{
					rt_lprintf("[energycon]  (%s) Storage BLE_ChgExe_Event, Successful!\n",__func__);
				}
				else
				{
					rt_lprintf("[energycon]  (%s) ������ִ���¼���ʧ�ܣ�\n",__func__);
					SetStorageData(Cmd_ChgExecuteWr,&BLE_ChgExe_Event,sizeof(CHARGE_EXE_EVENT));//�ٴ�һ��
				}

				b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeState,&BLE_ChgExe_Event,0);
				if(b_rst == SUCCESSFUL)
				{
					rt_lprintf("[strategy]  (%s) BLE_ChgExe_Event Apply, Successful!\n",__func__);				
				}
				else
				{
					rt_lprintf("[strategy]  (%s) �ظ�BLE���ִ���¼���ʧ�ܣ�\n",__func__);
				}
			}	
			else
			{
				rt_lprintf("[strategy]  (%s) �ظ�BLE��������¼���ʧ�ܣ�\n",__func__);
			}	
			break;
		}
		//�յ���������·��������״̬
		case AskState_EVENT:
		{
			rt_lprintf("[strategy]  (%s)  <AskState_EVENT> �յ� @����@ ��ѯ����״̬ ������  \n",__func__,BLE_EventCmd);  				
			
			ExeState_Update();				
			
			b_rst = BLE_CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,0);//�ظ�

			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Chg_ExeState Response to BLE, Successful!\n",__func__);
			}
			else
			{
				rt_lprintf("[strategy]  (%s) ���͸�BLE·��������״̬��ʧ�ܣ�\n",__func__);
			}
			break;
		}
		default:
			break;
	}
	BLE_EventCmd = 0;//��λ
}

/********************************************************************  
*	�� �� ��: RtState_Judge()
*	����˵��: ·�����ͳ��׮״̬�ж�
*	��    ��: ��
*	�� �� ֵ: ���ϴ���
********************************************************************/ 
static void RtState_Judge(void)
{
	if(ChargePilePara_Get.ChgPileState == PILE_STANDBY)
		PileIfo.WorkState = ChgSt_Standby;
	else if(ChargePilePara_Get.ChgPileState == PILE_WORKING)
		PileIfo.WorkState = ChgSt_InCharging;
	else if(ChargePilePara_Get.ChgPileState == PILE_FAU)
		PileIfo.WorkState = ChgSt_Fault;
	
	switch(PileIfo.WorkState)
	{
		case ChgSt_Fault:
		{
			Fault.Bit.ChgPile_Fau = TRUE;
			break;
		}
		case ChgSt_InCharging:
		case ChgSt_DisCharging:
		{
			RouterIfo.WorkState = RtSt_CtrlPower;
			break;
		}
		case ChgSt_Finished:
		{
			RouterIfo.WorkState = RtSt_StandbyOK;//�������ش���
			break;
		}
		default:
			break;
	}

	
	if(Fault.Total == TRUE)
		RouterIfo.WorkState = RtSt_Fault;	
}

static void strategy_thread_entry(void *parameter)
{
	rt_err_t res;

	RouterIfo.WorkState = RtSt_StandbyOK;//��������
	
	GetStorageData(Cmd_MeterNumRd,&RouterIfo.AssetNum,sizeof(RouterIfo.AssetNum));
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
	
	/*��ʼ������*/
	RouterIfo.WorkState = RtSt_Starting;//������
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



