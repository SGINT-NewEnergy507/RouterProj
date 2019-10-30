#include <rtthread.h>
#include <rtdevice.h>
#include "strategy.h"
#include "chargepile.h"
#include "698.h"
#include "meter.h"
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
CCMRAM static PLAN_OFFER_EVENT Plan_Offer_Event;
CCMRAM static CHARGE_APPLY_EVENT Chg_Apply_Event;
CCMRAM static CHARGE_EXE_EVENT BLE_ChgExe_Event;
	

//��ʱ����
CCMRAM static rt_timer_t StartChgResp;
CCMRAM static rt_timer_t StopChgResp;
CCMRAM static rt_timer_t PowerAdjResp;
CCMRAM static rt_timer_t ChgReqReportRsp;
//��ʱ��ѵ
CCMRAM static rt_timer_t ChgPileStateGet;

//ָ���־
CCMRAM static rt_bool_t startchg_flag;
CCMRAM static rt_bool_t stopchg_flag;
CCMRAM static rt_bool_t adjpower_flag;

CCMRAM static CTL_CHARGE Ctrl_Start;//�������ز���
CCMRAM static CTL_CHARGE Ctrl_Stop;
CCMRAM static CTL_CHARGE BLE_Stop;
CCMRAM static CTL_CHARGE Ctrl_PowerAdj;

//���ִ��״̬
CCMRAM CHARGE_EXE_STATE Chg_ExeState;

CCMRAM CTL_CHARGE_EVENT CtrlCharge_Event;
CCMRAM static CHARGE_EXE_EVENT Ctrl_ChgExe_Event;
CCMRAM static CHG_ORDER_EVENT ChgOrder_Event;//��������һ������

CCMRAM static unsigned char PlanSlotCount;
CCMRAM static unsigned char SetPowerFinishFlag[50];
CCMRAM static char cRequestNO_Old[17];
CCMRAM static char cRequestNO_New[17];

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
									1000, /* ��ʱ���ȣ���OS TickΪ��λ����1000��OS Tick */
									RT_TIMER_FLAG_PERIODIC); /* �����Զ�ʱ�� */
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
	Chg_ExeState.ucActualPower = stgMeter_Analog.ulAcPwr/100;
	Chg_ExeState.ucVoltage.A = stgMeter_Analog.ulVol;
	Chg_ExeState.ucCurrent.A = stgMeter_Analog.ulCur;
	
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
	Chg_ExeState.ChgPileState = ChargePilePara_Get.ChgPileState;
	memcpy(&Chg_ExeState.cUserID,&Chg_Apply_Event.UserAccount,sizeof(Chg_ExeState.cUserID));	
}
/********************************************************************  
*	�� �� ��: CtrlData_RecProcess()
*	����˵��: ����������մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void CtrlData_RecProcess(void)
{
	rt_uint8_t i;
	rt_bool_t PlanAccepted;
	rt_int8_t c_rst,b_rst,p_rst,s_rst;
	rt_uint32_t chgplanIssue,chgplanIssueAdj,startchg,stopchg;
	rt_uint32_t Ctrl_EventCmd=Cmd_Null,BLE_EventCmd=Cmd_Null;
	char Cmd[32];
	
	/***************************** ���տ��������� *******************************/
	Ctrl_EventCmd = strategy_event_get();
	memset(Cmd,0,sizeof(Cmd));
	itoa(Ctrl_EventCmd,Cmd,2);		  //ת�����ַ��������ƻ���2 
	rt_lprintf("[strategy]  (%s)  �յ�@������@����,�¼��ű���Ϊ��[%s]  \n",__func__,Cmd);
	for(i = 0; i <sizeof(rt_uint32_t)*8;i++)
	{
		if(Ctrl_EventCmd&(0x00000001<<i))
		{
			Ctrl_EventCmd = i;
			break;
		}
	}
	
	switch(Ctrl_EventCmd)
	{
		//�յ����ƻ�
		case Cmd_ChgPlanIssueAck:
		{
			PlanAccepted = TRUE;
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//ȡֵ
			
			if(Fault.Total != TRUE)
			{
				if(memcmp(&RouterIfo.AssetNum,&Chg_Strategy.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//У���ʲ�һ����
				{
					if((Chg_Strategy.ucChargeMode == ORDER)&&(Chg_Strategy.ucDecType == PLAN_CREATE))
					{
						memcpy(cRequestNO_New,Chg_Strategy.cRequestNO,sizeof(cRequestNO_New));//���յ����³��ƻ�
						memset(&SetPowerFinishFlag,0,50);//��ձ�־λ
						PlanSlotCount = 0;
					}
					
					memcpy(&Chg_StrategyRsp.cRequestNO,&Chg_Strategy.cRequestNO,sizeof(Chg_StrategyRsp.cRequestNO));
					memcpy(&Chg_StrategyRsp.cAssetNO,&Chg_Strategy.cAssetNO,sizeof(Chg_StrategyRsp.cAssetNO));
					Chg_StrategyRsp.cSucIdle = SUCCESSFUL;
				}
				else
				{
					Chg_StrategyRsp.cSucIdle = ORTHERS;
				}
			}
			else
			{
				Chg_StrategyRsp.cSucIdle = FAILED;
			}		
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,&Chg_StrategyRsp,0);//�ظ�	
		
			//�ظ��ɹ������¼�
			if(c_rst == SUCCESSFUL)
			{
				memcpy(&Plan_Offer_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//��¼�ϱ�ʱ��
				SetStorageData(Cmd_PlanOfferWr,&Plan_Offer_Event,sizeof(PLAN_OFFER_EVENT));
				
				Plan_Offer_Event.OrderNum++;		
				Plan_Offer_Event.ChannelState = 0;//ͨ��״̬
				memcpy(&Plan_Offer_Event.Chg_Strategy,&Chg_Strategy,sizeof(CHARGE_STRATEGY));
				memcpy(&Plan_Offer_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
				c_rst = CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//�ϱ����ƻ��¼�
				
				//ȱ�ȴ�ȷ��					
			}
			else
			{
				
			}
			break;
		}
		//�յ����ƻ�����
		case Cmd_ChgPlanAdjust:
		{
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanAdjust,&Adj_Chg_Strategy,0);//ȡֵ	
			if(Fault.Total != TRUE)
			{
				if(memcmp(&RouterIfo.AssetNum,&Adj_Chg_Strategy.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//У���ʲ�һ����
				{
					if((Adj_Chg_Strategy.ucChargeMode == ORDER)&&(Adj_Chg_Strategy.ucDecType == PLAN_ADJ))
					{
						memcpy(cRequestNO_New,Adj_Chg_Strategy.cRequestNO,sizeof(cRequestNO_New));//���յ����³��ƻ�
					}
					
					memcpy(&Adj_Chg_StrategyRsp.cRequestNO,&Adj_Chg_Strategy.cRequestNO,sizeof(Adj_Chg_Strategy.cRequestNO));
					memcpy(&Adj_Chg_StrategyRsp.cAssetNO,&Adj_Chg_Strategy.cAssetNO,sizeof(Adj_Chg_Strategy.cAssetNO));
					Adj_Chg_StrategyRsp.cSucIdle = SUCCESSFUL;
				}
				else
				{
					Adj_Chg_StrategyRsp.cSucIdle = ORTHERS;
				}
			}
			else
			{
				Adj_Chg_StrategyRsp.cSucIdle = FAILED;
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanAdjustAck,&Adj_Chg_StrategyRsp,0);//�ظ�	
			break;
		}
		//�յ������������
		case Cmd_StartChg:
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
		case Cmd_StopChg:
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
		case Cmd_PowerAdj:
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
		//�յ�����������·��������״̬
		case Cmd_RouterExeState:
		{
			rt_lprintf("[strategy]  (%s)  �յ�@������@��ѯ����״̬������  \n",__func__);  
				
			if(memcmp(ExeState_CtrlAsk.cAssetNO,RouterIfo.AssetNum,sizeof(ExeState_CtrlAsk.cAssetNO)) == 0)
			{
				ExeState_Update();	
			}
			else
			{
				Chg_ExeState.exeState = EXE_FAILED;//�ʲ��Ų�ƥ�䣬��Ϊ��ִ��ʧ��"
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
	memset(Cmd,0,sizeof(Cmd));
	itoa(BLE_EventCmd,Cmd,2);		  //ת�����ַ��������ƻ���2 
	rt_lprintf("[strategy]  (%s)  �յ�@APP@����,�¼��ű���Ϊ��[%s]  \n",__func__,Cmd);
	for(i = 0; i <sizeof(rt_uint32_t)*8;i++)
	{
		if(BLE_EventCmd&(0x00000001<<i))
		{
			BLE_EventCmd = i;
			break;
		}
	}
	switch(BLE_EventCmd)
	{
		//�������    Ȼ��ֱ��������磬��������������ִ���¼�
		case Cmd_ChgRequest:
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
				for(i=0;i<sizeof(Chg_Apply.cAssetNO);i++)
					rt_kprintf("[strategy]  (%s) Chg_Apply.cAssetNO = %0x\n",__func__,Chg_Apply.cAssetNO[i]);
				
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
			
			//�������ȷ����Ӧ
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestAck,&Chg_Apply_Rsp,0);	
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
				rt_lprintf("[strategy]  (%s) Sto+rage Chg_Apply_Event, Successful!\n",__func__);
			}
			else
			{
				rt_lprintf("[strategy]  (%s) ���� BLE�����¼���ʧ�ܣ�\n",__func__);
				SetStorageData(Cmd_ChgRequestWr,&Chg_Apply_Event,sizeof(CHARGE_APPLY_EVENT));//�ٴ�һ��
			}
			memcpy(&Chg_Apply_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
			c_rst = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//֪ͨ������
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReportAPP,&Chg_Apply_Event,0);//ͬʱ���¼��ش�APP
			
			
			//�ݶ������������
			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Charge_Apply Response to BLE, Successful!\n",__func__);
				memcpy(&BLE_ChgExe_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
				if(PileIfo.WorkState == ChgSt_Standby)
				{
					CtrlCharge_Event.CtrlType = CTRL_START;
					CtrlCharge_Event.StartSource = BLE_UNIT;
					p_rst = ChargepileDataGetSet(Cmd_ChargeStart,0);
					
					if(p_rst == SUCCESSFUL)
					{
						PileIfo.WorkState = ChgSt_InCharging;//�˴�������λ���·������ɹ���Ϊ���׮������
						Chg_ExeState.exeState = EXE_ING;
						rt_lprintf("[strategy]  (%s) Charge Started, Successful!\n",__func__);				
					}
					else
					{
						Chg_ExeState.exeState = EXE_FAILED;
						rt_lprintf("[strategy]  (%s) BLE���������������磬ʧ�ܣ�\n",__func__);
					}
				}
				else
				{
					Chg_ExeState.exeState = EXE_FAILED;
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
		case Cmd_RouterExeState:
		{
			rt_lprintf("[strategy]  (%s)  <AskState_EVENT> �յ� @����@ ��ѯ����״̬ ������  \n",__func__,BLE_EventCmd);  				
			b_rst = BLE_CtrlUnit_RecResp(Cmd_RouterExeState,0,0);//ȡֵ
			
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
		//�յ�����ͣ������
		case Cmd_StopChg:
		{
			CtrlCharge_Event.StopSource = BLE_UNIT;
				
			c_rst = BLE_CtrlUnit_RecResp(Cmd_StopChg,&BLE_Stop,0);//ȡֵ			
			rt_lprintf("[energycon]  (%s)  �յ�@����@ֹͣ�������  \n",__func__); 
			memcpy(&CtrlCharge_Event.OrderSn,&BLE_Stop.OrderSn,sizeof(CtrlCharge_Event.OrderSn));	
			memcpy(&CtrlCharge_Event.cAssetNO,&BLE_Stop.cAssetNO,sizeof(CtrlCharge_Event.cAssetNO));	
			CtrlCharge_Event.GunNum = BLE_Stop.GunNum;	
			CtrlCharge_Event.CtrlType = CTRL_STOP;
			
			if(Fault.Total != TRUE)
			{
				if(Ctrl_Stop.GunNum == GUN_SINGLE)		
				{
					p_rst = ChargepileDataGetSet(Cmd_ChargeStop,0);

					if(p_rst == SUCCESSFUL)
					{
						PileIfo.WorkState = ChgSt_Finished;	
						Chg_ExeState.exeState = EXE_END;
						rt_lprintf("[energycon] : APPͣ�����յ�Chargepile����\n");
					}					
				}
//				else
//				{
//					Ctrl_Stop.cSucIdle = ORTHERS;
//					c_rst = BLE_CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
//				}
			}
//			else
//			{
//				Ctrl_Stop.cSucIdle = FAILED;
//				c_rst = BLE_CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//�ظ�
//			}
		}
		default:
			break;
	}
	BLE_EventCmd = 0;//��λ
}

/********************************************************************  
*	�� �� ��: PileData_RecProcess()
*	����˵��: ��׮���ݽ��մ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static void PileData_RecProcess(void)
{
	rt_int8_t c_rst,p_rst;
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
*	�� �� ��: TimeCalculation
*	����˵��: ʱ�����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/ 
static time_t TimeCalculation(STR_SYSTEM_TIME TimeStamp)
{
	time_t time_s = 0;
	struct tm timep;
	
	BCD_toInt((unsigned char*)&timep.tm_sec,&TimeStamp.Second,1);
	BCD_toInt((unsigned char*)&timep.tm_min,&TimeStamp.Minute,1);
	BCD_toInt((unsigned char*)&timep.tm_hour,&TimeStamp.Hour,1);
	BCD_toInt((unsigned char*)&timep.tm_mday,&TimeStamp.Day,1);	
	BCD_toInt((unsigned char*)&timep.tm_wday,&TimeStamp.Week,1);
	BCD_toInt((unsigned char*)&timep.tm_mon,&TimeStamp.Month,1);
	BCD_toInt((unsigned char*)&timep.tm_year,&TimeStamp.Year,1);
	time_s = mktime(&timep);
	
	return time_s;
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
	rt_int8_t c_rst=0,p_rst=0,s_rst=0,b_rst=0;
	time_t NowTime_s;
	unsigned char PlanStartTime_s[50];
	
	NowTime_s = TimeCalculation(System_Time_STR);//��ȡ��ǰʱ������
	//��λ��ǰ�����ƻ�����ʼʱ���
	for(i=PlanSlotCount;i<Chg_Strategy.ucTimeSlotNum;i++)
	{
		if(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Day> 0)//У��ʱ��ĺϷ���
		{
			PlanStartTime_s[i] = TimeCalculation(Chg_Strategy.strChargeTimeSolts[i].strDecStartTime);
			
			if(NowTime_s >= PlanStartTime_s[i])//��⵽ִ�е���ǰʱ���	
			{
				Chg_ExeState.ucPlanPower = Chg_Strategy.strChargeTimeSolts[i].ulChargePow;		
				ChargePilePara_Set.PWM_Duty = Chg_ExeState.ucPlanPower*10/132;//���ʻ���
				
				if(SetPowerFinishFlag[i] == FALSE)//���Ʒ���һ��
				{
					if(PlanSlotCount == 0)
						memcpy(&Ctrl_ChgExe_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
					
					p_rst = ChargepileDataGetSet(Cmd_SetPower,&ChargePilePara_Set);//�·����׮�趨����
					if(p_rst == SUCCESSFUL)
						rt_kprintf("[energycon]  (%s) ChargePilePara_Set,Cmd_SetPower,Successful!\n",__func__);
					else
						rt_kprintf("[energycon]  (%s) ִ�г��ƻ�ʱ�����趨��ʧ�ܣ�\n",__func__);
					
					Ctrl_ChgExe_Event.OrderNum++;
					memcpy(&Ctrl_ChgExe_Event.Chg_ExeState,&Chg_ExeState,sizeof(CHARGE_EXE_STATE));
					
					c_rst = CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//�ϱ����ƻ�ִ���¼�
					if(c_rst == SUCCESSFUL)
						rt_kprintf("[energycon]  (%s) Ctrl_ChgExe_Event,Cmd_ChgPlanExeState,Successful!\n",__func__);
					else
						rt_kprintf("[energycon]  (%s) �ϱ�ִ�г��ƻ���ʧ�ܣ�\n",__func__);
					
					Chg_ExeState.exeState = EXE_ING;
					SetPowerFinishFlag[i] = TRUE;
					PlanSlotCount = i;
					//count++;
				}
				break;
			}
		}
		else
		{
			rt_kprintf("[energycon]  (%s) ִ�г��ƻ�ʱʱ�������쳣��ʧ�ܣ�\n",__func__);		
		}
	}
	
	if(c_rst !=0 )
		CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//��ʧ�����ϱ�һ��
		
	if(PlanSlotCount == Chg_Strategy.ucTimeSlotNum)//��⵽ִ����ƻ�
	{
		memcpy(cRequestNO_Old,cRequestNO_New,sizeof(cRequestNO_Old));//���ٸ�׮���͹����趨֡
		memcpy(&Ctrl_ChgExe_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//ִ���¼�����ʱ��
		
		c_rst = CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//�ϱ����ƻ�ִ���¼�
		if(c_rst == SUCCESSFUL)
			rt_kprintf("[energycon]  (%s) Ctrl_ChgExe_Event,Cmd_ChgPlanExeState,The last one,Successful!\n",__func__);
		else
			rt_kprintf("[energycon]  (%s) ���һ���ϱ�ִ�г��ƻ���ʧ�ܣ�\n",__func__);
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
			ChgOrder_Event.ChargeMode = Chg_Apply_Event.ChargeMode;
		}
		
		//�����ѳ����
		for(i=0;i<5;i++)
			ChgOrder_Event.ucChargeEle[i] = ChgOrder_Event.StopMeterValue[i] - ChgOrder_Event.StartMeterValue[i]; 
		//�����ѳ�ʱ��
		time_t start_time = 0;
		rt_kprintf("[energycon]  (%s) calculate start_time!\n",__func__);
		start_time = TimeCalculation(ChgOrder_Event.ChgStartTime);
		rt_kprintf("[energycon]  (%s) get start_time!\n",__func__);
		time_t stop_time = 0;
		stop_time = TimeCalculation(ChgOrder_Event.ChgStopTime);
		ChgOrder_Event.ucChargeTime = stop_time - start_time;
		
		memcpy(&ChgOrder_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//�¼�����ʱ��
//		s_rst = SetStorageData(Cmd_HistoryRecordWr,&ChgOrder_Event,sizeof(CHG_ORDER_EVENT));
//		if(s_rst == SUCCESSFUL)
//		{
//			rt_kprintf("[energycon]  (%s) Storage ChgOrder_Event, Successful!\n",__func__);
//		}
//		else
//		{
//			rt_kprintf("[energycon]  (%s) �����綩���¼���ʧ�ܣ�\n",__func__);
//			SetStorageData(Cmd_HistoryRecordWr,&ChgOrder_Event,sizeof(CHG_ORDER_EVENT));//�ٴ�һ��
//		}

		c_rst = CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//�ϱ���綩��
		if(c_rst == SUCCESSFUL)
			rt_kprintf("[energycon]  (%s) ChgOrder_Event Apply to Contrllor, Successful!\n",__func__);
		else
			rt_kprintf("[energycon]  (%s) ��綩���¼�������Contrllor��ʧ�ܣ�\n",__func__);
		
		b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//ͬʱ���¼��ش�APP
		if(b_rst == SUCCESSFUL)
			rt_kprintf("[energycon]  (%s) ChgOrder_Event Apply to BLE, Successful!\n",__func__);
		else
			rt_kprintf("[energycon]  (%s) ��綩���¼�������BLE��ʧ�ܣ�\n",__func__);
	}
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
	rt_err_t res,p_rst;

	RouterIfo.WorkState = RtSt_StandbyOK;//��������
	
	
	rt_thread_mdelay(3000);
	
	GetStorageData(Cmd_MeterNumRd,&RouterIfo.AssetNum,13);
	rt_thread_mdelay(100);
	
	rt_pin_mode(RELAYA_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RELAYB_PIN, PIN_MODE_OUTPUT);
	RELAY_ON();//�ϵ����ϼ̵���
	while (1)
	{
		RtState_Judge();
		ChgOrder_Apply();
		CtrlData_RecProcess();
		PileData_RecProcess();	
		
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

int strategy_thread_init(void)
{
	rt_err_t res;
	
	/*��ʼ������*/
	RouterIfo.WorkState = RtSt_Starting;//������
	Chg_ExeState.exeState = EXE_NULL;
	memset(&Plan_Offer_Event,0,sizeof(PLAN_OFFER_EVENT));
	memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
	CtrlCharge_Event.CtrlType = CTRL_NULL;
	
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
		rt_kprintf("[strategy] : Strategy Initialized!\n");
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

