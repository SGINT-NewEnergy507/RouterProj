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
//定义故障原因
static char *err_strfault[] = 
{
           "                       ",            /* ERR_OK          0  */
           "终端主板内存故障！   ",               /* ERR             1  */
           "时钟故障！         ",                 /* ERR             2  */
           "主板通信故障！	       ",            /* ERR             3  */
           "485抄表故障！         ",              /* ERR             4  */
           "显示板故障！         ",               /* ERR             5  */
           "载波通道异常！       ",               /* ERR             6  */
           "NandFLASH初始化错误！       ",        /* ERR             7  */
           "ESAM错误！         ",                 /* ERR             8  */
           "蓝牙模块故障！         ",             /* ERR             9  */
           "电源模块故障！         ",             /* ERR             10 */
           "充电桩通信故障！        ",            /* ERR             11 */
           "充电桩设备故障！         ",           /* ERR             12 */
	       "本地订单记录满！       ",             /* ERR             13 */
		   "RTC通信故障！       ",             	 /* ERR             14 */
};

	
static rt_uint8_t strategy_stack[THREAD_STRATEGY_STACK_SIZE];//线程堆栈
static struct rt_thread strategy;

CCMRAM ChargPilePara_TypeDef ChargePilePara_Set;
CCMRAM ChargPilePara_TypeDef ChargePilePara_Get;

CCMRAM static CHARGE_EXE_STATE_ASK ExeState_CtrlAsk;//控制器
CCMRAM static CHARGE_EXE_STATE_ASK ExeState_BleAsk;//蓝牙

CCMRAM CHARGE_STRATEGY Chg_Strategy;
CCMRAM static CHARGE_STRATEGY_RSP Chg_StrategyRsp;
CCMRAM CHARGE_STRATEGY Adj_Chg_Strategy;
CCMRAM static CHARGE_STRATEGY_RSP Adj_Chg_StrategyRsp;

CCMRAM static CHARGE_APPLY Chg_Apply;
CCMRAM static CHARGE_APPLY_RSP Chg_Apply_Rsp;

//上送事件
CCMRAM PLAN_OFFER_EVENT Plan_Offer_Event;
CCMRAM CHARGE_APPLY_EVENT Chg_Apply_Event;
CCMRAM static CHARGE_EXE_EVENT BLE_ChgExe_Event;
	


//超时结果
CCMRAM static rt_timer_t ChgReqReportRsp;
//定时轮训
CCMRAM static rt_timer_t ChgPileStateGet;
/**************************************************************
 * 函数名称: ChgReqReportResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 上送充电申请超时函数
 **************************************************************/
static void ChgReqReportResp_Timeout(void *parameter)
{
    rt_lprintf("[strategy] : ChgReqReportResp event is timeout!\n");
	
}
/**************************************************************
 * 函数名称: ChgReqReportResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 轮训充电桩状态函数
 **************************************************************/
static void ChgPileStateGet_Timeout(void *parameter)
{
    rt_lprintf("[strategy] : ChgPileStateGet event is timeout!\n");
	//查询充电桩状态
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
	
//	if(ChargePilePara_Get.ChgPileState == )
}
/**************************************************************
 * 函数名称: timer_create_init 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 定时器
 **************************************************************/
static void timer_create_init()
{
    /* 创建 充电申请上送回复 定时器 */
	 ChgReqReportRsp = rt_timer_create("ChgReqReportRsp",  /* 定时器名字是 ChgReqReportRsp */
									ChgReqReportResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* 一次性定时器 */
	/* 创建 充电桩状态查询 定时器 */
	ChgPileStateGet = rt_timer_create("ChgPileStateGet",  /* 定时器名字是 ChgPileStateGet */
									ChgPileStateGet_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									1000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* 一次性定时器 */
	/* 启动定时器 */
	if (ChgPileStateGet != RT_NULL)
		rt_timer_start(ChgPileStateGet);
}
/********************************************************************  
*	函 数 名: ExeState_Update()
*	功能说明: 路由器充电执行状态更新
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void ExeState_Update(void)
{	
	memcpy(Chg_ExeState.cRequestNO,Chg_Apply_Event.RequestNO,sizeof(Chg_ExeState.cRequestNO));
	memcpy(Chg_ExeState.cAssetNO,RouterIfo.AssetNum,sizeof(Chg_ExeState.cAssetNO));
	Chg_ExeState.GunNum = ExeState_BleAsk.GunNum;
	
	if(Chg_ExeState.exeState != EXE_ING)//非执行过程中，按计划中额定功率传
		Chg_ExeState.ucPlanPower = Chg_Strategy.ulChargeRatePow;
	
	ScmMeter_HisData stgMeter_HisData;
	cmMeter_get_data(EMMETER_HISDATA,&stgMeter_HisData);//获取电表计量数据
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
*	函 数 名: ChgPlan_RecProcess()
*	功能说明: 充电计划类数据_接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void ChgPlan_RecProcess(void)
{
	rt_uint8_t c_rst,b_rst,p_rst,s_rst;
	rt_uint32_t chgplanIssue,chgplanIssueAdj,startchg,stopchg;
	rt_uint32_t Ctrl_EventCmd,BLE_EventCmd;
	
	/***************************** 接收控制器命令 *******************************/
	Ctrl_EventCmd = strategy_event_get();
	switch(Ctrl_EventCmd)
	{
		//收到充电计划
		case ChgPlanIssue_EVENT:
		{
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//取值
			
			if((Chg_Strategy.ucChargeMode == 1)&&(Chg_Strategy.ucDecType == 1))
				rt_sem_release(&rx_sem_setpower);
			
			memcpy(&Chg_StrategyRsp,&Chg_Strategy,40);
			Chg_StrategyRsp.cSucIdle = 0;
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,&Chg_StrategyRsp,0);//回复	
			
			if(c_rst == 0)
			{
				memcpy(&Plan_Offer_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//记录上报时间
				SetStorageData(Cmd_PlanOfferWr,&Plan_Offer_Event,sizeof(PLAN_OFFER_EVENT));
				//回复成功上送事件
				Plan_Offer_Event.OrderNum++;		
				Plan_Offer_Event.ChannelState = 0;//通道状态
				memcpy(&Plan_Offer_Event.Chg_Strategy,&Chg_Strategy,sizeof(CHARGE_STRATEGY));
				memcpy(&Plan_Offer_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
				c_rst = CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//上报充电计划时间
				
				//缺等待确认		
			}
			break;
		}
		//收到充电计划调整
		case ChgPlanAdjust_EVENT:
		{
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanAdjust,&Adj_Chg_Strategy,0);//取值	
			if((Chg_Strategy.ucChargeMode == 1)&&(Chg_Strategy.ucDecType == 2))
				rt_sem_release(&rx_sem_adjpower);
			
			memcpy(&Adj_Chg_StrategyRsp,&Adj_Chg_Strategy,40);
			Chg_StrategyRsp.cSucIdle = 0;
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanAdjustAck,&Adj_Chg_StrategyRsp,0);//回复	
			break;
		}
		//收到控制器抄读路由器工作状态
		case AskState_EVENT:
		{
			rt_lprintf("[strategy]  (%s)  收到@控制器@查询工作状态的命令  \n",__func__);  
				
			if(memcmp(ExeState_CtrlAsk.cAssetNO,RouterIfo.AssetNum,sizeof(ExeState_CtrlAsk.cAssetNO)) == 0)
			{
				memcpy(Chg_ExeState.cAssetNO,RouterIfo.AssetNum,sizeof(Chg_ExeState.cRequestNO));
				
				if(Chg_ExeState.exeState != EXE_ING)//非执行过程中，按计划中额定功率传
					Chg_ExeState.ucPlanPower = Chg_Strategy.ulChargeRatePow;
				
				ScmMeter_HisData stgMeter_HisData;
				cmMeter_get_data(EMMETER_HISDATA,&stgMeter_HisData);//获取电表计量数据
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
				Chg_ExeState.exeState = EXE_FAILED;//申请单号不匹配，视为“执行失败"
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_RouterExeState,&Chg_ExeState,0);//回复			   	
			break;
		}
		//收到充电申请确认
		case ChgReqReportConfirm_EVENT:
		{	
				
			break;
		}		
		default:
			break;
	}
	Ctrl_EventCmd = 0;//清位
	
	/***************************** 接收APP命令 *******************************/
	BLE_EventCmd = Strategy_get_BLE_event();
	switch(BLE_EventCmd)
	{
		//充电申请    然后直接启动充电，启动后立即上送执行事件
		case ChgRequest_EVENT:
		{	
			rt_lprintf("[strategy]  (%s) <ChgRequest_EVENT> 收到@蓝牙@ 充电申请 的命令\n",__func__); 
			memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
			memset(&Chg_Apply_Rsp,0,sizeof(CHARGE_APPLY_RSP));
			memset(&BLE_ChgExe_Event,0,sizeof(CHARGE_EXE_EVENT));
			
			memcpy(&Chg_Apply_Event.RequestTimeStamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//充电申请时间
			memcpy(&Chg_Apply_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件发生时间
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequest,&Chg_Apply,0);//取值
			
			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Receive the data of Chg_Apply from BLE, Successful!\n",__func__);
				if(memcmp(&RouterIfo.AssetNum,&Chg_Apply.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//校验资产一致性
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
					memcpy(&Chg_Apply_Event.Token,&Chg_Apply.Token,sizeof(Chg_Apply.Token));		//按32有效数取值

					/* 充电申请上送回复计时 */
					if (ChgReqReportRsp != RT_NULL)
						rt_timer_start(ChgReqReportRsp);
					else
						rt_lprintf("ChgReqReportResp timer create error\n");

					Chg_Apply_Rsp.cSucIdle = SUCCESSFUL;
				}
				else
				{				
					Chg_Apply_Rsp.cSucIdle = FAILED;
					rt_lprintf("[strategy]  (%s) 校验资产一致性，失败！\n",__func__);
				}
				memcpy(&Chg_Apply_Rsp.cRequestNO,&Chg_Apply.cRequestNO,sizeof(Chg_Apply.cRequestNO));
				memcpy(&Chg_Apply_Rsp.cAssetNO,&Chg_Apply.cAssetNO,sizeof(Chg_Apply.cAssetNO));
				Chg_Apply_Rsp.GunNum = Chg_Apply.GunNum;
			}
			else
			{
				rt_lprintf("[strategy]  (%s) 获取BLE传输的数据，失败！\n",__func__);
			}
		
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestAck,&Chg_Apply_Rsp,0);//回复	
			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Chg_Apply Response, Successful!\n",__func__);				
			}
			else
			{
				rt_lprintf("[strategy]  (%s) 回复BLE充电申请确认响应，失败！\n",__func__);
			}
			
			
			//申请事件处理
			s_rst = SetStorageData(Cmd_ChgRequestWr,&Chg_Apply_Event,sizeof(CHARGE_APPLY_EVENT));
			if(s_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Storage Chg_Apply_Event, Successful!\n",__func__);
			}
			else
			{
				rt_lprintf("[strategy]  (%s) 保存 BLE申请事件，失败！\n",__func__);
				SetStorageData(Cmd_ChgRequestWr,&Chg_Apply_Event,sizeof(CHARGE_APPLY_EVENT));//再存一次
			}
			memcpy(&Chg_Apply_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件结束时间
//			c_rst = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//上送事件
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReportAPP,&Chg_Apply_Event,0);//同时将事件回传APP
			
			//暂定立即启动充电
			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Charge_Apply Response to BLE, Successful!\n",__func__);
				CtrlCharge_Event.CtrlType = CTRL_START;
				CtrlCharge_Event.StartSource = BLE_UNIT;
				p_rst = ChargepileDataGetSet(Cmd_ChargeStart,0);
				memcpy(&BLE_ChgExe_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件发生时间
				if(p_rst == SUCCESSFUL)
				{
					PileIfo.WorkState = ChgSt_InCharging;//此处优先置位，下发启动成功视为充电桩已启动
					rt_lprintf("[strategy]  (%s) Charge Started, Successful!\n",__func__);				
				}
				else
				{
					rt_lprintf("[strategy]  (%s) BLE申请后立即启动充电，失败！\n",__func__);
				}
				
				//上送充电执行事件
				BLE_ChgExe_Event.OrderNum++;
				BLE_ChgExe_Event.OccurSource = 0;
				ExeState_Update();
				memcpy(&BLE_ChgExe_Event.Chg_ExeState,&Chg_ExeState,sizeof(CHARGE_EXE_STATE));
				BLE_ChgExe_Event.Chg_ExeState.GunNum = Chg_Apply.GunNum;//注：这儿取值不同
			
				memcpy(&BLE_ChgExe_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件结束时间
				//存储充电执行事件
				s_rst = SetStorageData(Cmd_ChgExecuteWr,&BLE_ChgExe_Event,sizeof(CHARGE_EXE_EVENT));
				if(s_rst == SUCCESSFUL)
				{
					rt_lprintf("[energycon]  (%s) Storage BLE_ChgExe_Event, Successful!\n",__func__);
				}
				else
				{
					rt_lprintf("[energycon]  (%s) 保存充电执行事件，失败！\n",__func__);
					SetStorageData(Cmd_ChgExecuteWr,&BLE_ChgExe_Event,sizeof(CHARGE_EXE_EVENT));//再存一次
				}

				b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeState,&BLE_ChgExe_Event,0);
				if(b_rst == SUCCESSFUL)
				{
					rt_lprintf("[strategy]  (%s) BLE_ChgExe_Event Apply, Successful!\n",__func__);				
				}
				else
				{
					rt_lprintf("[strategy]  (%s) 回复BLE充电执行事件，失败！\n",__func__);
				}
			}	
			else
			{
				rt_lprintf("[strategy]  (%s) 回复BLE充电申请事件，失败！\n",__func__);
			}	
			break;
		}
		//收到蓝牙抄读路由器工作状态
		case AskState_EVENT:
		{
			rt_lprintf("[strategy]  (%s)  <AskState_EVENT> 收到 @蓝牙@ 查询工作状态 的命令  \n",__func__,BLE_EventCmd);  				
			
			ExeState_Update();				
			
			b_rst = BLE_CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,0);//回复

			if(b_rst == SUCCESSFUL)
			{
				rt_lprintf("[strategy]  (%s) Chg_ExeState Response to BLE, Successful!\n",__func__);
			}
			else
			{
				rt_lprintf("[strategy]  (%s) 上送给BLE路由器工作状态，失败！\n",__func__);
			}
			break;
		}
		default:
			break;
	}
	BLE_EventCmd = 0;//清位
}

/********************************************************************  
*	函 数 名: RtState_Judge()
*	功能说明: 路由器和充电桩状态判断
*	形    参: 无
*	返 回 值: 故障代码
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
			RouterIfo.WorkState = RtSt_StandbyOK;//充电结束回待机
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

	RouterIfo.WorkState = RtSt_StandbyOK;//待机正常
	
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
	
	/*初始化变量*/
	RouterIfo.WorkState = RtSt_Starting;//开机中
	Chg_ExeState.exeState = EXE_NULL;
	memset(&Plan_Offer_Event,0,sizeof(PLAN_OFFER_EVENT));
	memset(&Chg_Apply_Event,0,sizeof(CHARGE_APPLY_EVENT));
	
	/* 初始化定时器 */
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



