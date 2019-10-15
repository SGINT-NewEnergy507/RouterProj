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

ChargPilePara_TypeDef ChargePilePara_Set;
ChargPilePara_TypeDef ChargePilePara_Get;

CHARGE_STRATEGY Chg_Strategy;
CHARGE_STRATEGY_RSP Chg_StrategyRsp;
CHARGE_STRATEGY Adj_Chg_Strategy;
CHARGE_STRATEGY_RSP Adj_Chg_StrategyRsp;
CHARGE_EXE_STATE Chg_ExeState;

CHARGE_APPLY Chg_Apply;
CHARGE_APPLY_RSP Chg_Apply_Rsp;
CHARGE_APPLY_EVENT Chg_Apply_Event;//（上送）

//超时结果
static rt_timer_t ChgReqReportRsp;
/**************************************************************
 * 函数名称: ChgReqReportResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 上送充电申请超时函数
 **************************************************************/
static void ChgReqReportResp_Timeout(void *parameter)
{
    rt_lprintf("[strategy] : ChgReqReportResp event is timeout!\n");
//	ChargepileDataGetSet(Cmd_ChargeStartResp,0);[改成记录事件]
}
/**************************************************************
 * 函数名称: timer_create_init 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 定时器
 **************************************************************/
static void timer_create_init()
{
    /* 创建充电申请上送回复定时器 */
	 ChgReqReportRsp = rt_timer_create("ChgReqReportRsp",  /* 定时器名字是 ChgReqReportRsp */
									ChgReqReportResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* 一次性定时器 */
}
/********************************************************************  
*	函 数 名: ChgPlan_RecProcess()
*	功能说明: 充电计划类数据_接收处理函数
*	形    参: 无
*	返 回 值: 无
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
		//收到充电计划
		case ChgPlanIssue_EVENT:
		{
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//取值
			if((Chg_Strategy.ucChargeMode == 1)&&(Chg_Strategy.ucDecType == 1))
				rt_sem_release(&rx_sem_setpower);
			
			memcpy(&Chg_StrategyRsp,&Chg_Strategy,40);
			Chg_StrategyRsp.cSucIdle = 0;
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,&Chg_StrategyRsp,0);//回复	
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
		//收到查询工作状态的命令
		case AskState_EVENT:
		{
			rt_lprintf("[strategy]  (%s)  收到查询工作状态的命令  \n",__func__);  
				
			if((memcmp(Chg_ExeState.cRequestNO,Chg_Strategy.cRequestNO,sizeof(Chg_ExeState.cRequestNO)) == 0)
				||(memcpy(Chg_ExeState.cRequestNO,Adj_Chg_Strategy.cRequestNO,sizeof(Chg_ExeState.cRequestNO)) == 0))
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
				Chg_ExeState.ucVoltage = stgMeter_Analog.ulVol;
				Chg_ExeState.ucCurrent = stgMeter_Analog.ulCur;
				
				ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
				Chg_ExeState.ChgPileState = ChargePilePara_Get.ChgPileState;
			}
			else
			{
				Chg_ExeState.exeState = EXE_FAILED;//申请单号不匹配，视为“执行失败"
			}
			
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanExeStateAck,&Chg_ExeState,0);//回复			   	
			break;
		}
		//收到充电申请确认
		case ChgReqReportConfirm_EVENT:
		{	
			b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestConfirm,0,0);//通知蓝牙	
			break;
		}		
		default:
			break;
	}
	Ctrl_EventCmd = 0;//清位
	
	//收到充电申请
	if(BLE_EventCmd == ChgRequest_EVENT)
	{
		memcpy(&Chg_Apply_Event.RequestTimeStamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//申请时间
		b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequest,&Chg_Apply,0);//取值
		if(b_rst == 0)
		{
			if(memcmp(&RouterIfo.AssetNum,&Chg_Apply.cAssetNO,sizeof(RouterIfo.AssetNum)) == 0)//校验资产一致性
			{
				Chg_Apply_Event.OrderNum++;
				memcpy(&Chg_Apply_Event.StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
				memcpy(&Chg_Apply_Event.FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
				memcpy(&Chg_Apply_Event.RequestNO,&Chg_Apply.cRequestNO,51);//连续三个字段			
				Chg_Apply_Event.ChargeReqEle = Chg_Apply.ulChargeReqEle;			
				memcpy(&Chg_Apply_Event.PlanUnChg_TimeStamp,&Chg_Apply.PlanUnChg_TimeStamp,sizeof(STR_SYSTEM_TIME));
				Chg_Apply_Event.ChargeMode = Chg_Apply.ChargeMode;
				
				if(Chg_Apply.cUserID[0]<=9)//防止溢出
				{
					memcpy(&Chg_Apply_Event.UserAccount[1],&Chg_Apply.cUserID[1],Chg_Apply.cUserID[0]);		
					Chg_Apply_Event.UserAccount[0] = Chg_Apply.cUserID[0];
				}
				
				c_rst = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//上送
				b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReportAPP,&Chg_Apply_Event,0);//同时回传APP

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
			}
		}
		

		b_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRequestAck,&Chg_Apply_Rsp,0);//回复	
	}
	BLE_EventCmd = 0;//清位
}

/********************************************************************  
*	函 数 名: RtState_Judge()
*	功能说明: 路由器状态判断
*	形    参: 无
*	返 回 值: 故障代码
********************************************************************/ 
static void RtState_Judge(void)
{
	if((System_Time_STR.Year > 50)||(System_Time_STR.Month > 12)||(System_Time_STR.Day > 31)
		||(System_Time_STR.Hour > 23)||(System_Time_STR.Minute > 60))
	{
		Fault.Bit.Clock_Fau = TRUE;				
		rt_lprintf("[strategy] : %s\r\n",(const char*)err_strfault[CLOCK_FAU]);
		
	}
	
//	if(rt_device_find("lcd"))
//	{
//		fau |= (1<<(Screen_FAULT-1));				
//		rt_lprintf("%s\r\n",(const char*)err_strfault[Screen_FAULT]);
//	}
	
//	if(rt_sem_trytake(&rt_sem_meterfau) == RT_EOK)
//	{
//		fau |= (1<<(MeterCom_FAULT-1));
//		rt_lprintf("%s\r\n",(const char*)err_strfault[MeterCom_FAULT]);
//	}
//	
//	if(rt_sem_trytake(&rt_sem_nandfau) == RT_EOK)
//	{
//		fau |= (1<<(NandF_FAULT-1));		
//		rt_lprintf("%s\r\n",(const char*)err_strfault[NandF_FAULT]);
//	}
	
//	if(rt_sem_trytake(&rt_sem_bluetoothfau) == RT_EOK)
//	{
//		fau |= (1<<(Bluetooth_FAULT-1));
//		rt_lprintf("%s\r\n",(const char*)err_strfault[Bluetooth_FAULT]);
//	}	
}

static void strategy_thread_entry(void *parameter)
{
	rt_err_t res;
	
	Fault.Total = FALSE;
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
	
	
	Chg_ExeState.exeState = EXE_NULL;
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



