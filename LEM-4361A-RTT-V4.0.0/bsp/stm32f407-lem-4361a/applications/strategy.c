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
#define THREAD_STRATEGY_PRIORITY     8
#define THREAD_STRATEGY_STACK_SIZE   1024
#define THREAD_STRATEGY_TIMESLICE    5

#ifndef SUCCESSFUL
#define SUCCESSFUL    0
#endif
#ifndef FAILED
#define FAILED        1
#endif
#ifndef ORTHERS
#define ORTHERS       255
#endif

#define RELAYA_PIN    GET_PIN(F, 2)
#define RELAYB_PIN    GET_PIN(F, 3)


rt_uint8_t DeviceState;
rt_uint8_t ChgpileState;
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
};




void RELAY_ON(void)//吸合继电器
{
	rt_pin_write(RELAYA_PIN, PIN_LOW);
	rt_pin_write(RELAYB_PIN, PIN_HIGH);
}

void RELAY_OFF(void)//断开继电器
{
	rt_pin_write(RELAYB_PIN, PIN_LOW);
	rt_pin_write(RELAYA_PIN, PIN_HIGH);
}

//extern struct rt_event PowerCtrlEvent;
//extern struct rt_event HplcEvent;



//超时结果
static rt_timer_t StartChgResp;
static rt_timer_t StopChgResp;
static rt_timer_t PowerAdjResp;
	
static rt_uint8_t strategy_stack[THREAD_STRATEGY_STACK_SIZE];//线程堆栈
static struct rt_thread strategy;

ChargPilePara_TypeDef ChargePilePara_Set;
ChargPilePara_TypeDef ChargePilePara_Get;

CHARGE_STRATEGY Chg_Strategy;
CHARGE_STRATEGY_RSP Chg_StrategyRsp;
CHARGE_STRATEGY Adj_Chg_Strategy;
CHARGE_STRATEGY_RSP Adj_Chg_StrategyRsp;
CHARGE_EXE_STATE Chg_ExeState;

CTL_CHARGE Ctrl_Start;
CTL_CHARGE Ctrl_Stop;
CTL_CHARGE Ctrl_PowerAdj;
CTL_CHARGE_EVENT CtrlCharge_Event;

unsigned char PileWorkState;
rt_bool_t startchg_flag;
rt_bool_t stopchg_flag;
rt_bool_t adjpower_flag;
/**************************************************************
 * 函数名称: StartChgResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 启动充电超时函数
 **************************************************************/
static void StartChgResp_Timeout(void *parameter)
{
    rt_lprintf("StartChgResp event is timeout!\n");
	ChargepileDataGetSet(Cmd_ChargeStartResp,0);
}
/**************************************************************
 * 函数名称: StopChgResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 停止充电超时函数
 **************************************************************/
static void StopChgResp_Timeout(void *parameter)
{
    rt_lprintf("StopChgResp event is timeout!\n");
	ChargepileDataGetSet(Cmd_ChargeStopResp,0);
}
/**************************************************************
 * 函数名称: PowAdjResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 调整功率超时函数
 **************************************************************/
static void PowAdjResp_Timeout(void *parameter)
{
    rt_lprintf("PowerAdjResp event is timeout!\n");
	ChargepileDataGetSet(Cmd_SetPowerResp,0);
}
/**************************************************************
 * 函数名称: timer_create_init 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 定时器
 **************************************************************/
void timer_create_init()
{
    /* 创建启机回复定时器 */
	 StartChgResp = rt_timer_create("StartChgResp",  /* 定时器名字是 StartChgResp */
									StartChgResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* 一次性定时器 */
	/* 创建停机回复定时器 */
	 StopChgResp = rt_timer_create("StopChgResp",  /* 定时器名字是 StopChgResp */
									StopChgResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* 一次性定时器 */
	/* 创建调整功率回复定时器 */
	 PowerAdjResp = rt_timer_create("PowerAdjResp",  /* 定时器名字是 PowerAdjResp */
									PowAdjResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT); /* 一次性定时器 */
}
/*  */

/********************************************************************  
*	函 数 名: CtrlData_RecProcess()
*	功能说明: 控制器数据接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void CtrlData_RecProcess(void)
{
	rt_uint8_t c_rst;
	rt_uint32_t chgplanIssue,chgplanIssueAdj,startchg,stopchg;
	rt_uint32_t EventCmd;
	EventCmd = strategy_event_get();
	
	switch(EventCmd)
	{
		//收到充电计划
		case ChgPlanIssue_EVENT:
		{
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//取值
			if((Chg_Strategy.ucChargeMode == 1)&&(Chg_Strategy.ucDecType == 1))
				rt_sem_release(&rx_sem_energycon);
			
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
				rt_sem_release(&rx_sem_energycon);
			
			memcpy(&Adj_Chg_StrategyRsp,&Adj_Chg_Strategy,40);
			Chg_StrategyRsp.cSucIdle = 0;
			
			c_rst = CtrlUnit_RecResp(Cmd_ChgPlanAdjustAck,&Adj_Chg_StrategyRsp,0);//回复	
			break;
		}
		//收到查询工作状态的命令
		case AskState_EVENT:
		{
			rt_lprintf("[strategy]  (%s)  收到查询工作状态的命令  \n",__func__);  
				
			if((memcmp(Chg_ExeState.cRequestNO,Chg_Strategy.cRequestNO,22) == 0)
				||(memcpy(Chg_ExeState.cRequestNO,Adj_Chg_Strategy.cRequestNO,22) == 0))
			{
				memcpy(Chg_ExeState.cAssetNO,RouterIfo.AssetNum,22);
				
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
		
		
		//收到启动充电命令
		case StartChg_EVENT:
		{
			startchg_flag = TRUE;
			c_rst = CtrlUnit_RecResp(Cmd_StartChg,&Ctrl_Start,0);//取值
			rt_lprintf("[strategy]  (%s)  收到启动充电命令  \n",__func__);
			memcpy(&CtrlCharge_Event,&Ctrl_Start,41);
			CtrlCharge_Event.CtrlType = CTRL_START;
			
			if(Fault.Total != TRUE)
			{
				if(memcmp(&RouterIfo.AssetNum,&Ctrl_Start.cAssetNO,22) == 0)//校验资产一致性
				{
					ChargepileDataGetSet(Cmd_ChargeStart,0);	
				
					/* 开始启动回复计时 */
					if (StartChgResp != RT_NULL)
						rt_timer_start(StartChgResp);
					else
						rt_lprintf("StartChgResp timer create error\n");							
				}
				else
				{
					Ctrl_Start.cSucIdle = ORTHERS;
					c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//回复
				}			
			}
			else
			{
				Ctrl_Start.cSucIdle = FAILED;	
				c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//回复			
			}		
			CtrlCharge_Event.cSucIdle = Ctrl_Start.cSucIdle;			
			break;
		}
		//收到停止充电命令
		case StopChg_EVENT:
		{
			stopchg_flag = TRUE;	
			c_rst = CtrlUnit_RecResp(Cmd_StopChg,&Ctrl_Stop,0);//取值			
			rt_lprintf("[strategy]  (%s)  收到停止充电命令  \n",__func__); 
			memcpy(&CtrlCharge_Event,&Ctrl_Stop,41);			
			CtrlCharge_Event.CtrlType = CTRL_STOP;
			
			if(Fault.Total != TRUE)
			{
				if((memcmp(&RouterIfo.AssetNum,&Ctrl_Stop.cAssetNO,22) == 0)//校验资产一致性
					||(memcmp(&Ctrl_Start.OrderSn,&Ctrl_Stop.OrderSn,16) == 0))//校验启停单号一致性		
				{
					ChargepileDataGetSet(Cmd_ChargeStop,0);	
				
					/* 开始停机回复计时 */
					if (StopChgResp != RT_NULL)
						rt_timer_start(StopChgResp);
					else
						rt_lprintf("StopChgResp timer create error\n");
				}
				else
				{
					Ctrl_Stop.cSucIdle = ORTHERS;
					c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//回复
				}
			}
			else
			{
				Ctrl_Stop.cSucIdle = FAILED;
				c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//回复
			}
			CtrlCharge_Event.cSucIdle = Ctrl_Stop.cSucIdle;
			break;
		}
		//收到调整功率命令
		case PowerAdj_EVENT:
		{
			adjpower_flag = TRUE;
			c_rst = CtrlUnit_RecResp(Cmd_PowerAdj,&Ctrl_PowerAdj,0);//取值	
			rt_lprintf("[strategy]  (%s)  收到调整功率命令  \n",__func__);  
			memcpy(&CtrlCharge_Event,&Ctrl_PowerAdj,41);
			CtrlCharge_Event.CtrlType = CTRL_ADJPOW;
			
			if(Fault.Total != TRUE)
			{
				if(memcmp(&RouterIfo.AssetNum,&Ctrl_PowerAdj.cAssetNO,22) == 0)//校验资产一致性
				{
					ChargePilePara_Set.PWM_Duty = Ctrl_PowerAdj.SetPower*10/132;//功率换算: D(含一位小数)=I/60*1000=P/(60*220)*1000
					ChargepileDataGetSet(Cmd_SetPower,&ChargePilePara_Set);	
					
					/* 开始功率调整回复计时 */
					if (PowerAdjResp != RT_NULL)
						rt_timer_start(PowerAdjResp);
					else
						rt_lprintf("PowerAdjResp timer create error\n");
				}
				else
				{
					Ctrl_PowerAdj.cSucIdle = ORTHERS;
					c_rst = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);//回复
				}
			}
			else
			{
				Ctrl_PowerAdj.cSucIdle = FAILED;
				c_rst = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);//回复
			}
			Ctrl_Stop.cSucIdle = Ctrl_PowerAdj.cSucIdle;
			break;
		}

		default:
			break;
	}
}
/********************************************************************  
*	函 数 名: PileData_RecProcess()
*	功能说明: 电桩数据接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void PileData_RecProcess(void)
{
	rt_uint8_t c_rst,p_rst;
	rt_uint32_t start_result,stop_result,adjpow_result;
	
	if(startchg_flag == TRUE)
	{
		//启动成功
		if(rt_event_recv(&ChargePileEvent, ChargeStartOK_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &start_result) == RT_EOK)	
		{
			rt_timer_stop(StartChgResp);
			
			Ctrl_Start.cSucIdle = SUCCESSFUL;
			c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);
			
			if(c_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				startchg_flag = FALSE;//清位
				rt_lprintf("start charge successful!\n");
			}
		}
		//启动失败
		else if(rt_event_recv(&ChargePileEvent, ChargeStartER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &start_result) == RT_EOK)	
		{
			rt_timer_stop(StartChgResp);
			
			Ctrl_Start.cSucIdle = FAILED;
			p_rst = ChargepileDataGetSet(Cmd_ChargeStartResp,&ChargePilePara_Get);//获取失败原因
			
			if(p_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				startchg_flag = FALSE;//清位
				rt_lprintf("start charge failed,reason:%d!\n",ChargePilePara_Get.StartReson);
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);		
		}
		rt_lprintf("chargepile:ChargePileEvent 0x%02X\n", start_result);
	}
	
	if(stopchg_flag == TRUE)
	{
		//停机成功
		if(rt_event_recv(&ChargePileEvent, ChargeStopOK_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &stop_result) == RT_EOK)		
		{
			rt_timer_stop(StopChgResp);
			
			Ctrl_Stop.cSucIdle = SUCCESSFUL;
			c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);
						
			if(c_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				stopchg_flag = FALSE;//清位
				rt_lprintf("stop charge successful!\n");
			}
		}
		//停机失败
		else if(rt_event_recv(&ChargePileEvent, ChargeStopER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &stop_result) == RT_EOK)		
		{
			rt_timer_stop(StopChgResp);
			
			Ctrl_Start.cSucIdle = FAILED;
			p_rst = ChargepileDataGetSet(Cmd_ChargeStartResp,&ChargePilePara_Get);//获取失败原因
			
			if(p_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				stopchg_flag = FALSE;//清位
				rt_lprintf("stop charge failed,reason:%d!\n",ChargePilePara_Get.StopReson);
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);			
		}
		rt_lprintf("chargepile:ChargePileEvent 0x%02X\n", stop_result);
	}
	
	if(adjpower_flag == TRUE)
	{
		//调整功率成功
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
				adjpower_flag = FALSE;//清位
				rt_lprintf("start charge successful!\n");
			}
		}
		//调整功率失败
		else if(rt_event_recv(&ChargePileEvent, SetPowerER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &adjpow_result) == RT_EOK)	
		{
			rt_timer_stop(PowerAdjResp);
			
			Ctrl_PowerAdj.cSucIdle = FAILED;
			p_rst = ChargepileDataGetSet(Cmd_SetPowerResp,&ChargePilePara_Get);//获取失败原因
			
			if(p_rst != SUCCESSFUL)
			{
				
			}
			else
			{
				adjpower_flag = FALSE;//清位
				rt_lprintf("adjust power failed,reason:%d!\n",ChargePilePara_Get.AdjPowerReson);
			}
			
			c_rst = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);		
		}
		rt_lprintf("chargepile:ChargePileEvent 0x%02X\n", adjpow_result);
	}
}

/********************************************************************  
*	函 数 名: DevState_Judge()
*	功能说明: 路由器状态判断
*	形    参: 无
*	返 回 值: 故障代码
********************************************************************/ 
void DevState_Judge(void)
{
	if((System_Time_STR.Year > 50)||(System_Time_STR.Month > 12)||(System_Time_STR.Day > 31)
		||(System_Time_STR.Hour > 23)||(System_Time_STR.Minute > 60))
	{
		Fault.Bit.Clock_Fau = 1;				
		rt_lprintf("%s\r\n",(const char*)err_strfault[CLOCK_FAU]);
		
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
	rt_err_t res,fau;
	
	rt_pin_mode(RELAYA_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RELAYB_PIN, PIN_MODE_OUTPUT);
	RELAY_ON();//上电吸合继电器
	
	Fault.Total = FALSE;
	rt_thread_mdelay(100);
	
	while (1)
	{
		DevState_Judge();
		
		PileData_RecProcess();
		
		CtrlData_RecProcess();
		
		
		
		
						
		rt_thread_mdelay(1000);
	}
}

int strategy_thread_init(void)
{
	rt_err_t res;
	
	
	Chg_ExeState.exeState = EXE_NULL;
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



