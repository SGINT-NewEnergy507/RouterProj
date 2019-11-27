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

static rt_uint8_t strategy_stack[THREAD_STRATEGY_STACK_SIZE];//线程堆栈
static struct rt_thread strategy;

//static rt_mutex_t strategy_mutex = RT_NULL;

#define RELAYA_PIN    GET_PIN(F, 2)
#define RELAYB_PIN    GET_PIN(F, 3)

#define KC1_PIN     GET_PIN(F, 4)
#define KC2_PIN     GET_PIN(F, 5)
#define KC3_PIN     GET_PIN(F, 6)

extern rt_uint32_t Strategy_get_BLE_event(void);
extern rt_uint8_t BLE_CtrlUnit_RecResp(COMM_CMD_C cmd,void *STR_SetPara,int count);
////////////////////////////// 变量定义区 ////////////////////////////////
//故障原因
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

//const char* _hplc_event_char[]={//698 事件名称   打印日志用
//	"null",
//	"Charge_Request",							//蓝牙充电申请
//	"Charge_Request_Ack",						//蓝牙充电申请应答
//	
//	"Charge_PlanIssue", 					//充电计划下发
//	"Charge_PlanIssue_Ack",                 	//充电计划下发应答
//	"Charge_Plan_Offer", 						//充电计划事件上报
//	"Charge_Plan_Offer_Ack",                 	//充电计划上报事件应答
//	
//	"Charge_Plan_Adjust",                 		//充电计划调整
//	"Charge_Plan_Adjust_Ack",                 	//充电计划调整应答


//	"Charge_Request_Report",					//充电申请事件上送
//	"Charge_Request_Report_Ack",				//充电申请事件上送应答
////	"Charge_Request_Report_APP",				//充电申请事件告知APP
////	"Charge_Request_Confirm",					//充电申请确认（通知蓝牙）
//	
//	"Charge_Plan_Exe_State",                    //充电计划执行状态事件上报
//	"Charge_Plan_Exe_State_Ack",                 //充电计划执行状态事件上报应答
//	
//	"Start_Charge",							//启动充电参数下发
//	"Start_Charge_Ack",						//启动充电应答
//	"Stop_Charge",							//停止充电参数下发
//	"Stop_Charge_Ack",							//停止充电应答
//	"Power_Adj",							//功率调节参数下发
//	"Power_Adj_Ack",						//功率调节应答

//	"Charge_Record",							//上送充电订单
//	"Charge_RecordAck",						//上送充电订单事件确认
//	"Device_Fault",                      	//上送路由器异常状态
//	"Device_FaultAck",                      	//上送路由器异常状态
//	"Pile_Fault",                 			//上送充电桩异常状态
//	"Pile_FaultAck",                 			//上送充电桩异常状态
//	"Charge_Plan_Issue_Get_Ack",
//	
//	"Read_Router_State",                    	//路由器执行状态查询
//	"Read_Router_State_Ack",                 	//路由器执行状态应答
//	
//	"STAOnlineState",						//STA监测自身及路由器在线状态↓
//	"STAOnlineStateAck",					//STA监测自身及路由器在线状态确认↑
//};//业务传输流程命令号


CCMRAM ChargPilePara_TypeDef ChargePilePara_Set;
CCMRAM ChargPilePara_TypeDef ChargePilePara_Get;

CCMRAM static CHARGE_EXE_STATE_ASK ExeState_CtrlAsk;//控制器
//CCMRAM static CHARGE_EXE_STATE_ASK ExeState_BleAsk;//蓝牙

CCMRAM static CHARGE_STRATEGY Chg_Strategy;
CCMRAM static CHARGE_STRATEGY_RSP Chg_StrategyRsp;
CCMRAM static CHARGE_STRATEGY Chg_Strategy_Adj;
CCMRAM static CHARGE_STRATEGY_RSP Chg_StrategyRsp_Adj;

CCMRAM static CHARGE_APPLY Chg_Apply;
CCMRAM static CHARGE_APPLY_RSP Chg_Apply_Rsp;

//上送事件
CCMRAM static PLAN_OFFER_EVENT Plan_Offer_Event;
CCMRAM static CHARGE_APPLY_EVENT Chg_Apply_Event;
//CCMRAM static CHARGE_EXE_EVENT BLE_ChgExe_Event;
//表计在线状态变化事件
CCMRAM static ONLINE_STATE OnlineState_Event;	

CCMRAM static ROUTER_FAULT_EVENT Router_Pile_Fault_Event;

//超时处理
CCMRAM static rt_timer_t StartChgResp_Timer;
CCMRAM static rt_timer_t StopChgResp_Timer;
CCMRAM static rt_timer_t PowerAdjResp_Timer;

//定时轮训
CCMRAM static rt_timer_t ChgPileStateGet_Timer;
//等待上报事件确认
CCMRAM static rt_timer_t HPLC_ChgReqReportAck_Timer;
CCMRAM static rt_timer_t HPLC_ChgPlanOfferAck_Timer;
CCMRAM static rt_timer_t HPLC_ChgPlanExeStateAck_Timer;
CCMRAM static rt_timer_t HPLC_ChgRecordAck_Timer;

//等待上报事件确认
CCMRAM static rt_timer_t BLE_ChgReqReportAck_Timer;
CCMRAM static rt_timer_t BLE_ChgPlanOfferAck_Timer;
CCMRAM static rt_timer_t BLE_ChgPlanExeStateAck_Timer;
CCMRAM static rt_timer_t BLE_ChgRecordAck_Timer;


CCMRAM static CTL_CHARGE Ctrl_Start;//包括返回参数
CCMRAM static CTL_CHARGE Ctrl_Stop;
CCMRAM static CTL_CHARGE BLE_Stop;
CCMRAM static CTL_CHARGE Ctrl_PowerAdj;

//充电执行状态
CCMRAM static CHARGE_EXE_STATE Chg_ExeState;

CCMRAM static CHARGE_EXE_EVENT Ctrl_ChgExe_Event;
CCMRAM static CHG_ORDER_EVENT ChgOrder_Event;//订单共用一个变量

CCMRAM static unsigned char PlanSlotCount;
CCMRAM static unsigned char PlanExeNum;
CCMRAM static unsigned char PlanExeNum_Old;
CCMRAM static rt_uint32_t PlanSolts_ChargeTime[50];//时段充电时长
CCMRAM static rt_uint32_t PlanSolts_ChargeEle[50][5];//时段充电电量

//CCMRAM static unsigned char SetPowerFinishFlag[50];
//CCMRAM static char cRequestNO_Old[17];
//CCMRAM static char cRequestNO_New[17];


CCMRAM ScmMeter_HisData stMeter_HisData_Strategy;


//CCMRAM rt_uint32_t* Charge_Event_Ptr[] = {
//	0,
//	(rt_uint32_t*)&Chg_Apply,//充电申请
//	(rt_uint32_t*)&Chg_Apply,//充电申请响应
//	(rt_uint32_t*)&Chg_Strategy,//充电计划下发
//	(rt_uint32_t*)&Chg_StrategyRsp,//充电计划下发应答
//	(rt_uint32_t*)&Plan_Offer_Event,//充电计划事件上报
//	(rt_uint32_t*)&Plan_Offer_Event,//充电计划上报事件应答
//	(rt_uint32_t*)&Chg_Strategy_Adj,//充电计划调整
//	(rt_uint32_t*)&Chg_StrategyRsp_Adj,//充电计划调整应答
//	(rt_uint32_t*)&Chg_Apply_Event,//充电申请事件上送
//	(rt_uint32_t*)&Chg_Apply_Event,//充电申请事件上送应答
//	(rt_uint32_t*)&Chg_Apply_Event,//充电申请事件上送
//	
//	(rt_uint32_t*)&Plan_Offer_Event,//充电计划执行状态事件上报
//	(rt_uint32_t*)&Plan_Offer_Event,//充电计划执行状态事件上报应答
//	(rt_uint32_t*)&Ctrl_Start,//启动服务
//	(rt_uint32_t*)&Ctrl_Start,//启动服务应答
//	(rt_uint32_t*)&Ctrl_Stop,//停止服务
//	(rt_uint32_t*)&Ctrl_Stop,//停止服务应答
//	(rt_uint32_t*)&Ctrl_PowerAdj,//功率调节参数下发
//	(rt_uint32_t*)&Ctrl_PowerAdj,//功率调节应答
//	(rt_uint32_t*)&ChgOrder_Event,//上送充电订单事件
//	(rt_uint32_t*)&ChgOrder_Event,//上送充电订单事件确认
//	0,														//上送路由器异常状态
//	0,														//上送充电桩异常状态
//	0, 														//Cmd_ChgPlanIssueGetAck
//	(rt_uint32_t*)&Chg_ExeState,	//路由器执行状态查询↓
//	(rt_uint32_t*)&Chg_ExeState,//路由器执行状态应答↑
//	(rt_uint32_t*)&OnlineState_Event,//STA监测自身及路由器在线状态↓
//	(rt_uint32_t*)&OnlineState_Event,//STA监测自身及路由器在线状态确认↑
//	(rt_uint32_t*)&OnlineState_Event,//在线状态通知APP	
//};




/********************************************************************  
*	函 数 名: TimeCalculation
*	功能说明: 时间计算
*	形    参: 无
*	返 回 值: 无
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
 * 函数名称: RELAY_ON 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 吸合继电器
 **************************************************************/
 void RELAY_ON(void)
{
	rt_pin_write(RELAYB_PIN, PIN_LOW);
	rt_thread_mdelay(200);
	rt_pin_write(RELAYA_PIN, PIN_HIGH);
}
/**************************************************************
 * 函数名称: RELAY_OFF 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 断开继电器
 **************************************************************/
void RELAY_OFF(void)
{
	rt_pin_write(RELAYA_PIN, PIN_LOW);
	rt_thread_mdelay(200);
	rt_pin_write(RELAYB_PIN, PIN_HIGH);
}
/**************************************************************
 * 函数名称: StartChgResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 启动充电超时函数
 **************************************************************/
static void StartChgResp_Timeout(void *parameter)
{
	rt_int8_t p_rst;
	rt_lprintf("[strategy]: StartChgResp event is timeout!\n");
	p_rst = ChargepileDataGetSet(Cmd_ChargeStartResp,0);
	
//	if(p_rst != SUCCESSFUL)
	
}
/**************************************************************
 * 函数名称: StopChgResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 停止充电超时函数
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
 * 函数名称: PowAdjResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 调整功率超时函数
 **************************************************************/
static void PowAdjResp_Timeout(void *parameter)
{
    rt_uint8_t p_rst;
	rt_lprintf("[strategy]: PowerAdjResp event is timeout!\n");
	p_rst = ChargepileDataGetSet(Cmd_SetPowerResp,0);
}

/**************************************************************
 * 函数名称: ChgReqReportResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 轮训充电桩状态函数
 **************************************************************/
static void ChgPileStateGet_Timeout(void *parameter)
{
    rt_lprintf("[strategy]: ChgPileStateGet event is timeout!\n");
	//查询充电桩状态
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
}
/**************************************************************
 * 函数名称: ChgReqReportResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 上送充电申请超时函数
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgReqReportAck_count;
static void HPLC_ChgReqReportAck_Timeout(void *parameter)
{
	rt_int8_t res;

	res = CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//通知控制器
	
	//等待确认
	if(res == SUCCESSFUL)
	{	
		HPLC_ChgReqReportAck_count++;
	}
		
	if(HPLC_ChgReqReportAck_count>=3)
		rt_timer_stop(HPLC_ChgReqReportAck_Timer);
	
	rt_kprintf("[strategy]: HPLC_ChgReqReport event is timeout! count = %d\n",HPLC_ChgReqReportAck_count);
}
/**************************************************************
 * 函数名称: ChgPlanOfferAck_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 等待充电计划确认函数
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgPlanOfferAck_count;
static void HPLC_ChgPlanOfferAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;
	
	c_rst = CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//上报充电计划事件
	
	//等待确认
	if(c_rst == SUCCESSFUL)
	{	
		HPLC_ChgPlanOfferAck_count++;
	}
		
	if(HPLC_ChgPlanOfferAck_count>=3)
		rt_timer_stop(HPLC_ChgPlanOfferAck_Timer);
	
	rt_kprintf("[strategy]: HPLC_ChgPlanOffer event is timeout! count = %d\n",HPLC_ChgPlanOfferAck_count);
}

/**************************************************************
 * 函数名称: ChgPlanExeStateAck_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 等待充电计划执行状态确认函数
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgPlanExeStateAck_count;
static void HPLC_ChgPlanExeStateAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//上报充电计划执行事件
	
	//等待确认
	if(c_rst == SUCCESSFUL)
	{	
		HPLC_ChgPlanExeStateAck_count++;
	}
		
	if(HPLC_ChgPlanExeStateAck_count>=3)
		rt_timer_stop(HPLC_ChgPlanExeStateAck_Timer);
	
	rt_kprintf("[strategy]: HPLC_ChgPlanExeState event is timeout! count = %d\n",HPLC_ChgPlanExeStateAck_count);
}
/**************************************************************
 * 函数名称: ChgRecordAck_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 等待充电订单确认函数
 **************************************************************/
CCMRAM static rt_uint8_t HPLC_ChgRecordAck_count;
static void HPLC_ChgRecordAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//上报充电订单
	
	//等待确认
	if(c_rst == SUCCESSFUL)
	{	
		HPLC_ChgRecordAck_count++;
	}
		
	if(HPLC_ChgRecordAck_count>=3)
		rt_timer_stop(HPLC_ChgRecordAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgRecord event is timeout! count = %d\n",HPLC_ChgRecordAck_count);
}

/**************************************************************
 * 函数名称: ChgReqReportResp_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 上送充电申请超时函数
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgReqReportAck_count;
static void BLE_ChgReqReportAck_Timeout(void *parameter)
{
	rt_int8_t res;

	res = BLE_CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//同时将事件回传APP
	
	//等待确认
	if(res == SUCCESSFUL)
	{	
		BLE_ChgReqReportAck_count++;
	}
		
	if(BLE_ChgReqReportAck_count>=3)
		rt_timer_stop(BLE_ChgReqReportAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgReqReport event is timeout! count = %d\n",BLE_ChgReqReportAck_count);
}

/**************************************************************
 * 函数名称: ChgPlanOfferAck_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 等待充电计划确认函数
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgPlanOfferAck_count;
static void BLE_ChgPlanOfferAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = BLE_CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//上报充电计划事件
	
	//等待确认
	if(c_rst == SUCCESSFUL)
	{	
		BLE_ChgPlanOfferAck_count++;
	}
		
	if(BLE_ChgPlanOfferAck_count>=3)
		rt_timer_stop(BLE_ChgPlanOfferAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgPlanOffer event is timeout! count = %d\n",BLE_ChgPlanOfferAck_count);
}

/**************************************************************
 * 函数名称: ChgPlanExeStateAck_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 等待充电计划执行状态确认函数
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgPlanExeStateAck_count;
static void BLE_ChgPlanExeStateAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;

	c_rst = BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//同时将事件回传APP
	
	//等待确认
	if(c_rst == SUCCESSFUL)
	{	
		BLE_ChgPlanExeStateAck_count++;
	}
		
	if(BLE_ChgPlanExeStateAck_count>=3)
		rt_timer_stop(BLE_ChgPlanExeStateAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgPlanExeState event is timeout! count = %d\n",BLE_ChgPlanExeStateAck_count);
}
/**************************************************************
 * 函数名称: ChgRecordAck_Timeout 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 等待充电订单确认函数
 **************************************************************/
CCMRAM static rt_uint8_t BLE_ChgRecordAck_count;
static void BLE_ChgRecordAck_Timeout(void *parameter)
{
	rt_int8_t c_rst;
	
	c_rst = BLE_CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//同时将事件回传APP

	//等待确认
	if(c_rst == SUCCESSFUL)
	{	
		BLE_ChgRecordAck_count++;
	}
		
	if(BLE_ChgRecordAck_count>=3)
		rt_timer_stop(BLE_ChgRecordAck_Timer);
	
	rt_kprintf("[strategy]: BLE_ChgRecord event is timeout! count = %d\n",BLE_ChgRecordAck_count);
}

/**************************************************************
 * 函数名称: timer_create_init 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 定时器
 **************************************************************/
static void timer_create_init(void)
{
     /* 创建启机回复定时器 */
	 StartChgResp_Timer = rt_timer_create("StartChgResp",  /* 定时器名字是 StartChgResp */
									StartChgResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* 一次性定时器 */
	/* 创建停机回复定时器 */
	 StopChgResp_Timer = rt_timer_create("StopChgResp",  /* 定时器名字是 StopChgResp */
									StopChgResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* 一次性定时器 */
	/* 创建调整功率回复定时器 */
	 PowerAdjResp_Timer = rt_timer_create("PowerAdjResp",  /* 定时器名字是 PowerAdjResp */
									PowAdjResp_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									5000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* 一次性定时器 */
							
	/* 创建 充电桩状态查询 定时器 */
		ChgPileStateGet_Timer = rt_timer_create("ChgPileStateGet",  /* 定时器名字是 ChgPileStateGet */
									ChgPileStateGet_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									1000, /* 定时长度，以OS Tick为单位，即1000个OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* 周期性定时器 */
									
									
		/* 创建 充电申请上送回复 定时器 */
		HPLC_ChgReqReportAck_Timer = rt_timer_create("HPLC_ChgReqReportAck",  /* 定时器名字是 ChgReqReportRsp */
									HPLC_ChgReqReportAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									10000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* 一次性定时器 */
									
									
	/* 创建 充电计划确认 定时器 */
		HPLC_ChgPlanOfferAck_Timer = rt_timer_create("HPLC_ChgPlanOfferAck",  /* 定时器名字是 ChgPlanOfferAck */
									HPLC_ChgPlanOfferAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									10000, /* 定时长度，以OS Tick为单位，即3000个OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* 周期性定时器 */
	/* 创建 充电计划执行状态确认 定时器 */
		HPLC_ChgPlanExeStateAck_Timer = rt_timer_create("HPLC_ChgPlanExeStateAck",  /* 定时器名字是 ChgPlanExeStateAck */
									HPLC_ChgPlanExeStateAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									10000, /* 定时长度，以OS Tick为单位，即3000个OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* 周期性定时器 */
									
	/* 创建 充电订单确认 定时器 */
		HPLC_ChgRecordAck_Timer = rt_timer_create("HPLC_ChgRecordAck",  /* 定时器名字是 ChgRecordAck */
									HPLC_ChgRecordAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									20000, /* 定时长度，以OS Tick为单位，即3000个OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* 周期性定时器 */
									
									
		/* 创建 充电申请上送回复 定时器 */
		BLE_ChgReqReportAck_Timer = rt_timer_create("BLE_ChgReqReportAck",  /* 定时器名字是 ChgReqReportRsp */
									BLE_ChgReqReportAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									10000, /* 定时长度，以OS Tick为单位，即5000个OS Tick */
									RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER); /* 一次性定时器 */
									
									
	/* 创建 充电计划确认 定时器 */
		BLE_ChgPlanOfferAck_Timer = rt_timer_create("BLE_ChgPlanOfferAck",  /* 定时器名字是 ChgPlanOfferAck */
									BLE_ChgPlanOfferAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									10000, /* 定时长度，以OS Tick为单位，即3000个OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* 周期性定时器 */
	/* 创建 充电计划执行状态确认 定时器 */
		BLE_ChgPlanExeStateAck_Timer = rt_timer_create("BLE_ChgPlanExeStateAck",  /* 定时器名字是 ChgPlanExeStateAck */
									BLE_ChgPlanExeStateAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									10000, /* 定时长度，以OS Tick为单位，即3000个OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* 周期性定时器 */
	/* 创建 充电订单确认 定时器 */
		BLE_ChgRecordAck_Timer = rt_timer_create("BLE_ChgRecordAck",  /* 定时器名字是 ChgRecordAck */
									BLE_ChgRecordAck_Timeout, /* 超时时回调的处理函数 */
									RT_NULL, /* 超时函数的入口参数 */
									20000, /* 定时长度，以OS Tick为单位，即3000个OS Tick */
									RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER); /* 周期性定时器 */									
																
	/* 启动定时器 */
	if (ChgPileStateGet_Timer != RT_NULL)
		rt_timer_start(ChgPileStateGet_Timer);
}
/********************************************************************  
*	函 数 名: ExeState_Update()
*	功能说明: 路由器充电执行状态更新
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void ExeState_Update(void)
{
	memcpy(Chg_ExeState.cRequestNO,Plan_Offer_Event.Chg_Strategy.cRequestNO,sizeof(Chg_ExeState.cRequestNO));
	memcpy(Chg_ExeState.cUserID,Plan_Offer_Event.Chg_Strategy.cUserID,sizeof(Chg_ExeState.cUserID));
	
	memcpy(Chg_ExeState.cAssetNO,RouterInfo.AssetNum,sizeof(Chg_ExeState.cAssetNO));
	Chg_ExeState.GunNum = Plan_Offer_Event.Chg_Strategy.GunNum;
	
	if(Chg_ExeState.exeState != EXE_ING)//非执行过程中，按计划中额定功率传
		Chg_ExeState.ucPlanPower = Chg_Strategy.ulChargeRatePow;
	
	ScmMeter_HisData stgMeter_HisData;
	cmMeter_get_data(EMMETER_HISDATA,&stgMeter_HisData);//获取电表计量数据
	memcpy(&Chg_ExeState.ulEleBottomValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
	memcpy(&Chg_ExeState.ulEleActualValue[0],&stgMeter_HisData.ulMeter_Day,5*sizeof(long));
	
	ScmMeter_Analog stgMeter_Analog;
	cmMeter_get_data(EMMETER_ANALOG,&stgMeter_Analog);
	Chg_ExeState.ucActualPower = stgMeter_Analog.ulAcPwr/100;
	Chg_ExeState.ucVoltage.A = stgMeter_Analog.ulVol;
	Chg_ExeState.ucCurrent.A = stgMeter_Analog.ulCur;
	
	ChargepileDataGetSet(Cmd_GetPilePara,&ChargePilePara_Get);
	Router_WorkState.Pile_State = ChargePilePara_Get.ChgPileState;
	Chg_ExeState.ChgPileState = Router_WorkState.Pile_State;//直接取用充电桩的三态	
}

/********************************************************************  
*	函 数 名: Charge_PlanIssue_RSP()
*	功能说明: 充电计划响应
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

static void Charge_Apply_RSP(CHARGE_APPLY* chg_apply,CHARGE_APPLY_RSP* chg_apply_rsp)//充电请求响应
{
	memcpy(&chg_apply_rsp->cRequestNO,&chg_apply->cRequestNO,sizeof(chg_apply_rsp->cRequestNO));
	memcpy(&chg_apply_rsp->cAssetNO,&chg_apply->cAssetNO,sizeof(chg_apply_rsp->cAssetNO));
	chg_apply_rsp->GunNum = chg_apply->GunNum;

	if((memcmp(&RouterInfo.AssetNum,&chg_apply->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)
		&&(Router_WorkState.Router_Fault.Total_Fau == RT_FALSE))//校验资产一致性 并且无故障
	{
		chg_apply_rsp->cSucIdle = SUCCESSFUL;
	}
	else
		chg_apply_rsp->cSucIdle = ORTHERS;
	
	rt_kprintf("[strategy]: (%s)  Charge Apply response sucidle = %d \n",__func__,chg_apply_rsp->cSucIdle);
	
	BLE_CtrlUnit_RecResp(Cmd_ChgRequestAck,&Chg_Apply_Rsp,0);	//充电申请确认响应
}

/********************************************************************  
*	函 数 名: Charge_Apply_Event_Create()
*	功能说明: 生成充电请求事件记录
*	形    参: 无
*	返 回 值: 无
********************************************************************/

static rt_int8_t Charge_Apply_Event_Create(CHARGE_APPLY* chg_apply,CHARGE_APPLY_EVENT* chg_apply_event)
{
	rt_int8_t res;
	static rt_uint32_t OrderNum = 0;
	
	memset(chg_apply_event,0,sizeof(CHARGE_APPLY_EVENT));

	rt_kprintf("[strategy]:  (%s) Create charge apply event record...!\n",__func__);

	if(memcmp(&RouterInfo.AssetNum,&chg_apply->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//校验资产一致性
	{
		OrderNum++;

		chg_apply_event->OrderNum = OrderNum;

		memcpy(&chg_apply_event->RequestTimeStamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//充电申请时间
		memcpy(&chg_apply_event->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件发生时间
		memcpy(&chg_apply_event->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件结束时间		

		memcpy(&chg_apply_event->RequestNO,&chg_apply->cRequestNO,sizeof(chg_apply_event->RequestNO));
		memcpy(&chg_apply_event->AssetNO,&chg_apply->cAssetNO,sizeof(chg_apply_event->AssetNO));
		chg_apply_event->GunNum = chg_apply->GunNum;
		
		chg_apply_event->ChargeReqEle = chg_apply->ulChargeReqEle;			
		memcpy(&chg_apply_event->PlanUnChg_TimeStamp,&chg_apply->PlanUnChg_TimeStamp,sizeof(STR_SYSTEM_TIME));
		chg_apply_event->ChargeMode = chg_apply->ChargeMode;
		
		memcpy(&chg_apply_event->UserAccount,&chg_apply->cUserID,sizeof(chg_apply_event->UserAccount));							
		memcpy(&chg_apply_event->Token,&chg_apply->Token,sizeof(chg_apply_event->Token));		//按32有效数取值
		
		rt_kprintf("[strategy]:  (%s) Create charge apply event record sucess!\n",__func__);
		
		
		//申请事件处理
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
*	函 数 名: Charge_PlanIssue_RSP()
*	功能说明: 充电计划响应
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

static void Charge_PlanIssue_RSP(CHARGE_STRATEGY* charge_plan,CHARGE_STRATEGY_RSP* charge_plan_rsp)
{
	memcpy(&charge_plan_rsp->cRequestNO,&charge_plan->cRequestNO,sizeof(charge_plan_rsp->cRequestNO));
	memcpy(&charge_plan_rsp->cAssetNO,&charge_plan->cAssetNO,sizeof(charge_plan_rsp->cAssetNO));
	charge_plan_rsp->GunNum = charge_plan->GunNum;
	
	if((memcmp(&RouterInfo.AssetNum,&charge_plan->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)
		&&(Router_WorkState.Router_Fault.Total_Fau == RT_FALSE))//校验资产一致性 并且无故障
	{
		charge_plan_rsp->cSucIdle = SUCCESSFUL;
	}
	else
		charge_plan_rsp->cSucIdle = ORTHERS;
	
	rt_kprintf("[strategy]: (%s)  Charge Plan response sucidle = %d \n",__func__,charge_plan_rsp->cSucIdle);	
	
	CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,charge_plan_rsp,0);//充电计划响应
	
	
	CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Ack = RT_TRUE;//充电计划已响应
		
}


/********************************************************************  
*	函 数 名: Charge_Plan_Event_Create()
*	功能说明: 控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/

static rt_int8_t Charge_Plan_Event_Create(CHARGE_STRATEGY* charge_plan,PLAN_OFFER_EVENT* charge_plan_event)
{
	rt_int8_t res;
	static rt_uint32_t OrderNum = 0;
	
	memset(charge_plan_event,0,sizeof(PLAN_OFFER_EVENT));
	
	rt_kprintf("[strategy]:  (%s) Create charge plan event record...!\n",__func__);

	if(memcmp(&RouterInfo.AssetNum,&charge_plan->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//校验资产一致性
	{	
		OrderNum++;
		charge_plan_event->OrderNum = OrderNum;		
		charge_plan_event->ChannelState = 0;//通道状态
		charge_plan_event->OccurSource = 0;//事件发生源
		
		memcpy(&charge_plan_event->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//记录上报时间
		memcpy(&charge_plan_event->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
		
		memcpy(&charge_plan_event->Chg_Strategy,charge_plan,sizeof(CHARGE_STRATEGY));
			
		rt_kprintf("[strategy]:  (%s) Create charge plan event record sucess!\n",__func__);
		
		res = SetStorageData(Cmd_PlanOfferWr,charge_plan_event,sizeof(PLAN_OFFER_EVENT));//存储充电计划上报事件记录

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
*	函 数 名: Charge_PlanAdj_RSP()
*	功能说明: 充电计划调整响应
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

static void Charge_PlanAdj_RSP(CHARGE_STRATEGY* charge_plan_adj,CHARGE_STRATEGY_RSP* charge_plan_adj_rsp)
{
	if(memcmp(&RouterInfo.AssetNum,&charge_plan_adj->cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//校验资产一致性
	{
		memcpy(&charge_plan_adj_rsp->cRequestNO,&Chg_Strategy_Adj.cRequestNO,sizeof(Chg_Strategy_Adj.cRequestNO));
		memcpy(&charge_plan_adj_rsp->cAssetNO,&Chg_Strategy_Adj.cAssetNO,sizeof(Chg_Strategy_Adj.cAssetNO));
		charge_plan_adj_rsp->cSucIdle = SUCCESSFUL;
	}
	else
	{
		charge_plan_adj_rsp->cSucIdle = ORTHERS;
	}
	
	CtrlUnit_RecResp(Cmd_ChgPlanAdjustAck,&charge_plan_adj_rsp,0);//回复
}

/********************************************************************  
*	函 数 名: Charge_PlanAdj_RSP()
*	功能说明: 充电计划调整响应
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 


static void ChargePile_Start_Charge(ChargPilePara_TypeDef* PilePara)
{
	rt_int8_t res=0;
	
	ChargepileDataGetSet(Cmd_SetPower,&PilePara);//下发充电桩设定功率
	
	rt_kprintf("[strategy]:  (%s) set charge duty = %d !\n",__func__,PilePara->PWM_Duty);

	res = ChargepileDataGetSet(Cmd_ChargeStart,0);//发送启动命令
	
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
*	函 数 名: Charge_Plan_Event_Create()
*	功能说明: 控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
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

	memcpy(&charge_plan_exe_event->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//记录上报时间
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
*	函 数 名: Charge_Record_Create()
*	功能说明: 控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/

static rt_int8_t Charge_Record_Create(CHARGE_APPLY_EVENT* chg_apply_event,PLAN_OFFER_EVENT* charge_plan_event,CHG_ORDER_EVENT* charge_record)
{
	rt_uint8_t i;
	static rt_uint32_t OrderNum = 0;
	
	OrderNum++;
	charge_record->OrderNum = OrderNum;
	charge_record->ChannelState = 0;
	charge_record->OccurSource = 0;
	
	memcpy(&charge_record->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件发生时间
	memcpy(&charge_record->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件结束时间
	
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
//			rt_kprintf("[strategy]:  (%s) 保存充电订单事件，失败！\n",__func__);
//			SetStorageData(Cmd_HistoryRecordWr,&ChgOrder_Event,sizeof(CHG_ORDER_EVENT));//再存一次
//		}
}

/********************************************************************  
*	函 数 名: Charge_Record_Update()
*	功能说明: 控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
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
	
	memcpy(&charge_record->FinishTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//事件结束时间
	
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
*	函 数 名: Charge_Plan_Event_Create()
*	功能说明: 控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
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

	memcpy(&RouterFault->StartTimestamp,&System_Time_STR,sizeof(STR_SYSTEM_TIME));//记录上报时间
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
*	函 数 名: get_event_cmd()
*	功能说明: 查询当前指令
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: HPLC_Data_RecProcess()
*	功能说明: 控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void HPLC_Data_RecProcess(void)
{
	rt_int8_t i,res,cmd;
	rt_uint32_t EventCmd=Cmd_Null;
	
	/***************************** 接收控制器命令 *******************************/
	EventCmd = strategy_event_get();
	cmd = get_event_cmd(EventCmd);
	
	switch(cmd)
	{
		case Cmd_ChgPlanIssue://收到充电计划
		{
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Apply == RT_FALSE)
			{
				memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
			}
			
			if(CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan == RT_TRUE)
			{
				CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,-1);//充电计划取值
				Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//充电计划响应
				break;
			}
			
			memset(&Chg_Strategy,0,sizeof(Chg_Strategy));
			CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//充电计划取值
			
			Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//充电计划响应
			
			if(Chg_StrategyRsp.cSucIdle == SUCCESSFUL)//正确  生成充电计划事件
			{
				if(Charge_Plan_Event_Create(&Chg_Strategy,&Plan_Offer_Event) > 0 )
				{
					PlanSlotCount = Plan_Offer_Event.Chg_Strategy.ucTimeSlotNum;
					PlanExeNum = 0;
					PlanExeNum_Old = 0xff;					
					
					CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//上报充电计划事件
					if (HPLC_ChgPlanOfferAck_Timer != RT_NULL)
					{
						rt_timer_start(HPLC_ChgPlanOfferAck_Timer);
						HPLC_ChgPlanOfferAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  HPLC_ChgPlanOfferAck_Timer start!\n",__func__);	
					}					
				}				
				CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Event = RT_TRUE; //充电计划上报
			}
					
			CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan = RT_TRUE;//已接收到充电计划

			break;
		}
		
		case Cmd_ChgPlanOfferAck:////收到充电计划事件确认
		{	
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);

			CtrlUnit_RecResp(Cmd_ChgPlanOfferAck,0,0);//回复
			rt_timer_stop(HPLC_ChgPlanOfferAck_Timer);//收到确认响应 停止重发
			
			Router_WorkState.Router_State = ChgState_PlanSoltsStart;//已收到充电计划响应 开始判断充电计划 进入执行状态
					
			if(CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Adj_Event == RT_TRUE)
				CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Adj_Event_Ack = RT_TRUE;
			else	
				CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Plan_Event_Ack = RT_TRUE; //充电计划上报应答
			break;
		}
		case Cmd_ChgPlanExeStateAck://收到充电计划执行事件确认
		{
			CtrlUnit_RecResp(Cmd_ChgPlanExeStateAck,0,0);//取值	
			rt_timer_stop(HPLC_ChgPlanExeStateAck_Timer);		
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);
			break;
		}
		case Cmd_ChgRequestReportAck://充电申请事件应答
		{
			CtrlUnit_RecResp(Cmd_ChgRequestReportAck,0,0);//清除标志
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRequestReportAck clear!\n",__func__);
			break;
		}
		
		//收到充电计划调整
		case Cmd_ChgPlanAdjust:
		{
			res = CtrlUnit_RecResp(Cmd_ChgPlanAdjust,&Chg_Strategy_Adj,0);//取值	
			
			Charge_PlanAdj_RSP(&Chg_Strategy_Adj,&Chg_StrategyRsp_Adj);//充电计划调整响应
			
			if(memcmp(&Chg_Strategy_Adj.strChargeTimeSolts[0].strDecStartTime,\
				&Chg_Strategy_Adj.strChargeTimeSolts[0].strDecStopTime,sizeof(STR_SYSTEM_TIME)) == 0)
				{
					if(memcmp(&RouterInfo.AssetNum,&Chg_Strategy_Adj.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//校验资产一致性		
					{
						ChargePile_Stop_Charge();
						Router_WorkState.Router_State = ChgState_PlanExeEnd;//计划执行结束		
						Chg_ExeState.exeState = EXE_END;
					}
					break;
				}
			
			if(Chg_StrategyRsp_Adj.cSucIdle == SUCCESSFUL)//正确  更新充电计划
			{
				memcpy(&Chg_Strategy,&Chg_Strategy_Adj,sizeof(Chg_Strategy));//正确  更新充电计划
				
				if(Charge_Plan_Event_Create(&Chg_Strategy,&Plan_Offer_Event) > 0 )//生成充电计划事件
				{
					PlanSlotCount = Plan_Offer_Event.Chg_Strategy.ucTimeSlotNum;
					PlanExeNum = 0;
					PlanExeNum_Old = 0xff;
					
					CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//上报充电计划事件
					
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
		//收到启动充电命令
		case Cmd_StartChg:
		{		
			res = CtrlUnit_RecResp(Cmd_StartChg,&Ctrl_Start,0);//取启动充电参数

			if(memcmp(&RouterInfo.AssetNum,&Ctrl_Start.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//校验资产一致性
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
//			res = CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//回复
			
			////////////////////////预留 准备记录操作事件//////////////////////////
			
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Start = RT_TRUE;
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Start_Ack = RT_TRUE;
			
//			memcpy(&CtrlCharge_Event.Ctrl_ChgData,&Ctrl_Start,sizeof(Ctrl_Start));
//			CtrlCharge_Event.CtrlType = CTRL_START;
//			CtrlCharge_Event.StartSource = CTRL_UNIT;		
			//////////////////////////////////////////////////////////////////////
			break;
		}
		//收到停止充电命令
		case Cmd_StopChg:
		{	
			res = CtrlUnit_RecResp(Cmd_StopChg,&Ctrl_Stop,0);//取值			 

			if((memcmp(&RouterInfo.AssetNum,&Ctrl_Stop.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//校验资产一致性
				||(memcmp(&Ctrl_Start.OrderSn,&Ctrl_Stop.OrderSn,sizeof(Ctrl_Start.OrderSn)) == 0))//校验启停单号一致性		
			{
				ChargePile_Stop_Charge();
				Router_WorkState.Router_State = ChgState_PlanExeEnd;//计划执行结束		
				Chg_ExeState.exeState = EXE_END;
			}
			else
			{
				Ctrl_Stop.cSucIdle = ORTHERS;				
			}
			res = CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);//回复
			
			////////////////////////预留 准备记录操作事件//////////////////////////
			
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Stop = RT_TRUE;
			CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Stop_Ack = RT_TRUE;
			
			
//			memcpy(&CtrlCharge_Event.Ctrl_ChgData,&Ctrl_Stop,sizeof(Ctrl_Stop));			
//			CtrlCharge_Event.CtrlType = CTRL_STOP;
//			CtrlCharge_Event.StopSource = CTRL_UNIT;
			//////////////////////////////////////////////////////////////////////
			break;
		}
		//收到调整功率命令
		case Cmd_PowerAdj:
		{
			res = CtrlUnit_RecResp(Cmd_PowerAdj,&Ctrl_PowerAdj,0);//取值	
 	
			if(memcmp(&RouterInfo.AssetNum,&Ctrl_PowerAdj.cAssetNO,sizeof(RouterInfo.AssetNum)) == 0)//校验资产一致性
			{
				ChargePilePara_Set.PWM_Duty = Ctrl_PowerAdj.SetPower*10/132;//功率换算: D(含一位小数)=I/60*1000=P/(60*220)*1000
				ChargepileDataGetSet(Cmd_SetPower,&ChargePilePara_Set);	
			}
			else
			{
				Ctrl_PowerAdj.cSucIdle = ORTHERS;
			}
			res = CtrlUnit_RecResp(Cmd_PowerAdjAck,&Ctrl_PowerAdj,0);//回复
			
			break;
		}
		//收到控制器抄读路由器工作状态
		case Cmd_RouterExeState:
		{
			CtrlUnit_RecResp(Cmd_RouterExeState,&Chg_ExeState,0);//取值	
			
			ExeState_Update();	
		
			CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,1);//回复	 wyg191105
			
			break;
		}
		//收到充电订单事件确认
		case Cmd_ChgRecordAck:
		{
			CtrlUnit_RecResp(Cmd_ChgRecordAck,0,0);//清除标志
			rt_timer_stop(HPLC_ChgRecordAck_Timer);
			
			CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
					
			Charge_Event_Data_Clear();
			
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRecordAck clear!\n",__func__);
			break;
		}
		//路由器异常事件确认
		case Cmd_DeviceFaultAck:
		{
			CtrlUnit_RecResp(Cmd_DeviceFaultAck,0,0);//清除标志
//			rt_timer_stop(ChgRecordAck_Timer);
			
//			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
							
			rt_kprintf("[strategy]:  (%s)  Cmd_DeviceFaultAck clear!\n",__func__);
			break;
		}
				//充电桩异常事件确认
		case Cmd_PileFaultAck:
		{
			CtrlUnit_RecResp(Cmd_PileFaultAck,0,0);//清除标志
//			rt_timer_stop(ChgRecordAck_Timer);
			
//			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
			
			rt_kprintf("[strategy]:  (%s)  Cmd_PileFaultAck clear!\n",__func__);
			break;
		}
	
		//收到STA监测自身及路由器在线状态
		case Cmd_STAOnlineState:
		{			 
			res = CtrlUnit_RecResp(Cmd_STAOnlineState,&OnlineState_Event,0);//取值
			if(res == SUCCESSFUL)
			{
				rt_kprintf("[strategy]:  (%s) OnlineState_Event,Cmd_STAOnlineState,Successful!\n",__func__);
				CtrlUnit_RecResp(Cmd_STAOnlineStateAck,&OnlineState_Event,0);//回复确认
			}
		
			//存储在线状态变化事件
//			res = SetStorageData(Cmd_OnlineStateWr,&OnlineState_Event,sizeof(ONLINE_STATE));
//			if(res == SUCCESSFUL)
//			{
//				rt_kprintf("[strategy]:  (%s) Storage BLE_ChgExe_Event, Successful!\n",__func__);
//			}
//			else
//			{
//				rt_kprintf("[strategy]:  (%s) 保存在线状态，失败！\n",__func__);
//				SetStorageData(Cmd_OnlineStateWr,&OnlineState_Event,sizeof(ONLINE_STATE));//再存一次
//			}
					
//			res = BLE_CtrlUnit_RecResp(Cmd_STAOnlineStateAPP,&OnlineState_Event,0);//传送至蓝牙
//			if(res == SUCCESSFUL)
//				rt_kprintf("[strategy]:  (%s) OnlineState_Event Apply to BLE, Successful!\n",__func__);				
//			else
//				rt_kprintf("[strategy]:  (%s) 回复BLE 表计在线状态事件，失败！\n",__func__);		
			break;
		}			
		default:
			break;
	}
	EventCmd = 0;//清位
}

/********************************************************************  
*	函 数 名: BLE_Data_RecProcess()
*	功能说明: 蓝牙控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void BLE_Data_RecProcess(void)
{
	rt_int8_t i,res,cmd;
	rt_uint32_t EventCmd=Cmd_Null;
	
	/***************************** 接收控制器命令 *******************************/
	EventCmd = Strategy_get_BLE_event();
	cmd = get_event_cmd(EventCmd);
	
	if(cmd > Cmd_STAOnlineStateAck)
		return;

	switch(cmd)
	{	
		//充电申请    然后直接启动充电，启动后立即上送执行事件
		case Cmd_ChgRequest:
		{	
			memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));///////////////////////////////////////##############
			
			res = BLE_CtrlUnit_RecResp(Cmd_ChgRequest,&Chg_Apply,0);//取值
			
			Charge_Apply_RSP(&Chg_Apply,&Chg_Apply_Rsp);

			if(Chg_Apply_Rsp.cSucIdle == SUCCESSFUL)//充电申请响应成功 生成充电申请事件记录 否则直接跳过
			{
				res = Charge_Apply_Event_Create(&Chg_Apply,&Chg_Apply_Event);//生成充电申请事件记录
				if(res > 0)
				{					
					CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//通知控制器
					
					if (HPLC_ChgReqReportAck_Timer != RT_NULL)
					{
						rt_timer_start(HPLC_ChgReqReportAck_Timer);
						HPLC_ChgReqReportAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  HPLC_ChgReqReportAck_Timer start!\n",__func__);	
					}
					CtrlCharge_Event.Router_HPLC_Info.Bit.Charge_Apply_Event = RT_TRUE;
					
					
					BLE_CtrlUnit_RecResp(Cmd_ChgRequestReport,&Chg_Apply_Event,0);//同时将事件回传APP
					
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
		case Cmd_ChgRequestReportAck://充电申请事件应答
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgRequestReportAck,0,0);//清除标志
			rt_timer_stop(BLE_ChgReqReportAck_Timer);
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRequestReportAck clear!\n",__func__);
			break;
		}

		case Cmd_ChgPlanIssue://收到充电计划
		{
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Apply == RT_FALSE)
			{
				memset(&CtrlCharge_Event,0,sizeof(CtrlCharge_Event));
			}
			
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan == RT_TRUE)
			{
				BLE_CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,-1);//充电计划取值
				Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//充电计划响应
				break;
			}
			
			memset(&Chg_Strategy,0,sizeof(Chg_Strategy));
			BLE_CtrlUnit_RecResp(Cmd_ChgPlanIssue,&Chg_Strategy,0);//充电计划取值
			
			Charge_PlanIssue_RSP(&Chg_Strategy,&Chg_StrategyRsp);//充电计划响应
			
			if(Chg_StrategyRsp.cSucIdle == SUCCESSFUL)//正确  生成充电计划事件
			{
				if(Charge_Plan_Event_Create(&Chg_Strategy,&Plan_Offer_Event) > 0 )
				{
					
					PlanSlotCount = Plan_Offer_Event.Chg_Strategy.ucTimeSlotNum;
					PlanExeNum = 0;
					PlanExeNum_Old = 0xff;
					
					BLE_CtrlUnit_RecResp(Cmd_ChgPlanOffer,&Plan_Offer_Event,0);//上报充电计划事件
					if(BLE_ChgPlanOfferAck_Timer != RT_NULL)
					{
						rt_timer_start(BLE_ChgPlanOfferAck_Timer);
						BLE_ChgPlanOfferAck_count = 0;
						rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer start!\n",__func__);	
					}					
				}
				
				CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Event = RT_TRUE; //充电计划上报
			}
			CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan = RT_TRUE;//已接收到充电计划
			break;
		}
		
		case Cmd_ChgPlanOfferAck:////收到充电计划事件确认
		{	
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);

			BLE_CtrlUnit_RecResp(Cmd_ChgPlanOfferAck,0,0);//回复
			rt_timer_stop(BLE_ChgPlanOfferAck_Timer);//收到确认响应 停止重发
			
			Router_WorkState.Router_State = ChgState_PlanSoltsStart;//已收到充电计划响应 开始判断充电计划 进入执行状态
					
			if(CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Adj_Event == RT_TRUE)
				CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Adj_Event_Ack = RT_TRUE;
			else	
				CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Plan_Event_Ack = RT_TRUE; //充电计划上报应答
			break;
		}
		
		case Cmd_ChgPlanExeStateAck://收到充电计划执行事件确认
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeStateAck,0,0);//取值	
			rt_timer_stop(BLE_ChgPlanExeStateAck_Timer);		
			rt_kprintf("[strategy]:  (%s)  ChgPlanOfferAck_Timer stop!\n",__func__);
			break;
		}

		//收到蓝牙抄读路由器工作状态
		case Cmd_RouterExeState:
		{			
			res = BLE_CtrlUnit_RecResp(Cmd_RouterExeState,0,0);//取值
			
			ExeState_Update();				
			
			res = BLE_CtrlUnit_RecResp(Cmd_RouterExeStateAck,&Chg_ExeState,0);//回复

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
		//收到蓝牙停机命令
		case Cmd_StopChg:
		{	
			res = BLE_CtrlUnit_RecResp(Cmd_StopChg,&Ctrl_Stop,0);//取值	

			ChargePile_Stop_Charge();
			
			Router_WorkState.Router_State = ChgState_PlanExeEnd;//计划执行结束
			Chg_ExeState.exeState = EXE_END;		
			CtrlCharge_Event.Router_BLE_Info.Bit.Router_Svc_Stop = RT_TRUE;
			break;
		}
				//收到充电订单事件确认
		case Cmd_ChgRecordAck:
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgRecordAck,0,0);//清除标志
			rt_timer_stop(BLE_ChgRecordAck_Timer);
			
			CtrlCharge_Event.Router_BLE_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
					
//			Charge_Event_Data_Clear();
			
			rt_kprintf("[strategy]:  (%s)  Cmd_ChgRecordAck clear!\n",__func__);
			break;
		}
				//路由器异常事件确认
		case Cmd_DeviceFaultAck:
		{
			BLE_CtrlUnit_RecResp(Cmd_DeviceFaultAck,0,0);//清除标志
//			rt_timer_stop(ChgRecordAck_Timer);
			
//			CtrlCharge_Event.Ctrl_Chg_Info.Bit.Charge_Record_Event_Ack = RT_TRUE;
							
			rt_kprintf("[strategy]:  (%s)  Cmd_DeviceFaultAck clear!\n",__func__);
			break;
		}
				//充电桩异常事件确认
		case Cmd_PileFaultAck:
		{
			BLE_CtrlUnit_RecResp(Cmd_PileFaultAck,0,0);//清除标志
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
*	函 数 名: CtrlData_RecProcess()
*	功能说明: 控制命令接收处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void CtrlData_RecProcess(void)
{
	BLE_Data_RecProcess();//BLE 事件处理
	HPLC_Data_RecProcess();//HPLC事件处理
}

/********************************************************************  
*	函 数 名: Start_ChargePile_Process()
*	功能说明: 启动充电桩处理函数
*	形    参: 无
*	返 回 值: 无
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
		
		if(start_result|ChargeStartOK_EVENT)//启动充电成功
		{
			if(PlanExeNum	== 0)	//充电计划初始创建充电订单
				Charge_Record_Create(&Chg_Apply_Event,&Plan_Offer_Event,&ChgOrder_Event);//生成充电订单记录
			
			Router_WorkState.Router_State = ChgState_InCharging; //置状态 充电中
			
			Ctrl_Start.cSucIdle = SUCCESSFUL;//启动成功
			Chg_ExeState.exeState = EXE_ING;//计划执行中
		}
		else if(start_result|ChargeStartER_EVENT)//启动充电失败
		{
			ExeState_Update();//路由器状态更新
			Ctrl_Start.cSucIdle = FAILED;//启动失败
			Chg_ExeState.exeState = EXE_FAILED;//执行失败
			
			ChargepileDataGetSet(Cmd_ChargeStartResp,&ChargePilePara_Get);//获取失败原因
		}
		
		if(CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Start == RT_TRUE)//如果是主站启动 返回启动结果
			CtrlUnit_RecResp(Cmd_StartChgAck,&Ctrl_Start,0);//回复启动结果
		
		res = Charge_Plan_Exe_Event_Create(&Chg_ExeState,&Ctrl_ChgExe_Event);//生成充电执行事件记录
		
		if(res > 0)
		{
//			CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);//向控制器上报充电执行事件记录
			
			if (HPLC_ChgPlanExeStateAck_Timer != RT_NULL)
			{
				rt_timer_start(HPLC_ChgPlanExeStateAck_Timer);
				HPLC_ChgPlanExeStateAck_count = 0;
			}
			if(CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT == RT_TRUE)//如果蓝牙已连接 则向蓝牙上报充电执行事件记录
			{
				BLE_CtrlUnit_RecResp(Cmd_ChgPlanExeState,&Ctrl_ChgExe_Event,0);	//向APP上报充电执行事件记录
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
*	函 数 名: Stop_ChargePile_Process()
*	功能说明: 停止充电桩处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

static void Stop_ChargePile_Process(void)
{
	rt_int8_t res;
	rt_uint32_t stop_result;
	
	stop_result = 0;
	
	rt_kprintf("[strategy]: (%s) waiting for stop charge event!\n",__func__);
		//接收停机事件
	if(rt_event_recv(&ChargePileEvent, ChargeStopOK_EVENT|ChargeStopER_EVENT,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,100, &stop_result) == RT_EOK)		
	{
		rt_timer_stop(StopChgResp_Timer);
		
		if(stop_result|ChargeStopOK_EVENT)
		{
			Ctrl_Stop.cSucIdle = SUCCESSFUL;
			
			Charge_Record_Update(&ChgOrder_Event);//更新充电记录
			if(Router_WorkState.Router_State == ChgState_PlanExeEnd)//执行完成 转换为充电完成状态 等待上传订单
				Router_WorkState.Router_State = ChgState_Finished;//状态变更
			else
			{
				PlanExeNum++;
				Router_WorkState.Router_State = ChgState_PlanSoltsStart;//计划未执行完，等待下个时间段
			}
		}
		else if(stop_result|ChargeStopER_EVENT)
		{
			Ctrl_Stop.cSucIdle = FAILED;
			ChargepileDataGetSet(Cmd_ChargeStopResp,&ChargePilePara_Get);//获取失败原因			
		}
		
		if(CtrlCharge_Event.Router_HPLC_Info.Bit.Router_Svc_Stop == RT_TRUE)//如果是主站停止 返回启动结果
			CtrlUnit_RecResp(Cmd_StopChgAck,&Ctrl_Stop,0);
		
		rt_kprintf("[strategy]: (%s) stop chargepile event, sucidle = %d  .......!\n",__func__,Ctrl_Stop.cSucIdle);
	}
}


/********************************************************************  
*	函 数 名: Stop_ChargePile_Process()
*	功能说明: 停止充电桩处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

static void Adjust_ChargePile_Power(void)
{
	rt_int8_t res;
	rt_uint32_t adjpow_result;
	
	adjpow_result = 0;
	
	rt_kprintf("[strategy]: (%s) waiting for adjust power event!\n",__func__);
		//调整功率成功
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
			ChargepileDataGetSet(Cmd_SetPowerResp,&ChargePilePara_Get);//获取失败原因
		}
	}
	rt_kprintf("[strategy]: (%s) Adjust chargepile power, sucidle = %d  .......!\n",__func__,Ctrl_PowerAdj.cSucIdle);
}


/********************************************************************  
*	函 数 名: TimeSolt_PilePowerCtrl()
*	功能说明: 分时段进行电桩功率控制
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

static void Charge_Plan_Exe_Start(void)
{
	rt_uint8_t i;
	rt_uint32_t Time_s;
	rt_uint32_t PlanStartTime_start[50];
	rt_uint32_t PlanStartTime_stop[50];

	
	Time_s = TimeCalculation(&System_Time_STR);//获取当前时间秒数
	
	for(i = PlanExeNum; i<PlanSlotCount;i++)
	{
		PlanStartTime_start[i] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[i].strDecStartTime);
		PlanStartTime_stop[i] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[i].strDecStopTime);
		
		if((Time_s > PlanStartTime_start[i])&&(Time_s < PlanStartTime_stop[i]))
		{
			Chg_ExeState.ucPlanPower = Chg_Strategy.strChargeTimeSolts[i].ulChargePow;		
			ChargePilePara_Set.PWM_Duty = Chg_Strategy.strChargeTimeSolts[i].ulChargePow*10/132;//功率换算
			ChargePile_Start_Charge(&ChargePilePara_Set);			
			Router_WorkState.Router_State = ChgState_PlanSoltsStarting;//开始启动充电桩
			
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
*	函 数 名: TimeSolt_PilePowerCtrl()
*	功能说明: 分时段进行电桩功率控制
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

static void Charge_Plan_Running(void)
{
	rt_uint32_t Time_s;
	rt_uint32_t PlanStartTime_stop[50];
	
	Time_s = TimeCalculation(&System_Time_STR);//获取当前时间秒数
	
	PlanStartTime_stop[PlanExeNum] = TimeCalculation(&Chg_Strategy.strChargeTimeSolts[PlanExeNum].strDecStopTime);
	
	if(Time_s > PlanStartTime_stop[PlanExeNum])
	{
		rt_kprintf("[strategy]: (%s) ChargePile_Stop_Charge!\n",__func__,Time_s);
		ChargePile_Stop_Charge();
		if(PlanExeNum == (Chg_Strategy.ucTimeSlotNum-1))
			Router_WorkState.Router_State = ChgState_PlanExeEnd;//计划执行结束
		else
			Router_WorkState.Router_State = ChgState_PlanSoltsStoping;//计划执行未结束 暂时停机 不上传充电订单
	}
	
	rt_kprintf("[strategy]: (%s) Time_s = %d!\n",__func__,Time_s);
	rt_kprintf("[strategy]: (%s) Sys time: %02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day\
							,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
}
		
/********************************************************************  
*	函 数 名: ChgOrder_Apply()
*	功能说明: 形成充电订单上报
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void ChgOrder_Apply(void)
{	
		Charge_Record_Update(&ChgOrder_Event);//更新充电记录

//		CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//上报充电订单

		if (HPLC_ChgRecordAck_Timer != RT_NULL)
		{
			rt_timer_start(HPLC_ChgRecordAck_Timer);
			HPLC_ChgRecordAck_count = 0;	
			rt_kprintf("[strategy]:  (%s) HPLC_ChgOrder_Event Apply to Contrllor, Successful!\n",__func__);
		}
		
		if(CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT == RT_TRUE)//蓝牙已连接
		{
			BLE_CtrlUnit_RecResp(Cmd_ChgRecord,&ChgOrder_Event,0);//同时将事件回传APP
			
			if (BLE_ChgRecordAck_Timer != RT_NULL)
			{
				rt_timer_start(BLE_ChgRecordAck_Timer);
				BLE_ChgRecordAck_count = 0;	
				rt_kprintf("[strategy]:  (%s) BLE_ChgOrder_Event Apply to Contrllor, Successful!\n",__func__);
			}
		}
		Chg_ExeState.exeState = EXE_END;
		Router_WorkState.Router_State = ChgState_Standby;//状态变更
}


/********************************************************************  
*	函 数 名: ChgOrder_Apply()
*	功能说明: 形成充电订单上报
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 
static void Router_Alarm_Process(void)
{
	static rt_uint32_t l_RouterAlm,l_PileAlm;
	
	if(l_RouterAlm ^ Router_WorkState.Router_Fault.Total_Fau)//路由器故障
	{
		l_RouterAlm = Router_WorkState.Router_Fault.Total_Fau; 
		Router_Pile_Alarm_Event_Create(&Router_Pile_Fault_Event,&Router_WorkState);//创建异常信息事件记录
		
		
		CtrlUnit_RecResp(Cmd_DeviceFault,&Router_Pile_Fault_Event,0);//上报路由器故障信息	
			
		if(CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT == RT_TRUE)//蓝牙已连接
		{
			BLE_CtrlUnit_RecResp(Cmd_DeviceFault,&Router_Pile_Fault_Event,0);//上报路由器故障信息
		}
	}
	
	if(l_PileAlm ^ Router_WorkState.Pile_Fault.Total_Fau)//充电桩故障
	{
		l_PileAlm = Router_WorkState.Pile_Fault.Total_Fau;
	}
	
	if(Router_WorkState.Router_Fault.Total_Fau == RT_FALSE)
		Router_WorkState.Router_State = ChgState_Standby;
}



/********************************************************************  
*	函 数 名: RtState_Judge()
*	功能说明: 路由器和充电桩状态判断
*	形    参: 无
*	返 回 值: 故障代码
********************************************************************/ 
static void RtState_Judge(void)
{
	
	ExeState_Update();//路由器状态更新
//	
	if((Router_WorkState.Router_Fault.Total_Fau != RT_FALSE)||(Router_WorkState.Pile_State == PILE_FAU))
		Router_WorkState.Router_State = ChgState_Fault;
	
	switch(Router_WorkState.Router_State)
	{
		case ChgState_Standby://空闲
			break;
		
		case ChgState_PlanSoltsStart:          //已收到充电计划 判断充电计划
			Charge_Plan_Exe_Start();
			break;
		
		case ChgState_PlanSoltsStarting:    //已发送启动命令 等待充电控制器响应
			Start_ChargePile_Process();
		break;
		
		case ChgState_InCharging:           //充电中		
			Charge_Record_Update(&ChgOrder_Event);//更新充电记录
			Charge_Plan_Running();//判断是否满足停止条件
			break;
		
		case ChgState_PlanExeEnd:
		case ChgState_PlanSoltsStoping://已发送停止命令 等待充电控制器响应
			Stop_ChargePile_Process();
		break;
		
		case ChgState_Finished:				//充电计划执行完成  上传充电订单 返回待机状态
			ChgOrder_Apply();
			break;
		
		case ChgState_Fault:            	//路由器、充电桩故障
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

	/*初始化变量*/
	
			/* 创建互斥锁 */
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
	RELAY_ON();//上电吸合继电器
	
//	PlanExeNum_Old = 0xff;
//	PlanExeNum = 0;
//	PlanSlotCount = 1;
	while (1)
	{
		RtState_Judge();
		
		CtrlData_RecProcess();
		
//		Charge_Record_Update(&ChgOrder_Event);
		
//		time = TimeCalculation(&System_Time_STR);//获取当前时间秒数
//		rt_kprintf("[strategy]: (%s) time = %d  \n",__func__,time);
//		rt_kprintf("[strategy]: (%s) Sys time:%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day\
//								,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
		
		rt_thread_mdelay(500);
	}
}

int strategy_thread_init(void)
{
	rt_err_t res;
	
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



