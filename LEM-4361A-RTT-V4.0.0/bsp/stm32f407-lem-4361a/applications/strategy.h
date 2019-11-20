#ifndef __STRATEGY_H__
#define __STRATEGY_H__

#include <rtthread.h>
#include <rtconfig.h>
#include <string.h>
#include <stdio.h>
#include "global.h"
#include "chargepile.h"
#include "strategy.h"


//extern struct rt_semaphore rt_sem_bluetoothfau;
extern ChargPilePara_TypeDef ChargePilePara_Set;
extern ChargPilePara_TypeDef ChargePilePara_Get;



/******************************* 充电计划 *************************************/
typedef struct
{
	STR_SYSTEM_TIME strDecStartTime;//起始时间
	STR_SYSTEM_TIME strDecStopTime;	//结束时间
	unsigned long ulChargePow;		//充电功率设定值（单位：kW，换算：-4）
}CHARGE_TIMESOLT;/*时段负荷单元*/

typedef struct
{
	char cRequestNO[18];			//申请单号  octet-string（SIZE(16)）
	char cAssetNO[24];				//路由器资产编号  visible-string（SIZE(22)）
	GUN_NUM GunNum;					//枪序号	{A枪（1）、B枪（2）}
	char cUserID[66];   			//用户id  visible-string（SIZE(64)）	
	unsigned char ucDecMaker;		//决策者  {主站（1）、控制器（2）}
	unsigned char ucDecType; 		//决策类型{生成（1） 、调整（2）}
	STR_SYSTEM_TIME strDecTime;		//决策时间	
	
	unsigned long ulChargeReqEle;	//充电需求电量（单位：kWh，换算：-2）
	unsigned long ulChargeRatePow;	//充电额定功率 （单位：kW，换算：-4）
	unsigned char ucChargeMode;		//充电模式 {正常（0），有序（1）}
	unsigned char ucTimeSlotNum;	//时间段数量
	CHARGE_TIMESOLT strChargeTimeSolts[50];//时间段内容，最大50段

}CHARGE_STRATEGY;/*充电计划单*/
//CCMRAM extern CHARGE_STRATEGY Chg_Strategy;//下发计划单
//CCMRAM extern CHARGE_STRATEGY Adj_Chg_Strategy;//变更计划单

typedef struct
{
	char cRequestNO[18];	//申请单号  octet-string（SIZE(16)）
	char cAssetNO[24];		//路由器资产编号  visible-string（SIZE(22)）
	GUN_NUM GunNum;			//枪序号	{A枪（1）、B枪（2）}
	unsigned char cSucIdle;	//成功或失败原因:{0：成功 1：失败 255：其他}
}CHARGE_STRATEGY_RSP;/*充电计划单响应*/


/****************************************** 充电申请 **********************************************/
typedef struct
{
	char cRequestNO[18];				//	申请单号  octet-string（SIZE(16)）
	char cAssetNO[24];					//	路由器资产编号  visible-string（SIZE(22)） 
	GUN_NUM GunNum;						//	枪序号	{A枪（1）、B枪（2）}
	char cUserID[66];   				//	用户id  visible-string（SIZE(64)）
	unsigned long ulChargeReqEle;		//	充电需求电量（单位：kWh，换算：-2）
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	计划用车时间
	CHARGE_MODE ChargeMode;				//	充电模式 {正常（0），有序（1）}
	char Token[40];   					//	用户登录令牌  visible-string（SIZE(32)）
}CHARGE_APPLY;/*充电申请单(BLE)*/

typedef struct
{
	char cRequestNO[18];	//申请单号  octet-string（SIZE(16)）
	char cAssetNO[24];		//路由器资产编号  visible-string（SIZE(22)）
	GUN_NUM GunNum;			//枪序号	{A枪（1）、B枪（2）}
	unsigned char cSucIdle;	//成功或失败原因:{0：成功 1：失败 255：其他}
}CHARGE_APPLY_RSP;/*充电申请单响应*/

/******************************** 事件信息记录 ***********************************/
typedef struct
{
	unsigned long OfflinePeriod;		//本次离线时长（单位：秒）
	unsigned char OfflineReason;		//离线原因 {未知（0），停电（1），信道变化（2）}
}OFFLINE_IFO;/*离线信息*/

typedef struct
{
	unsigned long OrderNum;					//记录序号
	STR_SYSTEM_TIME OnlineTimestamp;		//上线时间
	STR_SYSTEM_TIME OfflineTimestamp;		//离线时间
	unsigned char OccurSource;				//事件发生源    NULL 
	unsigned char ChannelState;				//通道状态
	unsigned char AutualState;				//状态变化 {上线（0）， 离线（1）}
	OFFLINE_IFO OfflineIfo;					//离线信息
}ONLINE_STATE;/*表计在线状态事件*/

typedef struct
{
	unsigned long OrderNum;			//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;	//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;//  事件结束时间 
	unsigned char OccurSource;		//	事件发生源    NULL 	
//	unsigned char Reason;			//  事件发生原因     
	unsigned char ChannelState;		//  事件上报状态 = 通道上报状态
	char RequestNO[18];				//	充电申请单号   （SIZE(16)）
	char AssetNO[24];				//	路由器资产编号 visible-string（SIZE(22)）
	GUN_NUM GunNum;					//	枪序号	{A枪（1）、B枪（2）}
}PLAN_FAIL_EVENT;/*充电计划生成失败记录单元*/

typedef struct
{
	unsigned long OrderNum;				//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;		//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;	//  事件结束时间  
//	unsigned char Reason;				//  事件发生原因
	unsigned char OccurSource;			//	事件发生源    NULL 		
	unsigned char ChannelState;			//  事件上报状态 = 通道上报状态	
	CHARGE_STRATEGY Chg_Strategy; 
}PLAN_OFFER_EVENT;/*充电计划上报记录单元*/

typedef struct
{
	unsigned long OrderNum;				//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;		//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;	//  事件结束时间  
	unsigned char OccurSource;			//	事件发生源    NULL 
	unsigned char ChannelState;			//  事件上报状态 = 通道上报状态
	
	char RequestNO[18];					//	充电申请单号   （SIZE(16)） 
	char AssetNO[24];					//	路由器资产编号 visible-string（SIZE(22)）
	GUN_NUM GunNum;						//	枪序号	{A枪（1）、B枪（2）}
	
	STR_SYSTEM_TIME RequestTimeStamp;	//	充电申请时间
	unsigned long actSOC;				//	当前SOC（单位：%，换算：-2）
	unsigned long aimSOC;				//  目标SOC（单位：%，换算：-2）
	unsigned long CellCapacity;			//	电池容量（单位：kWh，换算：-2）
	unsigned long ChargeReqEle;			//	充电需求电量（单位：kWh，换算：-2）
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	计划用车时间
	CHARGE_MODE ChargeMode;				//	充电模式 {正常（0），有序（1）}
	char Token[40];   					//	用户登录令牌  visible-string（SIZE(38)）
	char UserAccount[66];				//  充电用户账号  visible-string（SIZE(9)）
}CHARGE_APPLY_EVENT;/*充电申请事件记录单元*/


typedef struct
{
	unsigned long OrderNum;				//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;		//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;	//  事件结束时间  
	unsigned char OccurSource;			//	事件发生源    NULL    
	unsigned char ChannelState;			//  事件上报状态 = 通道上报状态
	rt_err_t TotalFault;				//	故障状态
	char Fau[32];						//	各项故障状态
}ORDER_CHG_EVENT;/*有序充电事件记录单元*/


/******************************* 充电控制 *************************************/
typedef enum
{
	CTRL_NULL=0,
	CTRL_START,
	CTRL_STOP,
	CTRL_ADJPOW,
}CTRL_TYPE;/*{0：未控制 1：启动  2：停止  3：调整功率}*/

typedef enum 
{
	CTRL_UNIT=1,		//控制器操控
	BLE_UNIT,			//蓝牙操控
}CTRL_CMD_SOURCE;/*控制命令来源*/

typedef enum
{
	EXE_NULL=0,
	EXE_ING,
	EXE_END,
	EXE_FAILED,
}EXESTATE;/*执行状态 {0：未执行  1：正常执行 2：执行结束 3：执行失败}*/

typedef struct
{
	char OrderSn[18];			//订单号  octet-string（SIZE(16)）
	char cAssetNO[24];			//路由器资产编号  visible-string（SIZE(22)）
	GUN_NUM GunNum;				//枪序号	{A枪（1）、B枪（2）}
	unsigned long SetPower;		//设定充电功率（单位：W，换算：-1）
	unsigned char cSucIdle;		//成功或失败原因:{0：成功 1：失败 255：其他}
}CTL_CHARGE;/*控制器充电控制*/

typedef struct
{
	unsigned long A;
	unsigned long B;
	unsigned long C;
}PHASE_LIST;

typedef struct
{
	char cAssetNO[24];		//路由器资产编号  visible-string（SIZE(22)）
	GUN_NUM GunNum;	//枪序号	{A枪（1）、B枪（2）}
}CHARGE_EXE_STATE_ASK;/*路由器工作状态查询*/

typedef struct
{
	char cRequestNO[18];			//申请单号  octet-string（SIZE(16)）
	char cAssetNO[24];				//路由器资产编号  visible-string（SIZE(22)）
	GUN_NUM GunNum;					//枪序号	{A枪（1）、B枪（2）}
	EXESTATE exeState;				//执行状态 {1：正常执行 2：执行结束 3：执行失败}
	unsigned char ucTimeSlotNum;	//时间段数量
	unsigned long ulEleBottomValue[5]; 	//电能示值底值（充电首次执行时示值）（单位：kWh，换算：-2）
	unsigned long ulEleActualValue[5]; 	//当前电能示值（单位：kWh，换算：-2）
	unsigned long ucChargeEle[5];		//已充电量（单位：kWh，换算：-2）
	unsigned long ucChargeTime;		//已充时间（单位：s）
	unsigned long ucPlanPower;		//计划充电功率（单位：W，换算：-1）
	unsigned long ucActualPower;	//当前充电功率（单位：W，换算：-1）
	PHASE_LIST ucVoltage;			//当前充电电压（单位：V，换算：-1）
	PHASE_LIST ucCurrent;			//当前充电电流（单位：A，换算：-3）
	PILE_WORKSTATE ChgPileState;//充电桩状态（1：待机 2：工作 3：故障）
	char cUserID[66];   			//用户id  visible-string（SIZE(64)）
}CHARGE_EXE_STATE;/*路由器工作状态  即 充电计划单执行状态*/
//CCMRAM extern CHARGE_EXE_STATE Chg_ExeState;

/********************************** 自用记录单元 *************************************/

typedef union 
{
	rt_uint32_t Info;
	struct
	{
		rt_uint32_t Charge_Apply:1;		//	充电申请
		rt_uint32_t Charge_Apply_Ack:1;		//  充电申请应答
		rt_uint32_t Charge_Apply_Event:1;		//  充电申请事件上报
		rt_uint32_t Charge_Apply_Event_Ack:1;	//  充电申请事件上报应答
		rt_uint32_t Charge_Plan:1;		//  充电计划下发
		rt_uint32_t Charge_Plan_Ack:1;		//  充电计划下发应答
		rt_uint32_t Charge_Plan_Event:1;	//	充电计划上报事件
		rt_uint32_t Charge_Plan_Event_Ack:1;		//  充电计划上报事件应答
		rt_uint32_t Charge_Plan_Adj:1;			//	充电计划调整
		rt_uint32_t Charge_Plan_Adj_Ack:1;			//	充电计划调整应答
		rt_uint32_t Charge_Plan_Adj_Event:1;		// 充电计划调整事件上报
		rt_uint32_t Charge_Plan_Adj_Event_Ack:1;		//  充电计划调整事件应答
		rt_uint32_t Charge_Record_Event:1;		//  充电订单事件上报
		rt_uint32_t Charge_Record_Event_Ack:1; 	//	充电订单事件上报应答
		rt_uint32_t Router_Svc_Start:1;         //	路由器服务启动
		rt_uint32_t Router_Svc_Start_Ack:1;         //	路由器服务启动应答
		rt_uint32_t Router_Svc_Stop:1;         //	路由器服务停止
		rt_uint32_t Router_Svc_Stop_Ack:1;         //	路由器服务停止应答
		rt_uint32_t Charge_Power_Adj:1;					//充电功率调整
		rt_uint32_t Charge_Power_Adj_Ack:1;			//充电功率调整应答
		
		rt_uint32_t Router_Fault_Event:1;
		rt_uint32_t Router_Fault_Event_Ack:1;
		rt_uint32_t Pile_Fault_Event:1;
		rt_uint32_t Pile_Fault_Event_Ack:1;
	}
	Bit;
}CTRL_CHARGE_INFO;/*路由器故障*/


typedef struct
{
	CTRL_CHARGE_INFO Ctrl_Chg_Info;
	CTL_CHARGE Ctrl_ChgData;
	CTRL_TYPE CtrlType;			//控制类型{1：启动  2：停止  3：调整功率}
	CTRL_CMD_SOURCE StartSource;//启动源{1：4G启动  2:蓝牙启动}
	CTRL_CMD_SOURCE StopSource;	//停机源{1：4G停机  2:蓝牙停机}
}CTRL_CHARGE_EVENT;/*充电控制记录单元*/
CCMRAM extern CTRL_CHARGE_EVENT CtrlCharge_Event;

/********************************** 事件记录单元 *************************************/
typedef struct
{
	unsigned long OrderNum;				//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;		//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;	//  事件结束时间  
	unsigned char OccurSource;			//	事件发生源    NULL 		
	unsigned char ChannelState;			//  事件上报状态 = 通道上报状态
	CHARGE_EXE_STATE Chg_ExeState; 		//  当前充电执行状态 structure
}CHARGE_EXE_EVENT;/*充电执行事件记录单元*/

typedef struct
{
	unsigned long OrderNum;				//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;		//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;	//  事件结束时间  
	unsigned char OccurSource;			//	事件发生源    NULL     
	unsigned char ChannelState;			//  事件上报状态 = 通道上报状态
	
	char cUserID[66];   				//	用户id  visible-string（SIZE(64)）
	
	char RequestNO[18];					//	充电申请单号   （SIZE(16)）
	char AssetNO[24];					//	路由器资产编号 visible-string（SIZE(22)）
	GUN_NUM GunNum;						//	枪序号	{A枪（1）、B枪（2）}
	unsigned long ChargeReqEle;			//	充电需求电量（单位：kWh，换算：-2）
	STR_SYSTEM_TIME RequestTimeStamp;	//	充电申请时间
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	计划用车时间
	CHARGE_MODE ChargeMode;				//	充电模式 {正常（0），有序（1）}
	unsigned long StartMeterValue[5];	//	启动时电表表底值
	unsigned long StopMeterValue[5];	//	停止时电表表底值
	STR_SYSTEM_TIME	ChgStartTime;		//	充电启动时间
	STR_SYSTEM_TIME ChgStopTime;		//	充电停止时间
	unsigned long ucChargeEle[5];		//	已充电量（单位：kWh，换算：-2）
	unsigned long ucChargeTime;			//	已充时间（单位：s）
}CHG_ORDER_EVENT;/*充电订单事件记录单元*/


typedef struct
{
	unsigned long OrderNum;				//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;		//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;	//  事件结束时间  
	unsigned char OccurSource;			//	事件发生源    NULL     
	unsigned char ChannelState;			//  事件上报状态 = 通道上报状态
	
	ROUTER_FAULT Router_Fault;//路由器故障状态
	CHARGE_PILE_FAULT Pile_Fault;//充电桩故障状态
	
}ROUTER_FAULT_EVENT;/*充电订单事件记录单元*/

#endif

