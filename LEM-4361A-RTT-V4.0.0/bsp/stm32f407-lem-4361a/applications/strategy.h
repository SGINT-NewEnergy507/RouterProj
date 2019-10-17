#ifndef __STRATEGY_H__
#define __STRATEGY_H__

#include <string.h>
#include <stdio.h>
#include "global.h"
#include "chargepile.h"


extern struct rt_semaphore rt_sem_bluetoothfau;

/******************************** 与控制器交互信息 ***********************************/
typedef enum
{
	DISORDER=0,
	ORDER,
}CHARGE_MODE;/*充电模式 {正常（0），有序（1）}*/

typedef enum
{
	EXE_NULL=0,
	EXE_ING,
	EXE_END,
	EXE_FAILED,
}EXESTATE;/*执行状态 {0：未执行  1：正常执行 2：执行结束 3：执行失败}*/

typedef enum
{
	CONNECT=0,
	DISCONNECT,
}PILE_COM_STATE;/*与桩通信状态 {正常（0），异常（1）}*/

typedef enum
{
	CTRL_NULL=0,
	CTRL_START,
	CTRL_STOP,
	CTRL_ADJPOW,
}CTRL_TYPE;/*{0：未控制 1：启动  2：停止  3：调整功率}*/


/******************************* 充电计划 *************************************/
typedef struct
{
	STR_SYSTEM_TIME strDecStartTime;//起始时间
	STR_SYSTEM_TIME strDecStopTime;	//结束时间
	unsigned long ulChargePow;		//充电功率设定值（单位：kW，换算：-4）
}CHARGE_TIMESOLT;/*时段负荷单元*/

typedef struct
{
	char cRequestNO[17];			//申请单号  octet-string（SIZE(16)）
	char cAssetNO[23];				//路由器资产编号  visible-string（SIZE(22)）
	unsigned char GunNum;			//枪序号	{A枪（1）、B枪（2）}
	char cUserID[65];   			//用户id  visible-string（SIZE(64)）	
	unsigned char ucDecMaker;		//决策者  {主站（1）、控制器（2）}
	unsigned char ucDecType; 		//决策类型{生成（1） 、调整（2）}
	STR_SYSTEM_TIME strDecTime;		//决策时间	
	
	unsigned long ulChargeReqEle;	//充电需求电量（单位：kWh，换算：-2）
	unsigned long ulChargeRatePow;	//充电额定功率 （单位：kW，换算：-4）
	unsigned char ucChargeMode;		//充电模式 {正常（0），有序（1）}
	unsigned char ucTimeSlotNum;	//时间段数量
	CHARGE_TIMESOLT strChargeTimeSolts[50];//时间段内容，最大50段

}CHARGE_STRATEGY;/*充电计划单*/
extern CHARGE_STRATEGY Chg_Strategy;//下发计划单
extern CHARGE_STRATEGY Adj_Chg_Strategy;//变更计划单

typedef struct
{
	char cRequestNO[17];	//申请单号  octet-string（SIZE(16)）
	char cAssetNO[23];		//路由器资产编号  visible-string（SIZE(22)）
	unsigned char cSucIdle;	//成功或失败原因:{0：成功 1：失败 255：其他}
}CHARGE_STRATEGY_RSP;/*充电计划单响应*/


/****************************************** 充电申请 **********************************************/
typedef struct
{
	char cRequestNO[17];				//	申请单号  octet-string（SIZE(16)）
	char cAssetNO[23];					//	路由器资产编号  visible-string（SIZE(22)） 
	unsigned char GunNum;				//	枪序号	{A枪（1）、B枪（2）}
	char cUserID[65];   				//	用户id  visible-string（SIZE(64)）
	unsigned long ulChargeReqEle;		//	充电需求电量（单位：kWh，换算：-2）
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	计划用车时间
	unsigned char ChargeMode;			//	充电模式 {正常（0），有序（1）}
	char Token[33];   					//	用户登录令牌  visible-string（SIZE(32)）
}CHARGE_APPLY;/*充电申请单(BLE)*/

typedef struct
{
	char cRequestNO[17];	//申请单号  octet-string（SIZE(16)）
	char cAssetNO[23];		//路由器资产编号  visible-string（SIZE(22)）
	unsigned char cSucIdle;	//成功或失败原因:{0：成功 1：失败 255：其他}
}CHARGE_APPLY_RSP;/*充电申请单响应*/

/******************************** 事件信息记录 ***********************************/
typedef struct
{
	unsigned long OfflinePeriod;		//本次离线时常（单位：秒）
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
}ONLINE_STATE;/*表计在线状态*/

typedef struct
{
	unsigned long OrderNum;			//	事件记录序号 
	STR_SYSTEM_TIME StartTimestamp;	//  事件发生时间  
	STR_SYSTEM_TIME FinishTimestamp;//  事件结束时间 
	unsigned char OccurSource;		//	事件发生源    NULL 	
//	unsigned char Reason;			//  事件发生原因     
	unsigned char ChannelState;		//  事件上报状态 = 通道上报状态
	char RequestNO[17];				//	充电申请单号   （SIZE(16)）
	char AssetNO[23];				//	路由器资产编号 visible-string（SIZE(22)）
	unsigned char GunNum;			//	枪序号	{A枪（1）、B枪（2）}
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
	
	char RequestNO[17];					//	充电申请单号   （SIZE(16)） 
	char AssetNO[23];					//	路由器资产编号 visible-string（SIZE(22)）
	unsigned char GunNum;				//	枪序号	{A枪（1）、B枪（2）}
	
	STR_SYSTEM_TIME RequestTimeStamp;	//	充电申请时间
	unsigned long actSOC;				//	当前SOC（单位：%，换算：-2）
	unsigned long aimSOC;				//  目标SOC（单位：%，换算：-2）
	unsigned long CellCapacity;			//	电池容量（单位：kWh，换算：-2）
	unsigned long ChargeReqEle;			//	充电需求电量（单位：kWh，换算：-2）
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	计划用车时间
	unsigned char ChargeMode;			//	充电模式 {正常（0），有序（1）}
	char Token[39];   					//	用户登录令牌  visible-string（SIZE(38)）
	char UserAccount[10];				//  充电用户账号  visible-string（SIZE(9)）
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






#endif

