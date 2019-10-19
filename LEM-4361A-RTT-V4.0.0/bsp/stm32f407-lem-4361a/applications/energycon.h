#ifndef __ENERGYCON_H__
#define __ENERGYCON_H__

#include <string.h>
#include <stdio.h>
#include "global.h"
#include "chargepile.h"

extern ChargPilePara_TypeDef ChargePilePara_Set;
extern ChargPilePara_TypeDef ChargePilePara_Get;


extern struct rt_thread energycon;
extern struct rt_semaphore rx_sem_setpower;
extern struct rt_semaphore rx_sem_adjpower;


/******************************* 充电控制 *************************************/
typedef struct
{
	char OrderSn[17];			//订单号  octet-string（SIZE(16)）
	char cAssetNO[23];			//路由器资产编号  visible-string（SIZE(22)）
	unsigned char GunNum;		//枪序号	{A枪（1）、B枪（2）}
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
	char cAssetNO[23];		//路由器资产编号  visible-string（SIZE(22)）
	unsigned char GunNum;	//枪序号	{A枪（1）、B枪（2）}
}CHARGE_EXE_STATE_ASK;/*路由器工作状态查询*/

typedef struct
{
	char cRequestNO[17];			//申请单号  octet-string（SIZE(16)）
	char cAssetNO[23];				//路由器资产编号  visible-string（SIZE(22)）
	unsigned char GunNum;			//枪序号	{A枪（1）、B枪（2）}
	unsigned char exeState;			//执行状态 {1：正常执行 2：执行结束 3：执行失败}
	unsigned char ucTimeSlotNum;	//时间段数量
	unsigned long ulEleBottomValue[5]; 	//电能示值底值（充电首次执行时示值）（单位：kWh，换算：-2）
	unsigned long ulEleActualValue[5]; 	//当前电能示值（单位：kWh，换算：-2）
	unsigned long ucChargeEle[5];		//已充电量（单位：kWh，换算：-2）
	unsigned long ucChargeTime;		//已充时间（单位：s）
	unsigned long ucPlanPower;		//计划充电功率（单位：W，换算：-1）
	unsigned long ucActualPower;	//当前充电功率（单位：W，换算：-1）
	PHASE_LIST ucVoltage;			//当前充电电压（单位：V，换算：-1）
	PHASE_LIST ucCurrent;			//当前充电电流（单位：A，换算：-3）
	unsigned char ChgPileState;		//充电桩状态（1：待机 2：工作 3：故障）
}CHARGE_EXE_STATE;/*路由器工作状态  即 充电计划单执行状态*/
CCMRAM extern CHARGE_EXE_STATE Chg_ExeState;

/********************************** 自用记录单元 *************************************/
typedef struct
{
	char OrderSn[17];			//订单号  octet-string（SIZE(16)）
	char cAssetNO[23];			//路由器资产编号  visible-string（SIZE(22)）
	unsigned char GunNum;		//枪序号	{A枪（1）、B枪（2）}
	unsigned char CtrlType;		//控制类型{1：启动  2：停止  3：调整功率}
	unsigned char StartType;	//启动类型{1：4G启动  2:蓝牙启动}
	unsigned char StopType;		//停机类型{1：4G停机  2:蓝牙停机}
	unsigned long SetPower;		//设定充电功率（单位：W，换算：-1）
	unsigned char cSucIdle;		//成功或失败原因:{0：成功 1：失败 255：其他}
}CTL_CHARGE_EVENT;/*充电控制记录单元*/

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
	
	char cUserID[65];   				//	用户id  visible-string（SIZE(64)）
	
	char RequestNO[17];					//	充电申请单号   （SIZE(16)）
	char AssetNO[23];					//	路由器资产编号 visible-string（SIZE(22)）
	unsigned char GunNum;				//	枪序号	{A枪（1）、B枪（2）}
	unsigned long ChargeReqEle;			//	充电需求电量（单位：kWh，换算：-2）
	STR_SYSTEM_TIME RequestTimeStamp;	//	充电申请时间
	STR_SYSTEM_TIME	PlanUnChg_TimeStamp;//	计划用车时间
	unsigned char ChargeMode;			//	充电模式 {正常（0），有序（1）}
	unsigned long StartMeterValue[5];	//	启动时电表表底值
	unsigned long StopMeterValue[5];	//	停止时电表表底值
	STR_SYSTEM_TIME	ChgStartTime;		//	充电启动时间
	STR_SYSTEM_TIME ChgStopTime;		//	充电停止时间
	unsigned long ucChargeEle[5];		//	已充电量（单位：kWh，换算：-2）
	unsigned long ucChargeTime;			//	已充时间（单位：s）
}CHG_ORDER_EVENT;/*充电订单事件记录单元*/

#endif

