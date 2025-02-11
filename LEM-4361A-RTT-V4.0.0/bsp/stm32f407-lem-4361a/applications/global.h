#ifndef __GLOBAL_H
#define __GLOBAL_H	

#include <rtthread.h>
#include  <rtconfig.h>
#include  <string.h>
#include  <stdarg.h>

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//LEM_4242A Board
//系统时钟初始化	
//鲁能智能@LNINT
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗必究。
//Copyright(C) 山东鲁能智能技术有限公司 2017-2099
//All rights reserved
//********************************************************************************
//修改说明
//无
//////////////////////////////////////////////////////////////////////////////////


#define CCMRAM __attribute__((section("ccmram")))

#define MY_HEX	1
#define MY_CHAR 2

#define MY_DEFINE(R)	#R

extern unsigned char DEBUG_MSH;

typedef struct{
	rt_uint16_t DataRx_len;
	rt_uint8_t Rx_data[1024];
	rt_uint16_t DataTx_len;
	rt_uint8_t Tx_data[1024];

}ScmUart_Comm;

//////////////////////////////////////////////////////////////////////////////////
typedef struct 				
{
	unsigned char  Second;        // 秒
	unsigned char  Minute;        // 分
	unsigned char  Hour;          // 时
	unsigned char  Day;           // 日
	
	unsigned char  Week;          //星期
	
	unsigned char  Month;         // 月
	unsigned char  Year;          // 年 后两位
}STR_SYSTEM_TIME;
extern STR_SYSTEM_TIME System_Time_STR;

/******************************** 路由器相关信息 ***********************************/	//zcx190710
//typedef enum {
//	RtSt_Starting=0,			// 开机中
//	RtSt_StandbyOK,          	// 待机正常
//	RtSt_CtrlPower,				// 控制输出功率中（包括执行计划和充电控制）
//	RtSt_Fault,           		// 故障
//	RtSt_Update,				// 升级中
//}ROUTER_WORKSTATE;/*路由器状态*/


typedef struct
{
	char AssetNum[24];					//路由器资产编号 字符串 maxlen=22
	char Addr[14];					//路由器通讯地址 字符串 maxlen=13
//	ROUTER_WORKSTATE WorkState;			//路由器运行状态
}ROUTER_INFO_UNIT;/*路由器信息单元*/
extern ROUTER_INFO_UNIT RouterInfo;

typedef struct
{
	unsigned char PileNum[18];			//充电桩编号         visible-string（SIZE(16)），
	rt_uint8_t PileIdent;       		//充电接口标识(A/B)
	unsigned char PileInstallAddr[50];	//充电桩的安装地址   visible-string，
	unsigned long minChargePow;			//充电桩最低充电功率 double-long（单位：W，换算：-1），
	unsigned long ulChargeRatePow;		//充电额定功率额定功率 double-long（单位：W，换算：-1）
	rt_uint8_t AwakeSupport;			//是否支持唤醒    {0:不支持 1：支持}
//	PILE_WORKSTATE WorkState;			//运行状态
}PILE_INFO_UNIT;/*充电桩信息单元*/
extern PILE_INFO_UNIT PileInfo;

typedef enum 
{
	ChgState_Standby=0,            //正常待机
	ChgState_PlanSoltsStart,           //计划执行中
	ChgState_PlanSoltsStarting,           //计划执行中
	ChgState_PlanSoltsStartFail,           //计划执行失败
	ChgState_InCharging,           //充电中
	ChgState_PlanExeEnd,           //计划完成
	ChgState_PlanSoltsStop,           //计划执行中
	ChgState_PlanSoltsStoping,
	ChgState_PlanSoltsStopFail,           //计划执行失败
	ChgState_Finished,				//充放电完成（3秒回待机）
	ChgState_Fault,            	//故障
	ChgState_Update,
}ROUTER_WORKSTATE;/*充电桩状态（路由器自用）*/

typedef enum 
{
	PILE_STANDBY=1,         //正常待机
	PILE_WORKING,           //工作中
	PILE_FAU,            	//故障
}PILE_WORKSTATE;/*充电桩状态（数据上送）*/

typedef union 
{
	rt_uint32_t Total_Fau;
	struct
	{
		rt_uint32_t Memory_Fau:1;		//	终端主板内存故障（0）
		rt_uint32_t RTC_Fau:1;		//  时钟故障        （1）
		rt_uint32_t Board_Fau:1;		//  主板通信故障    （2）
		rt_uint32_t CallMeter_Fau:1;	//  485抄表故障     （3）
		rt_uint32_t Screen_Fau:1;		//  显示板故障      （4）
		rt_uint32_t Hplc_Fau:1;		//  载波通道异常    （5）
		rt_uint32_t RAM_Fau:1;	//	内存初始化错误  （6）
		rt_uint32_t ESAM_Fau:1;		//  ESAM错误        （7）
		rt_uint32_t Ble_Fau:1;			//	蓝牙模块故障     （8）
		rt_uint32_t MeterCom_Fau:1;		//	计量单元通讯故障 	（9）
		rt_uint32_t CanCom_Fau:1;		//  充电桩通信故障 （10）
		rt_uint32_t ChgPile_Fau:1;		//  充电桩设备故障 （11）
		rt_uint32_t OrdRecord_Fau:1; 	//	本地订单记录满 （12）
		rt_uint32_t PowOver_Fau:1;         //	超功率限制充电故障  (13)
	}
	Bit;
}ROUTER_FAULT;/*路由器故障*/

typedef union 
{
	rt_uint32_t Total_Fau;
	struct
	{
		rt_uint32_t Memory_Fau:1;		//	终端主板内存故障（0）
		rt_uint32_t RTC_Fau:1;		//  时钟故障        （1）
		rt_uint32_t Board_Fau:1;		//  主板通信故障    （2）
		rt_uint32_t CallMeter_Fau:1;	//  485抄表故障     （3）
		rt_uint32_t Screen_Fau:1;		//  显示板故障      （4）
		rt_uint32_t Hplc_Fau:1;		//  载波通道异常    （5）
		rt_uint32_t RAM_Fau:1;	//	内存初始化错误  （6）
		rt_uint32_t ESAM_Fau:1;		//  ESAM错误        （7）
		rt_uint32_t Ble_Fau:1;			//	蓝牙模块故障     （8）
		rt_uint32_t MeterCom_Fau:1;		//	计量单元通讯故障 	（9）
		
		rt_uint32_t StopEct_Fau:1;        //急停动作故障  （10）
		rt_uint32_t	Arrester_Fau:1;       //避雷器故障	    （11）
    rt_uint32_t GunOut_Fau:1;            //充电枪未归位		（12）
  	rt_uint32_t TemptOV_Fau:1;     //充电桩过温故障		（13）
		rt_uint32_t VOVWarn:1;           //输出电压过压 			（14）
		rt_uint32_t VLVWarn:1;           //输入电压欠压					（15）
		rt_uint32_t Pilot_Fau:1;		   //充电中车辆控制导引故障     （16）   
		rt_uint32_t ACCon_Fau:1;		   //交流接触器故障	    （17）
		rt_uint32_t OutCurr_Fau:1;		   //输出过流告警		   （18）         
		rt_uint32_t CurrAct_Fau:1;	   //输出过流保护动作	    （19）					
		rt_uint32_t ACCir_Fau:1;		   //交流断路器故障（20）
		rt_uint32_t Lock_Fau:1;	   //充电接口电子锁故障状态（21）
		rt_uint32_t GunTempt_Fau:1;     //充电接口过温故障				（22）
		rt_uint32_t CC_Fau:1;				   //充电连接状态CC检测点4    （23）
		rt_uint32_t CP_Fau:1;				   //充电控制状态CP检测点1（24）
		rt_uint32_t PE_Fau:1;			   //PE断线故障（25）
		rt_uint32_t OTHERS_Fau:1;			   //PE断线故障（25）
	}
	Bit;
}CHARGE_PILE_FAULT;/*充电桩故障*/


typedef struct
{
	ROUTER_WORKSTATE Router_State;
	PILE_WORKSTATE	Pile_State;
	
	ROUTER_FAULT Router_Fault;//路由器故障信息
	CHARGE_PILE_FAULT Pile_Fault;//充电桩故障信息
	
}ROUTER_WORK_STATE;//路由器工作状态

extern ROUTER_WORK_STATE Router_WorkState;


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
}CTRL_CHARGE_INFO;/*路由器xinxi*/


typedef union 
{
	rt_uint32_t Info;
	struct
	{
		rt_uint32_t BLE_CONNECT:1;		//	蓝牙连接状态
	}
	Bit;
}ROUTER_MODULE_INFO;/*路由器xinxi*/


typedef struct
{
	CTRL_CHARGE_INFO Router_HPLC_Info;
	CTRL_CHARGE_INFO Router_BLE_Info;
	ROUTER_MODULE_INFO Router_Module_Info;
}CTRL_CHARGE_EVENT;/*充电控制记录单元*/
CCMRAM extern CTRL_CHARGE_EVENT CtrlCharge_Event;


	
//typedef struct
//{
//	char UserID[66];   			//用户id  visible-string（SIZE(64)）
//	char Token[40];   			//用户登录令牌  visible-string（SIZE(38)）
//	unsigned char AccountState;	//账户状态 {0：正常，1：欠费}
//}WHITE_LIST;/*路由器白名单*/

//typedef struct
//{
//	char kAddress[17];   		//通讯地址	visible-string（SIZE(16)）
//	char MeterNum[9];   		//表号  visible-string（SIZE(8)）
//	char KeyNum[9];				//密钥信息	visible-string（SIZE(8)）	
//}KEY_INFO_UNIT;/*路由器密钥信息单元*/


/******************************** 与控制器交互信息 ***********************************/
typedef enum
{
	DISORDER=0,
	ORDER,
}CHARGE_MODE;/*充电模式 {正常（0），有序（1）}*/

typedef enum
{
	CONNECT=0,
	DISCONNECT,
}PILE_COM_STATE;/*与桩通信状态 {正常（0），异常（1）}*/

typedef enum
{
	PLAN_CREATE=1,
	PLAN_ADJ,
}PILE_TYPE;/*策略类型 {生成（1） 、调整（2）*/

/******************************** 充电桩相关信息 ***********************************/	//zcx190807


typedef enum
{
	GUN_SINGLE,
	GUN_A,
	GUN_B,
}GUN_NUM;/*枪序号 {A枪（1）、B枪（2）}*/

typedef enum
{
	SEV_ENABLE=0,
	SEV_DISABLE,
}PILE_SERVICE;/*桩充电服务 {可用（0），停用（1）}*/

//typedef enum 
//{
//	PILE_NOFAULT        =0x00000000,
//	PILE_Memory_FAULT	=0x00000001,	//	终端主板内存故障（0）
//	PILE_Clock_FAULT	=0x00000002,	//  时钟故障        （1）
//	PILE_Board_FAULT	=0x00000004,	//  主板通信故障    （2）
//	PILE_MeterCom_FAULT	=0x00000008,	//  485抄表故障    （3）
//	PILE_Screen_FAULT	=0x00000010,	//  显示板故障      （4）
//	CardOffLine_FAULT	=0x00000020,	//	读卡器通讯中断  （5）
//	PILE_ESAM_FAULT		=0x00000040,	//  ESAM错误        （6）
//	StopEct_FAULT		=0x00000080,	//  急停按钮动作故障（7）
//	Arrester_FAULT		=0x00000100,	//	避雷器故障		（8）
//	GunHoming_FAULT		=0x00000200,	//	充电枪未归位		（9）
//	OverV_FAULT			=0x00000400,	//	输入过压告警		（10）
//	UnderV_FAULT		=0x00000800,	//	输入欠压告警		（11）
//	Pilot_FAULT			=0x00001000,	//	充电中车辆控制导引告警（12）
//	Connect_FAULT		=0x00002000,	//	交流接触器故障	（13）
//	OverI_Warning		=0x00004000,	//	输出过流告警		（14）
//	OverI_FAULT			=0x00008000,	//	输出过流保护动作	（15）
//	ACCir_FAULT			=0x00010000,	//	交流断路器故障	（16）
//	GunLock_FAULT		=0x00020000,	//	充电接口电子锁故障（17）
//	GunOverTemp_FAULT	=0x00040000,	//	充电接口过温故障	（18）
//	CC_FAULT			=0x00080000,	//	充电连接状态CC异常（19）
//	CP_FAULT			=0x00100000,	//	充电控制状态CP异常（20）
//	PE_FAULT			=0x00200000,	//	PE断线故障		（21）
//	Dooropen_FAULT		=0x00400000,	//	柜门打开故障		(22)
//	Other_FAULT			=0x00800000,	//	充电机其他故障	（23）
//}PILE_FAULT;/*充电桩故障类型*/


/******************************** 故障信息 ***********************************/		//zcx190710
//typedef enum 
//{
//	NO_FAU=0,
//	MEMORY_FAU,		//	终端主板内存故障（0）
//	CLOCK_FAU,		//  时钟故障        （1）
//	BOARD_FAU,		//  主板通信故障    （2）
//	METER_FAU,		//  485抄表故障     （3）
//	SCREEN_FAU,		//  显示板故障      （4）
//	HPLC_FAU,		//  载波通道异常    （5）
//	NANDFLSH_FAU,	//	NandFLASH初始化错误  （6）
//	ESAM_FAU,		//  ESAM错误        （7）
//	BLE_FAU,		//	蓝牙模块故障     （8）
//	BATTERY_FAU,	//	电源模块故障 	（9）
//	CANCOM_FAU,		//  充电桩通信故障 （10）
//	CHGPILE_FAU,	//  充电桩设备故障 （11）
//	ORDRECOED_FAU, 	//	本地订单记录满 （12）
//	RTC_FAU,		//	RTC通信故障 （13）
//}ROUTER_FAU;/*路由器故障类型*/

/************************************** 有序充电业务 *******************************************/
typedef enum {
	Cmd_Null=0,								//未收到指令
	
	Cmd_ChgRequest,							//蓝牙充电申请
	Cmd_ChgRequestAck,						//蓝牙充电申请应答
	
	Cmd_ChgPlanIssue, 						//充电计划下发
	Cmd_ChgPlanIssueAck,                 	//充电计划下发应答
	Cmd_ChgPlanOffer, 						//充电计划事件上报
	Cmd_ChgPlanOfferAck,                 	//充电计划上报事件应答
	
	Cmd_ChgPlanAdjust,                 		//充电计划调整
	Cmd_ChgPlanAdjustAck,                 	//充电计划调整应答

	Cmd_ChgRequestReport,					//充电申请事件上送
	Cmd_ChgRequestReportAck,				//充电申请事件上送应答

	Cmd_ChgPlanExeState,                    //充电计划执行状态事件上报
	Cmd_ChgPlanExeStateAck,                 //充电计划执行状态事件上报应答
//	Cmd_ChgRequestConfirm,					//充电申请确认（通知蓝牙）
	
	Cmd_StartChg,							//启动充电参数下发
	Cmd_StartChgAck,						//启动充电应答
	Cmd_StopChg,							//停止充电参数下发
	Cmd_StopChgAck,							//停止充电应答
	Cmd_PowerAdj,							//功率调节参数下发
	Cmd_PowerAdjAck,						//功率调节应答

	Cmd_ChgRecord,							//上送充电订单事件
	Cmd_ChgRecordAck,						//上送充电订单事件确认
	Cmd_DeviceFault,                      	//上送路由器异常状态
	Cmd_DeviceFaultAck,                      	//上送路由器异常状态应答
	Cmd_PileFault,                 			//上送充电桩异常状态
	Cmd_PileFaultAck,                 			//上送充电桩异常状态应答
	Cmd_ChgPlanIssueGetAck,
	
	Cmd_RouterExeState,                    	//路由器执行状态查询↓
	Cmd_RouterExeStateAck,                 	//路由器执行状态应答↑
	
	Cmd_STAOnlineState,						//STA监测自身及路由器在线状态↓
	Cmd_STAOnlineStateAck,					//STA监测自身及路由器在线状态确认↑
}COMM_CMD_C;//业务传输流程命令号
#define COMM_CMD_C rt_uint32_t

//定义事件类型
typedef enum {
	CTRL_NO_EVENT             	=0x00000000,
	ChgPlanIssue_EVENT       	=0x00000001,        	//充电计划下发事件
	ChgPlanIssueGet_EVENT     	=0x00000002,      		//充电计划召测事件	
	ChgPlanAdjust_EVENT    		=0x00000004,        	//充电计划调整事件
	StartChg_EVENT				=0x00000008,          	//启动充电事件
	StopChg_EVENT				=0x00000010,          	//停止充电事件
	PowerAdj_EVENT				=0x00000020,          	//调整功率事件
	AskState_EVENT				=0x00000040,          	//查询工作状态事件
	ChgRequest_EVENT			=0x00000080,          	//发起充电申请事件
	ChgReqReportConfirm_EVENT	=0x00000100,          	//充电申请确认事件（控制器→路由器）
}CTRL_EVENT_TYPE;//业务传输事件

//////////////////////////////////////////////////////////////////////////////////


extern unsigned long timebin2long(unsigned char *buf);
CCMRAM extern char Printf_Buffer[1024];
CCMRAM extern char Sprintf_Buffer[1024];


extern const char ProgramVersion[8]; // 版本号
extern void Kick_Dog(void);

extern unsigned char str2bcd(char*src,unsigned char *dest);
extern void BCD_toInt(unsigned char *data1,unsigned char *data2,unsigned char len);
extern void Int_toBCD(unsigned char *data1,unsigned char *data2,unsigned char len);
extern unsigned char bcd2str(unsigned char *src,char *dest,unsigned char count);
extern unsigned char CRC7(unsigned char *ptr,unsigned int cnt);
//extern unsigned short CRC16_CCITT_ISO(unsigned char *ptr, unsigned int count);
extern unsigned int CRC_16(unsigned char *ptr, unsigned int nComDataBufSize);
extern unsigned char CRC7(unsigned char *ptr,unsigned int count);
extern unsigned char XOR_Check(unsigned char *pData, unsigned int Len);
extern char *itoa(int num,char *str,int radix);
extern char* comm_cmdtype_to_string(COMM_CMD_C cmd);
extern void my_printf(char* buf,rt_uint32_t datalenth,rt_uint8_t type,rt_uint8_t cmd,char* head,char* function,char* name);
////////////////////////////////////////////////////////////////////////////////// 

#endif
