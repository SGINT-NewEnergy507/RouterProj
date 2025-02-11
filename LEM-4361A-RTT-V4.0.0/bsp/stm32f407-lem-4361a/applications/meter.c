#include <rtthread.h>
#include <rtdevice.h>
#include <meter.h>
#include <string.h>
#include <stdio.h>
#include <global.h>
#include <storage.h>
#include <analog.h>

//#define DEVICE_NAME			"uart5"//串口5设备名 RT_METER_USART

/* 串口接收事件标志*/
#define UART_RX_EVENT (1 << 5)

#define THREAD_METER_PRIORITY     25
#define THREAD_METER_STACK_SIZE   2048
#define THREAD_METER_TIMESLICE    5

#define TASK_NAME "[Meter]:"

//rt_thread_t meter_sig;					//meter thread
//rt_uint8_t meter_signal_flag;

static rt_device_t meter_serial;				//meter device

static struct rt_thread meter;					//meter thread
static struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 串口默认配置参数*/

static rt_uint8_t meter_stack[THREAD_METER_STACK_SIZE];//线程堆栈

static struct rt_event meter_uartevent;//用于接收数据的事件

//static ScmStorage_Msg stStorage_msg;

CCMRAM static ScmUart_Comm stMeter;//meter 串口数据结构体
static ScmMeter_PriceModle stMeter_PriceModle;//计费模型
static ScmMeter_Analog stMeter_Analog;//meter 模拟量数据结构体
static ScmMeter_HisData stMeter_HisData;//meter历史数据缓存结构体
static rt_uint32_t ulMeter_Half[48];		//每天每半个小时的电量

//static ScmMeter_Power	ulMeter_Day;
static ScmMeter_Power	ulMeter_MonthOld;
static ScmMeter_Power	ulMeter_TotalOld;
//static ScmMeter_Power	ulMeter_Month;

static rt_uint32_t s_ulevmin_start;//每半个小时起始的电量值
static rt_uint8_t s_ucMeter_TotalStart;//每半个小时记录起始电量的标志
static rt_uint8_t s_ucMeter_ClearHalf;//每半个小时记录起始电量的标志
static rt_uint8_t s_ucMeter_ClearMonth;

rt_uint8_t g_ucMeter_rxbuf[100];//串口接收缓存
rt_uint8_t g_ucMeter_rxlenth;//接收数据长度

static void cmMeter_electricity_calc(void);//统计历史电量信息
static void cmMeter_GJFModeInit(ScmMeter_PriceModle *pSTR);

//unsigned char cmMeterStateUpdate(ScmUart_Comm *pSTR);
static void cmMeterModbus_RecProtocal(ScmUart_Comm *pSTR);

static Power_Analog_TypeDef	ScmAnalog;

static rt_uint8_t save_meter_flag;


static rt_uint32_t g_ulMeter_Rx_Count;//收发超时判断计数
static rt_uint32_t g_ulMeter_Tx_Count;


//static ScmMeter_Analog get_Analog;// 测试用

static rt_err_t meter_rx_ind(rt_device_t dev, rt_size_t size)
{
	g_ucMeter_rxlenth = size;//接收数据长度
	rt_event_send(&meter_uartevent, UART_RX_EVENT);//发送数据接收事件
  return RT_EOK;
}

static void save_meter_data(Power_Analog_TypeDef* analog)//系统掉电 保存电量信息
{
	rt_uint32_t l_ulmeter_time;
	
	get_analog_data(analog);
	
	if((analog->Pow_5V < 4000)&&(save_meter_flag))//系统掉电 发送信号给meter 存储电量
	{
		save_meter_flag = 0;
		
		l_ulmeter_time= 0x20;
		l_ulmeter_time = ((l_ulmeter_time<<8)&0xff00)|System_Time_STR.Year;
		l_ulmeter_time = ((l_ulmeter_time<<8)&0xffff00)|System_Time_STR.Month;
		l_ulmeter_time = ((l_ulmeter_time<<8)&0xffffff00)|System_Time_STR.Day;
		SetStorageData(Cmd_MeterPowerWr,&stMeter_HisData,l_ulmeter_time);//保存电量信息
	
		SetStorageData(Cmd_MeterHalfPowerWr,&ulMeter_Half,l_ulmeter_time);//保存电量信息
	}
	else if(analog->Pow_5V > 4500)//系统恢复  清除标志
	{
		save_meter_flag = 1;
	}
}

/********************************************************************  
*	函 数 名: Meter_Commit_TimeOut
*	功能说明: 电度表收发数据超时处理
*	形    参: 无
*	返 回 值: 无
********************************************************************/
void Meter_Commit_TimeOut(void)
{
	g_ulMeter_Rx_Count++;
	g_ulMeter_Tx_Count++;
	
	if((g_ulMeter_Rx_Count>60)||(g_ulMeter_Tx_Count>60))
	{
		g_ulMeter_Rx_Count = 61;
		g_ulMeter_Tx_Count = 61;
		Router_WorkState.Router_Fault.Bit.MeterCom_Fau = RT_TRUE;//计量通讯异常
	}
	else
	{
		Router_WorkState.Router_Fault.Bit.MeterCom_Fau = RT_FALSE;//计量通讯恢复
	}
}

static void meter_thread_entry(void *parameter)//meter 线程
{
	rt_err_t res;
	rt_uint8_t l_uchour;
	rt_uint32_t e,time;
	int ret;
	STR_SYSTEM_TIME Systime;
	
		
	rt_uint8_t buf[8]={0x01,0x03,0x00,0x48,0x00,0x08,0xC4,0x1A};
	
	meter_serial = rt_device_find(RT_METER_USART);//查找设备
	
	if(meter_serial != RT_NULL)
	{
		config.baud_rate = BAUD_RATE_4800;
		config.data_bits = DATA_BITS_8;
		config.stop_bits = STOP_BITS_2;
		config.parity = PARITY_NONE;
		rt_device_control(meter_serial, RT_DEVICE_CTRL_CONFIG, &config);//配置串口参数
		if (rt_device_open(meter_serial, RT_DEVICE_FLAG_DMA_RX) == RT_EOK)//打开设备
		{
			rt_lprintf("[Meter]: (%s) Open serial %s sucess!\n",__func__,RT_METER_USART);
		}
	}
	else
	{
		res = RT_ERROR;
		rt_lprintf("[Meter]: (%s) Open serial %s err!\n",__func__,RT_METER_USART);
		return;
	}
	
	rt_event_init(&meter_uartevent, "uart_rx_event", RT_IPC_FLAG_FIFO);//初始化事件对象
	
	rt_device_set_rx_indicate(meter_serial, meter_rx_ind);//设置串口接收回调函数
	
	memset(&stMeter_Analog,0,sizeof(ScmMeter_Analog));
	memset(&stMeter_HisData,0,sizeof(ScmMeter_HisData));
	memset(&ulMeter_Half,0,sizeof(ulMeter_Half));
	
	
//	rt_signal_install(SIGUSR1, meter_sig_handle);
//  rt_signal_unmask(SIGUSR1);
	
	rt_thread_mdelay(5000);
	
	memcpy(&Systime,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
	
	ret = GetStorageData(Cmd_MeterGJFModeRd,&stMeter_PriceModle,sizeof(stMeter_PriceModle));//读取计费模型
	if(ret < 0)
	{
		cmMeter_GJFModeInit(&stMeter_PriceModle);
		SetStorageData(Cmd_MeterGJFModeWr,&stMeter_PriceModle,sizeof(stMeter_PriceModle));
		rt_lprintf("[Meter]: (%s) JFmode is not exist,clear stMeter_PriceModle!\n",__func__);
	}
	
	if((Systime.Year > 0x19)&&(Systime.Month<13))
	{
		time = 0x20;
		time = (((time <<8)&0xff00) | Systime.Year);
		time = (((time <<8)&0xffff00) | Systime.Month);
		time = (((time <<8)&0xffffff00) | Systime.Day);
		
		ret = GetStorageData(Cmd_MeterPowerRd,&stMeter_HisData,sizeof(stMeter_HisData));//读取电量
		if(ret < 0)
		{
			memset(&stMeter_HisData.ulMeter_Day,0,sizeof(stMeter_HisData));
			SetStorageData(Cmd_MeterPowerWr,&stMeter_HisData,sizeof(stMeter_HisData));
			rt_lprintf("[Meter]: (%s) total meter is not exist,clear half meter!\n",__func__);
		}
		else
		{
			ulMeter_TotalOld.ulPowerT = stMeter_HisData.ulMeter_Total.ulPowerT;
			ulMeter_TotalOld.ulPowerJ = stMeter_HisData.ulMeter_Total.ulPowerJ;
			ulMeter_TotalOld.ulPowerF = stMeter_HisData.ulMeter_Total.ulPowerF;
			ulMeter_TotalOld.ulPowerP = stMeter_HisData.ulMeter_Total.ulPowerP;
			ulMeter_TotalOld.ulPowerG = stMeter_HisData.ulMeter_Total.ulPowerG;
		}
	}
	BCD_toInt(&l_uchour,&Systime.Hour,1);
	
	if(l_uchour<24)
	{
		ret = GetStorageData(Cmd_MeterHalfPowerRd,&ulMeter_Half,sizeof(ulMeter_Half));//读取半小时电量
		
		if(ret < 0)
		{
			SetStorageData(Cmd_MeterHalfPowerWr,&ulMeter_Half,sizeof(ulMeter_Half));
			rt_lprintf("[Meter]: (%s) half meter is not exist,clear half meter!\n",__func__);
		}
		s_ulevmin_start = stMeter_HisData.ulMeter_Total.ulPowerT - ulMeter_Half[l_uchour*2];//若半小时内存在重启 需减去半小时内存储的电量 作为该时段的起始值
	}
	
	g_ulMeter_Tx_Count = 0;
	g_ulMeter_Rx_Count = 0;
	
	while (1)
	{
		rt_device_write(meter_serial, 0, &buf, sizeof(buf));
		g_ulMeter_Tx_Count = 0;
		
		res = rt_event_recv(&meter_uartevent, UART_RX_EVENT, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, 1000, &e);
		if(res== RT_EOK)
		{		
			if(rt_device_read(meter_serial, 0, g_ucMeter_rxbuf, g_ucMeter_rxlenth) != 1)
			{		
				cmMeterModbus_RecProtocal(&stMeter);
				my_printf((char*)stMeter.Rx_data,stMeter.DataRx_len,MY_HEX,0,TASK_NAME,(char*)(__func__),"RX:");				
			}
		}
		else
		{
				rt_lprintf("[meter]:meter_serial sem timeout. reply:%s\r\n", res);
		}
		cmMeter_electricity_calc();//每月每日尖峰平谷电量计算
		save_meter_data(&ScmAnalog);
		
		Meter_Commit_TimeOut();
//		cmMeter_get_data(EMMETER_ANALOG,&get_Analog);
//		
		rt_thread_mdelay(2000);
	}
}


/*
*********************************************************************************************************
*	函 数 名: MeterModbusStateUpdate
*	功能说明: Modbus规约解析函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
unsigned char cmMeterStateUpdate(ScmUart_Comm *pSTR)
{
	rt_uint32_t temp = 0;
	rt_uint8_t tmp_arry[4];
	rt_uint16_t Crc16;
	
	Crc16 = (unsigned int)CRC_16(&pSTR->Rx_data[0],pSTR->DataRx_len-2);
	if(((Crc16&0xff)==pSTR->Rx_data[pSTR->DataRx_len-2])&&(((Crc16>>8)&0xff)==pSTR->Rx_data[pSTR->DataRx_len-1]))
	{		
		tmp_arry[3] = pSTR->Rx_data[3];
		tmp_arry[2] = pSTR->Rx_data[4];
		tmp_arry[1] = pSTR->Rx_data[5];
		tmp_arry[0] = pSTR->Rx_data[6];	
		memcpy(&temp,tmp_arry,sizeof(char)*4);
		stMeter_Analog.ulVol = (unsigned long)(temp/1000);//XXXXXXX.X 交流电压
				
		tmp_arry[3] = pSTR->Rx_data[7];
		tmp_arry[2] = pSTR->Rx_data[8];
		tmp_arry[1] = pSTR->Rx_data[9];
		tmp_arry[0] = pSTR->Rx_data[10];
		memcpy(&temp,tmp_arry,sizeof(char)*4);
		stMeter_Analog.ulCur = (unsigned long)(temp/10);//XXXX.XXX 交流电流 
		if(stMeter_Analog.ulCur < 65)
			stMeter_Analog.ulCur = 0;
			

		tmp_arry[3] = pSTR->Rx_data[11];
		tmp_arry[2] = pSTR->Rx_data[12];
		tmp_arry[1] = pSTR->Rx_data[13];
		tmp_arry[0] = pSTR->Rx_data[14];
		memcpy(&temp,tmp_arry,sizeof(char)*4);
		stMeter_Analog.ulAcPwr = (unsigned long)(temp/10);    // XXXXXX.XXX	有功功率
		
		if(stMeter_Analog.ulAcPwr < 5000)
			stMeter_Analog.ulAcPwr = 0;
									
		tmp_arry[3] = pSTR->Rx_data[15];
		tmp_arry[2] = pSTR->Rx_data[16];
		tmp_arry[1] = pSTR->Rx_data[17];
		tmp_arry[0] = pSTR->Rx_data[18];
		memcpy(&temp,tmp_arry,sizeof(char)*4);
		stMeter_Analog.ulMeterTotal = (unsigned long)(temp/10);    // XXXXXX.XXX		有功总电度
		
		
		tmp_arry[3] = pSTR->Rx_data[19];
		tmp_arry[2] = pSTR->Rx_data[20];
		tmp_arry[1] = pSTR->Rx_data[21];
		tmp_arry[0] = pSTR->Rx_data[22];
		memcpy(&temp,tmp_arry,sizeof(char)*4);
		stMeter_Analog.ulPwrFactor = (unsigned long)(temp/10);    // XXXXXX.XX		功率因数
		
		tmp_arry[3] = pSTR->Rx_data[31];
		tmp_arry[2] = pSTR->Rx_data[32];
		tmp_arry[1] = pSTR->Rx_data[33];
		tmp_arry[0] = pSTR->Rx_data[34];
		memcpy(&temp,tmp_arry,sizeof(char)*4);
		stMeter_Analog.ulFrequency = (unsigned long)(temp);    // XXXXXX.XX		频率
		
		rt_lprintf("[meter]:---ChargVa = %d.%dV  ChargIa = %d.%03dA  MeterTotal = %d.%03dKWH----\n",stMeter_Analog.ulVol/10,stMeter_Analog.ulVol%10,\
			stMeter_Analog.ulCur/1000,stMeter_Analog.ulCur%1000,stMeter_Analog.ulMeterTotal/1000,stMeter_Analog.ulMeterTotal%1000);

		rt_lprintf("[meter]:----AcPwr = %d.%03dW  PwrFactor = %d.%02d  Frequency = %d.%02dHZ----\n",stMeter_Analog.ulAcPwr/1000,stMeter_Analog.ulAcPwr%1000,\
			stMeter_Analog.ulPwrFactor/100,stMeter_Analog.ulPwrFactor%100,stMeter_Analog.ulFrequency/100,stMeter_Analog.ulFrequency%100);

		g_ulMeter_Rx_Count = 0;		
		return 0;
	}
	else
	{

		rt_lprintf("[meter]:CRC16 ERR!!! CRC16 = %04X  Rec_crc=%02X%02X\r\n",Crc16,pSTR->Rx_data[pSTR->DataRx_len-2],pSTR->Rx_data[pSTR->DataRx_len-1]);
		return 1;
	}
}
/********************************************************************  
*	函 数 名: MeterModbus_RecProtocal
*	功能说明: Modbus底层数据接收函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/ 

void cmMeterModbus_RecProtocal(ScmUart_Comm *pSTR)
{   
	rt_uint8_t lenth,i,rx_ptr;
	
	rx_ptr = 0;
	for(i=0;i<g_ucMeter_rxlenth;i++)
	{
		if((g_ucMeter_rxbuf[rx_ptr] == 0x01)&&(g_ucMeter_rxbuf[rx_ptr+1] == 0x03)&&(g_ucMeter_rxbuf[rx_ptr+2] == 0x20)&&(rx_ptr<98))
		{
			lenth = g_ucMeter_rxbuf[rx_ptr+2]+5;
			pSTR->DataRx_len = lenth;
			memcpy(pSTR->Rx_data,&g_ucMeter_rxbuf[rx_ptr],lenth);
			cmMeterStateUpdate(pSTR);
			break;
		}
		else
		{
			rx_ptr++;
			if(rx_ptr>97)
			{
				rx_ptr = 0;
				break;
			}
		}
	}		
}

static void cmMeter_GJFModeInit(ScmMeter_PriceModle *pSTR)//山东地区计费信息
{
	rt_uint8_t i;
	
	memset(&pSTR->uiJFmodID,0,8);
	pSTR->EffectiveTime.Year = 0x19;
	pSTR->EffectiveTime.Month = 0x01;
	pSTR->EffectiveTime.Day = 0x01;
	pSTR->EffectiveTime.Hour = 0x00;
	pSTR->EffectiveTime.Minute = 0x00;
	pSTR->EffectiveTime.Second = 0x00;
	
	pSTR->unEffectiveTime.Year = 0x99;
	pSTR->unEffectiveTime.Month = 0x12;
	pSTR->unEffectiveTime.Day = 0x31;
	pSTR->unEffectiveTime.Hour = 0x00;
	pSTR->unEffectiveTime.Minute = 0x00;
	pSTR->unEffectiveTime.Second = 0x00;
	
	pSTR->state = 0x01;
	pSTR->style = 0x01;
	pSTR->count = 4;
	pSTR->ulPriceNo[0] = 10947;
	pSTR->ulPriceNo[1] = 9693;
	pSTR->ulPriceNo[2] = 6559;
	pSTR->ulPriceNo[3] = 3425;
	
	for(i = 0; i < 48;	i++)
	{
		pSTR->ulTimeNo[i] = 0;
	}
	
//	for(i = 0;i < 2;i++)//10:30~11:30 尖 21-22
//		pSTR->ulTimeNo[21+i] = 0x02;
//		
//	for(i = 0;i < 4;i++)//19:00~21:00 尖 38-41
//		pSTR->ulTimeNo[38+i] = 0x02;
//	
//	for(i = 0;i < 4;i++)//8:30~10:30 峰 17-20
//		pSTR->ulTimeNo[17+i] = 0x02;
//		
//	for(i = 0;i < 6;i++)//16:00~19:00 峰 32-37
//		pSTR->ulTimeNo[32+i] = 0x02;
//	
//	
//	for(i = 0;i < 3;i++)//7:00~8:30 平 14-16
//		pSTR->ulTimeNo[14+i] = 0x02;
//		
//	for(i = 0;i < 9;i++)//11:30~16:00 平 23-31
//		pSTR->ulTimeNo[23+i] = 0x02;
//		
//	for(i = 0;i < 4;i++)//21:00~23:00 平 42-45
//		pSTR->ulTimeNo[42+i] = 0x02;
//	
//	for(i = 0;i < 14;i++)//23:00~7:00 谷 0-13
//		pSTR->ulTimeNo[i] = 0x03;
//		
//	pSTR->ulTimeNo[46] = 0x03;//23:00~7:00 谷
//	pSTR->ulTimeNo[47] = 0x03;
}

static void cmMeter_electricity_calc(void)
{
	rt_uint8_t i;
	rt_uint8_t l_ucmonth,l_ucDay,l_uchour,l_ucminute;
	rt_uint32_t l_ulmeter_day_sum[4],l_ulmeter_month_sum[4];
	rt_uint32_t l_ulmeter_time;
	
	STR_SYSTEM_TIME time;
	
	memcpy(&time,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
	
	memset(l_ulmeter_day_sum,0,sizeof(l_ulmeter_day_sum));
	memset(l_ulmeter_month_sum,0,sizeof(l_ulmeter_month_sum));
	
	BCD_toInt(&l_ucmonth,&time.Month,1);
	BCD_toInt(&l_ucDay,&time.Day,1);
	BCD_toInt(&l_uchour,&time.Hour,1);
	BCD_toInt(&l_ucminute,&time.Minute,1);
	
	if((l_ucmonth>12)||(l_ucmonth<1)||(l_ucDay>31)||(l_ucDay<1)||(l_uchour>24)||(l_ucminute>60))//时间合法开始计算电量
	{
		rt_lprintf("[meter]:meter electricity calc time error!!!\r\n");//时间段计算错误  返回
		return;
	}
	
	l_ulmeter_time= 0x20;
	l_ulmeter_time = l_ulmeter_time<<8|time.Year;
	l_ulmeter_time = l_ulmeter_time<<8|time.Month;
	l_ulmeter_time = l_ulmeter_time<<8|time.Day;
	
	if((!l_uchour)&&(!s_ucMeter_ClearHalf))//日初保存电量信息  清零日电量信息
	{
		s_ucMeter_ClearHalf = 1;
		memset(ulMeter_Half,0,sizeof(ulMeter_Half));
		memcpy(&ulMeter_MonthOld,&stMeter_HisData.ulMeter_Month,sizeof(ScmMeter_Power));
		
		SetStorageData(Cmd_MeterPowerWr,&stMeter_HisData,l_ulmeter_time);//保存电量信息

		rt_lprintf("[meter]:new day start,save day meter to nandflash!\r\n");
	}
	else if(l_uchour)
	{
		s_ucMeter_ClearHalf = 0;
	}
	
	if((!(l_ucDay-1))&&(!s_ucMeter_ClearMonth))//月初 保存上月电量信息 清零变量 从新统计
	{
		SetStorageData(Cmd_MeterPowerWr,&stMeter_HisData,l_ulmeter_time);//保存电量信息
		
		s_ucMeter_ClearMonth = 1;
		memset(&ulMeter_MonthOld,0,sizeof(ScmMeter_Power));
		memset(&stMeter_HisData.ulMeter_Month,0,sizeof(ScmMeter_Power));

		rt_lprintf("[meter]:new month start,save month meter to nandflash!\r\n");
	}
	else if(l_ucDay-1)
	{
		s_ucMeter_ClearMonth = 0;
	}

	if(l_ucminute%30)
	{
		s_ucMeter_TotalStart = 0;
		ulMeter_Half[2*l_uchour+l_ucminute/30] = stMeter_Analog.ulMeterTotal-s_ulevmin_start;
	}
	else
	{
		if(!s_ucMeter_TotalStart)
		{
			s_ucMeter_TotalStart = 1;
			s_ulevmin_start = stMeter_Analog.ulMeterTotal;
		}		
	}
	for(i=0;i<48;i++)
	{
		l_ulmeter_day_sum[stMeter_PriceModle.ulTimeNo[i]] +=ulMeter_Half[i];	//实时计算当日尖峰平谷总电量
	}
	stMeter_HisData.ulMeter_Day.ulPowerJ = l_ulmeter_day_sum[0];//某日总尖电量
	stMeter_HisData.ulMeter_Day.ulPowerF = l_ulmeter_day_sum[1];//某日总峰电量
	stMeter_HisData.ulMeter_Day.ulPowerP = l_ulmeter_day_sum[2];//某日总平电量
	stMeter_HisData.ulMeter_Day.ulPowerG = l_ulmeter_day_sum[3];//某日总谷电量
	
	stMeter_HisData.ulMeter_Day.ulPowerT = stMeter_HisData.ulMeter_Day.ulPowerJ+stMeter_HisData.ulMeter_Day.ulPowerF\
																				+stMeter_HisData.ulMeter_Day.ulPowerP+stMeter_HisData.ulMeter_Day.ulPowerG;//某日总电量
		
	stMeter_HisData.ulMeter_Month.ulPowerJ = ulMeter_MonthOld.ulPowerJ+stMeter_HisData.ulMeter_Day.ulPowerJ;//月尖总
	stMeter_HisData.ulMeter_Month.ulPowerF = ulMeter_MonthOld.ulPowerF+stMeter_HisData.ulMeter_Day.ulPowerF;//月峰总
	stMeter_HisData.ulMeter_Month.ulPowerP = ulMeter_MonthOld.ulPowerP+stMeter_HisData.ulMeter_Day.ulPowerP;//月平总
	stMeter_HisData.ulMeter_Month.ulPowerG = ulMeter_MonthOld.ulPowerG+stMeter_HisData.ulMeter_Day.ulPowerG;//月谷总
	
	stMeter_HisData.ulMeter_Month.ulPowerT = stMeter_HisData.ulMeter_Month.ulPowerJ+stMeter_HisData.ulMeter_Month.ulPowerF\
																					+stMeter_HisData.ulMeter_Month.ulPowerP+stMeter_HisData.ulMeter_Month.ulPowerG;//月总
	
	stMeter_HisData.ulMeter_Total.ulPowerJ = ulMeter_TotalOld.ulPowerJ+stMeter_HisData.ulMeter_Day.ulPowerJ;//月尖总
	stMeter_HisData.ulMeter_Total.ulPowerF = ulMeter_TotalOld.ulPowerF+stMeter_HisData.ulMeter_Day.ulPowerF;//月峰总
	stMeter_HisData.ulMeter_Total.ulPowerP = ulMeter_TotalOld.ulPowerP+stMeter_HisData.ulMeter_Day.ulPowerP;//月平总
	stMeter_HisData.ulMeter_Total.ulPowerG = ulMeter_TotalOld.ulPowerG+stMeter_HisData.ulMeter_Day.ulPowerG;//月谷总
	
	stMeter_HisData.ulMeter_Total.ulPowerT = stMeter_HisData.ulMeter_Total.ulPowerJ+stMeter_HisData.ulMeter_Total.ulPowerF\
																					+stMeter_HisData.ulMeter_Total.ulPowerP+stMeter_HisData.ulMeter_Total.ulPowerG;//月总
}

void cmMeter_get_data(unsigned char cmd,void* str_data)//其他线程调用函数 获取Meter数据信息
{
	switch(cmd)
	{
		case EMMETER_ANALOG:
//			memcpy(&((ScmMeter_Analog*)str_data)->ulVol,&stMeter_Analog.ulVol,sizeof(ScmMeter_Analog));
			*((ScmMeter_Analog*)str_data) = stMeter_Analog;
		break;
		case EMMETER_HISDATA:
			*((ScmMeter_HisData*)str_data) = stMeter_HisData;
//			memcpy(&((ScmMeter_HisData*)str_data)->ulMeter_Total,&stMeter_HisData.ulMeter_Total,sizeof(ScmMeter_HisData));
		break;
		default:
			break;
	}
}


int meter_thread_init(void)
{
	rt_err_t res;
	
	res=rt_thread_init(&meter,
											"meter",
											meter_thread_entry,
											RT_NULL,
											meter_stack,
											THREAD_METER_STACK_SIZE,
											THREAD_METER_PRIORITY,
											THREAD_METER_TIMESLICE);
	if (res == RT_EOK) 
	{
//			meter_sig = &meter;
			rt_thread_startup(&meter);
	}
	return res;
}


#if defined (RT_METER_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
	INIT_APP_EXPORT(meter_thread_init);
#endif
MSH_CMD_EXPORT(meter_thread_init, meter thread run);


