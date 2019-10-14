#include <rtthread.h>
#include <bluetooth.h>
#include <string.h>
#include <stdio.h>
#include <global.h>
//#include <meter.h>
//#include <analog.h>
#include <board.h>
#include <698.h>
#include <storage.h>


#define FUNC_PRINT_RX	"[bluetooth]:RX:"
#define FUNC_PRINT_TX	"[bluetooth]:TX:"

#define BLE_PIN    GET_PIN(E, 5)

#define BLE_PWR_ON()	{rt_pin_write(BLE_PIN, PIN_LOW);}//模块上电
#define BLE_PWR_OFF()	{rt_pin_write(BLE_PIN, PIN_HIGH);}//模块掉电

/* UART3接收事件标志*/
#define UART3_RX_EVENT (1 << 3)



#define THREAD_BLUETOOTH_PRIORITY     23
#define THREAD_BLUETOOTH_STACK_SIZE   1024
#define THREAD_BLUETOOTH_TIMESLICE    5

static struct rt_thread bluetooth;
static rt_device_t bluetooth_serial;
static rt_uint8_t bluetooth_stack[THREAD_BLUETOOTH_STACK_SIZE];//线程堆栈


char* AT_CmdDef[]={

	"ATE0\r\n",		//关闭回显功能
	"AT+BLEINIT=2\r\n",			//BLE 初始化，设置为Server模式
	"AT+BLENAME=\"LN000000000001\"\r\n",	//设置 BLE 设备名称
	"AT+BLEADDR=1,\"f1:f2:f3:f4:f5:f6\"\r\n",
	
	"AT+BLEGATTSSRVCRE\r\n",	//创建GATTS 服务
	"AT+BLEGATTSSRVSTART\r\n",	//开启GATTS 服务
	
	"AT+BLEADVPARAM=32,64,0,1,7\r\n",							//配置广播参数
	"AT+BLEADVDATA=\"0201060F094C4E3030303030303030303030310303E0FF\"\r\n",//配置扫描响应数据		
	"AT+BLEADVSTART\r\n",		//开始广播
	
	"AT+BLESPPCFG=1,1,6,1,6\r\n",	//配置BLE透传模式
	"AT+BLESPP\r\n",				//开启透传模式
	
	"+++",						//退出透传模式
	"AT+BLEDISCONN\r\n",	//断开连接
	
	"AT+RST\r\n",	//重启模块

};
typedef enum 		 //AT指令
{
	BLE_ATE = 0,
	BLE_INIT,
	BLE_NAME,
	BLE_ADDR_SET,
	
	BLE_GATTS_SRV,
	BLE_GATTS_START,
	
	BLE_ADV_PARAM,
	BLE_ADV_DATA,
	BLE_ADV_START,
	
	BLE_SPP_CFG,
	BLE_SPP,
	
	BLE_QUIT_TRANS,
	BLE_DISCONN,

	BLE_RESET,
	
	BLE_NULL,
}BLE_AT_CMD;

typedef enum 		 //后台连接数据类型
{
	AT_MODE = 1,
	TRANS_MODE,
}PROTOCOL_MODE;

static BLE_AT_CMD BLE_ATCmd;
static BLE_AT_CMD BLE_ATCmd_Old;
static rt_uint8_t BLE_ATCmd_Count;
static PROTOCOL_MODE g_ucProtocol;

static struct rt_event bluetooth_event;//用于接收数据的信号量

static ScmUart_Comm stBLE_Comm;

static rt_uint32_t g_ulRx_Count;
static rt_uint32_t g_ulTx_Count;
static rt_uint32_t g_ulRx_Size;
static rt_uint8_t g_ucRecv698_In_AT;//在AT指令模式下  收到了698协议数据
static rt_uint8_t _698_data_buf[255];

struct _698_BLE_FRAME _698_ble_frame;

static rt_err_t check_698_data_to_buf(ScmUart_Comm* stData,rt_uint8_t* buf);

extern int _698_HCS(unsigned char *data, int start_size,int size,unsigned short HCS);
extern int _698_FCS(unsigned char *data, int start_size,int size,unsigned short FCS);
extern int tryfcs16(unsigned char *cp, int len);

static void BLE_Trans_Send(rt_device_t dev,rt_uint32_t cmd,rt_uint8_t reason,ScmUart_Comm* stData);


/********************************************************************  
*	函 数 名: Uart_Recv_Data
*	功能说明: 串口数据接收
*	形    参: 无
*	返 回 值: 无
********************************************************************/
void BLE_Commit_TimeOut(void)
{
	g_ulRx_Count++;
	g_ulTx_Count++;
	
	if((g_ulRx_Count>60)||(g_ulTx_Count>60))
	{
		g_ulRx_Count = 100;
		g_ulTx_Count = 100;
		BLE_ATCmd = BLE_QUIT_TRANS;
		g_ucProtocol = AT_MODE;
	}
}

/********************************************************************  
*	函 数 名: Uart_Recv_Data
*	功能说明: 串口数据接收
*	形    参: 无
*	返 回 值: 无
********************************************************************/
rt_err_t Uart_Recv_Data(rt_device_t dev,ScmUart_Comm* stData)
{
//	rt_uint8_t i,rx_len,rx_ptr;
	
	if(dev == RT_NULL)
		return RT_ERROR;
	
//	rx_len = 0;
//	rx_ptr = 0;

//	while((rx_len = rt_device_read(dev, 0, stData->Rx_data+rx_ptr, 255)) > 0)//循环读取串口数据 一直读完  数据缓存在rx_data中
//	{
//		rx_ptr += rx_len;
//		if(rx_ptr>254)
//		{	
//			rx_ptr = 0;
//			rt_kprintf("[bluetooth]:ble_recv error!!!\n");
//			return RT_ERROR;
//		}
//		rt_kprintf("[bluetooth]:ble_recv: rx_ptr = %d\n",rx_ptr);
//	}
//	stData->DataRx_len = rx_ptr;
	stData->DataRx_len = rt_device_read(dev, 0, stData->Rx_data, g_ulRx_Size);
	g_ulRx_Count = 0;
	g_ulRx_Size = 0;
	
//	if(BLE_ATCmd == BLE_NULL)
//	{
//	}
//	else 
	
	if(g_ucProtocol == AT_MODE)
	{
		if(stData->DataRx_len)
		{
			my_printf((char*)stData->Rx_data,stData->DataRx_len,MY_CHAR,1,FUNC_PRINT_RX);
		}
	}
	else
	{
		if(stData->DataRx_len)
		{
			my_printf((char*)stData->Rx_data,stData->DataRx_len,MY_HEX,1,FUNC_PRINT_RX);
		}
	}
	
	return RT_EOK;
}

/********************************************************************  
*	函 数 名: BLE_Send_AT_TimeOut
*	功能说明: AT指令配置超时处理函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/

void BLE_ATCmd_TimeOut(BLE_AT_CMD at_cmd)
{
	if(BLE_ATCmd_Old == at_cmd)
	{
		BLE_ATCmd_Count++;
		if(BLE_ATCmd_Count>10)
		{
			BLE_PWR_ON();
			BLE_ATCmd = BLE_ATE;
			BLE_ATCmd_Old = BLE_NULL;
			BLE_ATCmd_Count = 0;
		}
		else if(BLE_ATCmd_Count>6)//蓝牙模块响应超时 掉电重启
		{
			BLE_PWR_OFF();		
		}
		rt_kprintf("[bluetooth]:ble_send repeate times is %d\n",BLE_ATCmd_Count);
	}
	else
	{
		BLE_ATCmd_Old = at_cmd;
		BLE_ATCmd_Count = 0;
	}
}

/********************************************************************  
*	函 数 名: BLE_ATCmd_Send
*	功能说明: AT指令配置函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/

void BLE_ATCmd_Send(rt_device_t dev,BLE_AT_CMD at_cmd)
{
	rt_size_t size;
	
	if((dev == RT_NULL)||(at_cmd == BLE_NULL))
		return;
	
	size = rt_device_write(dev, 0, AT_CmdDef[at_cmd], strlen(AT_CmdDef[at_cmd]));
	
	if(size == strlen(AT_CmdDef[at_cmd]))
	{
		rt_kprintf("[bluetooth]:ble_send: at_cmd is %s\n",AT_CmdDef[at_cmd]);
	}
	
	BLE_ATCmd_TimeOut(at_cmd);//AT指令发送超时处理
}
/********************************************************************  
*	函 数 名: BLE_ATCmd_Recv
*	功能说明: AT指令配置函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/

void BLE_ATCmd_Recv(rt_device_t dev,BLE_AT_CMD at_cmd,ScmUart_Comm* stData)//AT指令接收处理
{
	rt_uint32_t i;
	
	if(dev == RT_NULL)
		return;
	
	switch(at_cmd)
	{
		case BLE_ATE:
		case BLE_INIT:
		case BLE_NAME:
		case BLE_ADDR_SET:
		case BLE_GATTS_SRV:
		case BLE_GATTS_START:
		case BLE_ADV_PARAM:
		case BLE_ADV_DATA:
		case BLE_ADV_START:
		case BLE_SPP_CFG:
		case BLE_QUIT_TRANS:
		{
			if(BLE_ATCmd == BLE_SPP_CFG)
			{
				BLE_ATCmd =BLE_NULL;
			}
			else if(strstr((char*)(stData->Rx_data),"OK"))
			{
				BLE_ATCmd++;
			}
			break;
		}
		case BLE_SPP:
		{
			for(i=0;i<stData->DataRx_len;i++)
			{
				if((stData->Rx_data[i]== 0x68)&&(stData->Rx_data[i+1]== 0x17))
				{
					rt_uint8_t buf[]={0x68,0x17,0x00,0x43,0x45,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x10,0xDA,0x5F,0x05,0x01,0x00,0x40,0x01,0x02,0x00,0x00,0xED,0x03,0x16};
//					memcpy(_698_data_recv.priveData,buf,sizeof(buf));
						memcpy(_698_data_buf,buf,sizeof(buf));				
						g_ucRecv698_In_AT = 1;//收到698协议数据
				}
			}
			
			
			if(strstr((char*)(stData->Rx_data),"OK"))
			{
				BLE_ATCmd = BLE_NULL;
				g_ucProtocol = TRANS_MODE;
				rt_event_send(&bluetooth_event, UART3_RX_EVENT);
			}
			break;
		}
		case BLE_DISCONN:
		{
			if(strstr((char*)(stData->Rx_data),"OK"))
			{
				BLE_ATCmd = BLE_ADV_START;
			}
			break;
		}
		case BLE_RESET:
			break;
		case BLE_NULL:
		{		
			if(strstr((char*)(stData->Rx_data),"+BLECONN"))
			{
				BLE_ATCmd = BLE_SPP;
			}
			if(strstr((char*)(stData->Rx_data),"+BLEDISCONN"))
			{
				BLE_ATCmd = BLE_QUIT_TRANS;
				g_ucProtocol = AT_MODE;
			}
		}
			break;
		default:
			break;
	}
//	memset(stData->Rx_data,0,stData->DataRx_len);
//	stData->DataRx_len = 0;
}
rt_uint32_t BLE_698_Data_Package(struct _698_BLE_FRAME *dev_recv,rt_uint8_t user_data_len,ScmUart_Comm* stData)
{
//		rt_uint8_t i,lenth,size;
		rt_uint16_t total_lenth;
	
//		stData->Tx_data[0] 							= dev_recv->head;
//		stData->Tx_data[1] 							= dev_recv->datalenth.uclenth[0];
//		stData->Tx_data[2] 							= dev_recv->datalenth.uclenth[1];
//		
//		stData->Tx_data[3] 							= dev_recv->control.ucControl|0x80;
//		stData->Tx_data[4] 							= dev_recv->_698_ADDR.S_ADDR.SA;
//	
//		lenth = dev_recv->_698_ADDR.S_ADDR.B.uclenth+1;
//		for(i=0;i<lenth;i++)
//			stData->Tx_data[5+i] 					= dev_recv->_698_ADDR.addr[i];
//		stData->Tx_data[5+lenth]				= dev_recv->_698_ADDR.CA;
//			
//		stData->Tx_data[8+lenth]				= dev_recv->user_data.apdu_cmd|0x80;
//		stData->Tx_data[9+lenth]				= dev_recv->user_data.apdu_cmd_type;
//		stData->Tx_data[10+lenth]				= dev_recv->user_data.apdu_piid;
//		stData->Tx_data[11+lenth]				= dev_recv->user_data.apdu_oad.ucoad[3];
//		stData->Tx_data[12+lenth]				= dev_recv->user_data.apdu_oad.ucoad[2];
//		stData->Tx_data[13+lenth]				= dev_recv->user_data.apdu_oad.ucoad[1];
//		stData->Tx_data[14+lenth]				= dev_recv->user_data.apdu_oad.ucoad[0];
//		
//		for(i=0;i<user_data_len;i++)
//		{
//			stData->Tx_data[15+i+lenth] 	= dev_recv->user_data.apdu_usrdata[i];
//		}
//		
//		total_lenth = 16+lenth+user_data_len;
//		
//		stData->Tx_data[1] = total_lenth&0xff;
//		stData->Tx_data[2] = (total_lenth>>8)&0xff;
//		
//		tryfcs16(stData->Tx_data,lenth+6);
//		tryfcs16(stData->Tx_data,total_lenth-1);

//		stData->Tx_data[total_lenth+1] = dev_recv->end;
		
//		total_lenth += 2;
		
		return total_lenth;
}


rt_err_t BLE_698_Get_METER_ADDR_Package(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t i,lenth;
	rt_uint16_t total_lenth;
	
	struct _698_BLE_METER_ADDR meter_addr;
	
	
	GetStorageData(Cmd_MeterNumRd,meter_addr.addr,6);
	////////////////////////////////测试用///////////////////////////////
	meter_addr.data = 0x01;
	meter_addr.data_type=0x09;
	meter_addr.addr_len = 0x06;
	meter_addr.addr[0] = 0x00;
	meter_addr.addr[1] = 0x00;
	meter_addr.addr[2] = 0x00;
	meter_addr.addr[3] = 0x00;
	meter_addr.addr[4] = 0x00;
	meter_addr.addr[5] = 0x01;
	meter_addr.optional = 0x00;
	meter_addr.time = 0x00;
	//////////////////////////////////////////////////////////////////////
	

	stData->Tx_data[0] 							= dev_recv->head;
	stData->Tx_data[1] 							= dev_recv->datalenth.uclenth[0];
	stData->Tx_data[2] 							= dev_recv->datalenth.uclenth[1];
	
	stData->Tx_data[3] 							= dev_recv->control.ucControl|0x80;
	stData->Tx_data[4] 							= dev_recv->_698_ADDR.S_ADDR.SA;

	lenth = dev_recv->_698_ADDR.S_ADDR.B.uclenth+1;
	for(i=0;i<lenth;i++)
		stData->Tx_data[5+i] 					= dev_recv->_698_ADDR.addr[i];
	stData->Tx_data[5+lenth]				= dev_recv->_698_ADDR.CA;
	stData->Tx_data[6+lenth]				= dev_recv->HCS.ucHcs[0];
	stData->Tx_data[7+lenth]				= dev_recv->HCS.ucHcs[1];
	
		
	stData->Tx_data[8+lenth]				= dev_recv->apdu.apdu_cmd|0x80;
	
	stData->Tx_data[9+lenth]				= dev_recv->apdu.apdu_data[0];
	stData->Tx_data[10+lenth]				= dev_recv->apdu.apdu_data[1];
	stData->Tx_data[11+lenth]				= dev_recv->apdu.apdu_data[2];
	stData->Tx_data[12+lenth]				= dev_recv->apdu.apdu_data[3];
	stData->Tx_data[13+lenth]				= dev_recv->apdu.apdu_data[4];
	stData->Tx_data[14+lenth]				= dev_recv->apdu.apdu_data[5];
	
	stData->Tx_data[15+lenth]				= meter_addr.data;
	stData->Tx_data[16+lenth]				= meter_addr.data_type;
	stData->Tx_data[17+lenth]				= meter_addr.addr_len;
	stData->Tx_data[18+lenth]				= meter_addr.addr[0];
	stData->Tx_data[19+lenth]				= meter_addr.addr[1];
	stData->Tx_data[20+lenth]				= meter_addr.addr[2];
	stData->Tx_data[21+lenth]				= meter_addr.addr[3];
	stData->Tx_data[22+lenth]				= meter_addr.addr[4];
	stData->Tx_data[23+lenth]				= meter_addr.addr[5];
	stData->Tx_data[24+lenth]				= meter_addr.optional;
	stData->Tx_data[25+lenth]				= meter_addr.time;
		

	total_lenth = 27+lenth;
		
	stData->Tx_data[1] = total_lenth&0xff;
	stData->Tx_data[2] = (total_lenth>>8)&0xff;
		
	tryfcs16(stData->Tx_data,lenth+6);
	tryfcs16(stData->Tx_data,total_lenth-1);

	stData->Tx_data[total_lenth+1] = dev_recv->end;
	
	stData->DataTx_len = total_lenth+2;
	
	return RT_EOK;
}

rt_err_t BLE_698_Data_UnPackage(struct _698_BLE_FRAME *dev_recv,rt_uint8_t* buf)
{
	rt_uint32_t i,addr_lenth,apdu_lenth,total_lenth;
	
	if(buf[0] != 0x68)//数据头不正确  返回解析失败
		return RT_ERROR;
	
	dev_recv->head 												= buf[0];	//帧头
	dev_recv->datalenth.uclenth[0] 				= buf[1];
	dev_recv->datalenth.uclenth[1] 				= buf[2];	//数据长度
	dev_recv->control.ucControl 					= buf[3];	//控制字
	dev_recv->_698_ADDR.S_ADDR.SA 				= buf[4];	//服务器地址
	
	addr_lenth = dev_recv->_698_ADDR.S_ADDR.B.uclenth+1;			//地址域长度
	total_lenth = dev_recv->datalenth.ullenth+2;
	apdu_lenth = total_lenth-addr_lenth-12;

	for(i = 0;i<addr_lenth;i++)
		dev_recv->_698_ADDR.addr[i]					= buf[5+i];
	
	dev_recv->_698_ADDR.CA 								= buf[5+addr_lenth];
	dev_recv->HCS.ucHcs[0] 								= buf[6+addr_lenth];
	dev_recv->HCS.ucHcs[1] 								= buf[7+addr_lenth];
	
	dev_recv->apdu.apdu_cmd								= buf[8+addr_lenth];
	
	for(i=0;i<apdu_lenth;i++)
	{
		dev_recv->apdu.apdu_data[i] 				= buf[9+i+addr_lenth];
	}

	dev_recv->FCS.ucFcs[0] 								= buf[total_lenth-3];
	dev_recv->FCS.ucFcs[1] 								= buf[total_lenth-2];
	dev_recv->end 												= buf[total_lenth-1];
	
	
	my_printf((char*)buf,total_lenth,MY_HEX,1,FUNC_PRINT_RX);
	
	return RT_EOK;//数据解析完毕
}

rt_err_t BLE_698_Get_Request_Normal_Analysis(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint32_t apdu_oad;
	
	apdu_oad = dev_recv->apdu.apdu_data[2];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[3];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[4];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[5];
	
	switch(apdu_oad)
	{
		case 0x40010200:				//获取电表地址
		{
			BLE_698_Get_METER_ADDR_Package(dev_recv,stData);
			break;
		}
	}
	return RT_EOK;		
}



rt_err_t BLE_698_Get_Request_Analysis_and_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t apdu_attitude;
	
	apdu_attitude 	= dev_recv->apdu.apdu_data[0];
	switch(apdu_attitude)
	{
		case GET_REQUEST_NOMAL:
		{
			BLE_698_Get_Request_Normal_Analysis(dev_recv,stData);
		}
	}
	return RT_EOK;	
}


rt_err_t BLE_698_Data_Analysis_and_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	switch(dev_recv->apdu.apdu_cmd)
	{
		case GET_REQUEST:
		{
			BLE_698_Get_Request_Analysis_and_Response(dev_recv,stData);
			break;
		}
		case SECURITY_REQUEST:
		{
			break;
		}
	}
	
	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
	return RT_EOK;
}

/********************************************************************  
*	函 数 名: BLE_ATCmd_Recv
*	功能说明: AT指令配置函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/
void BLE_Trans_Send(rt_device_t dev,rt_uint32_t cmd,rt_uint8_t reason,ScmUart_Comm* stData)
{
	rt_uint8_t size;

	if(stData->DataTx_len)
	{			
		size = rt_device_write(dev, 0, stData->Tx_data, stData->DataTx_len);
						
		my_printf((char*)stData->Tx_data,stData->DataTx_len,MY_HEX,1,FUNC_PRINT_TX);
		
		if(size == stData->DataTx_len)
		{
			rt_kprintf("[bluetooth]:ble_send sucess!!!\n");
		}
	}
}


void BLE_Trans_Recv(rt_device_t dev,BLE_AT_CMD at_cmd,ScmUart_Comm* stData)//AT指令接收处理
{
	if(strstr((char*)(stData->Rx_data),"+BLEDISCONN"))
	{
		BLE_ATCmd = BLE_QUIT_TRANS;
		g_ucProtocol = AT_MODE;
		return;
	}
	if(check_698_data_to_buf(stData,_698_data_buf) == RT_EOK)
	{
		rt_kprintf("[bluetooth]:recv data 698 check ok!\n");
		if(BLE_698_Data_UnPackage(&_698_ble_frame,_698_data_buf) == RT_EOK)
		{		
			BLE_698_Data_Analysis_and_Response(&_698_ble_frame,stData);
		}								
	}
	memset(_698_data_buf,0,sizeof(_698_data_buf));
}



void BLE_SenData_Frame(rt_device_t dev,PROTOCOL_MODE protocol,BLE_AT_CMD at_cmd,ScmUart_Comm* stData)
{
	if(dev == RT_NULL)
		return;
	switch(protocol)
	{
		case AT_MODE:
			BLE_ATCmd_Send(dev,at_cmd);
			break;
		case TRANS_MODE:
//			BLE_Trans_Send(dev,0,0,stData);
			break;
		default:
			break;
	}
	g_ulTx_Count = 0;
}


/********************************************************************  
*	函 数 名: BLE_ATCmd_Recv
*	功能说明: AT指令配置函数
*	形    参: 无
*	返 回 值: 无
********************************************************************/

void BLE_RecvData_Process(rt_device_t dev,PROTOCOL_MODE protocol,BLE_AT_CMD at_cmd,ScmUart_Comm* stData)
{
	if(dev == RT_NULL)
		return;
	switch(protocol)
	{
		case AT_MODE:
			BLE_ATCmd_Recv(dev,at_cmd,stData);
			break;
		case TRANS_MODE:
			BLE_Trans_Recv(dev,at_cmd,stData);
			break;
		default:
			break;
	}
	memset(stData->Rx_data,0,stData->DataRx_len);
	stData->DataRx_len = 0;
}
static rt_err_t check_698_data_to_buf(ScmUart_Comm* stData,rt_uint8_t* buf)
//static rt_err_t check_698_data_to_buf(ScmUart_Comm* stData,struct CharPointDataManage* data_rev)
{
	rt_uint32_t i,lenth;
	
	if(g_ucRecv698_In_AT == 1)
	{
		g_ucRecv698_In_AT = 0;
		return RT_EOK;
	}
	
	for(i=0;i<stData->DataRx_len;i++)
	{
		if(stData->Rx_data[i] == 0x68)
		{
			lenth = stData->Rx_data[i+2];
			lenth = (lenth<<8)|(stData->Rx_data[i+1]);
			if(stData->Rx_data[i+lenth+1] == 0x16)//698一帧数据  接受完整
			{
				if((_698_HCS(stData->Rx_data,i+1,*(stData->Rx_data+i+4)&0x0f+6,0))&&(_698_FCS(stData->Rx_data,i+1,lenth-2,0)))
				{
					memcpy(buf,&stData->Rx_data[i],lenth+2);
					return RT_EOK;
				}
			}
		}
	}
	return RT_ERROR;
}


static rt_err_t bluetooth_rx_ind(rt_device_t dev, rt_size_t size)
{
	g_ulRx_Size = size;
	rt_event_send(&bluetooth_event, UART3_RX_EVENT);
  return RT_EOK;
}

static void bluetooth_thread_entry(void *parabluetooth)
{
	rt_err_t res,result;
	rt_uint32_t e;
	
	bluetooth_serial = rt_device_find(RT_BLUETOOTH_USART);
	
	if(bluetooth_serial != RT_NULL)
	{
		if (rt_device_open(bluetooth_serial, RT_DEVICE_FLAG_DMA_RX) == RT_EOK)
		{
			rt_lprintf("[bluetooth]:Open serial uart3 sucess!\r\n");
		}
	}
	else
	{
		res = RT_ERROR;
		rt_lprintf("[bluetooth]:Open serial uart3 err!\r\n");
		return;
	}
	//初始化事件对象
	rt_event_init(&bluetooth_event, "uart_rx_event", RT_IPC_FLAG_FIFO);

	/* 接收回调函数*/
	rt_device_set_rx_indicate(bluetooth_serial, bluetooth_rx_ind);
	
	rt_pin_mode(BLE_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(BLE_PIN, PIN_LOW);	//模块上电
	
	
	rt_thread_mdelay(3000);
	
	BLE_ATCmd = BLE_ATE;
	BLE_ATCmd_Old = BLE_NULL; 
	g_ucProtocol = AT_MODE;
	
//	g_ulRx_Ptr = 0;

	while (1)
	{
		res = rt_event_recv(&bluetooth_event, UART3_RX_EVENT, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, 10, &e);

		if(res== RT_EOK)
		{	
			result = Uart_Recv_Data(bluetooth_serial,&stBLE_Comm);
			if(result == RT_EOK)
			{
				rt_kprintf("[bluetooth]:ble uart recv event ok!\n");
				BLE_RecvData_Process(bluetooth_serial,g_ucProtocol,BLE_ATCmd,&stBLE_Comm);
			}
		}
		
		BLE_SenData_Frame(bluetooth_serial,g_ucProtocol,BLE_ATCmd,&stBLE_Comm);
		
//		BLE_Commit_TimeOut();
		rt_thread_mdelay(1000);
	}
}



int bluetooth_thread_init(void)
{
	rt_err_t res;
	
	res=rt_thread_init(&bluetooth,
											"bluetooth",
											bluetooth_thread_entry,
											RT_NULL,
											bluetooth_stack,
											THREAD_BLUETOOTH_STACK_SIZE,
											THREAD_BLUETOOTH_PRIORITY,
											THREAD_BLUETOOTH_TIMESLICE);
	if (res == RT_EOK) 
	{
		rt_thread_startup(&bluetooth);
	}
	
	return res;
}


#if defined (RT_BLUETOOTH_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
	INIT_APP_EXPORT(bluetooth_thread_init);
#endif
MSH_CMD_EXPORT(bluetooth_thread_init, bluetooth thread run);

void AT_CMD(int argc, char**argv)
{
	rt_size_t size;
	char* buf;
	
	buf = (char*)rt_malloc(50);
	
	memset(buf,0,50);
	
	strcpy(buf,argv[1]);
	if(strstr(buf,"+++"))
	{}
	else
		strcat(buf+strlen(argv[1]),"\r\n");
	
	
	size = rt_device_write(bluetooth_serial, 0, buf, strlen(buf));
	
	if(size == sizeof(buf))
	{
		rt_lprintf("size = %d,send cmd is %s",size,buf);
	}
	rt_free(buf);
}
MSH_CMD_EXPORT(AT_CMD, Send AT CMD);



