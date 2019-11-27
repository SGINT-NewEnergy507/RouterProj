#include <rtthread.h>
#include <bluetooth.h>
#include <string.h>
#include <stdio.h>
#include <global.h>
#include <board.h>
#include <698.h>
#include <storage.h>
#include <esam.h>
#include "strategy.h"
#include "chargepile.h"


//#define FUNC_PRINT_RX	"[bluetooth]:RX:"
//#define FUNC_PRINT_TX	"[bluetooth]:TX:"

#define TASK_NAME	"[bluetooth]:"

#define KPRINTF_ENABLE 1
#define KPRINTF_DISABLE 0

#define BLE_PIN    GET_PIN(E, 5)

#define BLE_PWR_ON()	{rt_pin_write(BLE_PIN, PIN_LOW);}//ģ���ϵ�
#define BLE_PWR_OFF()	{rt_pin_write(BLE_PIN, PIN_HIGH);}//ģ�����

/* UART3�����¼���־*/
//#define UART3_RX_EVENT (1 << 3)



#define THREAD_BLUETOOTH_PRIORITY     16
#define THREAD_BLUETOOTH_STACK_SIZE   1024*4
#define THREAD_BLUETOOTH_TIMESLICE    5

static struct rt_thread bluetooth;
static rt_device_t bluetooth_serial;
static rt_uint8_t bluetooth_stack[THREAD_BLUETOOTH_STACK_SIZE];//�̶߳�ջ

//static struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* ����Ĭ�����ò���*/

//const char* _698_event_char[]={//698 �¼�����   ��ӡ��־��
//	"null",
//	"Charge_Request",							//�����������
//	"Charge_Request_Ack",						//�����������Ӧ��
//	
//	"Charge_PlanIssue", 					//���ƻ��·�
//	"Charge_PlanIssue_Ack",                 	//���ƻ��·�Ӧ��
//	"Charge_Plan_Offer", 						//���ƻ��¼��ϱ�
//	"Charge_Plan_Offer_Ack",                 	//���ƻ��ϱ��¼�Ӧ��
//	
//	"Charge_Plan_Adjust",                 		//���ƻ�����
//	"Charge_Plan_Adjust_Ack",                 	//���ƻ�����Ӧ��


//	"Charge_Request_Report",					//��������¼�����
//	"Charge_Request_Report_Ack",				//��������¼�����Ӧ��
////	"Charge_Request_Report_APP",				//��������¼���֪APP
////	"Charge_Request_Confirm",					//�������ȷ�ϣ�֪ͨ������
//	
//	"Charge_Plan_Exe_State",                    //���ƻ�ִ��״̬�¼��ϱ�
//	"Charge_Plan_Exe_State_Ack",                 //���ƻ�ִ��״̬�¼��ϱ�Ӧ��
//	
//	"Start_Charge",							//�����������·�
//	"Start_Charge_Ack",						//�������Ӧ��
//	"Stop_Charge",							//ֹͣ�������·�
//	"Stop_Charge_Ack",							//ֹͣ���Ӧ��
//	"Power_Adj",							//���ʵ��ڲ����·�
//	"Power_Adj_Ack",						//���ʵ���Ӧ��

//	"Charge_Record",							//���ͳ�綩��
//	"Charge_RecordAck",						//���ͳ�綩���¼�ȷ��
//	"Device_Fault",                      	//����·�����쳣״̬
//	"Device_FaultAck",                      	//����·�����쳣״̬
//	"Pile_Fault",                 			//���ͳ��׮�쳣״̬
//	"Pile_FaultAck",                 			//���ͳ��׮�쳣״̬
//	"Charge_Plan_Issue_Get_Ack",
//	
//	"Read_Router_State",                    	//·����ִ��״̬��ѯ
//	"Read_Router_State_Ack",                 	//·����ִ��״̬Ӧ��
//	
//	"STAOnlineState",						//STA�������·��������״̬��
//	"STAOnlineStateAck",					//STA�������·��������״̬ȷ�ϡ�
//};//ҵ�������������

char* AT_CmdDef[]={

	"ATE0\r\n",		//�رջ��Թ���
	"AT+BLEINIT=2\r\n",			//BLE ��ʼ��������ΪServerģʽ
	"AT+BLENAME=",	//���� BLE �豸����
//	"AT+BLENAME=\"LN000000000001\"\r\n",	//���� BLE �豸����
//	"AT+BLENAME=\"[NR000000000001]\"\r\n",	//���� BLE �豸����
//	"AT+BLEADDR=1,\"f1:f2:f3:f4:f5:f6\"\r\n",
	"AT+BLEADDR=1,\"",
	
	"AT+BLEGATTSSRVCRE\r\n",	//����GATTS ����
	"AT+BLEGATTSSRVSTART\r\n",	//����GATTS ����
	
	"AT+BLEADVPARAM=32,64,0,1,7\r\n",							//���ù㲥����
	"AT+BLEADVDATA=\"0201060D09",//����ɨ����Ӧ����
//	"AT+BLEADVDATA=\"0201060F094C4E3030303030303030303030310303E0FF\"\r\n",//����ɨ����Ӧ����
//	"AT+BLEADVDATA=\"02010611095B4E523030303030303030303030315D0303E0FF\"\r\n",//����ɨ����Ӧ����		
	"AT+BLEADVSTART\r\n",		//��ʼ�㲥
	
	"AT+BLESPPCFG=1,1,1,1,1\r\n",	//����BLE͸��ģʽ
	"AT+BLESPP\r\n",				//����͸��ģʽ
	
	"+++",						//�˳�͸��ģʽ
	"AT+BLEDISCONN\r\n",	//�Ͽ�����
	
	"AT+RST\r\n",	//����ģ��

};

typedef enum 		 //ATָ��
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

typedef enum 		 //��̨������������
{
	AT_MODE = 1,
	TRANS_MODE,
}PROTOCOL_MODE;

static BLE_AT_CMD BLE_ATCmd;
static BLE_AT_CMD BLE_ATCmd_Old;
static rt_uint8_t BLE_ATCmd_Count;
static PROTOCOL_MODE g_ucProtocol;

static struct rt_event bluetooth_event;//���ڽ������ݵ��ź���

CCMRAM static ScmUart_Comm stBLE_Comm;
CCMRAM static ScmEsam_Comm stBLE_Esam_Comm;
//CCMRAM static rt_uint8_t BLE_698_data_buf[4][255];
//CCMRAM static rt_uint8_t BLE_698_data_buf[1024];


CCMRAM CHARGE_APPLY 				stBLE_Charge_Apply;//�������
CCMRAM CHARGE_APPLY_RSP			stBLE_Charge_Apply_RSP;//���������Ӧ
CCMRAM CHARGE_APPLY_EVENT 	stBLE_Charge_Apply_Event;//��������¼����ϱ�
CCMRAM PLAN_OFFER_EVENT			stBLE_Charge_Plan_Event;//���ƻ��ϱ��¼�
CCMRAM CHARGE_EXE_EVENT 		stBLE_Charge_Exe_Event;//���ִ���¼��ϱ�
CCMRAM ROUTER_FAULT_EVENT		stBLE_Router_Fault_Event;//·���������¼���¼


CCMRAM CHARGE_EXE_STATE			stBLE_Charge_State;//���״̬����
CCMRAM CHARGE_STRATEGY			stBLE_Charge_Plan;//���ƻ���
CCMRAM CHARGE_STRATEGY_RSP	stBLE_Charge_Plan_RSP;//���ƻ�����Ӧ
CCMRAM CTL_CHARGE						stBLE_Charge_Start;//�������
CCMRAM CTL_CHARGE						stBLE_Charge_Stop;//ֹͣ���
CCMRAM CHG_ORDER_EVENT      stBLE_Charge_Record;//��綩��
CCMRAM CHARGE_EXE_STATE_ASK stBLE_Charge_State_Ask;//·����״̬

CCMRAM CHARGE_STRATEGY stBLE_Charge_Plan_Adj;
CCMRAM CHARGE_STRATEGY_RSP stBLE_Charge_Plan_Adj_Rsp;


//CCMRAM static char printfbuf[100];

//CCMRAM ROUTER_IFO_UNIT stBLE_Router_Info;

static rt_uint8_t	Esam_KEY_R1[16];//R1����
static rt_uint8_t	Esam_KEY_R2[16];//R2����
static rt_uint8_t	Esam_KEY_R3[16];//R3����
static rt_uint8_t	Esam_KEY_DATA[32];//DATA2����

static rt_uint8_t Esam_KEY_Sess_State;//�ỰЭ����ɱ�־ =2 ����ỰЭ����� ���Խ��мӽ��ܲ���


rt_uint8_t BLE_698_Get_Addr[]={0x68,0x17,0x00,0x43,0x45,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x10,0xDA,0x5F,0x05,0x01,0x00,0x40,0x01,0x02,0x00,0x00,0xED,0x03,0x16};//��������ַ


static rt_uint32_t g_ulBLE_Rx_Count;//�շ���ʱ�жϼ���
static rt_uint32_t g_ulBLE_Tx_Count;
static rt_uint32_t g_ulBLE_Rx_Beg;
static rt_uint32_t g_ulBLE_Rx_Ptr;
static rt_uint32_t g_ulBLE_Rx_Pre;

//static rt_uint32_t g_ulBLE_RX_Write;
//static rt_uint32_t g_ulBLE_RX_Read;


//static rt_uint32_t g_ulRx_Size;
static rt_uint8_t g_ucRecv698_In_AT;//��ATָ��ģʽ��  �յ���698Э������
	
static rt_uint32_t g_BLE_Send_to_Strategy_event;
static rt_uint32_t g_BLE_Get_Strategy_event;
	
struct _698_BLE_FRAME _698_ble_frame;
struct _698_BLE_ADDR _698_ble_addr;
static rt_uint8_t _698_ble_control;
struct _698_BLE_METER_ADDR stBLE_meter_addr;

rt_uint8_t ble_meter_addr_buf[24] = {0x0C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x31};

static rt_err_t BLE_Check_Data_to_Buf(ScmUart_Comm* stData);

extern int _698_HCS(unsigned char *data, int start_size,int size,unsigned short HCS);
extern int _698_FCS(unsigned char *data, int start_size,int size,unsigned short FCS);
extern int tryfcs16(unsigned char *cp, int len);

static void BLE_Trans_Send(rt_device_t dev,rt_uint32_t cmd,rt_uint8_t reason,ScmUart_Comm* stData);

rt_err_t BLE_698_Data_Analysis_and_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData);
rt_err_t BLE_698_Data_UnPackage(struct _698_BLE_FRAME *dev_recv,rt_uint8_t* buf);


rt_err_t BLE_698_Charge_State_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData);

rt_uint8_t BLE_Get_Strategy_Event(void);//��ȡ�� ���Դ��ݹ������¼� ����Ӧ����
rt_uint8_t BLE_Send_Event_To_Strategy(COMM_CMD_C cmd);//�����¼�������
rt_uint32_t Strategy_get_BLE_event(void);
rt_uint8_t BLE_CtrlUnit_RecResp(COMM_CMD_C cmd,void *STR_SetPara,int count);
rt_uint8_t BLE_Send_To_Strategy_Event_Ack(void);

/********************************************************************  
*	�� �� ��: BLE_Commit_TimeOut
*	����˵��: �����շ����ݳ�ʱ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
void BLE_Commit_TimeOut(void)
{
	if(g_ucProtocol == TRANS_MODE)
	{
		g_ulBLE_Rx_Count++;
		g_ulBLE_Tx_Count++;
		
		if((g_ulBLE_Rx_Count>10)||(g_ulBLE_Tx_Count>10))
		{
			g_ulBLE_Rx_Count = 0;
			g_ulBLE_Tx_Count = 0;
			BLE_ATCmd = BLE_QUIT_TRANS;
			g_ucProtocol = AT_MODE;
		}
	}
}
/********************************************************************  
*	�� �� ��: BLE_Uart_Data_Recv_and_Process 
*	����˵��: �������ݽ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
static rt_err_t BLE_Uart_Data_Recv_and_Process(rt_device_t dev,ScmUart_Comm* stData)
{
	rt_uint8_t tmpsize,Res,hcs_size;
	rt_uint16_t ptr,lenth;
	static rt_uint8_t timeout;
	
	tmpsize = 0;
	ptr = 0;
	g_ulBLE_Rx_Pre = 0;
	g_ulBLE_Rx_Ptr = 0;
	
	
	while(1)
	{
		tmpsize = rt_device_read(dev, 0, &Res,1);
		
		switch(g_ucProtocol)
		{
			case AT_MODE:
			{
				if(tmpsize)
				{
					g_ulBLE_Rx_Ptr = 1;//�������յ�
					stData->Rx_data[ptr++] = Res;//�����ݴ��뻺��
					if(ptr == 1024)//���ݳ��ȳ�  ���ش���
						return RT_ERROR;
				}
				else if(g_ulBLE_Rx_Ptr)//�������յ�  atģʽ��ʱ�����մ���
				{
					rt_thread_mdelay(100);
										
					timeout++;
					if(timeout>5)
					{
						stBLE_Comm.DataRx_len = ptr;
						my_printf((char*)stBLE_Comm.Rx_data,stBLE_Comm.DataRx_len,MY_CHAR,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RX:");
						g_ulBLE_Rx_Beg = 1;
						g_ulBLE_Rx_Ptr = 0;
						timeout = 0;
						return RT_EOK; 
					}
				}
				else
				{
					rt_thread_mdelay(500);//��ʱ������  �˳�����
					timeout++;
					if(timeout>4)
					{
						timeout = 0;
						return RT_ERROR; 
					}
				}
			}	
			break;
			
			case TRANS_MODE:
			{
				if(g_ucRecv698_In_AT)
				{
					g_ucRecv698_In_AT = 0;
					g_ulBLE_Rx_Beg = 1;
					stData->DataRx_len = sizeof(BLE_698_Get_Addr);
					memcpy(&stData->Rx_data,BLE_698_Get_Addr,sizeof(BLE_698_Get_Addr));
					
					return RT_EOK;
				}
				
				if(tmpsize)
				{
					if(g_ulBLE_Rx_Pre == 1)
					{
						stData->Rx_data[ptr++] = Res;//�����ݴ��뻺��
						
						if(ptr == 3)
						{
							lenth = stData->Rx_data[2];
							lenth = (lenth<<8)|(stData->Rx_data[1]);
							
							if(lenth > 1024)
							{
								rt_kprintf("[bluetooth]: (%s)  recv data lenth error....\n",__func__);
								return RT_ERROR;
							}								
						}
						else if(ptr == 14)
						{
							hcs_size = (rt_uint32_t)((stData->Rx_data[4]&0x0f)+8);
							if(_698_HCS(stData->Rx_data,1,hcs_size,0) < 0)
							{
								rt_kprintf("[bluetooth]: (%s)  recv data hcs error....\n",__func__);
								return RT_ERROR;
							}
						}
						else if((lenth >0)&&(ptr >= (lenth+2)))
						{
							if(_698_FCS(stData->Rx_data,1,lenth,0) == 0)
							{
								stBLE_Comm.DataRx_len = ptr;
								my_printf((char*)stBLE_Comm.Rx_data,stBLE_Comm.DataRx_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RX:");
								g_ulBLE_Rx_Beg = 1;
								g_ulBLE_Rx_Pre = 0;
								timeout = 0;
								return RT_EOK;
							}
							else
							{
								rt_kprintf("[bluetooth]: (%s)  recv data fcs error....\n",__func__);
								return RT_ERROR;
							}							
						}	
					}
					else if(Res == 0x68)
					{
						ptr = 0;
						lenth = 0;
						stData->Rx_data[ptr++] = Res;//�����ݴ��뻺��
						g_ulBLE_Rx_Pre = 1;
					}	
				}
				else
				{
					rt_thread_mdelay(100);//��ʱ������  �˳�����
					timeout++;
					if(timeout>20)
					{
						timeout = 0;
						rt_kprintf("[bluetooth]: (%s)  no data for long time...\n",__func__);
						return RT_ERROR; 
					}
				}
			}		
			break;
			
			default:
				break;
		}
	}
}

/********************************************************************  
*	�� �� ��: BLE_Send_AT_TimeOut
*	����˵��: ATָ�����ó�ʱ������
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

void BLE_ATCmd_TimeOut(BLE_AT_CMD at_cmd)
{
	if(BLE_ATCmd_Old == at_cmd)
	{
		BLE_ATCmd_Count++;
		if(BLE_ATCmd_Count>5)
		{
			BLE_PWR_ON();
			BLE_ATCmd = BLE_ATE;
			BLE_ATCmd_Old = BLE_NULL;
			BLE_ATCmd_Count = 0;
		}
		else if(BLE_ATCmd_Count>3)//����ģ����Ӧ��ʱ ��������
		{
			BLE_PWR_OFF();	
		}
		rt_kprintf("[bluetooth]: (%s) ble_send repeate times is %d\n",__func__,BLE_ATCmd_Count);
	}
	else
	{
		BLE_ATCmd_Old = at_cmd;
		BLE_ATCmd_Count = 0;
	}
}

/********************************************************************  
*	�� �� ��: BLE_ATCmd_Send
*	����˵��: ATָ�����ú���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

void BLE_ATCmd_Send(rt_device_t dev,BLE_AT_CMD at_cmd)
{
	rt_size_t size;
	char at_buf[100];
	char i,buf[100];
	
	if((dev == RT_NULL)||(at_cmd == BLE_NULL))
	return;
	
	memset(at_buf,0,sizeof(at_buf));
	memset(buf,0,sizeof(buf));
	sprintf((char*)at_buf,"");
	strcpy(at_buf,AT_CmdDef[at_cmd]);
	
	if(at_cmd == BLE_NAME)
	{
		strcat(at_buf,"\"");
		strcat((char*)RouterInfo.Addr,"\0");
		strcat(at_buf,(char*)(&RouterInfo.Addr[1]));
		strcat(at_buf,"\"\r\n");
	}
	else if(at_cmd == BLE_ADDR_SET)
	{
		for(i = 0; i < 6;i++)
		{
			if(i<3)
				strncpy(buf,"f",1);
			else
				strncpy(buf,"a",1);
			
			strcat(at_buf,buf);
			strncpy(buf,(char*)(&RouterInfo.Addr[i+7]),1);
			strcat(at_buf,buf);
			if(i != 5)
				strcat(at_buf,":");
		}
		strcat(at_buf,"\"\r\n");		
	}
	else if(at_cmd == BLE_ADV_DATA)
	{
		sprintf((char*)buf,"");
		for(i = 0 ; i<RouterInfo.Addr[0];i++)
		{
			buf[i*2] = (char)(((RouterInfo.Addr[1+i]>>4)&0x0f)+0x30);
			buf[i*2+1] = (char)((RouterInfo.Addr[1+i]&0x0f)+0x30);
		}
		strcat((char*)buf,"\0");
		strcat(at_buf,(char*)(buf));
		strcat(at_buf,"0303E0FF\"\r\n");
	}
	strcat((char*)at_buf,"\0");
	
	size = rt_device_write(dev, 0, at_buf, strlen(at_buf));
	
	if(size == strlen(at_buf))
	{
		rt_kprintf("[bluetooth]: (%s) %s\n",__func__,at_buf);
	}
	
	BLE_ATCmd_TimeOut(at_cmd);//ATָ��ͳ�ʱ����
}
/********************************************************************  
*	�� �� ��: BLE_ATCmd_Recv
*	����˵��: ATָ�����ú���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

void BLE_ATCmd_Recv(rt_device_t dev,BLE_AT_CMD at_cmd,ScmUart_Comm* stData)//ATָ����մ���
{
	rt_uint32_t i;
	
	if(dev == RT_NULL)
		return;

	if(at_cmd == BLE_NULL)//������ cfg ����null �ȴ�APP����
	{
		if((strstr((char*)(stData->Rx_data),"+BLECONN"))||(strstr((char*)(stData->Rx_data),"+WRITE")))   //app�����Ӵ�͸��
		{				
			BLE_ATCmd = BLE_SPP;
			rt_thread_mdelay(1000);//����  spp֮ǰ �ȴ�2s
		}
		if(strstr((char*)(stData->Rx_data),"+BLEDISCONN"))//app�Ͽ�����
		{
			BLE_ATCmd = BLE_QUIT_TRANS;
			g_ucProtocol = AT_MODE;
		}
	}
	else
	{
		for(i = 0; i < stData->DataRx_len; i++)
		{
			if((stData->Rx_data[i] == 'O')&&(stData->Rx_data[i+1] == 'K'))
			{
				if(BLE_SPP_CFG == BLE_ATCmd)   //����cfg�ɹ� ����ȴ�
				{
					BLE_ATCmd =BLE_NULL;
					CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT = RT_FALSE;
				}
				else if(BLE_ATCmd == BLE_SPP)//��͸�� �ɹ�  ����͸��ģʽ
				{
					BLE_ATCmd = BLE_NULL;
					g_ucProtocol = TRANS_MODE;
					g_ucRecv698_In_AT = 1;//�յ�698Э������
				}
				else
					BLE_ATCmd++;
			}
		}
	}
	
	if(strstr((char*)(stData->Rx_data),"WRITE"))//����͸��ģʽ֮ǰ �յ�698 Э������
	{
		for(i = 0; i < stData->DataRx_len; i++)
		{
			if((stData->Rx_data[i] == 0x68)&&(stData->Rx_data[i+1] == 0x17))
			{
				g_ucRecv698_In_AT = 1;//�յ�698Э������
			}
		}
	}
	memset(stData->Rx_data,0,stData->DataRx_len);
	stData->DataRx_len = 0;
}

/********************************************************************  
*	�� �� ��: BLE_Trans_Send
*	����˵��: ���ڷ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
void BLE_Trans_Send(rt_device_t dev,rt_uint32_t cmd,rt_uint8_t reason,ScmUart_Comm* stData)
{
	rt_uint16_t size;

	if(stData->DataTx_len)
	{
//		strcpy(printfbuf,"");
//		sprintf(printfbuf,"[bluetooth]: (%s) TX:",__func__);
		my_printf((char*)stData->Tx_data,stData->DataTx_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"TX:");
		
		size = rt_device_write(dev, 0, stData->Tx_data, stData->DataTx_len);
		
		g_ulBLE_Tx_Count = 0;
	}
}
/********************************************************************  
*	�� �� ��: BLE_Trans_Recv
*	����˵��: �������ݽ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

void BLE_Trans_Recv(rt_device_t dev,BLE_AT_CMD at_cmd,ScmUart_Comm* stData)//ATָ����մ���
{
	if(BLE_698_Data_UnPackage(&_698_ble_frame,stData->Rx_data) == RT_EOK)
	{		
		BLE_698_Data_Analysis_and_Response(&_698_ble_frame,stData);
	}

//	if(strstr((char*)(stData->Rx_data),"+BLEDISCONN"))
//	{
//		BLE_ATCmd = BLE_QUIT_TRANS;
//		g_ucProtocol = AT_MODE;
//	}
	g_ulBLE_Rx_Count = 0;

}

/********************************************************************  
*	�� �� ��: BLE_ATCmd_Recv
*	����˵��: ATָ�����ú���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

void BLE_RecvData_Process(rt_device_t dev,PROTOCOL_MODE protocol,BLE_AT_CMD at_cmd,ScmUart_Comm* stData)
{
	if(dev == RT_NULL)
		return;
	
	if(strstr((char*)(stData->Rx_data),"start"))
	{
		ChargepileDataGetSet(Cmd_ChargeStart,NULL);
		memset(stData->Rx_data,0,1024);
		rt_kprintf("[bluetooth]:Send Start Charge cmd to ChargePile,start start start!\n");
	}
	else if(strstr((char*)(stData->Rx_data),"stop"))
	{
		ChargepileDataGetSet(Cmd_ChargeStop,NULL);
		memset(stData->Rx_data,0,1024);
		rt_kprintf("[bluetooth]:Send Stop Charge cmd to ChargePile,stop stop stop!\n");
	}
	
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
}

rt_err_t date_time_s_to_sys_time(struct _698_DATE_S* date_time_s,STR_SYSTEM_TIME* sys_time)
{
	rt_uint16_t _698_year;
		
	_698_year = date_time_s->year[0];
	_698_year = ((_698_year<<8) | date_time_s->year[1]);
	_698_year = _698_year%100;
	
	Int_toBCD(&sys_time->Year,(rt_uint8_t*)&_698_year,1);
	Int_toBCD(&sys_time->Month,&date_time_s->month,1);
	Int_toBCD(&sys_time->Day,&date_time_s->day,1);
	Int_toBCD(&sys_time->Hour,&date_time_s->hour,1);
	Int_toBCD(&sys_time->Minute,&date_time_s->minute,1);
	Int_toBCD(&sys_time->Second,&date_time_s->second,1);
	
	return RT_EOK;
}

rt_err_t Sys_time_to_date_time_s(STR_SYSTEM_TIME * sys_time,struct _698_DATE_S *date_time_s)
{
	rt_uint16_t _698_year;
	rt_uint8_t year;
	
	BCD_toInt(&year,&sys_time->Year,1);
	if(year == 0)
	{
		date_time_s->year[0] = 0;
		date_time_s->year[1] = 0;
	}
	else
	{		
		date_time_s->year[0] = (rt_uint8_t)((2000+year)>>8)&0xff;
		date_time_s->year[1] = (rt_uint8_t)(2000+year)&0xff;
	}
	BCD_toInt(&date_time_s->month,&sys_time->Month,1);
	BCD_toInt(&date_time_s->day,&sys_time->Day,1);
	BCD_toInt(&date_time_s->hour,&sys_time->Hour,1);
	BCD_toInt(&date_time_s->minute,&sys_time->Minute,1);
	BCD_toInt(&date_time_s->second,&sys_time->Second,1);
	
	return RT_EOK;
}


rt_uint32_t BLE_698_Data_Package(struct _698_BLE_FRAME *dev_recv,rt_uint16_t user_data_len,ScmUart_Comm* stData)
{
	rt_uint16_t i,total_lenth,lenth,ptr;
	
	ptr = 0;
	stData->Tx_data[ptr++] 							= _698_head;
	stData->Tx_data[ptr++]							= 0;//�ܳ���
	stData->Tx_data[ptr++] 							= 0;
	
	stData->Tx_data[ptr++] 							= _698_ble_control|0x80;
	
	_698_ble_addr.S_ADDR.SA							= 0x05;
	stData->Tx_data[ptr++] 							= _698_ble_addr.S_ADDR.SA;

	lenth = _698_ble_addr.S_ADDR.B.uclenth+1;
	
	for(i = 0; i< lenth; i++)
	_698_ble_addr.addr[lenth-1-i]					= stBLE_meter_addr.addr[i];								
	
	for(i=0;i<lenth;i++)
		stData->Tx_data[ptr++] 						= _698_ble_addr.addr[i];//������ ��ַ
	stData->Tx_data[ptr++]							= _698_ble_addr.CA;//�ͻ��˵�ַ
	stData->Tx_data[ptr++]							= 0;//HCSУ��
	stData->Tx_data[ptr++]							= 0;
	
	////////////////////////APDU data////////////////////////////////////////////////
	stData->Tx_data[ptr++]							= dev_recv->apdu.apdu_cmd | 0x80;
	for(i = 0;i < (user_data_len-1);i++)
	{
		stData->Tx_data[ptr++]						= dev_recv->apdu.apdu_data[i];
	}
	/////////////////////////////////////////////////////////////////////////////////
	stData->Tx_data[ptr++]							= 0;//FCSУ��
	stData->Tx_data[ptr++]							= 0;
	stData->Tx_data[ptr++]							= 0x16;//��β

	total_lenth = ptr;
		
	stData->Tx_data[1] = (total_lenth-2)&0xff;
	stData->Tx_data[2] = ((total_lenth-2)>>8)&0xff;
		
	tryfcs16(stData->Tx_data,lenth+6);
	tryfcs16(stData->Tx_data,total_lenth-3);

	stData->DataTx_len = total_lenth;
	return total_lenth;
}

/********************************************************************  
*	�� �� ��: BLE_698_Get_METER_ADDR_Package
*	����˵��: 698�������ַ��֡
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

rt_err_t BLE_698_Get_METER_ADDR_Package(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t i,lenth,ptr;
	rt_uint16_t total_lenth;
	rt_err_t result;
	
	ptr = 6;
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.data;
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.data_type;
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.addr_len;
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.addr[0];
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.addr[1];
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.addr[2];
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.addr[3];
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.addr[4];
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.addr[5];
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.optional;
	dev_recv->apdu.apdu_data[ptr++]	= stBLE_meter_addr.time;
	ptr++;//apdu_cmd
	
	result = BLE_698_Data_Package(dev_recv,ptr,stData);
	
	return RT_EOK;
}

/********************************************************************  
*	�� �� ��: BLE_698_Data_UnPackage
*	����˵��: 698���� ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

rt_err_t BLE_698_Data_UnPackage(struct _698_BLE_FRAME *dev_recv,rt_uint8_t* buf)
{
	rt_uint32_t i,addr_lenth,apdu_lenth,total_lenth;
	
	rt_kprintf("[bluetooth]: (%s) \n",__func__);
	
	if(buf[0] != 0x68)//����ͷ����ȷ  ���ؽ���ʧ��
		return RT_ERROR;
	
	dev_recv->head 												= buf[0];	//֡ͷ
	dev_recv->datalenth.uclenth[0] 				= buf[1];
	dev_recv->datalenth.uclenth[1] 				= buf[2];	//���ݳ���
	dev_recv->control.ucControl 					= buf[3];	//������
	dev_recv->_698_ADDR.S_ADDR.SA 				= buf[4];	//��������ַ
	
	addr_lenth = dev_recv->_698_ADDR.S_ADDR.B.uclenth+1;			//��ַ�򳤶�
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
	
	////////////////////////������յ��ĵ�ַ �������/////////////////////////
	_698_ble_addr.S_ADDR.SA = dev_recv->_698_ADDR.S_ADDR.SA;
	for(i = 0;i<addr_lenth;i++)
		_698_ble_addr.addr[i] = dev_recv->_698_ADDR.addr[i];
	_698_ble_addr.addr[i] = dev_recv->_698_ADDR.CA;
	
	_698_ble_control = dev_recv->control.ucControl;
	/////////////////////////////////////////////////////////////////////
	
	
	return RT_EOK;//���ݽ������
}
/********************************************************************  
*	�� �� ��: BLE_698_Get_Request_Normal_Analysis
*	����˵��: 698 get request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Get_Request_Normal_Analysis(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint32_t apdu_oad;
	
	apdu_oad = dev_recv->apdu.apdu_data[2];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[3];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[4];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[5];
	
	switch(apdu_oad)
	{
		case 0x40010200:				//��ȡ����ַ
		{
			BLE_698_Get_METER_ADDR_Package(dev_recv,stData);
			break;
		}
		case 0x60120300:
			BLE_Send_To_Strategy_Event_Ack();
			break;
		case 0x90030200:   //90030200 ��ǹ   90030201 ǹ1״̬  90030202 ǹ2״̬
		case 0x90030201:
		case 0x90030202:
		{			
			if(Esam_KEY_Sess_State == 2)
			{
				stBLE_Charge_State_Ask.GunNum = dev_recv->apdu.apdu_data[5];
				BLE_Send_Event_To_Strategy(Cmd_RouterExeState);
			}
			break;
		}
	}
	return RT_EOK;		
}

/********************************************************************  
*	�� �� ��: BLE_698_Get_Request_Analysis_and_Response
*	����˵��: 698 get request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

rt_err_t BLE_698_Get_Request_Analysis_and_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t apdu_attitude;
	
	apdu_attitude 	= dev_recv->apdu.apdu_data[0];
	switch(apdu_attitude)
	{
		case GET_REQUEST_NOMAL:
		{
			BLE_698_Get_Request_Normal_Analysis(dev_recv,stData);
			break;
		}
	}
	return RT_EOK;	
}
/********************************************************************  
*	�� �� ��: BLE_698_Security_Request_PlainText_Analysis
*	����˵��: 698 Security Request ���Ľ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Security_Request_PlainText_Analysis(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	return RT_EOK;
}

/********************************************************************  
*	�� �� ��: BLE_698_Security_Request_PlainText_Analysis
*	����˵��: 698 Security Request ���Ľ���
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Security_Request_CipherText_Analysis(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint32_t cipherdata_lenth,appenddata_lenth,lenth,temp;
	rt_uint8_t i,ptr;
//	rt_uint32_t apdu_oad;
	
	rt_kprintf("[bluetooth]: (%s) \n",__func__);
	
	ptr = 0;
//	cipherdata_lenth = dev_recv->apdu.apdu_data[1];

	memcpy(&stBLE_Esam_Comm.Tx_data[ptr],Esam_KEY_DATA,32);
	
	ptr+=32;
	
	if(dev_recv->apdu.apdu_data[1] == 0x81)
	{
		cipherdata_lenth = dev_recv->apdu.apdu_data[2];
		for(i=0;i<cipherdata_lenth;i++)
		{
			stBLE_Esam_Comm.Tx_data[ptr++] = dev_recv->apdu.apdu_data[3+i];
		}
	}
	else if(dev_recv->apdu.apdu_data[1] == 0x82)
	{
		cipherdata_lenth = dev_recv->apdu.apdu_data[2];
		cipherdata_lenth = ((cipherdata_lenth<<8)&0xff00)|(dev_recv->apdu.apdu_data[3]);
		for(i=0;i<cipherdata_lenth;i++)
		{
			stBLE_Esam_Comm.Tx_data[ptr++] = dev_recv->apdu.apdu_data[4+i];
		}
	}
	else
	{
		cipherdata_lenth = dev_recv->apdu.apdu_data[1];
		for(i=0;i<cipherdata_lenth;i++)
		{
			stBLE_Esam_Comm.Tx_data[ptr++] = dev_recv->apdu.apdu_data[2+i];
		}
	}

	stBLE_Esam_Comm.DataTx_len = ptr;
	

	if(ESAM_Communicattion(APP_SESS_VERI_MAC,&stBLE_Esam_Comm) == RT_EOK)//������֤mac
	{
		if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
		{			
				memcpy(&dev_recv->apdu.apdu_cmd,&stBLE_Esam_Comm.Rx_data[4],stBLE_Esam_Comm.DataRx_len-5);
				BLE_698_Data_Analysis_and_Response(dev_recv,stData);
		}
		else
		{
			rt_kprintf("[bluetooth]:esam return err,errcode is 0x%X%X!\n",stBLE_Esam_Comm.Rx_data[0],stBLE_Esam_Comm.Rx_data[1]);
		}
	}

	return RT_EOK;		
}
/********************************************************************  
*	�� �� ��: BLE_698_Security_Request_Analysis_and_Response
*	����˵��: 698 Security Request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Security_Request_Analysis_and_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t apdu_attitude;
	
	apdu_attitude 	= dev_recv->apdu.apdu_data[0];
	switch(apdu_attitude)
	{
		case PLAINTEXT://����
		{
//			BLE_698_Get_Request_Normal_Analysis(dev_recv,stData);
			break;
		}
		case CIPHERTEXT://����
		{
			BLE_698_Security_Request_CipherText_Analysis(dev_recv,stData);
			break;
		}
	}
	return RT_EOK;
}







/********************************************************************  
*	�� �� ��: BLE_698_ESAM_SESS_INTI_Package
*	����˵��: 698�������ַ��֡
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

rt_err_t BLE_698_ESAM_SESS_INTI_Package(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t	Esam_Version[5];//ESAM �汾��
//	rt_uint8_t	Esam_KEY_R2[16];//R2����
	rt_uint8_t	Esam_KEY_DATA2[16];//DATA2����
	rt_uint8_t 	i,ptr,lenth,total_lenth;
	rt_uint16_t Esam_endata_len;
	rt_uint8_t Version[2];
	rt_err_t result;
	
	rt_kprintf("[bluetooth]: (%s) \n",__func__);
	
	
	stBLE_Esam_Comm.DataTx_len = 0;
	ESAM_Communicattion(RD_INFO_01,&stBLE_Esam_Comm);
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		memcpy(Esam_Version,&stBLE_Esam_Comm.Rx_data[4],5);   //DATA1
	}
	else
	{
		rt_kprintf("[bluetooth]:(%s) read esam info fail,errcode =0x%X%X!\n",__func__,stBLE_Esam_Comm.Rx_data[0],stBLE_Esam_Comm.Rx_data[1]);
		return RT_ERROR;
	}
	
	ESAM_Communicattion(APP_KEY_AGREE_ONE,&stBLE_Esam_Comm);
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		memcpy(Esam_KEY_R2,&stBLE_Esam_Comm.Rx_data[4],16);
	}
	else
	{
		rt_kprintf("[bluetooth]:(%s) read esam r2 fail,errcode =0x%X%X!",__func__,stBLE_Esam_Comm.Rx_data[0],stBLE_Esam_Comm.Rx_data[1]);
		return RT_ERROR;
	}
	
	
	memcpy(stBLE_Esam_Comm.Tx_data,&dev_recv->apdu.apdu_data[11],16);//R1
	memcpy(Esam_KEY_R1,&dev_recv->apdu.apdu_data[11],16);
	memcpy(stBLE_Esam_Comm.Tx_data+16,Esam_Version,5);
	stBLE_Esam_Comm.DataTx_len = 21;
	
	ESAM_Communicattion(APP_KEY_AGREE_TWO,&stBLE_Esam_Comm);//R1+DATA1 
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		memcpy(Esam_KEY_DATA2,&stBLE_Esam_Comm.Rx_data[4],16);
	}
	else
	{
		rt_kprintf("[bluetooth]:(%s) read esam data2 fail,errcode =0x%X%X!\n",__func__,stBLE_Esam_Comm.Rx_data[0],stBLE_Esam_Comm.Rx_data[1]);
		return RT_ERROR;
	}
	
	memcpy(stBLE_Esam_Comm.Tx_data,Esam_KEY_DATA2,16);
	memcpy(stBLE_Esam_Comm.Tx_data+16,&dev_recv->apdu.apdu_data[11],16);//R1
	memcpy(stBLE_Esam_Comm.Tx_data+32,Esam_KEY_R2,16);
	stBLE_Esam_Comm.DataTx_len = 48;
	
	ESAM_Communicattion(APP_KEY_AGREE_THREE,&stBLE_Esam_Comm); //DATA2+R1+R2
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
			Esam_KEY_Sess_State = 1;
		
			Esam_endata_len = stBLE_Esam_Comm.Rx_data[2];
			Esam_endata_len = (rt_uint16_t)(((Esam_endata_len<<8)&0xff00)|(stBLE_Esam_Comm.Rx_data[3]));

		
			Version[0] = dev_recv->apdu.apdu_data[8];
			Version[1] = dev_recv->apdu.apdu_data[9];
		
			ptr = 6;
			dev_recv->apdu.apdu_data[ptr++]				= 0;//�ɹ�
			dev_recv->apdu.apdu_data[ptr++]				= 1;//optional

			dev_recv->apdu.apdu_data[ptr++]				= Data_octet_string;//��������
			dev_recv->apdu.apdu_data[ptr++]				= Esam_endata_len+2;
			
			dev_recv->apdu.apdu_data[ptr++]				= Version[0];
			dev_recv->apdu.apdu_data[ptr++]				= Version[1];//�汾��
			
			for(i = 0;i < Esam_endata_len;i++)
			{
				dev_recv->apdu.apdu_data[ptr++]			= stBLE_Esam_Comm.Rx_data[4+i];
			}	
			dev_recv->apdu.apdu_data[ptr++]				= 0;//��optional
			dev_recv->apdu.apdu_data[ptr++]				= 0;//��ʱ���ǩ
			ptr++;
			
			result = BLE_698_Data_Package(dev_recv,ptr,stData);
			
			return result;
	}
	else
	{
		rt_kprintf("[bluetooth]:(%s) read endata1 fail,errcode =0x%X%X!\n",__func__,stBLE_Esam_Comm.Rx_data[0],stBLE_Esam_Comm.Rx_data[1]);
		return RT_ERROR;
	}
}

/********************************************************************  
*	�� �� ��: BLE_698_ESAM_SESS_INTI_Package
*	����˵��: 698�������ַ��֡
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

rt_err_t BLE_698_ESAM_SESS_KEY_Package(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t 	i,ptr,lenth,total_lenth;
	rt_uint16_t Esam_Keydata_len;
	rt_uint8_t Version[2];
	rt_err_t result;
	
	rt_kprintf("[bluetooth]: (%s) \n",__func__);
	
	Esam_Keydata_len = dev_recv->apdu.apdu_data[7]-2;
	
	memcpy(stBLE_Esam_Comm.Tx_data,Esam_KEY_R1,16);
	memcpy(stBLE_Esam_Comm.Tx_data+16,Esam_KEY_R2,16);
	stBLE_Esam_Comm.DataTx_len = 32;
	
	ESAM_Communicattion(APP_SESS_AGREE_ONE,&stBLE_Esam_Comm);
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		memcpy(Esam_KEY_DATA,&stBLE_Esam_Comm.Rx_data[4],32);   //KEY DATA
	}
	else
	{
		rt_kprintf("[bluetooth]:(%s) sess agree state one fail,errcode =0x%X%X!\n",__func__,stBLE_Esam_Comm.Rx_data[0],stBLE_Esam_Comm.Rx_data[1]);
		return RT_ERROR;
	}
	
	memcpy(stBLE_Esam_Comm.Tx_data,Esam_KEY_DATA,32);
	memcpy(stBLE_Esam_Comm.Tx_data+32,&dev_recv->apdu.apdu_data[10],Esam_Keydata_len);
	stBLE_Esam_Comm.DataTx_len = 32+Esam_Keydata_len;
	
	ESAM_Communicattion(APP_SESS_AGREE_TWO,&stBLE_Esam_Comm);
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		memcpy(Esam_KEY_R3,&stBLE_Esam_Comm.Rx_data[4],16);   //KEY DATA
		
		if(memcmp(Esam_KEY_R3,Esam_KEY_R1,16) == 0)
		{
			Esam_KEY_Sess_State = 2;
			CtrlCharge_Event.Router_Module_Info.Bit.BLE_CONNECT = RT_TRUE;//����������
			
			Version[0] = dev_recv->apdu.apdu_data[8];
			Version[1] = dev_recv->apdu.apdu_data[9];
			
			ptr = 6;
			
			dev_recv->apdu.apdu_data[ptr++]				= 0;//�ɹ�
			dev_recv->apdu.apdu_data[ptr++]				= 1;//optional

			dev_recv->apdu.apdu_data[ptr++]				= Data_octet_string;//��������
			dev_recv->apdu.apdu_data[ptr++]				= 4;
			
			dev_recv->apdu.apdu_data[ptr++]				= Version[0];
			dev_recv->apdu.apdu_data[ptr++]				= Version[1];//�汾��
			
			dev_recv->apdu.apdu_data[ptr++]				= 0;
			dev_recv->apdu.apdu_data[ptr++]				= 0;//������� 0000 ��ȷ
			
			dev_recv->apdu.apdu_data[ptr++]				= 0;//��optional
			dev_recv->apdu.apdu_data[ptr++]				= 0;//��ʱ���ǩ
			ptr++;
						
			result = BLE_698_Data_Package(dev_recv,ptr,stData);

		}
	}
	else
	{
		rt_kprintf("[bluetooth]:%s sess agree state two fail,errcode =0x%X%X!\n",__func__,stBLE_Esam_Comm.Rx_data[0],stBLE_Esam_Comm.Rx_data[1]);
		return RT_ERROR;
	}
}

/********************************************************************  
*	�� �� ��: BLE_698_Action_Request_Normal_Analysis
*	����˵��: 698 get request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Normal_Charge_Appply(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t ptr,data_len;
	rt_uint32_t power;
	rt_uint16_t time;
	
	struct _698_DATE_S _698_date_s;
	
//	rt_kprintf("[bluetooth]: ----\033[31m%s\033[0m---- \n",__func__);
	
	
			
	ptr = 0;
	data_len = dev_recv->apdu.apdu_data[9];//������뵥��
	stBLE_Charge_Apply.cRequestNO[0] = data_len;
	if(data_len > sizeof(stBLE_Charge_Apply.cRequestNO))
	{
		rt_kprintf("[bluetooth]: (%s) RequestNO lenth error!!!\n",__func__);
		return RT_ERROR;
	}
	memcpy(&stBLE_Charge_Apply.cRequestNO[1],&dev_recv->apdu.apdu_data[10],data_len);//������뵥��
	
//	rt_sprintf(printfbuf,"");
//	rt_sprintf(printfbuf,"[bluetooth]: (%s) RequestNO:",__func__);
	my_printf((char*)&stBLE_Charge_Apply.cRequestNO[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO:");
	
	
	
	ptr += data_len+2;
	data_len = dev_recv->apdu.apdu_data[9+ptr];
	stBLE_Charge_Apply.cUserID[0] = data_len;
	
	if(data_len > sizeof(stBLE_Charge_Apply.cUserID))
	{
		rt_kprintf("[bluetooth]: (%s) UserID lenth error!!!\n",__func__);
		return RT_ERROR;
	}
	
	memcpy(&stBLE_Charge_Apply.cUserID[1],&dev_recv->apdu.apdu_data[10+ptr],data_len);//�û�ID
	my_printf((char*)&stBLE_Charge_Apply.cUserID[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"UserID:");
	
	
	ptr += data_len+2;
	data_len = dev_recv->apdu.apdu_data[9+ptr];
	stBLE_Charge_Apply.cAssetNO[0] = data_len;
	
	if(data_len > sizeof(stBLE_Charge_Apply.cAssetNO))
	{
		rt_kprintf("[bluetooth]: (%s) AssetNO lenth error!!!\n",__func__);
		return RT_ERROR;
	}
	
	memcpy(&stBLE_Charge_Apply.cAssetNO[1],&dev_recv->apdu.apdu_data[10+ptr],data_len);//·�����ʲ����
	my_printf((char*)&stBLE_Charge_Apply.cAssetNO[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");
	

	ptr += data_len+1;
	
	if(dev_recv->apdu.apdu_data[10+ptr] == 0x11)//������ ������汾
	{
		data_len = 11;
		stBLE_Charge_Apply.GunNum = 0;
		rt_kprintf("[bluetooth]: (%s) GUNNUM: %d\n",__func__,stBLE_Charge_Apply.GunNum);
	}
	else
	{
		data_len = 1;
		stBLE_Charge_Apply.GunNum = dev_recv->apdu.apdu_data[10+ptr];
		rt_kprintf("[bluetooth]: (%s) GUNNUM: %d\n",__func__,stBLE_Charge_Apply.GunNum);
		if(stBLE_Charge_Apply.GunNum > 2)
		{
			rt_kprintf("[bluetooth]: (%s) GUNNUM  error!!!\n",__func__);
			return RT_ERROR;
		}
	}
		
	
	ptr += data_len+1;
	data_len = 4;
	power = dev_recv->apdu.apdu_data[10+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[11+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[12+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[13+ptr];//��繦��
	stBLE_Charge_Apply.ulChargeReqEle = power;
	
	rt_kprintf("[bluetooth]: (%s) ChargeReqEle: %08X \n",__func__,stBLE_Charge_Apply.ulChargeReqEle);

	
	ptr += data_len+1;
	data_len = 7;
	memcpy(&_698_date_s,&dev_recv->apdu.apdu_data[10+ptr],data_len);
	date_time_s_to_sys_time(&_698_date_s,&stBLE_Charge_Apply.PlanUnChg_TimeStamp);
	
	rt_kprintf("[bluetooth]: (%s) StartTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,\
					stBLE_Charge_Apply.PlanUnChg_TimeStamp.Year,\
					stBLE_Charge_Apply.PlanUnChg_TimeStamp.Month,\
					stBLE_Charge_Apply.PlanUnChg_TimeStamp.Day,\
					stBLE_Charge_Apply.PlanUnChg_TimeStamp.Hour,\
					stBLE_Charge_Apply.PlanUnChg_TimeStamp.Minute,\
					stBLE_Charge_Apply.PlanUnChg_TimeStamp.Second);
		
	ptr += data_len+1;
	data_len = 1;
	stBLE_Charge_Apply.ChargeMode = dev_recv->apdu.apdu_data[10+ptr];
	rt_kprintf("[bluetooth]: (%s) ChargeMode: %02X \n",__func__,stBLE_Charge_Apply.ChargeMode);	

	ptr += data_len+2;
	data_len = dev_recv->apdu.apdu_data[9+ptr];
	stBLE_Charge_Apply.Token[0] = data_len;
	
	if(data_len > sizeof(stBLE_Charge_Apply.Token))
	{
		rt_kprintf("[bluetooth]: (%s) Token lenth error!!!\n",__func__);
		return RT_ERROR;
	}
	memcpy(&stBLE_Charge_Apply.Token[1],&dev_recv->apdu.apdu_data[10+ptr],data_len);//��½����
	
	my_printf((char*)&stBLE_Charge_Apply.Token[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"Token:");
	
	BLE_Send_Event_To_Strategy(Cmd_ChgRequest);

	return RT_EOK;
}

/********************************************************************  
*	�� �� ��: BLE_698_Action_Request_Normal_Charge_Plan
*	����˵��: 698 get request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Normal_Charge_Plan(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t i,ptr,data_len;
	rt_uint32_t power;
	rt_uint16_t time;
	
	struct _698_DATE_S _698_date_s;
	
	ptr = 0;
	data_len = dev_recv->apdu.apdu_data[9];//������뵥��
	stBLE_Charge_Plan.cRequestNO[0] = data_len;
	
	if(data_len > sizeof(stBLE_Charge_Plan.cRequestNO))
	{
		rt_kprintf("[bluetooth]: (%s) RequestNO lenth error!!!\n",__func__);
		return RT_ERROR;
	}
	
	memcpy(&stBLE_Charge_Plan.cRequestNO[1],&dev_recv->apdu.apdu_data[10],data_len);//������뵥��
	
//	rt_sprintf(printfbuf,"");
//	rt_sprintf(printfbuf,"[bluetooth]: (%s) RequestNO:",__func__);
	my_printf((char*)&stBLE_Charge_Plan.cRequestNO[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO:");
	
	ptr += data_len+2;
	data_len = dev_recv->apdu.apdu_data[9+ptr];
	stBLE_Charge_Plan.cUserID[0] = data_len;
	if(data_len > sizeof(stBLE_Charge_Plan.cUserID))
	{
		rt_kprintf("[bluetooth]: (%s) UserID lenth error!!!\n",__func__);
		return RT_ERROR;
	}

	memcpy(&stBLE_Charge_Plan.cUserID[1],&dev_recv->apdu.apdu_data[10+ptr],data_len);//�û�ID
	
//	rt_sprintf(printfbuf,"");
//	rt_sprintf(printfbuf,"[bluetooth]: (%s) UserID:",__func__);
	my_printf((char*)&stBLE_Charge_Plan.cUserID[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"UserID:");
	
	
	ptr += data_len+1;
	data_len = 1;
	stBLE_Charge_Plan.ucDecMaker = dev_recv->apdu.apdu_data[10+ptr];//���ߵ�Ԫ
	rt_kprintf("[bluetooth]: (%s) DecMaker: %d\n",__func__,stBLE_Charge_Plan.GunNum);
		
	ptr += data_len+1;
	data_len = 1;
	stBLE_Charge_Plan.ucDecType = dev_recv->apdu.apdu_data[10+ptr];
	rt_kprintf("[bluetooth]: (%s) DecType: %d\n",__func__,stBLE_Charge_Plan.GunNum);//��������
	
	
	ptr += data_len+1;
	data_len = 7;
	
	memcpy(&_698_date_s,&dev_recv->apdu.apdu_data[10+ptr],data_len);//����ʱ��
	date_time_s_to_sys_time(&_698_date_s,&stBLE_Charge_Plan.strDecTime);
	
	ptr += data_len+2;
	data_len = dev_recv->apdu.apdu_data[9+ptr];
	stBLE_Charge_Plan.cAssetNO[0] = data_len;
	if(data_len > sizeof(stBLE_Charge_Plan.cAssetNO))
	{
		rt_kprintf("[bluetooth]: (%s) AssetNO lenth error!!!\n",__func__);
		return RT_ERROR;
	}
	
	memcpy(&stBLE_Charge_Plan.cAssetNO[1],&dev_recv->apdu.apdu_data[10+ptr],data_len);//·�����ʲ����
	
//	rt_sprintf(printfbuf,"");
//	rt_sprintf(printfbuf,"[bluetooth]: (%s) AssetNO:",__func__);
	my_printf((char*)&stBLE_Charge_Plan.cAssetNO[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");
	
	ptr += data_len+1;
	data_len = 1;
	stBLE_Charge_Plan.GunNum = dev_recv->apdu.apdu_data[10+ptr];
	rt_kprintf("[bluetooth]: (%s) GunNum: %d\n",stBLE_Charge_Plan.GunNum);//ǹ��
	
	ptr += data_len+1;
	data_len = 4;
	power = dev_recv->apdu.apdu_data[10+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[11+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[12+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[13+ptr];//����������
	stBLE_Charge_Plan.ulChargeReqEle = power;
	
	rt_kprintf("[bluetooth]: (%s) ChargeReqEle = %08X \n",stBLE_Charge_Plan.ulChargeReqEle);
	
	ptr += data_len+1;
	data_len = 4;
	power = dev_recv->apdu.apdu_data[10+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[11+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[12+ptr];
	power = (power<<8) | dev_recv->apdu.apdu_data[13+ptr];//�������
	stBLE_Charge_Plan.ulChargeRatePow = power;
	
	rt_kprintf("[bluetooth]: (%s) ChargeRatePow = %08X \n",stBLE_Charge_Plan.ulChargeRatePow);

	ptr += data_len+1;
	data_len = 1;
	stBLE_Charge_Plan.ucChargeMode = dev_recv->apdu.apdu_data[10+ptr];
	rt_kprintf("[bluetooth]: (%s) ChargeMode: %d\n",stBLE_Charge_Plan.ucChargeMode);
	
	ptr += data_len+1;
	data_len = 1;
	stBLE_Charge_Plan.ucTimeSlotNum = dev_recv->apdu.apdu_data[10+ptr];
	rt_kprintf("[bluetooth]: (%s) TimeSlotNum: %d\n",stBLE_Charge_Plan.ucTimeSlotNum);

	for(i = 0; i <stBLE_Charge_Plan.ucTimeSlotNum;i++)
	{
		ptr += data_len+1;
		data_len = 7;
		
		memcpy(&_698_date_s,&dev_recv->apdu.apdu_data[10+ptr],data_len);
		date_time_s_to_sys_time(&_698_date_s,&stBLE_Charge_Plan.strChargeTimeSolts[i].strDecStartTime);
		
//		rt_sprintf(printfbuf,"");
//		rt_sprintf(printfbuf,"[bluetooth]: (%s) DecStartTime:",__func__);
		my_printf((char*)&stBLE_Charge_Plan.strChargeTimeSolts[i].strDecStartTime,data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"DecStartTime:");
		
		ptr += data_len+1;
		data_len = 7;
		
		memcpy(&_698_date_s,&dev_recv->apdu.apdu_data[10+ptr],data_len);
		date_time_s_to_sys_time(&_698_date_s,&stBLE_Charge_Plan.strChargeTimeSolts[i].strDecStopTime);

//		rt_sprintf(printfbuf,"");
//		rt_sprintf(printfbuf,"[bluetooth]: (%s) DecStopTime:",__func__);
		my_printf((char*)&stBLE_Charge_Plan.strChargeTimeSolts[i].strDecStopTime,data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"DecStopTime:");
		
		ptr += data_len+1;
		data_len = 4;
		power = dev_recv->apdu.apdu_data[10+ptr];
		power = (power<<8) | dev_recv->apdu.apdu_data[11+ptr];
		power = (power<<8) | dev_recv->apdu.apdu_data[12+ptr];
		power = (power<<8) | dev_recv->apdu.apdu_data[13+ptr];//�������
		stBLE_Charge_Plan.strChargeTimeSolts[i].ulChargePow = power;
		
		rt_kprintf("[bluetooth]: (%s) ChargePow = %08X \n",stBLE_Charge_Plan.strChargeTimeSolts[i].ulChargePow);
		
	}

	BLE_Send_Event_To_Strategy(Cmd_ChgPlanIssue);
	
	return RT_EOK;
}

/********************************************************************  
*	�� �� ��: BLE_698_Action_Request_Normal_Charge_Plan
*	����˵��: 698 get request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Normal_Charge_Start(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t i,ptr,data_len;
	
//	rt_kprintf("[bluetooth]: ----\033[32m%s\033[0m---- \n",__func__);

	ptr = 0;
	data_len = dev_recv->apdu.apdu_data[9+ptr];
	stBLE_Charge_Stop.cAssetNO[0] = data_len;
	if(data_len > sizeof(stBLE_Charge_Stop.cAssetNO))
	{
		rt_kprintf("[bluetooth]: (%s) AssetNO lenth error!!!\n",__func__);
		return RT_ERROR;
	}
	
	memcpy(&stBLE_Charge_Stop.cAssetNO[1],&dev_recv->apdu.apdu_data[10+ptr],data_len);//·�����ʲ����
	
//	rt_sprintf(printfbuf,"");
//	rt_sprintf(printfbuf,"[bluetooth]: (%s) AssetNO:",__func__);
	my_printf((char*)&stBLE_Charge_Stop.cAssetNO[1],data_len,MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");

	ptr += data_len+1;
	data_len = 1;
	stBLE_Charge_Stop.GunNum = dev_recv->apdu.apdu_data[10+ptr];
	rt_kprintf("[bluetooth]: (%s) GunNum: %d\n",__func__,stBLE_Charge_Stop.GunNum);
	
	BLE_Send_Event_To_Strategy(Cmd_StartChg);
	
	return RT_EOK;
}

/********************************************************************  
*	�� �� ��: BLE_698_Action_Request_Normal_Charge_Plan
*	����˵��: 698 get request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Normal_Charge_Stop(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{	
//	rt_kprintf("[bluetooth]: ----\033[31m%s\033[0m---- \n",__func__);
	
	stBLE_Charge_Stop.GunNum = dev_recv->apdu.apdu_data[7];
	
	rt_kprintf("[bluetooth]: (%s) GunNum: %d\n",stBLE_Charge_Stop.GunNum);
	
	BLE_Send_Event_To_Strategy(Cmd_StopChg);
	
	
	return RT_EOK;
}

/********************************************************************  
*	�� �� ��: BLE_698_Action_Request_Normal_Analysis
*	����˵��: 698 get request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Normal_Analysis(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint32_t apdu_oad;
	
	apdu_oad = dev_recv->apdu.apdu_data[2];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[3];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[4];
	apdu_oad = apdu_oad<<8|dev_recv->apdu.apdu_data[5];
	
	switch(apdu_oad)
	{
		case 0xF1000B00://�ỰЭ��  ��һ��
		{
			BLE_698_ESAM_SESS_INTI_Package(dev_recv,stData);
			break;
		}
		case 0xF1000C00://�ỰЭ�� �ڶ���
		{
			BLE_698_ESAM_SESS_KEY_Package(dev_recv,stData);
			break;
		}
		case 0x90027F00://�������
		{
			BLE_698_Action_Request_Normal_Charge_Appply(dev_recv,stData);
			break;
		}
		case 0x90017F00://���ƻ�
		{
			BLE_698_Action_Request_Normal_Charge_Plan(dev_recv,stData);
			break;
		}
		case 0x90047F00://������ ����
		{
			BLE_698_Action_Request_Normal_Charge_Start(dev_recv,stData);
			break;
		}
		case 0x90048000://������ ֹͣ
		{
			BLE_698_Action_Request_Normal_Charge_Stop(dev_recv,stData);
			break;
		}
		default:
			break;
	}
	return RT_EOK;		
}

/********************************************************************  
*	�� �� ��: BLE_698_Security_Request_Analysis_and_Response
*	����˵��: 698 Security Request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Analysis_and_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t apdu_attitude;
	
	apdu_attitude 	= dev_recv->apdu.apdu_data[0];
	switch(apdu_attitude)
	{	
		case ACTION_REQUEST_NOMAL:
		{
			BLE_698_Action_Request_Normal_Analysis(dev_recv,stData);
			break;
		}
	}
	return RT_EOK;
}

/********************************************************************  
*	�� �� ��: BLE_698_Data_Analysis_and_Response
*	����˵��: 698 ���� ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/

rt_err_t BLE_698_Data_Analysis_and_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	
//	rt_kprintf("[bluetooth]: ----\033[31m%s\033[0m---- \n",__func__);
	
	switch(dev_recv->apdu.apdu_cmd)
	{
		case GET_REQUEST:
		{
			BLE_698_Get_Request_Analysis_and_Response(dev_recv,stData);
			break;
		}
		case SECURITY_REQUEST:
		{
			BLE_698_Security_Request_Analysis_and_Response(dev_recv,stData);
			break;
		}
		case ACTION_REQUEST:
		{
			BLE_698_Action_Request_Analysis_and_Response(dev_recv,stData);
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
*	�� �� ��: BLE_698_Charge_Apply_Event_Response
*	����˵��: �ϱ���������¼�
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Charge_Apply_Event_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint16_t i,ptr,lenth;
	rt_err_t res;
	struct _698_DATE_S date_time_s;
	
	rt_kprintf("[bluetooth]: (%s) OrderNum: %08X \n",__func__,stBLE_Charge_Apply_Event.OrderNum);
	
	rt_kprintf("[bluetooth]: (%s) StartTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Apply_Event.StartTimestamp.Year,
																																								stBLE_Charge_Apply_Event.StartTimestamp.Month,\
																																									stBLE_Charge_Apply_Event.StartTimestamp.Day,\
																																									stBLE_Charge_Apply_Event.StartTimestamp.Hour,\
																																									stBLE_Charge_Apply_Event.StartTimestamp.Minute,\
																																									stBLE_Charge_Apply_Event.StartTimestamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) FinishTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Apply_Event.FinishTimestamp.Year,
																																								stBLE_Charge_Apply_Event.FinishTimestamp.Month,\
																																									stBLE_Charge_Apply_Event.FinishTimestamp.Day,\
																																									stBLE_Charge_Apply_Event.FinishTimestamp.Hour,\
																																									stBLE_Charge_Apply_Event.FinishTimestamp.Minute,\
																																									stBLE_Charge_Apply_Event.FinishTimestamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) RequestTimeStamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Apply_Event.RequestTimeStamp.Year,
																																								stBLE_Charge_Apply_Event.RequestTimeStamp.Month,\
																																									stBLE_Charge_Apply_Event.RequestTimeStamp.Day,\
																																									stBLE_Charge_Apply_Event.RequestTimeStamp.Hour,\
																																									stBLE_Charge_Apply_Event.RequestTimeStamp.Minute,\
																																									stBLE_Charge_Apply_Event.RequestTimeStamp.Second);\

	rt_kprintf("[bluetooth]: (%s) PlanUnChg_TimeStamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Apply_Event.PlanUnChg_TimeStamp.Year,
																																								stBLE_Charge_Apply_Event.PlanUnChg_TimeStamp.Month,\
																																									stBLE_Charge_Apply_Event.PlanUnChg_TimeStamp.Day,\
																																									stBLE_Charge_Apply_Event.PlanUnChg_TimeStamp.Hour,\
																																									stBLE_Charge_Apply_Event.PlanUnChg_TimeStamp.Minute,\
																																									stBLE_Charge_Apply_Event.PlanUnChg_TimeStamp.Second);\
	
	rt_kprintf("[bluetooth]: (%s) OccurSource: %d \n",__func__,stBLE_Charge_Apply_Event.OccurSource);
	rt_kprintf("[bluetooth]: (%s) ChannelState: %d \n",__func__,stBLE_Charge_Apply_Event.ChannelState);
	
	rt_kprintf("[bluetooth]: (%s) ChargeMode: %d \n",__func__,stBLE_Charge_Apply_Event.ChargeMode);
	rt_kprintf("[bluetooth]: (%s) CellCapacity: %d \n",__func__,stBLE_Charge_Apply_Event.CellCapacity);
	rt_kprintf("[bluetooth]: (%s) ChargeReqEle: %d \n",__func__,stBLE_Charge_Apply_Event.ChargeReqEle);
	
	rt_kprintf("[bluetooth]: (%s) actSOC: %d \n",__func__,stBLE_Charge_Apply_Event.actSOC);
	rt_kprintf("[bluetooth]: (%s) aimSOC: %d \n",__func__,stBLE_Charge_Apply_Event.aimSOC);
	

	my_printf((char*)&stBLE_Charge_Apply_Event.Token[1],stBLE_Charge_Apply_Event.Token[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"Token:");

	my_printf((char*)&stBLE_Charge_Apply_Event.UserAccount[1],stBLE_Charge_Apply_Event.UserAccount[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"UserID:");
	
	
	
	_698_ble_control = 0x83;//�����ϱ�
	
	ptr = 0;
	dev_recv->apdu.apdu_cmd    = REPORT_RESPONSE |0x80;//��������¼ʱ�䷵��
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//���ɸ���¼�Զ���
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;  //piid-acd
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;		//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x60;//OAD 60120300
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;  //len ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;  //csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;		//OAD 202A0200
	dev_recv->apdu.apdu_data[ptr++]    = 0x2A;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x34;//OAD 34030200
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x11;// len
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼���¼��� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x22;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x1E;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����Դ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x24;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x33;//OI ͨ���ϱ�״̬ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	for(i = 0; i< 12; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = 0x35;//OI 3505��������¼� ����2 ���6~16
		dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x06+i;//
	}

	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�������
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_TSA;// 0x85 tsa����
	dev_recv->apdu.apdu_data[ptr++]    = 0x07;//����
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	for(i = 0; i < 6; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_meter_addr.addr[i];//����ַ
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x11;//����16
	

	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//���� 06//�¼���¼���
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.OrderNum>>24)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.OrderNum>>16)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.OrderNum>>8)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.OrderNum)&0xff;
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Apply_Event.StartTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Apply_Event.FinishTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//�¼�Դ
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;// ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_OAD;//OAD 81
	
	dev_recv->apdu.apdu_data[ptr++]    = 0xF2;//OI F209 �ز����߽ӿ�
	dev_recv->apdu.apdu_data[ptr++]    = 0x09;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_unsigned;//���� 08 ֵ 00
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_octet_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.RequestNO[0];//����
	
	for(i = 0; i < stBLE_Charge_Apply_Event.RequestNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.RequestNO[1+i];//
	}
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.AssetNO[0];//����
	
	for(i = 0; i < stBLE_Charge_Apply_Event.AssetNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = (rt_uint8_t)(stBLE_Charge_Apply_Event.AssetNO[1+i]);
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//ǹ��
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.GunNum;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Apply_Event.RequestTimeStamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.actSOC>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.actSOC)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.aimSOC>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.aimSOC)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.CellCapacity>>24)&0xff;//�������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.CellCapacity>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.CellCapacity>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.CellCapacity)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.ChargeReqEle>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.ChargeReqEle>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.ChargeReqEle>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Apply_Event.ChargeReqEle)&0xff;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Apply_Event.PlanUnChg_TimeStamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//ǹ��
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.ChargeMode;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.Token[0];//����
	
	for(i = 0; i < stBLE_Charge_Apply_Event.Token[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.Token[1+i];//
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.UserAccount[0];//����
	
	for(i = 0; i < stBLE_Charge_Apply_Event.UserAccount[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Apply_Event.UserAccount[1+i];//
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	ptr++; //apdu cmd

	res = BLE_698_Data_Package(dev_recv,ptr,stData);

	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
	return res;
}

/********************************************************************  
*	�� �� ��: BLE_698_Charge_Plan_Event_Response
*	����˵��: �ϱ����ִ���¼� ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Charge_Plan_Event_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint16_t i,ptr,lenth,str_data_len;
	rt_err_t res;
	struct _698_DATE_S date_time_s;
	
	rt_kprintf("[bluetooth]: (%s) OrderNum: %08X \n",stBLE_Charge_Plan_Event.OrderNum);
	
	rt_kprintf("[bluetooth]: (%s) StartTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Plan_Event.StartTimestamp.Year,
																																								stBLE_Charge_Plan_Event.StartTimestamp.Month,\
																																									stBLE_Charge_Plan_Event.StartTimestamp.Day,\
																																									stBLE_Charge_Plan_Event.StartTimestamp.Hour,\
																																									stBLE_Charge_Plan_Event.StartTimestamp.Minute,\
																																									stBLE_Charge_Plan_Event.StartTimestamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) FinishTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Plan_Event.FinishTimestamp.Year,
																																								stBLE_Charge_Plan_Event.FinishTimestamp.Month,\
																																									stBLE_Charge_Plan_Event.FinishTimestamp.Day,\
																																									stBLE_Charge_Plan_Event.FinishTimestamp.Hour,\
																																									stBLE_Charge_Plan_Event.FinishTimestamp.Minute,\
																																									stBLE_Charge_Plan_Event.FinishTimestamp.Second);\
	
	rt_kprintf("[bluetooth]: (%s) OccurSource: %d \n",__func__,stBLE_Charge_Plan_Event.OccurSource);
	rt_kprintf("[bluetooth]: (%s) ChannelState: %d \n",__func__,stBLE_Charge_Plan_Event.ChannelState);
	
	my_printf((char*)&stBLE_Charge_Plan_Event.Chg_Strategy.cRequestNO[1],stBLE_Charge_Plan_Event.Chg_Strategy.cRequestNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO:");
	my_printf((char*)&stBLE_Charge_Plan_Event.Chg_Strategy.cUserID[1],stBLE_Charge_Plan_Event.Chg_Strategy.cUserID[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"UserID:");
	
	rt_kprintf("[bluetooth]: (%s) DMU: %d \n",__func__,stBLE_Charge_Plan_Event.Chg_Strategy.ucDecMaker);
	rt_kprintf("[bluetooth]: (%s) Decision Style: %d \n",__func__,stBLE_Charge_Plan_Event.Chg_Strategy.ucDecType);
	rt_kprintf("[bluetooth]: (%s) Decision Time: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Plan_Event.Chg_Strategy.strDecTime.Year,
																																								stBLE_Charge_Plan_Event.Chg_Strategy.strDecTime.Month,\
																																									stBLE_Charge_Plan_Event.Chg_Strategy.strDecTime.Day,\
																																									stBLE_Charge_Plan_Event.Chg_Strategy.strDecTime.Hour,\
																																									stBLE_Charge_Plan_Event.Chg_Strategy.strDecTime.Minute,\
																																									stBLE_Charge_Plan_Event.Chg_Strategy.strDecTime.Second);\
	
	
	my_printf((char*)&stBLE_Charge_Plan_Event.Chg_Strategy.cAssetNO[1],stBLE_Charge_Plan_Event.Chg_Strategy.cAssetNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");
	
	rt_kprintf("[bluetooth]: (%s) GunNum: %d \n",__func__,stBLE_Charge_Plan_Event.Chg_Strategy.GunNum);
	rt_kprintf("[bluetooth]: (%s) ChargeReqEle: %d \n",__func__,stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeReqEle);
	
	rt_kprintf("[bluetooth]: (%s) ChargeRatePow: %d \n",__func__,stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeRatePow);
	rt_kprintf("[bluetooth]: (%s) ChargeMode: %d \n",__func__,stBLE_Charge_Plan_Event.Chg_Strategy.ucChargeMode);
	
	
	
	for(i = 0; i < stBLE_Charge_Plan_Event.Chg_Strategy.ucTimeSlotNum; i++)
	{
			rt_kprintf("[bluetooth]: (%s) DecStartTime: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Year,
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Month,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Day,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Hour,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Minute,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Second);\
		rt_kprintf("[bluetooth]: (%s) Decision Time: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Year,
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Month,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Day,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Hour,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Minute,\
								stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Second);\
		
		rt_kprintf("[bluetooth]: (%s) ActualPower: %d \n",stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].ulChargePow);	
	}



	_698_ble_control = 0x83;//�����ϱ�
	
	str_data_len = 0x10;//���ƻ��ϱ��¼��ṹ���������
	
	ptr = 0;
	dev_recv->apdu.apdu_cmd    = REPORT_RESPONSE |0x80;//���ִ��״̬����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//���ɸ���¼�Զ���
	dev_recv->apdu.apdu_data[ptr++]    = 0x10;  //piid-acd
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;		//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x60;//OAD 60120300
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;  //len ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;  //csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;		//OAD 202A0200
	dev_recv->apdu.apdu_data[ptr++]    = 0x2A;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x34;//OAD 34020200���ƻ���¼��Ԫ
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = str_data_len;// len
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼���¼��� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x22;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x1E;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����Դ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x24;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x33;//OI ͨ���ϱ�״̬ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	for(i = 0; i< (str_data_len-5); i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = 0x35;//OI 3505��������¼� ����2 ���6~16
		dev_recv->apdu.apdu_data[ptr++]    = 0x04;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x06+i;//
	}

	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�������
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_TSA;// 0x85 tsa����
	dev_recv->apdu.apdu_data[ptr++]    = 0x07;//����
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	for(i = 0; i < 6; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_meter_addr.addr[i];//����ַ
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = str_data_len;//����16
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//���� 06//�¼���¼���
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.OrderNum>>24)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.OrderNum>>16)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.OrderNum>>8)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.OrderNum)&0xff;
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Plan_Event.StartTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Plan_Event.FinishTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.OccurSource;//�¼�Դ
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�ϱ�״̬
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;// ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_OAD;//OAD 81
	
	dev_recv->apdu.apdu_data[ptr++]    = 0xF2;//OI F209 �ز����߽ӿ�
	dev_recv->apdu.apdu_data[ptr++]    = 0x09;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_unsigned;//���� 08 ֵ 00
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_octet_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.cRequestNO[0];//����
	
	for(i = 0; i < stBLE_Charge_Plan_Event.Chg_Strategy.cRequestNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.cRequestNO[1+i];//
	}
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.cUserID[0];//����
	
	for(i = 0; i < stBLE_Charge_Plan_Event.Chg_Strategy.cUserID[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = (rt_uint8_t)(stBLE_Charge_Plan_Event.Chg_Strategy.cUserID[1+i]);//
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//���ߵ�Ԫ 1:��վ 2:������
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.ucDecMaker;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//�������� 1:���� 2:����
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.ucDecType;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Plan_Event.Chg_Strategy.strDecTime,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����ʱ��
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.cAssetNO[0];//����
	
	for(i = 0; i < stBLE_Charge_Plan_Event.Chg_Strategy.cAssetNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = (rt_uint8_t)(stBLE_Charge_Plan_Event.Chg_Strategy.cAssetNO[1+i]);//
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//ǹ��
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.GunNum;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeReqEle>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeReqEle>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeReqEle>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeReqEle)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeRatePow>>24)&0xff;//�������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeRatePow>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeRatePow>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.ulChargeRatePow)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//���ģʽ
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.ucChargeMode;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//seq
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Plan_Event.Chg_Strategy.ucTimeSlotNum;//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//����  �ṹ��
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;//�ṹ���������
	
	
	for(i = 0;i < stBLE_Charge_Plan_Event.Chg_Strategy.ucTimeSlotNum;i++)
	{
		Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStartTime,(struct _698_DATE_S*)&date_time_s);	
		dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////�ƻ���ʼʱ��
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
		
		Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].strDecStopTime,(struct _698_DATE_S*)&date_time_s);	
		dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////�ƻ�����ʱ��
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
		dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
		
		dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
		dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].ulChargePow>>24)&0xff;//����������
		dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].ulChargePow>>16)&0xff;//
		dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].ulChargePow>>8)&0xff;//��ǰSOC
		dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Plan_Event.Chg_Strategy.strChargeTimeSolts[i].ulChargePow)&0xff;//
		
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	ptr++; //apdu cmd

	res = BLE_698_Data_Package(dev_recv,ptr,stData);

	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
	return res;
}


/********************************************************************  
*	�� �� ��: BLE_698_Charge_Exe_Event_Response
*	����˵��: �ϱ����ִ���¼� ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Charge_Exe_Event_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint16_t i,ptr,lenth;
	rt_err_t res;
	struct _698_DATE_S date_time_s;
	
	rt_kprintf("[bluetooth]: (%s) OrderNum: %08X \n",__func__,stBLE_Charge_Exe_Event.OrderNum);
	
	rt_kprintf("[bluetooth]: (%s) StartTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Exe_Event.StartTimestamp.Year,
																																								stBLE_Charge_Exe_Event.StartTimestamp.Month,\
																																									stBLE_Charge_Exe_Event.StartTimestamp.Day,\
																																									stBLE_Charge_Exe_Event.StartTimestamp.Hour,\
																																									stBLE_Charge_Exe_Event.StartTimestamp.Minute,\
																																									stBLE_Charge_Exe_Event.StartTimestamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) FinishTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Exe_Event.FinishTimestamp.Year,
																																								stBLE_Charge_Exe_Event.FinishTimestamp.Month,\
																																									stBLE_Charge_Exe_Event.FinishTimestamp.Day,\
																																									stBLE_Charge_Exe_Event.FinishTimestamp.Hour,\
																																									stBLE_Charge_Exe_Event.FinishTimestamp.Minute,\
																																									stBLE_Charge_Exe_Event.FinishTimestamp.Second);\
	
	rt_kprintf("[bluetooth]: (%s) OccurSource: %d \n",__func__,stBLE_Charge_Exe_Event.OccurSource);
	rt_kprintf("[bluetooth]: (%s) ChannelState: %d \n",__func__,stBLE_Charge_Exe_Event.ChannelState);
	
	my_printf((char*)&stBLE_Charge_Exe_Event.Chg_ExeState.cRequestNO[1],stBLE_Charge_Exe_Event.Chg_ExeState.cRequestNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO:");

	my_printf((char*)&stBLE_Charge_Exe_Event.Chg_ExeState.cAssetNO[1],stBLE_Charge_Exe_Event.Chg_ExeState.cAssetNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");
	
	rt_kprintf("[bluetooth]: (%s) GunNum: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.GunNum);
	rt_kprintf("[bluetooth]: (%s) exeState: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.exeState);
	
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[0]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[0]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[1]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[1]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[2]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[2]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[3]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[3]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[4]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[4]);
	
	rt_kprintf("[bluetooth]: (%s) EleActualValue[0]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[0]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[1]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[1]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[2]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[2]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[3]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[3]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[4]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[4]);
	
	rt_kprintf("[bluetooth]: (%s) ChargeEle[0]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[0]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[1]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[1]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[2]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[2]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[3]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[3]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[4]: 0x%08X kwh \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[4]);
	
	rt_kprintf("[bluetooth]: (%s) ChargeTime: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeTime);
	
	rt_kprintf("[bluetooth]: (%s) PlanPower: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucPlanPower);
	rt_kprintf("[bluetooth]: (%s) ActualPower: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucActualPower);
	
	rt_kprintf("[bluetooth]: (%s) Voltage: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucVoltage);
	rt_kprintf("[bluetooth]: (%s) Current: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucCurrent);

	rt_kprintf("[bluetooth]: (%s) ChgPileState: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ChgPileState);

	rt_kprintf("[bluetooth]: (%s) TimeSlotNum: %d \n",__func__,stBLE_Charge_Exe_Event.Chg_ExeState.ucTimeSlotNum);



	_698_ble_control = 0x83;//�����ϱ�
	
	ptr = 0;
	dev_recv->apdu.apdu_cmd    = REPORT_RESPONSE |0x80;//���ִ��״̬����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//���ɸ���¼�Զ���
	dev_recv->apdu.apdu_data[ptr++]    = 0x10;  //piid-acd
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;		//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x60;//OAD 60120300
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;  //len ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;  //csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;		//OAD 202A0200
	dev_recv->apdu.apdu_data[ptr++]    = 0x2A;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x34;//OAD 34040200���ִ�м�¼��Ԫ
	dev_recv->apdu.apdu_data[ptr++]    = 0x04;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;// len
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼���¼��� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x22;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x1E;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����Դ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x24;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x33;//OI ͨ���ϱ�״̬ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	for(i = 0; i< 13; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = 0x35;//OI 3505��������¼� ����2 ���6~16
		dev_recv->apdu.apdu_data[ptr++]    = 0x06;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x06+i;//
	}

	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�������
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_TSA;// 0x85 tsa����
	dev_recv->apdu.apdu_data[ptr++]    = 0x07;//����
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	for(i = 0; i < 6; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_meter_addr.addr[i];//����ַ
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;//����16
	

	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//���� 06//�¼���¼���
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.OrderNum>>24)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.OrderNum>>16)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.OrderNum>>8)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.OrderNum)&0xff;
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Exe_Event.StartTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Exe_Event.FinishTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//�¼�Դ
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;// ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_OAD;//OAD 81
	
	dev_recv->apdu.apdu_data[ptr++]    = 0xF2;//OI F209 �ز����߽ӿ�
	dev_recv->apdu.apdu_data[ptr++]    = 0x09;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_unsigned;//���� 08 ֵ 00
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_octet_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Exe_Event.Chg_ExeState.cRequestNO[0];//����
	
	for(i = 0; i < stBLE_Charge_Exe_Event.Chg_ExeState.cRequestNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Exe_Event.Chg_ExeState.cRequestNO[1+i];//
	}
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Exe_Event.Chg_ExeState.cAssetNO[0];//����
	
	for(i = 0; i < stBLE_Charge_Exe_Event.Chg_ExeState.cAssetNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = (rt_uint8_t)(stBLE_Charge_Exe_Event.Chg_ExeState.cAssetNO[1+i]);//
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//ǹ��
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Exe_Event.Chg_ExeState.GunNum;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//ִ��״̬
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Exe_Event.Chg_ExeState.exeState;//
	
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[0]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[0]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[0]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[0])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[1]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[1]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[1]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[1])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[2]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[2]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[2]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[2])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[3]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[3]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[3]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[3])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[4]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[4]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[4]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleBottomValue[4])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[0]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[0]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[0]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[0])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[1]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[1]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[1]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[1])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[2]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[2]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[2]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[2])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[3]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[3]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[3]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[3])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[4]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[4]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[4]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ulEleActualValue[4])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[0]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[0]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[0]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[0])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[1]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[1]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[1]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[1])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[2]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[2]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[2]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[2])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[3]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[3]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[3]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[3])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[4]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[4]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[4]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeEle[4])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeTime>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeTime>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeTime>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucChargeTime)&0xff;//
	
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucPlanPower>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucPlanPower>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucPlanPower>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucPlanPower)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucActualPower>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucActualPower>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucActualPower>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucActualPower)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucVoltage.A>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucVoltage.A)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucCurrent.A>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucCurrent.A>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucCurrent.A>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Exe_Event.Chg_ExeState.ucCurrent.A)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//ִ��״̬
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Exe_Event.Chg_ExeState.ChgPileState;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	ptr++; //apdu cmd

	res = BLE_698_Data_Package(dev_recv,ptr,stData);

	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
	return res;
}

/********************************************************************  
*	�� �� ��: BLE_698_Action_Request_Charge_Apply_Response
*	����˵��: 698 Security Request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Charge_Apply_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t i,ptr,lenth,apdu[100];
	rt_err_t res;

	my_printf((char*)&stBLE_Charge_Apply_RSP.cRequestNO[1],stBLE_Charge_Apply_RSP.cRequestNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO:");

	my_printf((char*)&stBLE_Charge_Apply_RSP.cAssetNO[1],stBLE_Charge_Apply_RSP.cAssetNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");
	rt_kprintf("[bluetooth]: (%s) GunNum: %d\n",__func__,stBLE_Charge_Apply_RSP.GunNum);
	rt_kprintf("[bluetooth]: (%s) SucIdle: %d\n",__func__,stBLE_Charge_Apply_RSP.cSucIdle);
	

	ptr = 0;
	apdu[ptr++]    = ACTION_REQUEST |0x80;
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x90;		//oad 90027f00
	apdu[ptr++]    = 0x02;
	apdu[ptr++]    = 0x7F;
	apdu[ptr++]    = 0x00;
	
	apdu[ptr++]    = stBLE_Charge_Apply_RSP.cSucIdle;  //���  00�ɹ�
	apdu[ptr++]    = 0x01;  //optional
	
	apdu[ptr++]    = 0x02;		//�ṹ��
	apdu[ptr++]    = 0x03;   //�ṹ���Ա2
	apdu[ptr++]    = 0x09;   //����
	
	lenth = stBLE_Charge_Apply_RSP.cRequestNO[0];   //����
	apdu[ptr++]    = lenth;
	
	for(i = 0;i < lenth;i++)
	{
		apdu[ptr++] = stBLE_Charge_Apply_RSP.cRequestNO[1+i];//���뵥��   00 11 91 01 61 65 34 20
	}
	apdu[ptr++]    = 0x0A;   //����
	
	lenth = stBLE_Charge_Apply_RSP.cAssetNO[0];   //����
	apdu[ptr++]    = lenth;
	
	for(i = 0;i < lenth;i++)
	{
		apdu[ptr++] = stBLE_Charge_Apply_RSP.cAssetNO[1+i];//�ʲ����  31 32 33
	}
	
	apdu[ptr++]    = Data_enum;
	apdu[ptr++] 	 = stBLE_Charge_Apply_RSP.GunNum;
	
	
	apdu[ptr++]    = 0x00;
	apdu[ptr++]    = 0x00;
	
	memcpy(stBLE_Esam_Comm.Tx_data,Esam_KEY_DATA,32);
	memcpy(stBLE_Esam_Comm.Tx_data+32,apdu,ptr);
	stBLE_Esam_Comm.DataTx_len = 32+ptr;
	
	ESAM_Communicattion(APP_SESS_CALC_MAC_A4,&stBLE_Esam_Comm);
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		ptr = 0;
		lenth = stBLE_Esam_Comm.Rx_data[3];
		
		dev_recv->apdu.apdu_cmd				=	SECURITY_REQUEST |0x80;    
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;//  00 ����  01  ����
		if(lenth >0x7f)
		{
			dev_recv->apdu.apdu_data[ptr++]		= 0x81;  //���ĳ���
		}
		else if(lenth>0xff)
		{
			dev_recv->apdu.apdu_data[ptr++]		= 0x82;  //���ĳ���
		}
		
		dev_recv->apdu.apdu_data[ptr++]		= lenth;  //���ĳ���
		for(i = 0;i < lenth;i++)
		{
			dev_recv->apdu.apdu_data[ptr++]		= stBLE_Esam_Comm.Rx_data[4+i];
		}
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //mac optional
		dev_recv->apdu.apdu_data[ptr++]		= 0x00;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x04;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		ptr++;
		
		res = BLE_698_Data_Package(dev_recv,ptr,stData);
	}
	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
}


/********************************************************************  
*	�� �� ��: BLE_698_Charge_State_Response
*	����˵��: 698 Security Request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Charge_State_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t apdu[300];
	rt_uint16_t i,ptr,lenth;
	rt_err_t res;
	
	
	my_printf((char*)&stBLE_Charge_State.cRequestNO[1],stBLE_Charge_State.cRequestNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO:");
	my_printf((char*)&stBLE_Charge_State.cAssetNO[1],stBLE_Charge_State.cAssetNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");
	
	rt_kprintf("[bluetooth]: (%s) GunNum: %d \n",__func__,stBLE_Charge_State.GunNum);
	rt_kprintf("[bluetooth]: (%s) exeState: %d \n",__func__,stBLE_Charge_State.exeState);

	rt_kprintf("[bluetooth]: (%s) EleBottomValue[0]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleBottomValue[0]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[1]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleBottomValue[1]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[2]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleBottomValue[2]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[3]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleBottomValue[3]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[4]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleBottomValue[4]);
	
	rt_kprintf("[bluetooth]: (%s) EleActualValue[0]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleActualValue[0]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[1]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleActualValue[1]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[2]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleActualValue[2]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[3]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleActualValue[3]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[4]: %08X kwh \n",__func__,stBLE_Charge_State.ulEleActualValue[4]);
	
	rt_kprintf("[bluetooth]: (%s) ChargeEle[0]: %08X kwh \n",__func__,stBLE_Charge_State.ucChargeEle[0]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[1]: %08X kwh \n",__func__,stBLE_Charge_State.ucChargeEle[1]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[2]: %08X kwh \n",__func__,stBLE_Charge_State.ucChargeEle[2]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[3]: %08X kwh \n",__func__,stBLE_Charge_State.ucChargeEle[3]);
	rt_kprintf("[bluetooth]: (%s) ChargeEle[4]: %08X kwh \n",__func__,stBLE_Charge_State.ucChargeEle[4]);
	
	rt_kprintf("[bluetooth]: (%s) ChargeTime: %d \n",__func__,stBLE_Charge_State.ucChargeTime);
	
	rt_kprintf("[bluetooth]: (%s) PlanPower: %d \n",__func__,stBLE_Charge_State.ucPlanPower);
	rt_kprintf("[bluetooth]: (%s) ActualPower: %d \n",__func__,stBLE_Charge_State.ucActualPower);
	
	rt_kprintf("[bluetooth]: (%s) Voltage: %d \n",__func__,stBLE_Charge_State.ucVoltage);
	rt_kprintf("[bluetooth]: (%s) Current: %d \n",__func__,stBLE_Charge_State.ucCurrent);
	rt_kprintf("[bluetooth]: (%s) ChgPileState: %d \n",__func__,stBLE_Charge_State.ChgPileState);
	
	my_printf((char*)&stBLE_Charge_State.cUserID[1],stBLE_Charge_State.cUserID[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"UserID:");
	

	ptr = 0;
	apdu[ptr++]    = GET_REQUEST |0x80;
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x0E;
	apdu[ptr++]    = 0x90;		//oad 90027f00
	apdu[ptr++]    = 0x03;
	apdu[ptr++]    = 0x02;
	apdu[ptr++]    = 0x00;     
	
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x01;  //array����
	apdu[ptr++]    = 0x02;  //�ṹ��
	apdu[ptr++]    = 0x0E;   //�ṹ���Ա2
	
	apdu[ptr++]    = Data_octet_string;   //����
	
	lenth = stBLE_Charge_State.cRequestNO[0];   //����
	apdu[ptr++]    = lenth;
	
	for(i = 0;i < lenth;i++)
	{
		apdu[ptr++] = stBLE_Charge_State.cRequestNO[1+i];//���뵥��   00 11 91 01 61 65 34 20
	}
	
	apdu[ptr++]    = 0x0A;   //����
	
	apdu[ptr++]    = stBLE_Charge_State.cAssetNO[0];//����
	
	for(i = 0; i < stBLE_Charge_State.cAssetNO[0];i++)
	{
		apdu[ptr++]    = (rt_uint8_t)(stBLE_Charge_State.cAssetNO[1+i]);
//		apdu[ptr++]    = (rt_uint8_t)(((stBLE_Charge_State.cAssetNO[1+i]>>4)&0x0f)+0x30);//
//		apdu[ptr++]    = (rt_uint8_t)((stBLE_Charge_State.cAssetNO[1+i]&0x0f)+0x30);//
	}
	

	apdu[ptr++]    = Data_enum;//ǹ��
	apdu[ptr++]    = stBLE_Charge_State.GunNum;//
	
	apdu[ptr++]    = Data_enum;//ִ��״̬
	apdu[ptr++]    = stBLE_Charge_State.exeState;//
	
	apdu[ptr++]    = 0x01;//
	apdu[ptr++]    = 0x05;//
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[0]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[0]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[0]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[0])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[1]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[1]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[1]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[1])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[2]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[2]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[2]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[2])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[3]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[3]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[3]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[3])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[4]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[4]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[4]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleBottomValue[4])&0xff;//
	
	apdu[ptr++]    = 0x01;//
	apdu[ptr++]    = 0x05;//
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[0]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[0]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[0]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[0])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[1]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[1]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[1]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[1])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[2]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[2]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[2]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[2])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[3]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[3]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[3]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[3])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[4]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[4]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[4]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ulEleActualValue[4])&0xff;//
	
	apdu[ptr++]    = 0x01;//
	apdu[ptr++]    = 0x05;//
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[0]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[0]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[0]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[0])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[1]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[1]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[1]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[1])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[2]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[2]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[2]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[2])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[3]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[3]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[3]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[3])&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[4]>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[4]>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[4]>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeEle[4])&0xff;//
	
	apdu[ptr++]    = Data_double_long_unsigned;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeTime>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeTime>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeTime>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucChargeTime)&0xff;//
	
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucPlanPower>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucPlanPower>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucPlanPower>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucPlanPower)&0xff;//
	
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucActualPower>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucActualPower>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucActualPower>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucActualPower)&0xff;//
	
	apdu[ptr++]    = 0x01;//
	apdu[ptr++]    = 0x01;//
	apdu[ptr++]    = Data_long_unsigned;//
	apdu[ptr++]    = (stBLE_Charge_State.ucVoltage.A>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucVoltage.A)&0xff;//
	
	apdu[ptr++]    = 0x01;//
	apdu[ptr++]    = 0x01;//
	apdu[ptr++]    = Data_double_long;//
	apdu[ptr++]    = (stBLE_Charge_State.ucCurrent.A>>24)&0xff;//����������
	apdu[ptr++]    = (stBLE_Charge_State.ucCurrent.A>>16)&0xff;//
	apdu[ptr++]    = (stBLE_Charge_State.ucCurrent.A>>8)&0xff;//��ǰSOC
	apdu[ptr++]    = (stBLE_Charge_State.ucCurrent.A)&0xff;//
	
	apdu[ptr++]    = Data_enum;//���׮״̬
	apdu[ptr++]    = stBLE_Charge_State.ChgPileState;//
	
	apdu[ptr++]    = Data_visible_string;//
	apdu[ptr++]    = stBLE_Charge_State.cUserID[0];//����
	
	for(i = 0; i < stBLE_Charge_State.cUserID[0];i++)
	{
		apdu[ptr++]  = stBLE_Charge_State.cUserID[1+i];//
	}
	
	apdu[ptr++]    = 0x00;//
	apdu[ptr++]    = 0x00;//
	
	memcpy(stBLE_Esam_Comm.Tx_data,Esam_KEY_DATA,32);
	memcpy(stBLE_Esam_Comm.Tx_data+32,apdu,ptr);
	stBLE_Esam_Comm.DataTx_len = 32+ptr;
	
	ESAM_Communicattion(APP_SESS_CALC_MAC_A4,&stBLE_Esam_Comm);
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		ptr = 0;
		lenth = stBLE_Esam_Comm.Rx_data[2];
		lenth = (((lenth<<8)&0xff00) | stBLE_Esam_Comm.Rx_data[3]);
		
		dev_recv->apdu.apdu_cmd				=	SECURITY_REQUEST |0x80;    
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;//  00 ����  01  ����		
		
		if(lenth < 0x80)
		{
			dev_recv->apdu.apdu_data[ptr++]		= lenth;  //���ĳ���
		}
		else if(lenth > 0xff)
		{
			dev_recv->apdu.apdu_data[ptr++]		= 0x82;  //���ĳ���
			
			dev_recv->apdu.apdu_data[ptr++]		= (lenth>>8)&0xff;  //���ĳ���
			dev_recv->apdu.apdu_data[ptr++]		= lenth&0xff;  //���ĳ���
		}
		else
		{
			dev_recv->apdu.apdu_data[ptr++]		= 0x81;  //���ĳ���
			dev_recv->apdu.apdu_data[ptr++]		= lenth;  //���ĳ���
		}
	
		for(i = 0;i < lenth;i++)
		{
			dev_recv->apdu.apdu_data[ptr++]		= stBLE_Esam_Comm.Rx_data[4+i];
		}
//		dev_recv->apdu.apdu_data[ptr++]		= 0x00;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //mac optional
		dev_recv->apdu.apdu_data[ptr++]		= 0x00;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x04;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0xAE;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0xC1;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x39;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x0A;  //
		ptr++;
		
		res = BLE_698_Data_Package(dev_recv,ptr,stData);
	}
	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
}

/********************************************************************  
*	�� �� ��: BLE_698_Action_Request_Charge_Plan_Response
*	����˵��: 698 Security Request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Action_Request_Charge_Plan_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint8_t i,ptr,lenth,apdu[100];
	rt_err_t res;

	ptr = 0;
	apdu[ptr++]    = ACTION_REQUEST |0x80;
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x90;		//oad 90017f00
	apdu[ptr++]    = 0x01;
	apdu[ptr++]    = 0x7F;
	apdu[ptr++]    = 0x00;
	
	apdu[ptr++]    = stBLE_Charge_Plan_RSP.cSucIdle;  //���  00�ɹ�
	apdu[ptr++]    = 0x01;  //optional
	
	apdu[ptr++]    = 0x02;		//�ṹ��
	apdu[ptr++]    = 0x03;   //�ṹ���Ա3
	
	apdu[ptr++]    = Data_octet_string;   //����
	lenth = stBLE_Charge_Plan_RSP.cRequestNO[0];   //����
	apdu[ptr++]    = lenth;
	for(i = 0;i < lenth;i++)
	{
		apdu[ptr++] = stBLE_Charge_Plan_RSP.cRequestNO[1+i];//���뵥��   00 11 91 01 61 65 34 20
	}
	
	
	apdu[ptr++]    = Data_visible_string;   //����
	lenth = stBLE_Charge_Plan_RSP.cAssetNO[0];   //����
	apdu[ptr++]    = lenth;
	for(i = 0;i < lenth;i++)
	{
		apdu[ptr++] = stBLE_Charge_Plan_RSP.cAssetNO[1+i];//�ʲ����  31 32 33
	}
	
	apdu[ptr++]    = Data_enum;
	apdu[ptr++] 	 = stBLE_Charge_Plan_RSP.GunNum;
	
	apdu[ptr++]    = 0x00;
	apdu[ptr++]    = 0x00;
	
	memcpy(stBLE_Esam_Comm.Tx_data,Esam_KEY_DATA,32);
	memcpy(stBLE_Esam_Comm.Tx_data+32,apdu,ptr);
	stBLE_Esam_Comm.DataTx_len = 32+ptr;
	
	ESAM_Communicattion(APP_SESS_CALC_MAC_A4,&stBLE_Esam_Comm);
	if((stBLE_Esam_Comm.Rx_data[0] == 0x90)&&(stBLE_Esam_Comm.Rx_data[1] == 0x00))
	{
		ptr = 0;
		lenth = stBLE_Esam_Comm.Rx_data[3];
		
		dev_recv->apdu.apdu_cmd				=	SECURITY_REQUEST |0x80;    
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;//  00 ����  01  ����
		if(lenth >0x7f)
		{
			dev_recv->apdu.apdu_data[ptr++]		= 0x81;  //���ĳ���
		}
		else if(lenth>0xff)
		{
			dev_recv->apdu.apdu_data[ptr++]		= 0x82;  //���ĳ���
		}
		
		dev_recv->apdu.apdu_data[ptr++]		= lenth;  //���ĳ���
		for(i = 0;i < lenth;i++)
		{
			dev_recv->apdu.apdu_data[ptr++]		= stBLE_Esam_Comm.Rx_data[4+i];
		}
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //mac optional
		dev_recv->apdu.apdu_data[ptr++]		= 0x00;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x04;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		dev_recv->apdu.apdu_data[ptr++]		= 0x01;  //
		ptr++;
		
		res = BLE_698_Data_Package(dev_recv,ptr,stData);
	}
	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
}


/********************************************************************  
*	�� �� ��: BLE_698_Charge_Record_Event_Response
*	����˵��: 698 Security Request ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Charge_Record_Event_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint16_t i,ptr,lenth;
	rt_err_t res;
	struct _698_DATE_S date_time_s;
	
	
	rt_kprintf("[bluetooth]: (%s) OrderNum: %08X \n",__func__,stBLE_Charge_Record.OrderNum);
	
	rt_kprintf("[bluetooth]: (%s) StartTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Record.StartTimestamp.Year,
																																								stBLE_Charge_Record.StartTimestamp.Month,\
																																									stBLE_Charge_Record.StartTimestamp.Day,\
																																									stBLE_Charge_Record.StartTimestamp.Hour,\
																																									stBLE_Charge_Record.StartTimestamp.Minute,\
																																									stBLE_Charge_Record.StartTimestamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) FinishTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Record.FinishTimestamp.Year,
																																								stBLE_Charge_Record.FinishTimestamp.Month,\
																																									stBLE_Charge_Record.FinishTimestamp.Day,\
																																									stBLE_Charge_Record.FinishTimestamp.Hour,\
																																									stBLE_Charge_Record.FinishTimestamp.Minute,\
																																									stBLE_Charge_Record.FinishTimestamp.Second);\
	
	rt_kprintf("[bluetooth]: (%s) OccurSource: %d \n",__func__,stBLE_Charge_Record.OccurSource);
	rt_kprintf("[bluetooth]: (%s) ChannelState: %d \n",__func__,stBLE_Charge_Record.ChannelState);

	my_printf((char*)&stBLE_Charge_Record.RequestNO[1],stBLE_Charge_Record.RequestNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO:");

	my_printf((char*)&stBLE_Charge_Record.AssetNO[1],stBLE_Charge_Record.AssetNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO:");
	
	
	rt_kprintf("[bluetooth]: (%s) GunNum: %d \n",__func__,stBLE_Charge_Record.GunNum);
	rt_kprintf("[bluetooth]: (%s) ChargeReqEle: %d \n",__func__,stBLE_Charge_Record.ChargeReqEle);
	
	rt_kprintf("[bluetooth]: (%s) RequestTimeStamp: %02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Record.RequestTimeStamp.Year,
																																								stBLE_Charge_Record.RequestTimeStamp.Month,\
																																									stBLE_Charge_Record.RequestTimeStamp.Day,\
																																									stBLE_Charge_Record.RequestTimeStamp.Hour,\
																																									stBLE_Charge_Record.RequestTimeStamp.Minute,\
																																									stBLE_Charge_Record.RequestTimeStamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) PlanUnChg_TimeStamp: %02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Record.PlanUnChg_TimeStamp.Year,
																																								stBLE_Charge_Record.PlanUnChg_TimeStamp.Month,\
																																									stBLE_Charge_Record.PlanUnChg_TimeStamp.Day,\
																																									stBLE_Charge_Record.PlanUnChg_TimeStamp.Hour,\
																																									stBLE_Charge_Record.PlanUnChg_TimeStamp.Minute,\
																																									stBLE_Charge_Record.PlanUnChg_TimeStamp.Second);\
	rt_kprintf("[bluetooth]: (%s) ChargeMode: %d \n",__func__,stBLE_Charge_Record.ChargeMode);
	
	
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[0]: %08X kwh \n",__func__,stBLE_Charge_Record.StartMeterValue[0]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[1]: %08X kwh \n",__func__,stBLE_Charge_Record.StartMeterValue[1]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[2]: %08X kwh \n",__func__,stBLE_Charge_Record.StartMeterValue[2]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[3]: %08X kwh \n",__func__,stBLE_Charge_Record.StartMeterValue[3]);
	rt_kprintf("[bluetooth]: (%s) EleBottomValue[4]: %08X kwh \n",__func__,stBLE_Charge_Record.StartMeterValue[4]);
	
	rt_kprintf("[bluetooth]: (%s) EleActualValue[0]: %08X kwh \n",__func__,stBLE_Charge_Record.StopMeterValue[0]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[1]: %08X kwh \n",__func__,stBLE_Charge_Record.StopMeterValue[1]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[2]: %08X kwh \n",__func__,stBLE_Charge_Record.StopMeterValue[2]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[3]: %08X kwh \n",__func__,stBLE_Charge_Record.StopMeterValue[3]);
	rt_kprintf("[bluetooth]: (%s) EleActualValue[4]: %08X kwh \n",__func__,stBLE_Charge_Record.StopMeterValue[4]);
	
	rt_kprintf("[bluetooth]: (%s) ChgStartTime: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Record.ChgStartTime.Year,
																																								stBLE_Charge_Record.ChgStartTime.Month,\
																																									stBLE_Charge_Record.ChgStartTime.Day,\
																																									stBLE_Charge_Record.ChgStartTime.Hour,\
																																									stBLE_Charge_Record.ChgStartTime.Minute,\
																																									stBLE_Charge_Record.ChgStartTime.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) ChgStopTime: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Charge_Record.ChgStopTime.Year,
																																								stBLE_Charge_Record.ChgStopTime.Month,\
																																									stBLE_Charge_Record.ChgStopTime.Day,\
																																									stBLE_Charge_Record.ChgStopTime.Hour,\
																																									stBLE_Charge_Record.ChgStopTime.Minute,\
																																									stBLE_Charge_Record.ChgStopTime.Second);\
	

	rt_kprintf("[bluetooth]: (%s) ucChargeEle[0]: %08X kwh \n",__func__,stBLE_Charge_Record.ucChargeEle[0]);
	rt_kprintf("[bluetooth]: (%s) ucChargeEle[1]: %08X kwh \n",__func__,stBLE_Charge_Record.ucChargeEle[1]);
	rt_kprintf("[bluetooth]: (%s) ucChargeEle[2]: %08X kwh \n",__func__,stBLE_Charge_Record.ucChargeEle[2]);
	rt_kprintf("[bluetooth]: (%s) ucChargeEle[3]: %08X kwh \n",__func__,stBLE_Charge_Record.ucChargeEle[3]);
	rt_kprintf("[bluetooth]: (%s) ucChargeEle[4]: %08X kwh \n",__func__,stBLE_Charge_Record.ucChargeEle[4]);
	
	rt_kprintf("[bluetooth]: (%s) ucChargeTime: %d \n",__func__,stBLE_Charge_Record.ucChargeTime);
	

	_698_ble_control = 0x83;//�����ϱ�
	
	ptr = 0;
	dev_recv->apdu.apdu_cmd    = REPORT_RESPONSE |0x80;//���ִ��״̬����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//���ɸ���¼�Զ���
	dev_recv->apdu.apdu_data[ptr++]    = 0x14;  //piid-acd
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;		//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x60;//OAD 60120300
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;  //len ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;  //csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;		//OAD 202A0200
	dev_recv->apdu.apdu_data[ptr++]    = 0x2A;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x34;//OAD 34040200���ִ�м�¼��Ԫ
	dev_recv->apdu.apdu_data[ptr++]    = 0x08;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x13;// len
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼���¼��� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x22;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x1E;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����Դ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x24;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x33;//OI ͨ���ϱ�״̬ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	for(i = 0; i< 14; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = 0x35;//OI 3505��������¼� ����2 ���6~16
		dev_recv->apdu.apdu_data[ptr++]    = 0x08;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x06+i;//
	}

	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�������
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_TSA;// 0x55 tsa����
	dev_recv->apdu.apdu_data[ptr++]    = 0x07;//����
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	for(i = 0; i < 6; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_meter_addr.addr[i];//����ַ
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x13;//����16
	

	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//���� 06//�¼���¼���
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.OrderNum>>24)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.OrderNum>>16)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.OrderNum>>8)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.OrderNum)&0xff;
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Record.StartTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Record.FinishTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//�¼�Դ
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//ͨ��״̬
//	
//	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
//	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
//	
//	dev_recv->apdu.apdu_data[ptr++]    = 0x02;// ����
//	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
//	
//	dev_recv->apdu.apdu_data[ptr++]    = Data_OAD;//OAD 81
//	
//	dev_recv->apdu.apdu_data[ptr++]    = 0xF2;//OI F209 �ز����߽ӿ�
//	dev_recv->apdu.apdu_data[ptr++]    = 0x09;//
//	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
//	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
//	
//	dev_recv->apdu.apdu_data[ptr++]    = Data_unsigned;//���� 08 ֵ 00
//	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_octet_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Record.RequestNO[0];//����
	
	for(i = 0; i < stBLE_Charge_Record.RequestNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Record.RequestNO[1+i];//
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Record.cUserID[0];//����
	
	for(i = 0; i < stBLE_Charge_Record.cUserID[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]  = stBLE_Charge_Record.cUserID[1+i];//
	}
	
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_visible_string;//
//	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Record.AssetNO[0]*2;//����
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Record.AssetNO[0];
	
	for(i = 0; i < stBLE_Charge_Record.AssetNO[0];i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = (rt_uint8_t)(stBLE_Charge_Record.AssetNO[1+i]);//
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//ǹ��
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Record.GunNum;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ChargeReqEle>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ChargeReqEle>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ChargeReqEle>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ChargeReqEle)&0xff;//
	
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Record.RequestTimeStamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Record.PlanUnChg_TimeStamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_enum;//���ģʽ
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Charge_Record.ChargeMode;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[0]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[0]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[0]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[0])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[1]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[1]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[1]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[1])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[2]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[2]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[2]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[2])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[3]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[3]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[3]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[3])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[4]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[4]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[4]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StartMeterValue[4])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[0]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[0]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[0]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[0])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[1]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[1]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[1]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[1])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[2]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[2]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[2]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[2])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[3]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[3]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[3]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[3])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[4]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[4]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[4]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.StopMeterValue[4])&0xff;//
	
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Record.ChgStartTime,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Charge_Record.ChgStopTime,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[0]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[0]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[0]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[0])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[1]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[1]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[1]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[1])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[2]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[2]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[2]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[2])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[3]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[3]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[3]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[3])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[4]>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[4]>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[4]>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeEle[4])&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeTime>>24)&0xff;//����������
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeTime>>16)&0xff;//
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeTime>>8)&0xff;//��ǰSOC
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Charge_Record.ucChargeTime)&0xff;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	ptr++; //apdu cmd

	res = BLE_698_Data_Package(dev_recv,ptr,stData);

	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
	return res;
}

/********************************************************************  
*	�� �� ��: BLE_698_Router_Fault_Event_Response
*	����˵��: ·�����쳣�¼� ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Router_Fault_Event_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint16_t i,ptr,lenth,str_data_len;
	rt_err_t res;
	rt_uint32_t temp;
	struct _698_DATE_S date_time_s;
	
	rt_kprintf("[bluetooth]: (%s) OrderNum: %08X \n",stBLE_Charge_Plan_Event.OrderNum);
	
	rt_kprintf("[bluetooth]: (%s) StartTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Router_Fault_Event.StartTimestamp.Year,
																																								stBLE_Router_Fault_Event.StartTimestamp.Month,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Day,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Hour,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Minute,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) FinishTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Router_Fault_Event.FinishTimestamp.Year,
																																								stBLE_Router_Fault_Event.FinishTimestamp.Month,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Day,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Hour,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Minute,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Second);\
	
	rt_kprintf("[bluetooth]: (%s) OccurSource: %d \n",__func__,stBLE_Router_Fault_Event.OccurSource);
	rt_kprintf("[bluetooth]: (%s) ChannelState: %d \n",__func__,stBLE_Router_Fault_Event.ChannelState);
	
	rt_kprintf("[bluetooth]: (%s) Router_Fault: %04X \n",__func__,stBLE_Router_Fault_Event.Router_Fault.Total_Fau);
	

	_698_ble_control = 0x83;//�����ϱ�
	
	str_data_len = 0x07;//���ƻ��ϱ��¼��ṹ���������
	
	ptr = 0;
	dev_recv->apdu.apdu_cmd    = REPORT_RESPONSE |0x80;//���ִ��״̬����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//���ɸ���¼�Զ���
	dev_recv->apdu.apdu_data[ptr++]    = 0x10;  //piid-acd
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;		//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x60;//OAD 60120300
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;  //len ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;  //csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;		//OAD 202A0200
	dev_recv->apdu.apdu_data[ptr++]    = 0x2A;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x34;//OAD 34020200���ƻ���¼��Ԫ
	dev_recv->apdu.apdu_data[ptr++]    = 0x06;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = str_data_len;// len
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼���¼��� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x22;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x1E;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����Դ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x24;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x33;//OI ͨ���ϱ�״̬ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	for(i = 0; i< (str_data_len-5); i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = 0x35;//OI 3505����쳣�¼� ����2 ���6~7
		dev_recv->apdu.apdu_data[ptr++]    = 0x07;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x06+i;//
	}

	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�������
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_TSA;// 0x85 tsa����
	dev_recv->apdu.apdu_data[ptr++]    = 0x07;//����
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	for(i = 0; i < 6; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_meter_addr.addr[i];//����ַ
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = str_data_len;//����16
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//���� 06//�¼���¼���
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum>>24)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum>>16)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum>>8)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum)&0xff;
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Router_Fault_Event.StartTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Router_Fault_Event.FinishTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Router_Fault_Event.OccurSource;//�¼�Դ
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�ϱ�״̬
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;// ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_OAD;//OAD 81
	
	dev_recv->apdu.apdu_data[ptr++]    = 0xF2;//OI F209 �ز����߽ӿ�
	dev_recv->apdu.apdu_data[ptr++]    = 0x09;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_unsigned;//���� 08 ֵ 00
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_bit_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = 15;//����
	
	
	temp = stBLE_Router_Fault_Event.Router_Fault.Total_Fau;
	dev_recv->apdu.apdu_data[ptr++]    = ((temp>>8)&0xff);//����
	dev_recv->apdu.apdu_data[ptr++]    = (temp&0xff);//����
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_bit_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = 15;//����
	
	
	temp = stBLE_Router_Fault_Event.Router_Fault.Total_Fau;
	dev_recv->apdu.apdu_data[ptr++]    = ((temp>>8)&0xff);//����
	dev_recv->apdu.apdu_data[ptr++]    = (temp&0xff);//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	ptr++; //apdu cmd

	res = BLE_698_Data_Package(dev_recv,ptr,stData);

	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
	return res;
}

/********************************************************************  
*	�� �� ��: BLE_698_Pile_Fault_Event_Response
*	����˵��: ���׮�쳣�¼� ����
*	��    ��: ��
*	�� �� ֵ: ��
********************************************************************/
rt_err_t BLE_698_Pile_Fault_Event_Response(struct _698_BLE_FRAME *dev_recv,ScmUart_Comm* stData)
{
	rt_uint16_t i,ptr,lenth,str_data_len;
	rt_err_t res;
	rt_uint32_t temp;
	struct _698_DATE_S date_time_s;
	
	rt_kprintf("[bluetooth]: (%s) OrderNum: %08X \n",stBLE_Charge_Plan_Event.OrderNum);
	
	rt_kprintf("[bluetooth]: (%s) StartTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Router_Fault_Event.StartTimestamp.Year,
																																								stBLE_Router_Fault_Event.StartTimestamp.Month,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Day,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Hour,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Minute,\
																																									stBLE_Router_Fault_Event.StartTimestamp.Second);\
																																									
	rt_kprintf("[bluetooth]: (%s) FinishTimestamp: 20%02X-%02X-%02X-%02X-%02X-%02X!\n",__func__,stBLE_Router_Fault_Event.FinishTimestamp.Year,
																																								stBLE_Router_Fault_Event.FinishTimestamp.Month,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Day,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Hour,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Minute,\
																																									stBLE_Router_Fault_Event.FinishTimestamp.Second);\
	
	rt_kprintf("[bluetooth]: (%s) OccurSource: %d \n",__func__,stBLE_Router_Fault_Event.OccurSource);
	rt_kprintf("[bluetooth]: (%s) ChannelState: %d \n",__func__,stBLE_Router_Fault_Event.ChannelState);
	
	rt_kprintf("[bluetooth]: (%s) Router_Fault: %04X \n",__func__,stBLE_Router_Fault_Event.Router_Fault.Total_Fau);
	

	_698_ble_control = 0x83;//�����ϱ�
	
	str_data_len = 0x07;//���ƻ��ϱ��¼��ṹ���������
	
	ptr = 0;
	dev_recv->apdu.apdu_cmd    = REPORT_RESPONSE |0x80;//���ִ��״̬����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//���ɸ���¼�Զ���
	dev_recv->apdu.apdu_data[ptr++]    = 0x10;  //piid-acd
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;		//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x60;//OAD 60120300
	dev_recv->apdu.apdu_data[ptr++]    = 0x12;
	dev_recv->apdu.apdu_data[ptr++]    = 0x03;
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;  //len ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;  //csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;		//OAD 202A0200
	dev_recv->apdu.apdu_data[ptr++]    = 0x2A;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;   //
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//csd type
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x34;//OAD 34020200���ƻ���¼��Ԫ
	dev_recv->apdu.apdu_data[ptr++]    = 0x07;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = str_data_len;// len
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼���¼��� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x22;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x1E;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����ʱ�� ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x20;//OI �¼�����Դ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x24;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x33;//OI ͨ���ϱ�״̬ ����2
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	for(i = 0; i< (str_data_len-5); i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = 0x35;//OI 3505����쳣�¼� ����2 ���6~7
		dev_recv->apdu.apdu_data[ptr++]    = 0x07;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
		dev_recv->apdu.apdu_data[ptr++]    = 0x06+i;//
	}

	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�������
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//len
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_TSA;// 0x85 tsa����
	dev_recv->apdu.apdu_data[ptr++]    = 0x07;//����
	dev_recv->apdu.apdu_data[ptr++]    = 0x05;//
	for(i = 0; i < 6; i++)
	{
		dev_recv->apdu.apdu_data[ptr++]    = stBLE_meter_addr.addr[i];//����ַ
	}
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	dev_recv->apdu.apdu_data[ptr++]    = str_data_len;//����16
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_double_long_unsigned;//���� 06//�¼���¼���
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum>>24)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum>>16)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum>>8)&0xff;
	dev_recv->apdu.apdu_data[ptr++]    = (stBLE_Router_Fault_Event.OrderNum)&0xff;
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Router_Fault_Event.StartTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;//����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	Sys_time_to_date_time_s((STR_SYSTEM_TIME*)&stBLE_Router_Fault_Event.FinishTimestamp,(struct _698_DATE_S*)&date_time_s);	
	dev_recv->apdu.apdu_data[ptr++]    = Data_date_time_s;////����
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[0];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.year[1];//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.month;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.day;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.hour;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.minute;//
	dev_recv->apdu.apdu_data[ptr++]    = date_time_s.second;//
	
	dev_recv->apdu.apdu_data[ptr++]    = stBLE_Router_Fault_Event.OccurSource;//�¼�Դ
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//�ϱ�״̬
	dev_recv->apdu.apdu_data[ptr++]    = 0x01;//
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;// ����
	dev_recv->apdu.apdu_data[ptr++]    = 0x02;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_OAD;//OAD 81
	
	dev_recv->apdu.apdu_data[ptr++]    = 0xF2;//OI F209 �ز����߽ӿ�
	dev_recv->apdu.apdu_data[ptr++]    = 0x09;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_unsigned;//���� 08 ֵ 00
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_bit_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = 27;//����
	
	
	temp = stBLE_Router_Fault_Event.Pile_Fault.Total_Fau;
	dev_recv->apdu.apdu_data[ptr++]    = ((temp>>8)&0xff);//����
	dev_recv->apdu.apdu_data[ptr++]    = (temp&0xff);//����
	
	dev_recv->apdu.apdu_data[ptr++]    = Data_bit_string;//����
	dev_recv->apdu.apdu_data[ptr++]    = 27;//����
	
	
	temp = stBLE_Router_Fault_Event.Pile_Fault.Total_Fau;
	dev_recv->apdu.apdu_data[ptr++]    = ((temp>>8)&0xff);//����
	dev_recv->apdu.apdu_data[ptr++]    = (temp&0xff);//����
	
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	dev_recv->apdu.apdu_data[ptr++]    = 0x00;//
	
	ptr++; //apdu cmd

	res = BLE_698_Data_Package(dev_recv,ptr,stData);

	if(stData->DataTx_len)
	{
		BLE_Trans_Send(bluetooth_serial,0,0,stData);
		memset(stData->Tx_data,0,stData->DataTx_len);
		memset(dev_recv,0,sizeof(struct _698_BLE_FRAME));
		stData->DataTx_len = 0;
	}
	return res;
}



/******************************************��������ݴ��ݽӿ�******************************************/

rt_uint8_t BLE_Send_To_Strategy_Event_Ack(void)
{
	rt_uint32_t i,cmd;
	
	if(g_BLE_Send_to_Strategy_event == 0)//���¼�����
		return 0;
	
	for(i = 0; i <sizeof(rt_uint32_t)*8;i++)
	{
		if(g_BLE_Send_to_Strategy_event&(0x00000001<<i))
		{
			cmd = i;
			break;
		}
	}
	switch(cmd)
	{
		case Cmd_ChgRequestReport:
			BLE_Send_Event_To_Strategy(Cmd_ChgRequestReportAck);
		break;
		
		case Cmd_ChgPlanOffer:
			BLE_Send_Event_To_Strategy(Cmd_ChgPlanOfferAck);
		break;
		
		case Cmd_ChgPlanExeState:
			BLE_Send_Event_To_Strategy(Cmd_ChgPlanExeStateAck);
		break;
			
		case Cmd_ChgRecord:
			BLE_Send_Event_To_Strategy(Cmd_ChgRecordAck);
		break;
		
		case Cmd_DeviceFault:
			BLE_Send_Event_To_Strategy(Cmd_DeviceFaultAck);
		break;
		
		case Cmd_PileFault:
			BLE_Send_Event_To_Strategy(Cmd_PileFaultAck);
		break;

		default:
			break;
	}
	
}


rt_uint8_t BLE_Send_Event_To_Strategy(COMM_CMD_C cmd)//�����¼�������
{
	g_BLE_Send_to_Strategy_event |= (0x00000001<<cmd);
	
	rt_kprintf("[bluetooth]: Send cmd = %d      Strategy_event = 0x%08X\n",cmd,g_BLE_Send_to_Strategy_event);	
	return 0;	
}


rt_uint32_t Strategy_get_BLE_event(void)
{	
	if(g_BLE_Send_to_Strategy_event == 0)//���¼�����
	return 0;
		
	return g_BLE_Send_to_Strategy_event;
}
	
rt_uint8_t BLE_Get_Strategy_Event(void)//��ȡ�� ���Դ��ݹ������¼� ����Ӧ����
{
	rt_uint32_t i,cmd;
	
	if(g_BLE_Get_Strategy_event == 0)//���¼�����
		return 0;
	
	for(i = 0; i <sizeof(rt_uint32_t)*8;i++)
	{
		if(g_BLE_Get_Strategy_event&(0x00000001<<i))
		{
			cmd = i;
			break;
		}
	}
	rt_kprintf("[bluetooth]: (%s) get cmd: %s\n",__func__,comm_cmdtype_to_string(cmd));
	
	switch(cmd)
	{
		case Cmd_ChgRequestAck:
			BLE_698_Action_Request_Charge_Apply_Response(&_698_ble_frame,&stBLE_Comm);
		break;
		
		case Cmd_ChgRequestReport:
			BLE_698_Charge_Apply_Event_Response(&_698_ble_frame,&stBLE_Comm);
		break;
		
		case Cmd_ChgPlanOffer:
			BLE_698_Charge_Plan_Event_Response(&_698_ble_frame,&stBLE_Comm);
		break;
		
		case Cmd_ChgPlanExeState:
			BLE_698_Charge_Exe_Event_Response(&_698_ble_frame,&stBLE_Comm);
		break;
		
		case Cmd_ChgPlanIssueAck:
				BLE_698_Action_Request_Charge_Plan_Response(&_698_ble_frame,&stBLE_Comm);//δ����
		break;
		
		case Cmd_RouterExeStateAck:
			BLE_698_Charge_State_Response(&_698_ble_frame,&stBLE_Comm);
		break;
		
		case Cmd_ChgRecord:
			BLE_698_Charge_Record_Event_Response(&_698_ble_frame,&stBLE_Comm);
		break;
		
		case Cmd_DeviceFault:
			BLE_698_Router_Fault_Event_Response(&_698_ble_frame,&stBLE_Comm);
		break;
		
		case Cmd_PileFault:
			BLE_698_Pile_Fault_Event_Response(&_698_ble_frame,&stBLE_Comm);
		break;

		default:
			break;
	}
	
	g_BLE_Get_Strategy_event &= (rt_uint32_t)(~(0x00000001<<cmd));
	
	if((g_BLE_Get_Strategy_event&(0x00000001<<cmd)) == (0x00000001<<cmd))
	{
		rt_kprintf("[bluetooth]: (%s)  %s clear fail\n",__func__,comm_cmdtype_to_string(cmd));
	}
	else
	{
		rt_kprintf("[bluetooth]: (%s)  %s clear sucess\n",__func__,comm_cmdtype_to_string(cmd));
	}
	return 0;
}

rt_uint8_t BLE_CtrlUnit_RecResp(COMM_CMD_C cmd,void *STR_SetPara,int count)
{
	rt_uint8_t result=1;
	
	rt_kprintf("[bluetooth]: (%s) process cmd: %s\n",__func__,comm_cmdtype_to_string(cmd));

	switch(cmd){
/////////////////////////////////////////////BLE to strategy////////////////////////////////////		
		case Cmd_ChgRequest:	//������� ���뵥  CHARGE_APPLY stBLE_Charge_Apply;
			if(count >= 0)
			{
				*((CHARGE_APPLY*)STR_SetPara) = stBLE_Charge_Apply;				
			}
			result=0;			
		break;
		
		case Cmd_RouterExeState:
			if(count >= 0)
			{
				*((CHARGE_EXE_STATE_ASK*)STR_SetPara) = stBLE_Charge_State_Ask;
			}
			result=0;	
		break;
		
		case Cmd_ChgPlanIssue:
			if(count >= 0)
			{
				*((CHARGE_STRATEGY*)STR_SetPara) = stBLE_Charge_Plan;
			}
			result=0;		
		break;
		
		case Cmd_StopChg:
			if(count >= 0)
			{
				*((CTL_CHARGE*)STR_SetPara) = stBLE_Charge_Stop;
			}
			result=0;	
		break;
		
////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////strategy to ble////////////////////////////////////////////		
		case Cmd_ChgRequestAck:
		{
			stBLE_Charge_Apply_RSP = *((CHARGE_APPLY_RSP*)STR_SetPara);

			my_printf((char*)&stBLE_Charge_Apply_RSP.cRequestNO[1],stBLE_Charge_Apply_RSP.cRequestNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"RequestNO");

			my_printf((char*)&stBLE_Charge_Apply_RSP.cAssetNO[1],stBLE_Charge_Apply_RSP.cAssetNO[0],MY_HEX,KPRINTF_ENABLE,TASK_NAME,(char*)(__func__),"AssetNO");
			rt_kprintf("[bluetooth]: (%s) GunNum: %d\n",__func__,stBLE_Charge_Apply_RSP.GunNum);
			rt_kprintf("[bluetooth]: (%s) SucIdle: %d",__func__,stBLE_Charge_Apply_RSP.cSucIdle);
		
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_ChgRequestAck);
			result=0;
		}
		break;
		
		case Cmd_ChgPlanIssueAck:
			stBLE_Charge_Plan_RSP = *((CHARGE_STRATEGY_RSP*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_ChgPlanIssueAck);
			result=0;
		break;
		
		case Cmd_ChgPlanOffer:
			stBLE_Charge_Plan_Event = *((PLAN_OFFER_EVENT*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_ChgPlanOffer);
			result=0;
		break;
		
		case Cmd_ChgPlanExeState:
			stBLE_Charge_Exe_Event = *((CHARGE_EXE_EVENT*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_ChgPlanExeState);
		result=0;
		break;
		
		case Cmd_ChgRequestReport:
			stBLE_Charge_Apply_Event = *((CHARGE_APPLY_EVENT*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_ChgRequestReport);
			result=0;
		break;
		
		case Cmd_RouterExeStateAck://CHARGE_EXE_STATE			stBLE_Charge_State;//���״̬����
			stBLE_Charge_State = *((CHARGE_EXE_STATE*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_RouterExeStateAck);
			result=0;
		break;
		
		case Cmd_ChgRecord:
			stBLE_Charge_Record = *((CHG_ORDER_EVENT*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_ChgRecord);
			result=0;
		break;
		
		case Cmd_DeviceFault:
			stBLE_Router_Fault_Event = *((ROUTER_FAULT_EVENT*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_DeviceFault);
			result=0;
		break;
		
		case Cmd_PileFault:
			stBLE_Router_Fault_Event = *((ROUTER_FAULT_EVENT*)STR_SetPara);
			g_BLE_Get_Strategy_event |= (0x00000001<<Cmd_PileFault);
			result=0;
		break;
		
///////////////////////////////////////////////////////////////////////////////////////////////////			
		
		default:
			result=1;
			break;	
	}
	
	//////////////////////////��� ���͸����Ե�ָ����־/////////////////////////////////////////
	g_BLE_Send_to_Strategy_event &= (rt_uint32_t)(~(0x00000001<<cmd));
	
	if((g_BLE_Send_to_Strategy_event&(0x00000001<<cmd)) == (0x00000001<<cmd))
	{
		rt_kprintf("[bluetooth]: (%s)  %s clear fail\n",__func__,comm_cmdtype_to_string(cmd));
	}
	else
	{
		rt_kprintf("[bluetooth]: (%s)  %s clear sucess\n",__func__,comm_cmdtype_to_string(cmd));
	}
	//////////////////////////////////////////////////////////////////////////////////////////////
	
	return result;			
}

/*************************************************************************************************************/

//rt_uint8_t addr_buf[24] = {0x0C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x31};

//static rt_err_t bluetooth_rx_ind(rt_device_t dev, rt_size_t size)
//{
//	BLE_Uart_Data_Recv(dev,&stBLE_Comm,size);
//  return RT_EOK;
//}

static void bluetooth_thread_entry(void *parabluetooth)
{
	rt_err_t res,result;
	rt_uint8_t i;
	
	bluetooth_serial = rt_device_find(RT_BLUETOOTH_USART);
	
	if(bluetooth_serial != RT_NULL)
	{
		if (rt_device_open(bluetooth_serial, RT_DEVICE_FLAG_DMA_RX) == RT_EOK)
		{
			rt_kprintf("[bluetooth]: (%s) Open serial uart3 sucess!\r\n",__func__);
		}
	}
	else
	{
		res = RT_ERROR;
		rt_kprintf("[bluetooth]: (%s) Open serial uart3 err!\r\n",__func__);
		return;
	}
	//��ʼ���¼�����
//	rt_event_init(&bluetooth_event, "uart_rx_event", RT_IPC_FLAG_FIFO);

	/* ���ջص�����*/
//	rt_device_set_rx_indicate(bluetooth_serial, bluetooth_rx_ind);
	
	rt_pin_mode(BLE_PIN, PIN_MODE_OUTPUT);
	BLE_PWR_ON()	//ģ���ϵ�
	
	
	rt_thread_mdelay(3000);
	
	BLE_ATCmd = BLE_ATE;
	BLE_ATCmd_Old = BLE_NULL; 
	g_ucProtocol = AT_MODE;
	
	
	////////////////////////////////������///////////////////////////////
	stBLE_meter_addr.data = 0x01;
	stBLE_meter_addr.data_type=0x09;
	stBLE_meter_addr.addr_len = 0x06;
	stBLE_meter_addr.addr[0] = (((RouterInfo.Addr[1]-0x30)<<4) | (RouterInfo.Addr[2]-0x30));
	stBLE_meter_addr.addr[1] = (((RouterInfo.Addr[3]-0x30)<<4) | (RouterInfo.Addr[4]-0x30));
	stBLE_meter_addr.addr[2] = (((RouterInfo.Addr[5]-0x30)<<4) | (RouterInfo.Addr[6]-0x30));
	stBLE_meter_addr.addr[3] = (((RouterInfo.Addr[7]-0x30)<<4) | (RouterInfo.Addr[8]-0x30));
	stBLE_meter_addr.addr[4] = (((RouterInfo.Addr[9]-0x30)<<4) | (RouterInfo.Addr[10]-0x30));
	stBLE_meter_addr.addr[5] = (((RouterInfo.Addr[11]-0x30)<<4) | (RouterInfo.Addr[12]-0x30));
	stBLE_meter_addr.optional = 0x00;
	stBLE_meter_addr.time = 0x00;
//////////////////////////////////////////////////////////////////////	
	
	Esam_KEY_Sess_State= 0;// �ỰЭ�̱�־  �ỰЭ����� ���ܽ�������ҵ��
	g_ulBLE_Rx_Count = 0;
	g_ulBLE_Tx_Count = 0;
		
	while (1)
	{
		if(BLE_Uart_Data_Recv_and_Process(bluetooth_serial,&stBLE_Comm) == RT_EOK)
		{
			BLE_RecvData_Process(bluetooth_serial,g_ucProtocol,BLE_ATCmd,&stBLE_Comm);
		}
			
		BLE_Get_Strategy_Event();

		if(g_ucProtocol == AT_MODE)
		{
			BLE_ATCmd_Send(bluetooth_serial,BLE_ATCmd);
//			rt_thread_mdelay(1000);
		}
//		else
//			rt_thread_mdelay(50);
		
		BLE_Commit_TimeOut();
//		rt_thread_mdelay(1000);
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
		rt_kprintf("size = %d,send cmd is %s",size,buf);
	}
	rt_free(buf);
}
MSH_CMD_EXPORT(AT_CMD, Send AT CMD);



