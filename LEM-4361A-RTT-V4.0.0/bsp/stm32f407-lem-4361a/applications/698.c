
#include "drv_gpio.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <698.h>
#include <esam.h>
#include <storage.h>
//extern rt_uint8_t GetStorageData(STORAGE_CMD_ENUM cmd,void *STO_GetPara,rt_uint32_t datalen);
//测试数据
int test_dis_check=0;//对接收数据不进行头尾校验的开关

int test_report=0;
int first_time=0;


//esam
CCMRAM ScmEsam_Comm hplc_ScmEsam_Comm;

CCMRAM ESAM_CMD hplc_current_ESAM_CMD;
//other


CCMRAM struct rt_event PowerCtrlEvent;


#define PPPINITFCS16 0xffff // initial FCS value /
#define PPPGOODFCS16 0xf0b8 // Good final FCS value 
#define my_debug 1
//struct rt_event PowerCtrlEvent;
struct rt_event HplcEvent;
#define THREAD_HPLC_PRIORITY     15
#define THREAD_HPLC_STACK_SIZE   1024*8
#define THREAD_HPLC_TIMESLICE    7
//下发命令
CCMRAM struct  _698_FRAME _698_ChgPlanIssue;
unsigned char _698_ChgPlanIssue_data[1024];







CCMRAM CHARGE_STRATEGY charge_strategy_ChgPlanIssue;//充电计划下发

CCMRAM CHARGE_STRATEGY charge_strategy_ChgRecord;   //上送充电订单
CCMRAM CHARGE_STRATEGY_RSP ChgPlanIssue_rsp;


CCMRAM struct  _698_FRAME _698_ChgPlanIssueGet;
CCMRAM unsigned char _698_ChgPlanIssueGet_data[100];


CCMRAM _698_CHARGE_STRATEGY _698_charge_strategy;



CCMRAM ROUTER_FAULT_EVENT _698_router_fail_event;

CCMRAM ROUTER_FAULT_EVENT _698_pile_fail_event;



CCMRAM _698_CHARGE_EXE_STATE _698_charge_exe_state;

CCMRAM struct  _698_FRAME _698_RouterExeState;////路由器执行状态查询Cmd_RouterExeState
CCMRAM unsigned char _698_RouterExeState_data[200];


CCMRAM struct  _698_FRAME _698_ChgPlanAdjust;
CCMRAM unsigned char _698_ChgPlanAdjust_data[1024];
CCMRAM CHARGE_STRATEGY charge_strategy_ChgPlanAdjust;

CCMRAM CHARGE_STRATEGY_RSP ChgPlanAdjust_rsp;

//功率调节
CCMRAM struct  _698_FRAME _698_CTL_CHARGE_Adj;
CCMRAM unsigned char _698_CTL_CHARGE_Adj_data[50];
CCMRAM CTL_CHARGE CTL_CHARGE_Adj;

//表计在线状态事件
CCMRAM struct  _698_FRAME _698_ONLINE_STATE;
CCMRAM unsigned char _698_ONLINE_STATE_data[200];
CCMRAM ONLINE_STATE ONLINE_STATE_hplc;

//启动充电
CCMRAM struct  _698_FRAME _698_StartChg;
CCMRAM unsigned char _698_StartChg_data[50];
CCMRAM CTL_CHARGE CTL_CHARGE_START;

//停止充电
CCMRAM struct  _698_FRAME _698_StopChg;
CCMRAM unsigned char _698_StopChg_data[50];
CCMRAM CTL_CHARGE CTL_CHARGE_STOP;


//上送
/*充电申请事件记录单元*/
CCMRAM CHARGE_APPLY_EVENT hplc_CHARGE_APPLY_EVENT;

CCMRAM PLAN_OFFER_EVENT hplc_PLAN_OFFER_EVENT;

CCMRAM CHG_ORDER_EVENT hplc_CHG_ORDER_EVENT;

//CHARGE_EXE_STATE hplc_CHARGE_EXE_STATE;
CCMRAM CHARGE_EXE_EVENT hplc_CHARGE_EXE_EVENT;


CCMRAM struct _698_STATE hplc_698_state;
int hplc_lock1=0,hplc_lock2=0;
unsigned int strategyEvent[2];
rt_uint32_t hplc_event=0;
rt_uint32_t event_no=1;

rt_uint8_t hplc_stack[THREAD_HPLC_STACK_SIZE];//线程堆栈
rt_device_t hplc_serial; 	// 串口设备句柄 
rt_device_t blue_tooth_serial; 	// 串口设备句柄 
struct rt_thread hplc;

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：	 
*	 函数功能：主函数；等待读取一帧；解析该帧，填充到接收结构体中；
	 设定hplcDataWaitList为了保存等待回应的帧，包括等待需要分针的下一帧，
	 接收用不能超过2个，实际位客户机接收帧最大窗口尺寸；
	 添加有链接超时管理，还未实际用；
	 先接收一帧，进来的都是整帧，需要处理645协议获取电表，发送电表地址（不发
	 这一帧，hplc不会发送698的要标号协议），并置电表发送表号标志位备用；
	 其他情况认为是长时间没有收到数据,大约5秒退出接收程序，到主函数处理其他事
	 事，目前主要处理的是，50秒左右打印一次，证明线程存在；
	 如果正确接收到了一帧，进行帧解析	,给结构体，并且校验	，是直接赋值，不是
	 强制类型转换，赋完值后就可以直接操作privFrameRev;
	 处理函数要区分是由后台还是服务器发来的帧,如果需要需要回复将报文拼接到发
	 送结构的数据缓存中；
*	 
*	
*/
void hplc_thread_entry(void * parameter){
	int result=0,i=0,j=0,time=0,time_past=0,times=0;
	struct CharPointDataManage hplc_data_rev,hplc_data_tx;
	struct CharPointDataManage hplc_data_wait_list;//用不能超过2个，实际位客户机接收帧最大窗口尺寸
	
	rt_kprintf("[hplc]  (%s)  start !!! \n",__func__);
	
	result=rt_event_init(&HplcEvent,"HplcEvent",RT_IPC_FLAG_FIFO);
	if(result!=RT_EOK){		
			rt_kprintf("[hplc]  (%s)  rt_event Hplc faild! \n",__func__);
	}
	rt_thread_mdelay(5000);
	
	hplc_inition(&hplc_data_wait_list,&hplc_data_rev,&hplc_data_tx);
	init_698_state(&hplc_698_state);
	rt_memset(hplc_698_state.last_link_requset_time.data,0,10);	//将时间减去

	while(1){
		
//	if(1){
//		rt_thread_mdelay(2000);	
//		test_dis_check=1;
	
		result=get_single_frame_frome_hplc(&hplc_698_state,&hplc_data_rev,&hplc_data_tx);
		if(result!=0){
			if(result==1){//645协议获取电表
				hplc_645_addr_response(&hplc_698_state,&hplc_data_rev,&hplc_data_tx);				
				hplc_tx_frame(&hplc_698_state,hplc_serial,&hplc_data_tx);
				hplc_698_state.meter_addr_send_ok=1;//重启的时候重新置0
				clear_data(&hplc_698_state,&hplc_data_rev,&hplc_data_rev._698_frame);
			}			
			if(result==2){//退出，是因为受到了事件，不执行clear_data
				//打包发送				
			}	
		}else{//接收正确帧
			if(hplc_priveData_analysis(&hplc_data_rev,&hplc_data_tx)==0){//进行帧解析	,给结构体，并且校验					
				rt_kprintf("[hplc]  (%s)  	RX:\n",__func__);
				printmy(&hplc_data_rev._698_frame);	//测试
				if(judge_meter_no(&hplc_698_state,&hplc_data_rev)==0){
					result=_698_analysis(&hplc_698_state,&hplc_data_tx,&hplc_data_rev,&hplc_data_wait_list);
					
					if( result!=0){//是直接赋值，不是强制类型转换，赋完值后就可以直接操作_698_frame_rev	
						if(result==1){//
							rt_kprintf("[hplc]  (%s)  not need to send and out\n",__func__);//在收到的是应答的时候就不许要去处理了
						}else if(result==2){
							rt_kprintf("[hplc]  (%s)  for charge\n",__func__);
							
						}else{
							rt_kprintf("[hplc]  (%s)  error result=%d\n",__func__,result);//						
						}						
					}else{//下面是需要回复的情况
						rt_kprintf("[hplc]  (%s)  TX:\n",__func__);
						printmy(&hplc_data_tx._698_frame);
						hplc_tx_frame(&hplc_698_state,hplc_serial,&hplc_data_tx);//发送数据
					}					
				}					
			}	
			clear_data(&hplc_698_state,&hplc_data_rev,&hplc_data_rev._698_frame);//处理完了后清理数据	,只有这个是动态的后面的申请了空间后就不变了									
		}	
	
		if(1){
			times++;
			if(times>20){
				times=0;
				rt_kprintf("[hplc]  (%s)  no data for very long time! \n",__func__);				
			}
		}	
	}
}



/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	从hplc中读取一个帧.
	 先查看底层有没有事件上传上来，处理上传上来的上报事件等操作；
	 程序一直在这里读串口缓存中的数据，直到读到一个数据或者超时退出，整体超过5s后就
	 退出；
	 等待接收68开头的帧，收到68后，68以及后面收到数据都存到缓存里面；
	 下一步是接收数据长度,缓存中需要接收的len是接收的数据+2长度的字节，接收到数据长
	 度，置标志位；
	 有了长度后，接着接收，接收到地址的长度后，就知道头的长度，进行头校验；
	 头校验后，一直接收直到接收完，数据全部接收完了，判断最后一位是不是16；
	 接收完一个整帧后，查看底层有没有事件上传上来，处理上传上来的上报事件等操作；
	 是为了方便其他接口可能的调用，如果错误，接着去接收接收数据；

*/
int get_single_frame_frome_hplc(struct _698_STATE  * priv_698_state,struct CharPointDataManage *data_rev,struct CharPointDataManage *data_tx){
	
	unsigned char Res;
	int times=0;      					
	int tempSize;
	int result=0;
	if(1){	
		result=check_afair_from_botom(priv_698_state,data_tx);//查看底层有没有事件上传上来
	}	
	result=init_698_FRAME(&data_rev->_698_frame);		
	while(1){
		tempSize=rt_device_read(hplc_serial, 0, &Res, 1) ;//一直在这里读数据，直到读空，超时退出。

//	tempSize=1;//测试
//	priv_698_state->connect_flag=1;
//	for(int i=0;i<1024;i++){
//		Res=esam_data[i];	
		
		if(tempSize==1){		
			times=0;    //判断超时用 ，读到数就清零			 		
			if(priv_698_state->USART_RX_STA&0x40000000){             //已经接收到了一帧的第一个0x68				
				if(save_hplc_data(data_rev,data_rev->dataSize,Res)<0)//将收到的数据全部接收过来，方便发送		
				{
					rt_kprintf("[hplc]  (%s) get_single_frame save_hplc_data error\n",__func__);
					clear_data(priv_698_state,data_rev,&data_rev->_698_frame); 	//其他原因退出，也用这个函数清理。           													
				}else{					
					if(priv_698_state->USART_RX_STA&0x20000000){    //接收到数据长度,接收len是接收的数据+2长度的字节

						if(!(priv_698_state->USART_RX_STA&0x08000000)){//还未进行头校验
					
							if(priv_698_state->USART_RX_STA&0x10000000){    //sa地址长度，也就知道了头校验的位置，
								if(data_rev->dataSize==(8+priv_698_state->len_sa)){    //表示收完头校验位
									priv_698_state->USART_RX_STA|=0x08000000;        //置标志位	
									////测试							
									if(!test_dis_check){		
										if(_698_HCS(data_rev->priveData,1,7+priv_698_state->len_sa,0)<0){             //校验不过
											result=hplc_645_addr_receive(data_rev);
											if(result==1){
												rt_kprintf("[hplc]  (%s)  is 645!!!!\n",__func__);
												return result;
											}
											clear_data(priv_698_state,data_rev,&data_rev->_698_frame); 	//其他原因退出，也用这个函数清理。    			
											//continue;//最好不用这个,可以不用	
										}	
									}
								}								
							}else{
								if((data_rev->dataSize)==5){    //表示收完第5个数，接收到sa数据长度
									priv_698_state->USART_RX_STA|=0x10000000;        //置标志位
									if((data_rev->priveData[1]==0xaa)&&(data_rev->priveData[2]==0xaa)){
										rt_kprintf("[hplc]  (%s) [1]==0xaa [2]==0xaa \n",__func__);
										priv_698_state->len_sa=4;//电表是12个数
									}else{									
										if(priv_698_state->len_all > HPLC_DATA_MAX_SIZE){
											rt_kprintf("[hplc]  (%s) len_all=%d>HPLC_DATA_MAX_SIZE \n",__func__,priv_698_state->len_all);
											clear_data(priv_698_state,data_rev,&data_rev->_698_frame);
											return -1;								
										}
										priv_698_state->len_sa=(data_rev->priveData[4]&0X0F)+1;//sa地址长度，需要多加一位
//										rt_kprintf("[hplc]  (%s) len_sa=%d\n",__func__,priv_698_state->len_sa);
									}	
								}
							}
						}											
						priv_698_state->len_left--;		       //判断是否接收完了		上面如果清除了就多清除一次。    				
						if(priv_698_state->len_left<=0){     //数据全部接收完了，判断最后一位是不是19，并校验								
							if(data_rev->priveData[(data_rev->dataSize-1)]!=0x16){//结束位不对								
								clear_data(priv_698_state,data_rev,&data_rev->_698_frame); 	//其他原因退出，也用这个函数清理。此处用作打日志											
								rt_kprintf("[hplc]  (%s)   last date!=0x16\n",__func__);
								//可能是645
								return -1;//最好不用这个			
							}else {									
								priv_698_state->USART_RX_STA|=0x04000000;      //接收完成，不再接收其他帧？？			
								rt_kprintf("[hplc]  (%s)  get right frame!!!!! not check use’s business\n",__func__);								
//								if(1){//查看底层有没有事件上传上来	
//									check_afair_from_botom(priv_698_state,data_tx);
//								}									
								return 0;														
							}											
						}												
					}else{                               //等待接收数据长度		
						if((data_rev->dataSize)==3){    //表示收完第三个数，接收到数据长度
							priv_698_state->USART_RX_STA|=0x20000000;        //接收到数据长度，置标志位
						
							priv_698_state->len_all=data_rev->priveData[2]*256+data_rev->priveData[1]+2;//获取len是接收的静荷+后面的一位校验位和一位0x19								
							if(priv_698_state->len_all<12){//最少得这些数
								clear_data(priv_698_state,data_rev,&data_rev->_698_frame);	
								return -1;
							}else{
								priv_698_state->len_left=priv_698_state->len_all-3;						
							}														
//							rt_kprintf("[hplc]  (%s)  len_all=%d \n",__func__,priv_698_state->len_all);													
						}
					}					
				}
			}else{//还未收到第一个68
//				if(Res==0x68&&(priv_698_state->FE_no>=4)){                                    //第一个68
//					rt_kprintf("[hplc]  (%s)    FE_no>=4 \n",__func__);
				if(Res==0x68){ 
					rt_kprintf("[hplc]  (%s)    get 0x68 \n",__func__);
					priv_698_state->FE_no=0;
					if(save_hplc_data(data_rev,data_rev->dataSize,Res)<0)//收到的数据全部接收过来，方便发送		
					{
						clear_data(priv_698_state,data_rev,&data_rev->_698_frame);										
					}else{
						priv_698_state->USART_RX_STA|=0x40000000;                       //收到第一帧中第一个08的标志位
						times=0;                                        //判断超时用 ，此时定时器开启					
					}															
				}	
//				if(Res==0xfe){ //开头连着四个fe
//					priv_698_state->FE_no+=1;
//					times=0;		
//				}else{
//					priv_698_state->FE_no=0;				
//				}				
			}			
		}else{//很长时间没收到数超时退出，处理其他业务
			if(1){//查看底层有没有事件上传上来
				check_afair_from_botom(priv_698_state,data_tx);		
			}			
			rt_thread_mdelay(200);							
			times++;
			if(times==25){//10秒判断超时,全清空
				times=0;
				clear_data(priv_698_state,data_rev,&data_rev->_698_frame);//处理完了后清理数据	,只有这个是动态的后面的申请了空间后就不变了
				return -1;
			}
		}
	}	
}


/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	应答645获取电表的帧
//68 32 00 00 00 00 00 68 93 06 65 33 33 33 33 33 FF 16 //18个
*/

int hplc_645_addr_response(struct _698_STATE  * priv_698_state,struct CharPointDataManage *data_rev,struct CharPointDataManage *data_tx){

	int i;
	data_tx->priveData[0]=0x68;
	my_strcpy(data_tx->priveData+1,priv_698_state->addr.s_addr,0,priv_698_state->addr.s_addr_len);
	data_tx->priveData[7]=0x68;
	data_tx->priveData[8]=0x93;
	data_tx->priveData[9]=0x06;		
	my_strcpy(data_tx->priveData+10,priv_698_state->addr.s_addr,0,priv_698_state->addr.s_addr_len);
	for(i=0;i<6;i++){
		data_tx->priveData[10+i]+=0x33;	
	}
	data_tx->priveData[16]=0;
	for(i=0;i<16;i++){
		//rt_kprintf("\n[hplc]  (%s)  hplc_645_addr_response  priveData[%d]=%02x\n",__func__,i,data_tx->priveData[i]);
		data_tx->priveData[16]+=data_tx->priveData[i];	
	}

	data_tx->priveData[17]=0x16;
	data_tx->dataSize=18;
	return 0;
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	路由器上电
*/
void hplc_PWR_ON(void)	
{					
	rt_pin_mode(GET_PIN(G,4),PIN_MODE_OUTPUT);//power-on,
	rt_pin_write(GET_PIN(G,4),PIN_HIGH);

	rt_pin_mode(GET_PIN(G,3),PIN_MODE_OUTPUT);//	
	rt_pin_write(GET_PIN(G,3),PIN_HIGH);//RESET,低电平有效,只要是低电平就有效,不用置高

	rt_pin_mode(GET_PIN(G,2),PIN_MODE_OUTPUT);//		
	rt_pin_write(GET_PIN(G,2),PIN_HIGH);//SET,低电平有效,没啥反应不知道干啥用的		
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	路由器断电
*/
void hplc_PWR_OFF(void)	
{		
	rt_pin_write(GET_PIN(G,4),PIN_LOW);
	rt_pin_write(GET_PIN(G,3),PIN_LOW);//RESET,低电平有效			
	rt_pin_write(GET_PIN(G,2),PIN_HIGH);//SET,低电平有效		
}//模块掉电

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	初始化
*/
int hplc_inition(struct CharPointDataManage *  data_wait_list,struct CharPointDataManage * data_rev,struct CharPointDataManage * data_tx){

	struct serial_configure hplc_config = RT_SERIAL_CONFIG_DEFAULT; /* 配置参数 */	
	hplc_serial = rt_device_find(HPLC_UART_NAME);//开启相应的串口
 
	if(hplc_serial!=RT_NULL){
		hplc_config.baud_rate = BAUD_RATE_9600;
		hplc_config.data_bits = DATA_BITS_9;
		hplc_config.stop_bits = STOP_BITS_1;
		hplc_config.parity = PARITY_EVEN  ;
		hplc_config.bufsz=1124;
		rt_device_control(hplc_serial, RT_DEVICE_CTRL_CONFIG, &hplc_config);//配置串口参数
		if(rt_device_open(hplc_serial, RT_DEVICE_FLAG_DMA_RX)==0){//以dam发送模式打开串口设备

				rt_kprintf("[hplc]  (%s)   SET SERIAL AS 9600 E 8 1 \n",__func__);//
		}	
	}else{
		rt_kprintf("[hplc]  (%s)   rt_device_find is error \n",__func__);//	
	}	
		
	if(1){
		hplc_PWR_ON();
	}else{
		hplc_PWR_OFF();			
	}
	//整体发送数据的初始化	
	init_CharPointDataManage(data_rev);	
	init_CharPointDataManage(data_tx);
	init_CharPointDataManage(data_wait_list);
	init_698_FRAME(&_698_ChgPlanIssue);	
	_698_ChgPlanIssue.usrData=_698_ChgPlanIssue_data;
	_698_ChgPlanIssue.usrData_size=1024;

		
	init_698_FRAME(&_698_ChgPlanIssueGet);
	_698_ChgPlanIssueGet.usrData=_698_ChgPlanIssueGet_data;
	_698_ChgPlanIssueGet.usrData_size=100;
	
	init_698_FRAME(&_698_RouterExeState);
	_698_RouterExeState.usrData=_698_RouterExeState_data;
	_698_RouterExeState.usrData_size=200;	
	
	
	init_698_FRAME(&_698_ChgPlanAdjust);
	_698_ChgPlanAdjust.usrData=_698_ChgPlanAdjust_data;
	_698_ChgPlanAdjust.usrData_size=1024;

	init_698_FRAME(&_698_CTL_CHARGE_Adj);
	_698_CTL_CHARGE_Adj.usrData=_698_CTL_CHARGE_Adj_data;
	_698_CTL_CHARGE_Adj.usrData_size=50;

	
	
	init_698_FRAME(&_698_ONLINE_STATE);
	_698_CTL_CHARGE_Adj.usrData=_698_ONLINE_STATE_data;
	_698_CTL_CHARGE_Adj.usrData_size=200;



	init_698_FRAME(&_698_StartChg);
	_698_StartChg.usrData=_698_StartChg_data;
	_698_StartChg.usrData_size=50;

	init_698_FRAME(&_698_StopChg);
	_698_StopChg.usrData=_698_StopChg_data;
	_698_StopChg.usrData_size=50;	
	
	return 0;	
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	68 AA AA AA AA AA AA 68 13 00 DF 16 
   实际收到了18个,因为广播用了AA
*/
int hplc_645_addr_receive(struct CharPointDataManage *data_rev){
	if(data_rev->dataSize>=12){
		if(data_rev->priveData[0]==0x68&&(data_rev->priveData[7]==0x68)
			&&(data_rev->priveData[11]==0x16)){
			rt_kprintf("\n[hplc]  (%s)   good struct\n",__func__);//		
		}else{
			return -1;
		}
		if(data_rev->priveData[1]==0xAA&&(data_rev->priveData[8]==0x13)
			&&(data_rev->priveData[10]==0xDF)){
			rt_kprintf("\n[hplc]  (%s)   good orde check\n",__func__);//		
		}else{
			return -1;
		}	
		return 1;	
	}
	return 0;		
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	hplc接口操作，主要进行帧校验,将结构体赋给static struct  _698_FRAME  
			_698_frame_rev，调用响应函数，收到了帧，记录收到的时间
*/
int hplc_priveData_analysis(struct CharPointDataManage * data_rev,struct CharPointDataManage * data_tx){	
	//_698_frame_rev->strategy.cmd_type=0;

	//rt_kprintf("[hplc]  (%s)   hplc_priveData_analysis data_tx.size= %d \n",__func__,data_tx->size);
	get_current_time(data_rev->_698_frame.rev_tx_frame_date_time.data);
	return priveData_analysis(data_rev,data_tx);//将数解析出来	
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	发送完后至少要33位的空闲间隔
*/


int hplc_package(unsigned char * data,int size){
	return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：辅助函数,用于数组扩容,默认增容1024
*/

int clear_data(struct _698_STATE  * priv_698_state,struct CharPointDataManage *data_rev,struct  _698_FRAME  *_698_frame){

//	rt_kprintf("[hplc]  (%s)    \n",__func__);
	priv_698_state->USART_RX_STA=0;                       //收到第一帧中第一个08的标志位
//	times=0;                                        //判断超时用 ,此时定时器开启	
	data_rev->dataSize=0;//这个用于判断0
	rt_memset(data_rev->priveData,0,data_rev->size);
	priv_698_state->len_left=0;
	priv_698_state->len_sa=0;
	priv_698_state->len_all=0;
	priv_698_state->FE_no=0;
	//if((data_rev->size!=0)&&(data_rev->priveData!=RT_NULL)){
	//	rt_memset(data_rev->priveData,0,data_rev->size);		
	//}
	/*if(array_deflate(data_rev->priveData, data_rev->size,0)==0){
		data_rev->size=1024;
		rt_memset(data_rev->priveData,0,data_rev->size);//清空		
	}else{
		rt_memset(data_rev->priveData,0,data_rev->size);
		return -1;
	}*/	
	return 0;	
}	
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：用户来调用这个获取事件strategyEvent[0]
*/
rt_uint32_t strategy_event_get(void){
	rt_uint32_t result=CTRL_NO_EVENT;
//	rt_uint32_t event;
//锁资源
	while(hplc_lock1==1){
		rt_kprintf("[hplc]  (%s)   lock1==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_lock1=1;
	while(hplc_lock2==1){
		rt_kprintf("[hplc]  (%s)   lock2==1  \n",__func__);		
		rt_thread_mdelay(20);
	}	
	hplc_lock2=1;	
	result=strategyEvent[0];
	
//	strategyEvent[0]=CTRL_NO_EVENT;	
	hplc_lock2=0;
	hplc_lock1=0;		
	return result;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：
*/
int save_hplc_data(struct CharPointDataManage *hplc_data,int position,unsigned char Res){
		return save_char_point_data(hplc_data,position,&Res,1);	
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：打印698帧和时间
*/
int printmy(struct  _698_FRAME  *_698_frame){
	int i;
	rt_kprintf("[hplc]  (%s)  ######################################################################\n",__func__);
	
	rt_kprintf("[hplc]  (%s)  time = %0x:%0x:%0x\n",__func__,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);

	if(1){
		rt_kprintf("%0x ",_698_frame->head);	
		rt_kprintf("%0x ",_698_frame->length0);
		rt_kprintf("%0x ",_698_frame->length1);
		rt_kprintf("%0x ",_698_frame->control);		
		rt_kprintf("%0x ",_698_frame->addr.sa);	
	
		for(i=0;i<_698_frame->addr.s_addr_len;i++){
		 		rt_kprintf("%0x ",_698_frame->addr.s_addr[i]);	
		}	
		rt_kprintf("%0x ",_698_frame->addr.ca);
		rt_kprintf("%0x ",_698_frame->HCS0);
		rt_kprintf("%0x ",_698_frame->HCS1);

		for(i=0;i<_698_frame->usrData_len;i++){
			rt_kprintf("%02x ",_698_frame->usrData[i]);	
			
		}
		rt_kprintf("%0x ",_698_frame->FCS0);
		rt_kprintf("%0x ",_698_frame->FCS1);
		rt_kprintf("%0x ",_698_frame->end);
		rt_kprintf("\n");	

	}else{
		rt_kprintf("[hplc]  (%s)  _698_frame->head=    %0x\n",__func__,_698_frame->head);	
		rt_kprintf("[hplc]  (%s)  _698_frame->length0= %0x\n",__func__,_698_frame->length0);
		rt_kprintf("[hplc] (%s)  _698_frame->length1= %0x\n",__func__,_698_frame->length1);
		rt_kprintf("[hplc] (%s)  _698_frame->control= %0x\n",__func__,_698_frame->control);		
		rt_kprintf("[hplc] (%s)  _698_frame->addr.sa=  %0x\n",__func__,_698_frame->addr.sa);	
		//rt_kprintf("[hplc]  (%s)  _698_frame->addr.s_addr_len= %0x\n",__func__,_698_frame->addr.s_addr_len);	
		
		for(i=0;i<_698_frame->addr.s_addr_len;i++){
		 rt_kprintf("[hplc] (%s)  _698_frame->addr->s_addr[%d]= %0x\n",__func__,i,_698_frame->addr.s_addr[i]);	
		}	
		rt_kprintf("[hplc] (%s)  _698_frame->addr.ca=%0x \n",__func__,_698_frame->addr.ca);
		rt_kprintf("[hplc] (%s)  _698_frame->HCS0= %0x\n",__func__,_698_frame->HCS0);
		rt_kprintf("[hplc] (%s)  _698_frame->HCS1= %0x\n",__func__,_698_frame->HCS1);
		
	//	rt_kprintf("[hplc]  (%s)  _698_frame->usrData_len= %d\n",__func__,_698_frame->usrData_len);	
		for(i=0;i<_698_frame->usrData_len;i++){
	//		rt_kprintf("[hplc] (%s)  __698_frame->usrData[%d]= %0x\n",__func__,i,_698_frame->usrData[i]);
			rt_kprintf("%02x ",_698_frame->usrData[i]);	
			
		}
		rt_kprintf("\n");			
		rt_kprintf("[hplc]  (%s)  _698_frame->FCS0= %0x\n",__func__,_698_frame->FCS0);
		rt_kprintf("[hplc]  (%s)  _698_frame->FCS1= %0x\n",__func__,_698_frame->FCS1);
		rt_kprintf("[hplc]  (%s)  _698_frame->end=  %0x\n",__func__,_698_frame->end);
	}	
	return 0;
}


/*
*	 函数名：
*	 函数参数：size,返回组帧之后的帧长度。
*	 函数返回值：
*	 函数功能：对分针进行处理。只处理分帧的，整帧的或者打包完了的，就直接往下走

		分帧应答需要处理的情况：
			//1:服务器发起    来的是分帧      需要接收且应答每个分帧，并组帧     是不是需要超时处理
			//2:服务器发起    发送的是分帧    发送完一帧后需要等待应答          iterate_wait_response_list（）
			//3:客户机发起的  发送的是分帧    需要等待应答，收到应答发送下一帧。 iterate_wait_response_list（）
			//4:客户机发起的  来的是分帧      需要接收且应答每个分帧，并组帧					

		//处理的是来的帧，
		如果是由服务器发起的普通帧，回复即可 return 0


		如果是由服务器发起的分针   看是不是第一帧，和最后一帧，如果是第一帧就直接保存（成功后返回应答 return 0）；
															如果不是第一帧，去查看有没有已经保存的帧，没有返回 -1（超时给清了怎么办），有保存数据，增长长度，返回应答 return 0


		如果是由用户发起的普通帧    判断处理一下，不用回复 return 1

		如果是由用户发起的        当发送时分针（发送第一帧就存到链表），收到应答后，发下一针（长度都是计算的按照上一帧的帧好乘以最大帧长度）  
														 当收到的是分帧是，跟 由服务器发起的分针 一样处理
														 														 
		收到分帧和分帧响应。												 

*/

int iterate_wait_response_list(struct _698_STATE  * priv_698_state,struct CharPointDataManage * data_tx,struct CharPointDataManage * data_rev,struct CharPointDataManage * hplc_data_list){
	int i;
	struct CharPointDataManage *above,*node,*want=RT_NULL;
//	unsigned char *want_affair;
	unsigned char want_affair;
	above=hplc_data_list;//头
	node=hplc_data_list;
	
	if(0x81<=data_rev->_698_frame.usrData[0]<=0x89){
		want_affair=data_rev->_698_frame.usrData[0]-0x80;//还原到发送的帧		
	}else if(data_rev->_698_frame.usrData[0]==security_response){
		want_affair=security_request;				
	}else{
		rt_kprintf("[hplc]  (%s)  no this kind of affair or not support! \n",__func__);
		return -3;				
	}

	if((hplc_data_list->list_no==0)||(node->_698_frame.usrData==RT_NULL)){//方便下面减一操作,空指针会有致命操作，单不会有空的
		rt_kprintf("[hplc]  (%s)   there is no hplc_data_list \n",__func__);
		return -3;
	}
	
	if(!(data_rev->_698_frame.control&CON_MORE_FRAME)){//如果不分帧，就简单的删去那一帧(心跳帧单独处理)，让后往下走，让下面的策略来处理		
		for(i=0;i<hplc_data_list->list_no;i++){//遍历链表
			if(want_affair==node->_698_frame.usrData[0]&&(!node->_698_frame.control&CON_START_MASK)){//判断类型和oad，和是不是应答帧，如果是就发送下一帧。且不删除，直到发送完。
				//判断是不是想要的帧，找列表中的比较旧的找到就退出，重复的需要超时的程序去检索，相同的需要每个发送去删除（加到发送中）。			
				want=node;
				break;
			}else{
				above=node;//保存上一个			
			}
			if(node->next==RT_NULL){//后面的是空就退出	,认为到最后了
				rt_kprintf("[hplc]  (%s)   node->next==RT_NULL \n",__func__);
				break;			
			}else{
				node=node->next;
			}
		}		
		if((hplc_data_list->list_no==1)&&(want!=RT_NULL)){//如果只有一个，并且这个是想要的
			//head->list_no-=1;//按照语境来。
			init_CharPointDataManage(want);			
			return 0;
		}
		
		if(want!=RT_NULL&&(want->next==RT_NULL)){//有想要的，不是第一个，而是最后一个
			above->next=RT_NULL;//前一个指空		
			hplc_data_list->list_no-=1;
			free_CharPointDataManage(want);
			return 0;
		
		}
		
		if(want!=RT_NULL){//有想要的，是中间的
			above->next=want->next;//指向下一个
			hplc_data_list->list_no-=1;
			free_CharPointDataManage(want);
			return 0;	
		}
		return -3;//没有找到合适的帧。
	}else{	//如果是分帧的，需要将数据加长，组成完整帧后完了之后，把数据给data_rev,并按照
		//if(finish){
		//数据给data_rev
		//到下面去处理事务,
		//return 0;
		//}else if （没完）{
		//发送应答帧 
		//return -3;//应答帧就是处理了，下面的不处理了；
		//}	
	}
	return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：函数调用的统一接口；test_dis_check测试用，当不为零时就不进行校验
	 测试时需要先计算 ,校验成功进行下一步
*/
int priveData_analysis(struct CharPointDataManage * data_rev,struct CharPointDataManage * data_tx){
	if(!test_dis_check){
		if(_698_FCS(data_rev->priveData, 1,data_rev->dataSize-2,0)!=0	){
			rt_kprintf("[hplc]  (%s)    _698_FCS if failed \n",__func__);	
			return -1;		
		} 
	}		
	if( _698_unPackage(data_rev->priveData,&data_rev->_698_frame,data_rev->dataSize)!=0){//是直接赋值，不是强制类型转换，赋完值后就可以直接操作_698_frame_rev	
		rt_kprintf("[hplc]  (%s)   _698_unPackage(data,_698_frame_rev,size)!=0\n",__func__);
		return -1;
	}
	return 0;	
}
/*
*	 函数名：
*	 函数参数：size,返回组帧之后的帧长度。
*	 函数返回值：
*	 函数功能：	只处理分帧的，因为不分帧直接就处理了。
*/
int iterate_wait_request_list(struct _698_STATE  * priv_698_state,struct CharPointDataManage * data_tx,struct CharPointDataManage * data_rev,struct CharPointDataManage * hplc_data_list){
	int i;
	struct CharPointDataManage *above,*node,*want=RT_NULL;
	above=hplc_data_list;//头
	node=hplc_data_list;
//  unsigned char * want_affair;
	unsigned char want_affair;
	want_affair=data_rev->_698_frame.usrData[0];	
	if((hplc_data_list->list_no==0)||(node->_698_frame.usrData==RT_NULL)){//方便下面减一操作,空指针会有致命操作，单不会有空的
		rt_kprintf("[hplc]  (%s)   there is no hplc_data_list \n",__func__);
		return -3;
	}	
	if((data_rev->_698_frame.control&CON_MORE_FRAME)){//如果分帧，* want_affair与hplc_data_list->_698_frame.usrData[0]比较		
		for(i=0;i<hplc_data_list->list_no;i++){//遍历链表
			if(want_affair==node->_698_frame.usrData[0]&&(node->_698_frame.control&CON_START_MASK)){//判断类型和oad，和是不是应答帧，如果是就发送下一帧。且不删除，直到发送完。
				//判断是不是想要的帧，找列表中的比较旧的找到就退出，重复的需要超时的程序去检索，相同的需要每个发送去删除（加到发送中）。			
				want=node;
				break;
			}else{
				above=node;//保存上一个			
			}
			if(node->next==RT_NULL){//后面的是空就退出	,认为到最后了
				rt_kprintf("[hplc]  (%s)   node->next==RT_NULL \n",__func__);
				break;			
			}else{
				node=node->next;
			}
		}		
		if((hplc_data_list->list_no==1)&&(want!=RT_NULL)){//如果只有一个，并且这个是想要的：判断收完了么，收完了，初始化 返回0；没收完，返回分帧应答帧，返回 1；收到应答分帧，发送下一针，并返回2，下同
			init_CharPointDataManage(want);			
			return 0;
		}
		
		if(want!=RT_NULL&&(want->next==RT_NULL)){//有想要的，不是第一个，而是最后一个：判断收完了么，收完了，初始化 返回0；没收完，返回分帧应答帧data_tx，返回 1
			above->next=RT_NULL;//前一个指空		
			hplc_data_list->list_no-=1;
			free_CharPointDataManage(want);
			return 0;				
		}
		
		if(want!=RT_NULL){//有想要的，是中间的：判断收完了么，收完了，初始化 返回0；没收完，返回分帧应答帧，返回 1
			above->next=want->next;//指向下一个
			hplc_data_list->list_no-=1;
			free_CharPointDataManage(want);
			return 0;		
		}
		//没有找到合适的帧。就将这一帧加入到新帧中。		
		return 1;		
	}else{	//如果是不分帧的，不处理，只是加一个判断
			
	}
	return 0;
}	

/*
*	 函数名：
*	 函数参数：size,返回组帧之后的帧长度。
*	 函数返回值：result==1，不需要发送
*	 函数功能：	拦截处理分帧
//先判断是从客户机到服务器还是从服务器到客户机，但是对控制器和对电表时是不同的，还得要区分这个？
	由服务器发起的情况(CON_UTS_S)：
		显然是回应，处理应答发送的分帧；处理来的应答，但是分帧发送的（回应答），整合后就可以处理了；
		跟分帧应答整合后是一个过程，往下走；如果want_affair涉及到oad就找到是那种oad，后面用到，这个
		暂时不实现，毕竟服务器主动发送的有限，而且，连接好像也不用实现了；只处理分帧	，和分针的应答；
		只有分帧情况才进去；打包完后让后面的策略来处理；如果组帧成功了，就赋值给data_rev，然后处理；	
		找到那个帧并且删掉,如果来的是分帧就拼帧，并返回应答	；目前这三种情况：收到的不是想要的帧；
		请求帧是分帧，且还没有发完所有帧，收应答帧；应答帧是分帧，而且还没组成整包；如果不是上述情况，
		让_698_del_affairs处理。

	由控制器发起的情况(CON_UTS_U)：
		即客户机发起。来的是分帧的数据，到hplc_data_wait_list，并返回分帧应答；如果满了复制回data_rev，
		删除list正常应答，返回0；分帧的应答也在iterate_wait_request_list处理，发下一分帧分帧的应答也在
		这里处理，发下一分帧；目前这三种情况：收到的是没有的帧，且不是第一帧；应答帧是分帧，且不是最后
		一帧；请求帧是分帧，而且还没组成整包；如果不是上述情况，让_698_del_affairs处理。如果来的不是分
		帧，什么也不做，向下面处理事务	
							
*/

int _698_analysis(struct _698_STATE  * priv_698_state,struct CharPointDataManage * data_tx,struct CharPointDataManage * data_rev,struct CharPointDataManage * hplc_data_wait_list){
	int result=0;
// current_meter_serial=get_meter_serial();
		
	switch ( data_rev->_698_frame.control&CON_DIR_START_MASK){//先打印为了调试，后期可能会用到
		case(CON_UTS_S):
			rt_kprintf("[hplc]  (%s)  UTS_S \n",__func__);						
			result=iterate_wait_response_list(priv_698_state,data_tx,data_rev,hplc_data_wait_list);	
			if(result!=0){
				return result;				
			}
			break;
		case(CON_UTS_U):
			rt_kprintf("[hplc]  (%s)  UTS_U \n",__func__);
			if((data_rev->_698_frame.control&CON_MORE_FRAME)){
				
				rt_kprintf("[hplc]  (%s)   CON_MORE_FRAME  so go on ! \n",__func__);				
				result=iterate_wait_request_list(priv_698_state,data_tx,data_rev,hplc_data_wait_list);
			
				if(result!=0){					return result;				
				}					
			}else{
				rt_kprintf("[hplc]  (%s) UTS_U is not  CON_MORE_FRAME   ! \n",__func__);							
			}			
			break;				
		default:
			rt_kprintf("[hplc]  (%s)   not real UTS \n",__func__);		
			break;
	}
	result=rev_698_del_affairs(priv_698_state,data_tx,data_rev);//实际处理，用户可以调用这个来处理接收的
//	rt_kprintf("[hplc]  (%s)   result %d \n",__func__,result);	
	return result;
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：		
	 注：已有代码就不更改了。						
*/
int copy_698_FactoryVersion(struct _698_FactoryVersion *des,struct _698_FactoryVersion *src){
//	*des=*src;
	my_strcpy(des->manufacturer_code,src->manufacturer_code,0,4);
	my_strcpy(des->soft_version,src->soft_version,0,4);
	my_strcpy(des->soft_date,src->soft_date,0,6);
	my_strcpy(des->hard_version,src->hard_version,0,4);
	my_strcpy(des->hard_date,src->hard_date,0,6);
	my_strcpy(des->manufacturer_ex_info,src->manufacturer_ex_info,0,8);	
	return 0;
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：		
	 信息由客户机发来						
*/

int unPackage_698_connect_request(struct _698_STATE  * priv_698_state,struct  _698_FRAME  * _698_frame,struct _698_connect_response * prive_struct){

	
	rt_kprintf("[hplc]  (%s)    \n",__func__);
	prive_struct->type=link_response;
	prive_struct->piid_acd=_698_frame->usrData[1];
	copy_698_FactoryVersion(&prive_struct->connect_res_fv,&priv_698_state->FactoryVersion);
	my_strcpy(prive_struct->apply_version,_698_frame->usrData,2,2);
	my_strcpy(priv_698_state->version,_698_frame->usrData,2,2);
	
	my_strcpy(prive_struct->connect_res_pro.protocolconformance,_698_frame->usrData,4,8);
	my_strcpy(priv_698_state->protocolconformance,prive_struct->connect_res_pro.protocolconformance,0,8);
		
	my_strcpy(prive_struct->connect_res_func.functionconformance,_698_frame->usrData,12,16);
	my_strcpy(priv_698_state->functionconformance,prive_struct->connect_res_func.functionconformance,0,16);
	
	//不拷贝也没事，每次响应是收到那边的值然后反应
	my_strcpy(prive_struct->max_size_send,_698_frame->usrData,28,2);	
	my_strcpy(prive_struct->max_size_rev,_698_frame->usrData,30,2);
	my_strcpy(&prive_struct->max_size_rev_windown,_698_frame->usrData,32,1);
	my_strcpy(prive_struct->max_size_handle,_698_frame->usrData,33,2);
	my_strcpy(prive_struct->connect_overtime,_698_frame->usrData,35,4);
	prive_struct->connect_res_info.connectresult=0x00;//允许建立应用连接
	prive_struct->connect_res_info.connectresponseinfo_sd.type=_698_frame->usrData[39];//认证附加信息=	认证请求对象
	if(prive_struct->connect_res_info.connectresponseinfo_sd.type!=NullS){
		prive_struct->connect_res_info.connectresponseinfo_sd.type=0;//这个暂时不处理
			rt_kprintf("[hplc]  (%s)  not control  type!=NullS \n",__func__);
	}
	prive_struct->FollowReport=0;//OPTIONAL=0表示没有上报信息
	if(prive_struct->FollowReport!=0){
					rt_kprintf("[hplc]  (%s)  not control  FollowReport!=0 \n",__func__);
	}else{
	
	}	
	if(_698_frame->usrData[39]==0){
		prive_struct->time_tag=_698_frame->usrData[39];	
	}else{//不处理
			rt_kprintf("[hplc]  (%s)  not control  time_tag!=0 \n",__func__);		
	}		
	return 0;
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：		
	 将结构体赋给static struct _698_link_request hplc_698_link_request;
	 调用响应函数						
*/

int unPackage_698_link_request(struct  _698_FRAME  *_698_frame,struct _698_link_request * request,int * size){
	request->type=_698_frame->usrData[0];
	request->piid_acd=_698_frame->usrData[1];
	request->work_type=_698_frame->usrData[2];
	_698_frame->strategy.heart_beat_time0=request->heartbeat_time[0]=_698_frame->usrData[3];
	_698_frame->strategy.heart_beat_time1=request->heartbeat_time[1]=_698_frame->usrData[4];
	request->date_time.year[0]=_698_frame->usrData[5];
	request->date_time.year[1]=_698_frame->usrData[6];
	request->date_time.month=_698_frame->usrData[7];
	request->date_time.day=_698_frame->usrData[8];
	request->date_time.week=_698_frame->usrData[9];
	request->date_time.hour=_698_frame->usrData[10];
	request->date_time.minute=_698_frame->usrData[11];
	request->date_time.second=_698_frame->usrData[12];
	request->date_time.millisconds[0]=_698_frame->usrData[13];
	request->date_time.millisconds[1]=_698_frame->usrData[14];
	
	request->position=15;
	my_strcpy(request->date_time.data,_698_frame->usrData,5,10);	
	return 0;	
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：		
	 其他场景用结构体赋值：* des=* source					
*/
int copy_698_frame(struct  _698_FRAME * des,struct  _698_FRAME  * source){
	
	des->head=source->head;//起始帧头 = 0x68	
	
	des->length0=source->length0;//hplc_data->size<1024时
	des->length1=source->length1;		
	des->control=source->control;   //控制域c,bit7,传输方向位
	des->addr.sa=source->addr.sa;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度

	//拷贝主机地址
	des->addr.s_addr_len=source->addr.s_addr_len;
	my_strcpy(des->addr.s_addr,source->addr.s_addr,0,source->addr.s_addr_len);//拷贝数组

	des->addr.ca=source->addr.ca;

//校验头
	des->HCS0=source->HCS0;	
	des->HCS1=source->HCS1;

	des->usrData_len=source->usrData_len;//用户数据长度归零

	//充电计划单不能太长
	
	save_char_point_usrdata(des->usrData,&des->usrData_size,source->usrData,0,source->usrData_len);	
	
	rt_kprintf("[hplc]  (%s)  need length=%d  the total size = %d \n",__func__,source->usrData_len,des->usrData_size);

	des->FCS0=source->FCS0;
	des->FCS1=source->FCS1;
		
	des->end=source->end;/**/
	return 0;
}
/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：		
	 其他场景用结构体赋值：* des=* source					
*/
int copy_char_point_data(struct CharPointDataManage * des,struct CharPointDataManage * source){

	des->_698_frame.head=source->_698_frame.head;//起始帧头 = 0x68		
	des->_698_frame.length0=source->_698_frame.length0;//hplc_data->size<1024时
	des->_698_frame.length1=source->_698_frame.length1;		
	des->_698_frame.control=source->_698_frame.control;   //控制域c,bit7,传输方向位
	des->_698_frame.addr.sa=source->_698_frame.addr.sa;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度

	//拷贝主机地址
	des->_698_frame.addr.s_addr_len=source->_698_frame.addr.s_addr_len;
	//if(des->addr.s_addr!=RT_NULL){//先释放	
	//	rt_free(des->addr.s_addr);//不判断错误，顶多多吃点内存		
	//}
	//des->addr.s_addr=(unsigned char *)rt_malloc(sizeof(unsigned char)*(source->addr.s_addr_len));//分配空间		
	my_strcpy(des->_698_frame.addr.s_addr,source->_698_frame.addr.s_addr,0,source->_698_frame.addr.s_addr_len);//拷贝数组

	des->_698_frame.addr.ca=source->_698_frame.addr.ca;
//校验头
	des->_698_frame.HCS0=source->_698_frame.HCS0;	
	des->_698_frame.HCS1=source->_698_frame.HCS1;
	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	des->_698_frame.usrData_len=source->_698_frame.usrData_len;//用户数据长度归零
	                             	
//	if(des->usrData!=RT_NULL){//先释放
//		rt_kprintf("[hplc]  (%s)   des->usrData!=RT_NULL\n",__func__);
	//	rt_free(des->usrData);
//	}else{
//		rt_kprintf("[hplc]  (%s)   des->usrData==RT_NULL\n",__func__);	
	
//	}
	//des->usrData=(unsigned char *)rt_malloc(source->usrData_len);//给用户分配空间	
	//des->usrData_size=source->usrData_len;//空间大小	
	des->_698_frame.usrData=des->priveData+(8+des->_698_frame.usrData_len);
	save_char_point_usrdata(des->_698_frame.usrData,&des->_698_frame.usrData_size,source->_698_frame.usrData,0,source->_698_frame.usrData_len);		
	des->_698_frame.FCS0=source->_698_frame.FCS0;
	des->_698_frame.FCS1=source->_698_frame.FCS1;
		
	des->_698_frame.end=source->_698_frame.end;/**/
	return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：发了上报等函数等待应答。		 				
*/

int copy_to_work_wait_list(struct CharPointDataManage *hplc_data,struct CharPointDataManage * hplc_data_wait_list){
	int i;
	struct CharPointDataManage * new_struct,*head,*node,*end;
	head=hplc_data_wait_list;//头	
	if(hplc_data_wait_list->list_no!=0){//如果第一个没有
		new_struct=(struct CharPointDataManage *)rt_malloc(sizeof(struct CharPointDataManage));//开辟新空间
		if(new_struct==RT_NULL){//先释放
			rt_kprintf("[hplc]  (%s)  new_struct==RT_NULL \n",__func__);
		}else{	
			init_CharPointDataManage(new_struct);//指针赋空值
		}
		//传给下一个
		node=hplc_data_wait_list;//指向第一个
		
		for(i=0;i<hplc_data_wait_list->list_no;i++){//遍历到最后一个
			if(node->next==RT_NULL){//这里也可以是认为找到了最后一个，这个错误是致命错误，空指针错误太严重，不能退出				
				if(hplc_data_wait_list->list_no!=i+1){//没有到最后一个就是个空指针，
					rt_kprintf("[hplc]  (%s)   opy_to_work_wait_list_no is not right\n",__func__);
					hplc_data_wait_list->list_no=i+1;//如果数量不对，这里来调整个数，后面的不要了,顶多第一个也是个空的。可以在任务里面删去				
				}
				break;			
			}else{//判断是不是重复发送的帧，如果是就覆盖，主要覆盖时间,也可以由重发超时不回复帧的功能来实现。
				//return 0;			
			}
			node=node->next;
		}		
		node->next=new_struct;
		end=new_struct;//end指向最后一个				
	}else{//如果第一个还没有赋值，就将new_struct指向第一个。
		rt_kprintf("[hplc]  (%s)   new_struct=hplc_data_wait_list\n",__func__);
		end=hplc_data_wait_list;//第一个也是最后一个
	}
	end->next=RT_NULL;//指向第一个	
	my_strcpy(end->priveData,hplc_data->priveData,0,hplc_data->dataSize);//拷贝数组	
	copy_char_point_data(end,hplc_data);
	head->list_no+=1;//按照语境来。
	return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：登录帧打包返回可用的data_tx。		
	 				
*/
int link_request_package(struct CharPointDataManage *hplc_data,struct _698_STATE  * priv_698_state){
	int result=0;
	
	struct _698_link_request user_date_struct;
	struct _698_date_time current_time;
	short past_time;
	unsigned char temp_char;
	//结构体赋值，共同部分

	get_current_time(current_time.data);
	past_time=(current_time.data[5]-priv_698_state->last_link_requset_time.data[5])*60*60  
						+(current_time.data[6]-priv_698_state->last_link_requset_time.data[6])*60
						+(current_time.data[7]-priv_698_state->last_link_requset_time.data[7]);
	
	if(past_time >= (priv_698_state->heart_beat_time0*256+priv_698_state->heart_beat_time1-5)){//如果接近心跳超时帧，发送心跳帧,这个大小位是顺序的
		
		rt_kprintf("[hplc]  (%s)   past_time >= overtime sent link_request= %d   \n",__func__,priv_698_state->try_link_type);
		result=0;
	
	}else{
		rt_kprintf("[hplc]  (%s)   past_time <= overtime sent link_request= %d   \n",__func__,priv_698_state->try_link_type);
		return result;
	}
	
	hplc_data->dataSize=0;	
	temp_char=hplc_data->_698_frame.head = 0x68;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	int len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的长度	
	
	temp_char=hplc_data->_698_frame.control=CON_STU_S|CON_LINK_MANAGE;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa ;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//拷贝服务器地址
	hplc_data->_698_frame.addr.s_addr_len=priv_698_state->addr.s_addr_len;
//	if(hplc_data->_698_frame.addr.s_addr!=RT_NULL){//先释放
//		rt_free(hplc_data->_698_frame.addr.s_addr);//不判断错误，顶多多吃点内存
//	}
//	hplc_data->_698_frame.addr.s_addr=(unsigned char *)rt_malloc(sizeof(unsigned char)*(hplc_data->_698_frame.addr.s_addr_len));//分配空间	
	
	my_strcpy(hplc_data->_698_frame.addr.s_addr,priv_698_state->addr.s_addr,0,hplc_data->_698_frame.addr.s_addr_len);//拷贝数组
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);

	temp_char=hplc_data->_698_frame.addr.ca=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位	
	

	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=(hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len));	                              
	
//	if(hplc_data->_698_frame.usrData!=RT_NULL){//先释放	,不是空的就不管了
//		rt_kprintf("[hplc]  (%s)   hplc_data->_698_frame.usrData!=RT_NULL \n",__func__,__func__);
		//rt_free(hplc_data->_698_frame.usrData);

//	}
	//hplc_data->_698_frame.usrData=(unsigned char *)rt_malloc(sizeof(unsigned char)*(1024));//给用户分配空间	
	//hplc_data->_698_frame.usrData_size=1024;//空间大小		
	
	temp_char=user_date_struct.type=link_request;//加用户结构体，只是为了不遗漏，和方便获取数据
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	if(priv_698_state->link_flag==0){//还没有登录，打包登录帧
		temp_char=user_date_struct.piid_acd=0x00;//连接的默认优先级
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
		temp_char=user_date_struct.work_type=link_request;
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
	  priv_698_state->try_link_type=link_request_load;
		get_current_time(priv_698_state->last_link_requset_time.data);//给心跳帧的起始时间赋值	
		
	}else if (priv_698_state->connect_flag==1){//已经登录且链结成功了判断是不是需要发送心跳	
		temp_char=user_date_struct.piid_acd=0x01;//连接的默认优先级
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
		
		temp_char=user_date_struct.work_type=link_request_heart_beat;
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		priv_698_state->try_link_type=link_request_heart_beat;	
		get_current_time(priv_698_state->last_link_requset_time.data);//给心跳帧的起始时间赋值							
		
	}	else{
		return -1;
	}	
	temp_char=user_date_struct.heartbeat_time[0]=priv_698_state->heart_beat_time0;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=user_date_struct.heartbeat_time[1]=priv_698_state->heart_beat_time1;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	
	get_current_time(user_date_struct.date_time.data);//给心跳帧的起始时间赋值		
	result=save_char_point_data(hplc_data,hplc_data->dataSize,user_date_struct.date_time.data,10);	
	
	hplc_data->_698_frame.usrData_len=(hplc_data->dataSize-HCS_position-2);//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
	//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);			
	
	int FCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验
		
	temp_char=hplc_data->_698_frame.end=0x16;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

//给长度结构体赋值,这里判断是不是需要分针
	hplc_data->priveData[len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

	hplc_data->priveData[len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

//校验头
	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(hplc_data->priveData, HCS_position);
	hplc_data->_698_frame.HCS0=hplc_data->priveData[HCS_position];	
	hplc_data->_698_frame.HCS1=hplc_data->priveData[HCS_position+1];

	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(hplc_data->priveData, FCS_position);
	
	hplc_data->_698_frame.FCS0=hplc_data->priveData[FCS_position];
	hplc_data->_698_frame.FCS1=hplc_data->priveData[FCS_position+1];		

	return result;//不发送
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：返回可用的data_tx。		
	 				
*/

int connect_request_package(struct CharPointDataManage *hplc_data,struct _698_STATE  * priv_698_state){
	int result;
	struct _698_connect_request user_date_struct;
	unsigned char temp_char;
	//结构体赋值，共同部分
	rt_kprintf("[hplc]  (%s) \n",__func__);

	hplc_data->dataSize=0;	
	temp_char=hplc_data->_698_frame.head = 0x68;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好

	int len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的长度
	
	temp_char=hplc_data->_698_frame.control=CON_UTS_U|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

	
	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa ;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	//拷贝主机地址
	hplc_data->_698_frame.addr.s_addr_len=priv_698_state->addr.s_addr_len;
	//if(hplc_data->_698_frame.addr.s_addr==RT_NULL){//先释放
	//	rt_free(hplc_data->_698_frame.addr.s_addr);//不判断错误，顶多多吃点内存
	//	hplc_data->_698_frame.addr.s_addr=(unsigned char *)rt_malloc(sizeof(unsigned char)*(hplc_data->_698_frame.addr.s_addr_len));//分配空间	
	//}	
	my_strcpy(hplc_data->_698_frame.addr.s_addr,priv_698_state->addr.s_addr,0,hplc_data->_698_frame.addr.s_addr_len);//拷贝数组
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);


	temp_char=hplc_data->_698_frame.addr.ca=priv_698_state->addr.ca;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位



	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);		                               
	
//	if(hplc_data->_698_frame.usrData==RT_NULL){//先释放	,不是空的就不管了
//		rt_kprintf("[hplc]  (%s)     hplc_data->_698_frame.usrData==RT_NULL \n",__func__);
		//rt_free(hplc_data->_698_frame.usrData);
	//	hplc_data->_698_frame.usrData=(unsigned char *)rt_malloc(sizeof(unsigned char)*(1024));//给用户分配空间	
	//	hplc_data->_698_frame.usrData_size=1024;//空间大小	
//	}

	
	temp_char=user_date_struct.type=connect_request;//加用户结构体，只是为了不遗漏，和方便获取数据
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	
	temp_char=user_date_struct.piid=priv_698_state->piid;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//获取协议版本号，在hpcl的状态结构体中，似乎是自定义的

	//my_strcpy(user_date_struct.version,priv_698_state->version,0,2);//10个数的请求时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->version,2);//这样打包好么

	//my_strcpy(user_date_struct.connect_req_pro.protocolconformance,priv_698_state->protocolconformance,0,8);//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->protocolconformance,8);
	
	//my_strcpy(user_date_struct.connect_req_func.functionconformance,priv_698_state->functionconformance,0,16);//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->functionconformance,16);
	
	
	temp_char=user_date_struct.max_size_send[0]=0x04;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=user_date_struct.max_size_send[1]=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=user_date_struct.max_size_rev[0]=0x04;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=user_date_struct.max_size_rev[1]=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=user_date_struct.max_size_rev_windown=0x01;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
	
	temp_char=user_date_struct.max_size_handle[0]=0x04;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=user_date_struct.max_size_handle[1]=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	//my_strcpy(user_date_struct.connect_overtime,priv_698_state->connect_overtime,0,4);//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->connect_overtime,4);
	
	temp_char=user_date_struct.connect_req_cmi.NullSecurity=NullS;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//	
	

	temp_char=0x00;//没有时间标签
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//	



	hplc_data->_698_frame.usrData_len=hplc_data->dataSize-HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
	//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

	int FCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验

		
	temp_char=hplc_data->_698_frame.end=0x16;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么



//给长度结构体赋值,这里判断是不是需要分针
	hplc_data->priveData[len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

	hplc_data->priveData[len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	


//校验头
	//rt_kprintf("[hplc]  (%s)  calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(hplc_data->priveData, HCS_position);
	hplc_data->_698_frame.HCS0=hplc_data->priveData[HCS_position];	
	hplc_data->_698_frame.HCS1=hplc_data->priveData[HCS_position+1];


	//rt_kprintf("[hplc]  (%s)  calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(hplc_data->priveData, FCS_position);
	
	hplc_data->_698_frame.FCS0=hplc_data->priveData[FCS_position];
	hplc_data->_698_frame.FCS1=hplc_data->priveData[FCS_position+1];
	
	return result;
	

}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：返回可用的data_tx。		
	 				
*/

int connect_response_del(struct  _698_FRAME  *_698_frame_rev ,struct _698_FRAME  * _698_frame_send,struct _698_STATE  * priv_698_state,struct CharPointDataManage * data_tx){
	//struct _698_connect_response connect_response;
  //save_FactoryVersion(priv_698_state->FactoryVersion,_698_frame_rev->usrData,2)
  //保存厂家版本信息 
//	priv_698_state->connect_flag=1;//连接成功
	return 0 ;
}	

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	omd传递值
	 				
*/

int omd_package(struct _698_omd *priv_struct,struct  _698_FRAME  *_698_frame_rev,int position){
	int result=0;//不发送返回1,如果发送，result=0;

	rt_kprintf("[hplc]  (%s)   \n",__func__);
	priv_struct->oi[0]=_698_frame_rev->usrData[position];	
	priv_struct->oi[1]=_698_frame_rev->usrData[position+1];
	priv_struct->method_id=_698_frame_rev->usrData[position+2];	
	priv_struct->op_mode=_698_frame_rev->usrData[position+3];
	result=0;

	return result;	
	
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	oad传递值
	 				
*/

int oad_package(struct _698_oad *priv_struct,struct  _698_FRAME  *_698_frame_rev,int position){
	int result=0;//不发送返回1,如果发送，result=0;

	rt_kprintf("[hplc]  (%s)   \n",__func__);
	priv_struct->oi[0]=_698_frame_rev->usrData[position];	
	priv_struct->oi[1]=_698_frame_rev->usrData[position+1];
	priv_struct->attribute_id=_698_frame_rev->usrData[position+2];	
	priv_struct->attribute_index=_698_frame_rev->usrData[position+3];
	result=0;

	return result;	
	
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	get时返回打包数据的头两个字节。	 				
*/

int get_data_class(struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data,enum Data_T data_type){

	unsigned char temp_char,result=0;
	temp_char=0x01;//get_response时，返回的数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=data_type;//数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	return result;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	从接口获取，获取后再赋值给date_time_s。	 				
*/
int get_date_time_s(struct _698_date_time_s *date_time_s){
	
		STR_SYSTEM_TIME time;
	
		memcpy(&time,&System_Time_STR,sizeof(STR_SYSTEM_TIME));
//STR_SYSTEM_TIME_to_date_time_s(&priv_struct->StartTimestamp,&date_time_s);
		date_time_s->data[0]=20;//20年
		date_time_s->data[1]=(time.Year>>4)*10+time.Year&0x0f;//年	
		date_time_s->data[2]=(time.Month>>4)*10+time.Year&0x0f;//月
		date_time_s->data[3]=(time.Day>>4)*10+time.Day&0x0f;//日	
		date_time_s->hour=date_time_s->data[4]=(time.Hour>>4)*10+time.Hour&0x0f;//时
		date_time_s->minute=date_time_s->data[5]=(time.Minute>>4)*10+time.Minute&0x0f;//分	
		date_time_s->second=date_time_s->data[6]=(time.Second>>4)*10+time.Second&0x0f;//秒	

		return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	获取实时时间并打包。	 				
*/
int oi_parameter_get_time(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	struct _698_date_time_s date_time_s;
	int result=0;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	
	get_date_time_s(&date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,date_time_s.data,7);			
	return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	获取地址并打包。	 				
*/
int oi_parameter_get_addr(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){

	unsigned char temp_char;
	int result=0;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=priv_698_state->addr.s_addr_len;//地址长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);//保存地址

	return result;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	打印CHARGE_STRATEGY的结构体，用于数据结构解析时调用。	 				
*/

int print_charge_strategy(CHARGE_STRATEGY * charge_strategy){
	int i=0,count=0;
	
	count=charge_strategy->cRequestNO[0]+1;
	rt_kprintf("[hplc]  (%s)  scharge_strategy->cRequestNO \n",__func__);			
	for(i=0;i<count;i++){
		rt_kprintf("%x ",charge_strategy->cRequestNO[i]);
	
	}	
	rt_kprintf("\n");
	
	count=charge_strategy->cUserID[0]+1;
	rt_kprintf("[hplc]  (%s)  charge_strategy->cUserID \n",__func__);			
	for(i=0;i<count;i++){
		rt_kprintf("%x ",charge_strategy->cUserID[i]);
	
	}	
	rt_kprintf("\n");
	
	rt_kprintf("[hplc]  (%s)  charge_strategy->ucDecMaker \n",__func__);			
	rt_kprintf("%x ",charge_strategy->ucDecMaker);
	rt_kprintf("\n");
	
	rt_kprintf("[hplc]  (%s)  charge_strategy->ucDecType \n",__func__);			
	rt_kprintf("%x ",charge_strategy->ucDecType);
	rt_kprintf("\n");
	
	rt_kprintf("[hplc]  (%s)  charge_strategy->strDecTime \n",__func__);			
	rt_kprintf("%2x ",charge_strategy->strDecTime.Year);
	rt_kprintf("%2x ",charge_strategy->strDecTime.Month);
	rt_kprintf("%2x ",charge_strategy->strDecTime.Day);
	rt_kprintf("%2x ",charge_strategy->strDecTime.Hour);
	rt_kprintf("%2x ",charge_strategy->strDecTime.Minute);
	rt_kprintf("%2x ",charge_strategy->strDecTime.Second);	
	rt_kprintf("\n");
		
	count=charge_strategy->cAssetNO[0]+1;
	rt_kprintf("[hplc]  (%s)  charge_strategy->cAssetNO \n",__func__);			
	for(i=0;i<count;i++){
		rt_kprintf("%x ",charge_strategy->cAssetNO[i]);
	
	}	
	rt_kprintf("\n");	
	
	rt_kprintf("[hplc]  (%s)  charge_strategy->GunNum \n",__func__);			
	rt_kprintf("%x ",charge_strategy->GunNum);
	rt_kprintf("\n");
	
	rt_kprintf("[hplc]  (%s)  charge_strategy->ulChargeReqEle \n",__func__);			
	rt_kprintf("%8x ",charge_strategy->ulChargeReqEle);
	rt_kprintf("\n");
		
	rt_kprintf("[hplc]  (%s)  charge_strategy->ulChargeRatePow \n",__func__);			
	rt_kprintf("%8x ",charge_strategy->ulChargeRatePow);
	rt_kprintf("\n");
	
	rt_kprintf("[hplc]  (%s)  charge_strategy->ucChargeMode \n",__func__);			
	rt_kprintf("%x ",charge_strategy->ucChargeMode);
	rt_kprintf("\n");
	
	count=charge_strategy->ucTimeSlotNum;
	for(int j=0;j<count;j++){
		rt_kprintf("[hplc]  (%s)  charge_strategy->strChargeTimeSolts[%d].strDecStartTime \n",__func__,j);			
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStartTime.Year);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStartTime.Month);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStartTime.Day);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStartTime.Hour);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStartTime.Minute);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStartTime.Second);	
		rt_kprintf("\n");
		
		rt_kprintf("[hplc]  (%s)  charge_strategy->strChargeTimeSolts[%d].strDecStopTime \n",__func__,j);			
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStopTime.Year);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStopTime.Month);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStopTime.Day);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStopTime.Hour);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStopTime.Minute);
		rt_kprintf("%2x ",charge_strategy->strChargeTimeSolts[j].strDecStopTime.Second);	
		rt_kprintf("\n");
		

		rt_kprintf("[hplc]  (%s)  charge_strategy->strChargeTimeSolts[%d].ulChargePow \n",__func__,j);				
		rt_kprintf("%x ",charge_strategy->strChargeTimeSolts[j].ulChargePow);
		rt_kprintf("\n");		

	}	
	
	
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	保存下发的启动和停止充电参数。	
	 路由器资产编号  visible-string（SIZE(22)）	和枪序号

*/


int action_response_charge_StartStop(CTL_CHARGE * priv_struct ,struct  _698_FRAME  *_698_frame_rev){
	int i=0,j=0,count,len=0,position;
	rt_kprintf("[hplc]  (%s) \n",__func__);
	i=_698_frame_rev->time_flag_positon;//指向了这个数,注意这个数
	i++;//—— 结构体位置
	if(_698_frame_rev->usrData[++i]!=2){//—— 成员数量位置
		rt_kprintf("[hplc]  (%s)  struct no. is not right i=%d!! \n",__func__,i);				
	}
	
	i+=2;//跳过上面的一个成员数量，一个类型
	len=(_698_frame_rev->usrData[i]+1);//将长度也存起来
	if(len>sizeof(priv_struct->cAssetNO)){
		rt_kprintf("[hplc]  (%s) len=%d>sizeof(priv_struct->cAssetNO)=%d  \n",__func__,len,sizeof(priv_struct->cAssetNO));						
		return -1;	
	}		
	my_strcpy_char(priv_struct->cAssetNO,(char *)_698_frame_rev->usrData,i,len);	
	
	i+=len+1;//跳过上面的16位，一个类型
	len=1;//
	priv_struct->GunNum=_698_frame_rev->usrData[i];
	return 0;	
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	保存下发的功率调节参数

*/

int action_response_power_adjust(CTL_CHARGE * CTL_CHARGE_Adj ,struct  _698_FRAME  *_698_frame_rev){
	int i=0,j=0,count,len=0,position;
	rt_kprintf("[hplc]  (%s) \n",__func__);
	i=6;//指向了这个数,注意这个数
	i++;//
	i++;
	if(_698_frame_rev->usrData[i]!=3){//—— 成员数量位置
		rt_kprintf("[hplc]  (%s)  struct no. is not right _698_frame_rev->usrData[%d]=%x!! \n",__func__,i,_698_frame_rev->usrData[i]);				

	}
	//路由器资产编号 visible-stringg（SIZE(22)）
	i+=2;//跳过上面的一个成员数量，一个类型
	len=_698_frame_rev->usrData[i]+1;//
	if(len>sizeof(CTL_CHARGE_Adj->cAssetNO)){
		rt_kprintf("[hplc]  (%s) len=%d>sizeof(charge_strategy->cRequestNO)=%d  \n",__func__,len,sizeof(CTL_CHARGE_Adj->cAssetNO));						
		return -1;	
	}
	
	my_strcpy_char(CTL_CHARGE_Adj->cAssetNO,(char *)_698_frame_rev->usrData,i,len);	

	//枪序号
	i+=len+1;//跳过上面的位，一个类型
	//my_strcpy(&charge_strategy->ucDecMaker,_698_frame_rev->usrData,i,1);
	CTL_CHARGE_Adj->GunNum=_698_frame_rev->usrData[i];
	
	//有功功率设定
	i+=1+1;//跳过上面的位，一个类型,
	unsigned_char_to_int(&CTL_CHARGE_Adj->SetPower,_698_frame_rev->usrData+i);	
	
}




/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	下发充电计划单时，给结构体赋值

*/

int report_response_strategy(ONLINE_STATE * online_state,struct  _698_FRAME  *_698_frame_rev){
	int i=0,j=0,count,len=0,position;
	rt_kprintf("[hplc]  (%s) \n",__func__);
	i=7;//指向了这个数,注意这个数,多一个
	i++;//rcsd 长度
	i++;//csd  type
	
	i++;//oad
	i+=4;//csd  type
	i++;//oad
	
	i+=4;//成员数量位置
	if(_698_frame_rev->usrData[i]!=6){//—— 成员数量位置
		rt_kprintf("[hplc]  (%s)  struct no. is not right _698_frame_rev->usrData[%d]=%x!! \n",__func__,i,_698_frame_rev->usrData[i]);				
		return -1;
	}
	i++;//oad数组
	i+=6*4;//result record type
	i+=13;//记录序号
	if(_698_frame_rev->usrData[i]!=6){//—— 成员数量位置
		rt_kprintf("[hplc]  (%s)  struct ->OrderNum. is not right _698_frame_rev->usrData[%d]=%x!! \n",__func__,i,_698_frame_rev->usrData[i]);				
		return -1;
	}	
	online_state->OrderNum=0;	
	for(j=0;j<4;j++){
		online_state->OrderNum=online_state->OrderNum<<4;
		online_state->OrderNum=_698_frame_rev->usrData[++i];
	}
	
	i++;//上线时间的类型位置
	i++;//上线时间
	date_time_s_to_STR_SYSTEM_TIME(&online_state->OnlineTimestamp,(_698_frame_rev->usrData+i));
	i+=7;//上线时间的类型位置
	i++;//上线时间	
	date_time_s_to_STR_SYSTEM_TIME(&online_state->OfflineTimestamp,(_698_frame_rev->usrData+i));	
	i+=7;//空oad(没数)的位置	
	i+=12;//oad（33 17 02 06）的位置，取数	
	i+=2;//的类型位置	
	i+=3;//AutualState的类型位置
	i++;//AutualState的值
	online_state->OfflineIfo.AutualState=_698_frame_rev->usrData[i];
	if(_698_frame_rev->usrData[i]==1){
		hplc_698_state.connect_flag=0;
	}
	
	
	
	i++;//OfflinePeriod的类型位置
	online_state->OfflineIfo.OfflinePeriod=0;	
	for(j=0;j<4;j++){
		online_state->OfflineIfo.OfflinePeriod=online_state->OrderNum<<4;
		online_state->OfflineIfo.OfflinePeriod=_698_frame_rev->usrData[++i];
	}
	
	i++;//OfflineReason的类型位置
	i++;//OfflineReason的值
	online_state->OfflineIfo.OfflineReason=_698_frame_rev->usrData[i];	
	
	i++;//DeviceType的类型位置
	i++;//DeviceType的值
	online_state->OfflineIfo.DeviceType=_698_frame_rev->usrData[i];	

	return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	下发充电计划单时，给结构体赋值

*/

int action_response_charge_strategy(CHARGE_STRATEGY * charge_strategy,struct  _698_FRAME  *_698_frame_rev){
	int i=0,j=0,count,len=0,position;
	rt_kprintf("[hplc]  (%s) \n",__func__);
	i=6;//指向了这个数,注意这个数
	i++;//—— 结构体位置
	i++;
	if(_698_frame_rev->usrData[i]!=11){//—— 成员数量位置
		rt_kprintf("[hplc]  (%s)  struct no. is not right _698_frame_rev->usrData[%d]=%x!! \n",__func__,i,_698_frame_rev->usrData[i]);				
	}
	//充电申请单号 octet-string（SIZE(16)）
	i+=2;//跳过上面的一个成员数量，一个类型
	len=_698_frame_rev->usrData[i]+1;//
	if(len>sizeof(charge_strategy->cRequestNO)){
		rt_kprintf("[hplc]  (%s) len=%d>sizeof(charge_strategy->cRequestNO)=%d  \n",__func__,len,sizeof(charge_strategy->cRequestNO));						
		return -1;	
	}
	my_strcpy_char(charge_strategy->cRequestNO,(char *)_698_frame_rev->usrData,i,len);	
	
	//用户ID visible-string（SIZE(64)）
	i+=len+1;//跳过上面的16位，一个类型
	len=_698_frame_rev->usrData[i]+1;//
	if(len>sizeof(charge_strategy->cUserID)){
		rt_kprintf("[hplc]  (%s) len>sizeof(charge_strategy->cUserID)  \n",__func__);						
		return -1;	
	}	
	my_strcpy_char(charge_strategy->cUserID,(char *)_698_frame_rev->usrData,i,len);
	
	//决策者  {主站（1）、控制器（2）}
	i+=len+1;//跳过上面的位，一个类型
	//my_strcpy(&charge_strategy->ucDecMaker,_698_frame_rev->usrData,i,1);
	charge_strategy->ucDecMaker=_698_frame_rev->usrData[i];
	
	//决策类型{生成（1） 、调整（2）}
	i+=2;//跳过上面的位，一个类型
	charge_strategy->ucDecType=_698_frame_rev->usrData[i];
	
	//决策时间
	i+=2;//跳过上面的位，一个类型,
	date_time_s_to_STR_SYSTEM_TIME(&charge_strategy->strDecTime,(_698_frame_rev->usrData+i));

	
	//路由器资产编号  visible-string（SIZE(22)）	
	i+=7+1;//跳过上面的位，一个类型,
	len=_698_frame_rev->usrData[i]+1;
	if(len>sizeof(charge_strategy->cAssetNO)){
		rt_kprintf("[hplc]  (%s) len>sizeof(charge_strategy->cAssetNO)  \n",__func__);						
		return -1;	
	}	
	my_strcpy_char(charge_strategy->cAssetNO,(char *)_698_frame_rev->usrData,i,len);


	//枪序号	enum{A枪（1）、B枪（2）}
	i+=len+1;//跳过上面的位，一个类型,
	charge_strategy->GunNum=_698_frame_rev->usrData[i];

	//充电需求电量（单位：kWh，换算：-2）double-long-unsigned
	i+=1+1;//跳过上面的位，一个类型,
	unsigned_char_to_int(&charge_strategy->ulChargeReqEle,_698_frame_rev->usrData+i);	

	//充电额定功率  double-long（单位：kW，换算：-4），//不用判断负数原样转发就可以了
	i+=4+1;//跳过上面的位，一个类型,
	unsigned_char_to_int(&charge_strategy->ulChargeRatePow,_698_frame_rev->usrData+i);

	//充电模式      enum{正常（0），有序（1）}
	i+=4+1;//跳过上面的位，一个类型,
	charge_strategy->ucChargeMode=_698_frame_rev->usrData[i];

	//充电时段  array时段充电功率		
	i+=2;//跳过上面的位，一个类型,
	count=_698_frame_rev->usrData[i];//个数
	charge_strategy->ucTimeSlotNum=count;
	i+=3;//跳过上面的位,再跳过结构体类型,一个数量，
	for(j=0;j<count;j++){
		//开始时间    date_time_s
		i+=1;//跳过一个类型
		date_time_s_to_STR_SYSTEM_TIME(&charge_strategy->strChargeTimeSolts[j].strDecStartTime,_698_frame_rev->usrData+i);
		//结束时间    date_time_s，
		i+=7+1;//跳过上面的位，一个类型,
		date_time_s_to_STR_SYSTEM_TIME(&charge_strategy->strChargeTimeSolts[j].strDecStopTime,_698_frame_rev->usrData+i);
		//充电功率    double-long（单位：kW，换算：-4）
		i+=7+1;//跳过上面的位，一个类型,
		unsigned_char_to_int(&charge_strategy->strChargeTimeSolts[j].ulChargePow,_698_frame_rev->usrData+i);
		i+=4;//跳过上面的位
	}
//	print_charge_strategy(charge_strategy);	
	return 0;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	打包计划失败事件，还未完成


*/
int plan_fail_event_package(PLAN_FAIL_EVENT *priv_struct,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;//,*temp_array;
//	CHARGE_TIMESOLT *priv_struct_TIMESOLT;
	struct _698_date_time_s date_time_s;
	rt_kprintf("[hplc]  (%s) \n",__func__);
	
	temp_char=Data_structure;//结构体
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=7;//结构体成员数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

//	Value=priv_struct->OrderNum;
	//事件记录序号 
	temp_char=Data_double_long_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=((priv_struct->OrderNum&0xff000000)>>24);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((priv_struct->OrderNum&0x00ff0000)>>16);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((priv_struct->OrderNum&0x0000ff00)>>8);//超了是不是溢出，不用与也可以？
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=((priv_struct->OrderNum&0x000000ff));//超了是不是溢出，不用与也可以？
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	//  事件发生时间 
	temp_char=Data_date_time_s;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	STR_SYSTEM_TIME_to_date_time_s(&priv_struct->StartTimestamp,&date_time_s);
	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,date_time_s.data,7);	

	// 事件结束时间
	temp_char=Data_date_time_s;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	STR_SYSTEM_TIME_to_date_time_s(&priv_struct->FinishTimestamp,&date_time_s);
	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,date_time_s.data,7);	

	//事件发生源    enum，
	temp_char=Data_enum;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
//	temp_char=priv_struct->Reason;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);




	
	return result;
}


/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	路由器工作状态  即 充电计划单执行状态


*/

int charge_exe_state_package(CHARGE_EXE_STATE *priv_struct,struct CharPointDataManage * hplc_data){
	int i=0,j=0,len=0;
	int result=0;
	
	unsigned char temp_char,*temp_array;
	CHARGE_TIMESOLT *priv_struct_TIMESOLT;
	struct _698_date_time_s date_time_s;
rt_kprintf("[hplc]  (%s) \n",__func__);	
	temp_char=Data_structure;//结构体
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=14;//结构体成员数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	


  len=temp_char=priv_struct->cRequestNO[0];//充电申请单号   octet-string（SIZE(16)）
	if(len>(sizeof(priv_struct->cRequestNO)-1)){
		rt_kprintf("[hplc]  (%s) len> array size cRequestNO \n",__func__);
		return -1;
	}	
	temp_array=( unsigned char *) (priv_struct->cRequestNO+1);
	_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);
	
	
	//路由器资产编号   visible-string（SIZE(22)）

  len=temp_char=priv_struct->cAssetNO[0];
	if(len>(sizeof(priv_struct->cAssetNO)-1)){
		rt_kprintf("[hplc]  (%s) len> array size cAssetNO\n",__func__);
		return -1;
	}	
	temp_array=( unsigned char *) (priv_struct->cAssetNO+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);	
	

	temp_char=Data_enum;//枪序号	enum{A枪（1）、B枪（2）}，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_struct->GunNum; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	

	temp_char=Data_enum;//执行状态	enum{正常执行（1）、执行结束（2）、执行失败（3）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_struct->exeState; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	
	temp_char=Data_array;//电能示值底值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_struct->ulEleBottomValue[j];		
		_698_double_long((unsigned int) len, hplc_data);		
	}
	
	
	temp_char=Data_array;//当前电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_struct->ulEleActualValue[j];
		_698_double_long((unsigned int) len, hplc_data);		
	}
	
	
	temp_char=Data_array;//已充电量	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_struct->ucChargeEle[j];
		if(j>sizeof(priv_struct->ucChargeEle)){
			rt_kprintf("[hplc]  (%s) len> array size priv_struct->ucChargeEle\n",__func__);
			return -1;
		}	
		_698_double_long((unsigned int) len, hplc_data);		
	}	
	

	//已充时间	double-long-unsigned单位：秒

  len=priv_struct->ucChargeTime;
  _698_double_long_unsigned((unsigned int) len, hplc_data);	
	
	
	//计划充电功率	double-long（单位：W，换算：-1）

  len=priv_struct->ucPlanPower;
  _698_double_long((unsigned int) len, hplc_data);	
	
	//当前充电功率	double-long（单位：W，换算：-1）

  len=priv_struct->ucActualPower;
  _698_double_long((unsigned int) len, hplc_data);
	
	//电压	arraylong-unsigned，（单位：V，换算：-1）
	temp_char=Data_array;//当前电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=1;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<1;j++){//单相或三相

		len=priv_struct->ucVoltage.A;
		_698_double_long((unsigned int) len, hplc_data);		
	}	
	
	
	//电流	arraydouble-long，（单位：A 换算：-3）
	temp_char=Data_array;//当前电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=1;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<1;j++){

		len=priv_struct->ucCurrent.A;
		_698_double_long((unsigned int) len, hplc_data);		
	}	
	
	//充电桩状态	enum{待机（1）、工作（2）、故障（3）}
	temp_char=Data_enum;//执行状态	enum{正常执行（1）、执行结束（2）、执行失败（3）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_struct->ChgPileState; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	
	//用户ID visible-string（SIZE(64)）//王云光说加上了

  len=temp_char=priv_struct->cUserID[0];
	//这个的零返回
	if(len>(sizeof(priv_struct->cUserID)-1)){
		rt_kprintf("[hplc]  (%s) len> array size cRequestNO\n",__func__);
		return -1;
	}	
	temp_array=( unsigned char *) (priv_struct->cUserID+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);		
	
	return result;
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	充电计划单

*/

int charge_strategy_package(CHARGE_STRATEGY *priv_struct,struct CharPointDataManage * hplc_data){
	int i=0,j=0,len=0;
	int result=0;
	unsigned char temp_char,*temp_array;
	CHARGE_TIMESOLT *priv_struct_TIMESOLT;
	struct _698_date_time_s date_time_s;
	rt_kprintf("[hplc]  (%s) \n",__func__);	
	
	temp_char=Data_structure;//结构体
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=10;//结构体成员数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//充电申请单号 octet-string（SIZE(16)）

  len=temp_char=priv_struct->cRequestNO[0];//充电申请单号   octet-string（SIZE(16)）
	if(len>(sizeof(priv_struct->cRequestNO)-1)){
		rt_kprintf("[hplc]  (%s) len> array size cRequestNO\n",__func__);
		return -1;
	}	
	temp_array=( unsigned char *) (priv_struct->cRequestNO+1);
	_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);	
	
	//用户ID visible-string（SIZE(64)）

  len=temp_char=priv_struct->cUserID[0];
	if(len>(sizeof(priv_struct->cUserID)-1)){
		rt_kprintf("[hplc]  (%s) len> array size cRequestNO\n",__func__);
		return -1;
	}	
	temp_array=( unsigned char *) (priv_struct->cUserID+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);	

	
	
	//决策者  {主站（1）、控制器（2）}
	temp_char=Data_enum;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=priv_struct->ucDecMaker;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	//决策类型{生成（1） 、调整（2）}
	temp_char=Data_enum;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=priv_struct->ucDecType;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	//决策时间
	temp_char=Data_date_time_s;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	STR_SYSTEM_TIME_to_date_time_s(&priv_struct->strDecTime,&date_time_s);
	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,date_time_s.data,7);			

	//路由器资产编号   visible-string（SIZE(22)）

  len=temp_char=priv_struct->cAssetNO[0];
	if(len>(sizeof(priv_struct->cAssetNO)-1)){
		rt_kprintf("[hplc]  (%s) len> array size cAssetNO \n",__func__);
		return -1;
	}		
	temp_array=( unsigned char *) (priv_struct->cAssetNO+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);	


	//充电需求电量（单位：kWh，换算：-2）double-long-unsigned
	temp_char=Data_double_long_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=((priv_struct->ulChargeReqEle&0xff000000)>>24);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((priv_struct->ulChargeReqEle&0x00ff0000)>>16);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((priv_struct->ulChargeReqEle&0x0000ff00)>>8);//超了是不是溢出，不用与也可以？
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=((priv_struct->ulChargeReqEle&0x000000ff));//超了是不是溢出，不用与也可以？
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	
	//充电额定功率  double-long（单位：kW，换算：-4），//不用判断负数原样转发就可以了	
	temp_char=Data_double_long;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
		
	temp_char=((priv_struct->ulChargeRatePow&0xff000000)>>24);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((priv_struct->ulChargeRatePow&0x00ff0000)>>16);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((priv_struct->ulChargeRatePow&0x0000ff00)>>8);//超了是不是溢出，不用与也可以？
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=((priv_struct->ulChargeRatePow&0x000000ff));//超了是不是溢出，不用与也可以？
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
	
	//充电模式      enum{正常（0），有序（1）}
	temp_char=Data_enum;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=priv_struct->ucDecType;//数组数量，由上传者决定默认是一
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
			
	//充电时段  array时段充电功率
	temp_char=Data_array;//数组类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=priv_struct->ucTimeSlotNum;//数组数量//测试
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	if(priv_struct->ucTimeSlotNum==0){
		return 0;
	}
	
	for(j=0;j<priv_struct->ucTimeSlotNum;j++){
		priv_struct_TIMESOLT=(CHARGE_TIMESOLT *)priv_struct->strChargeTimeSolts+i;	
		temp_char=Data_structure;//结构体
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

		temp_char=3;//结构体成员数
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		//开始时间    date_time_s
		temp_char=Data_date_time_s;//
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

		STR_SYSTEM_TIME_to_date_time_s(&priv_struct_TIMESOLT->strDecStartTime,&date_time_s);
		
		result=save_char_point_data(hplc_data,hplc_data->dataSize,date_time_s.data,7);					
		//结束时间    date_time_s，
		temp_char=Data_date_time_s;//
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
		STR_SYSTEM_TIME_to_date_time_s(&priv_struct_TIMESOLT->strDecStopTime,&date_time_s);
		
		result=save_char_point_data(hplc_data,hplc_data->dataSize,date_time_s.data,7);		

		//充电功率    double-long（单位：kW，换算：-4）
//		Value=priv_struct_TIMESOLT->ulChargePow;
		temp_char=Data_double_long;//
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
			
		temp_char=((priv_struct_TIMESOLT->ulChargePow&0xff000000)>>24);//超了是不是溢出
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

		temp_char=((priv_struct_TIMESOLT->ulChargePow&0x00ff0000)>>16);//超了是不是溢出
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

		temp_char=((priv_struct_TIMESOLT->ulChargePow&0x0000ff00)>>8);//超了是不是溢出，不用与也可以？
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

		temp_char=((priv_struct_TIMESOLT->ulChargePow&0x000000ff));//超了是不是溢出，不用与也可以？
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	}	
	return result;
}


/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	充电计划单

*/

int oi_esam_info_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0;
	unsigned char temp_char;
	CHARGE_STRATEGY *priv_struct_STRATEGY;
	rt_kprintf("[hplc]  (%s)   \n",__func__);
	
	hplc_ScmEsam_Comm.DataTx_len=0;
	hplc_current_ESAM_CMD=RD_INFO_FF;
	ESAM_Communicattion(hplc_current_ESAM_CMD,&hplc_ScmEsam_Comm);
	
	temp_char=(hplc_ScmEsam_Comm.DataRx_len-1);//数组数量，由上传者决定默认是一
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	if(hplc_ScmEsam_Comm.DataRx_len<1){
	rt_kprintf("[hplc]  (%s)   no data received ! \n",__func__);		
		return -1;//没读出数据
	}else{		
		result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+4),(hplc_ScmEsam_Comm.DataRx_len-1));
	}
		return result;		
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	路由器执行状态应答

*/
	
int oi_router_exe_state_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0;
	unsigned char temp_char;
	CHARGE_EXE_STATE *priv_CHARGE_EXE_STATE;
	
	temp_char=_698_charge_exe_state.array_size;//数组数量，由上传者决定默认是一
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	if(_698_charge_exe_state.array_size==0){
		return 0;
	}
	rt_kprintf("[hplc]  (%s)  array_size=%0x  \n",__func__,_698_charge_exe_state.array_size);
	for(i=0;i<_698_charge_exe_state.array_size;i++){
		priv_CHARGE_EXE_STATE=(CHARGE_EXE_STATE *)(_698_charge_exe_state.charge_exe_state+i);
//		charge_strategy_package(priv_struct_STRATEGY,hplc_data);
		result=charge_exe_state_package(priv_CHARGE_EXE_STATE,hplc_data);	
	}
	if(result == 0)	{//继续打包		
	}	
	return result;		
}


/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：	

*/

int oi_charge_strategy_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0;
	unsigned char temp_char;
	CHARGE_STRATEGY *priv_struct_STRATEGY;
	
	temp_char=_698_charge_strategy.array_size;//数组数量，由上传者决定默认是一
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	if(_698_charge_strategy.array_size==0){
		return 0;
	}
	rt_kprintf("[hplc]  (%s)  \n",__func__);	
	for(i=0;i<_698_charge_strategy.array_size;i++){
		priv_struct_STRATEGY=(CHARGE_STRATEGY *)(_698_charge_strategy.charge_strategy+i);
		charge_strategy_package(priv_struct_STRATEGY,hplc_data);	
	}
	if(result == 0)	{//继续打包
		
	}	
	return result;		
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：正向有功电能
	 测试用，为改成正常的

*/
int oi_electrical_pap(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0;
	unsigned char temp_char,temp_array[20]={0,0,8,4,0,0,2,1,0,0,1,1,0,0,3,1,0,0,2,1};
	temp_char=(priv_698_state->oad_omd.oi[1]&OI2_MASK)>>4;
	switch (temp_char){//判别了两位，还要判断属性或者方法，很少，用if在里面判断
		case(0)://合相
			rt_kprintf("[hplc]  (%s)  conjunction    \n",__func__);
			//判断属性,只处理属性2
			if(priv_698_state->oad_omd.attribute_id==2){//

				result=get_data_class(priv_698_state,hplc_data,Data_array);

				//result=get_rap(temp_array);
				temp_char=5;//5组数据
				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
				for(i=0;i<20;i++){
					if(i%4==0){//总尖峰平谷，反向电压有尖峰平谷么？
						temp_char=Data_double_long_unsigned;//数字类型
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
					}					
					temp_char=temp_array[i];//
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);				
				}
				result =0;				
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==1  \n",__func__);
				return -1;
			}			
			break;
		case(1)://A 相
			result=-1;
			rt_kprintf("[hplc]  (%s)  a phase   \n",__func__);
			break;
		case(2)://B 相
			result=-1;
			rt_kprintf("[hplc]  (%s)  b phase  \n",__func__);		
			break;			
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  only supply communication addr \n",__func__);
			break;		
	}	
	if(result == 0)	{//继续打包
		
	}	
	return result;		
}

/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：反向有功电能
	 测试用，为改成正常的

*/
int oi_electrical_rap(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0;
	unsigned char temp_char,temp_array[20]={8,0,0,4,2,0,0,1,2,0,0,1,2,0,0,1,2,0,0,1};
	temp_char=(priv_698_state->oad_omd.oi[1]&OI2_MASK)>>4;
	switch (temp_char){//判别了两位，还要判断属性或者方法，很少，用if在里面判断
		case(0)://合相
			rt_kprintf("[hplc]  (%s)  conjunction    \n",__func__);
			//判断属性,只处理属性2
			if(priv_698_state->oad_omd.attribute_id==2){//

				result=get_data_class(priv_698_state,hplc_data,Data_array);

				//result=get_rap(temp_array);
				temp_char=5;//5组数据
				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
				for(i=0;i<20;i++){
					if(i%4==0){//总尖峰平谷，反向电压有尖峰平谷么？
						temp_char=Data_double_long_unsigned;//数字类型
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
					}					
					temp_char=temp_array[i];//
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);				
				}			
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==1  \n",__func__);
				return -1;
			}			
			break;
		case(1)://A 相
			result=-1;
			rt_kprintf("[hplc]  (%s)  a phase   \n",__func__);
			break;
		case(2)://B 相
			result=-1;
			rt_kprintf("[hplc]  (%s)  b phase  \n",__func__);		
			break;			
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  only supply communication addr \n",__func__);
			break;		
	}	
	if(result == 0)	{//继续打包
		
	}	
	return result;		
}


/*
*	 函数名：
*	 函数参数：
*	 函数返回值：
*	 函数功能：反向有功电能
	 测试用，为改成正常的

*/
int oi_variable_oib_meterage(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0,j;
	unsigned char temp_char;
	ScmMeter_Analog prv_ScmMeter_Analog;
	switch (priv_698_state->oad_omd.oi[1]){//判别了两位，还要判断属性或者方法，很少，用if在里面判断
		case(0x00)://电压
			rt_kprintf("[hplc]  (%s)  voltage    \n",__func__);
			//判断属性,只处理属性2
			if(priv_698_state->oad_omd.attribute_id==2){//

				result=get_data_class(priv_698_state,hplc_data,Data_array);


				temp_char=1;//a相电压
				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
				
				cmMeter_get_data(EMMETER_ANALOG,&prv_ScmMeter_Analog);	


				_698_long_unsigned((unsigned int)prv_ScmMeter_Analog.ulVol,hplc_data);				
//				temp_char=Data_long_unsigned;//数字类型
//				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);					

				rt_kprintf("[hplc]  (%s)  prv_ScmMeter_Analog.ulVol=%d    \n",__func__,prv_ScmMeter_Analog.ulVol);

//				temp_char=((prv_ScmMeter_Analog.ulVol&(0x0000ff00))>>8);//
//				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);				

//				temp_char=(prv_ScmMeter_Analog.ulVol&(0x0000ff));//
//				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}			
			break;
		case(0x01)://电流
			rt_kprintf("[hplc]  (%s)  current    \n",__func__);
			//判断属性,只处理属性2
			if(priv_698_state->oad_omd.attribute_id==2){//

				get_data_class(priv_698_state,hplc_data,Data_array);


				temp_char=1;//a相电流
				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
				
				cmMeter_get_data( EMMETER_ANALOG,&prv_ScmMeter_Analog);	


				_698_double_long((unsigned int)prv_ScmMeter_Analog.ulCur,hplc_data);				
//				temp_char=Data_double_long;//数字类型
//				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//				for(i=3;i>=0;i--){
//										
//					temp_char=((prv_ScmMeter_Analog.ulCur&(0xff000000>>((3-(i%4))*8)))>>((i%4)*8));//
//					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);				
//				}				
	
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}			
			break;
		case(0x04)://有功功率

			rt_kprintf("[hplc]  (%s)  active  power    \n",__func__);
			//判断属性,只处理属性2
			if(priv_698_state->oad_omd.attribute_id==2){//

				result=get_data_class(priv_698_state,hplc_data,Data_array);


				temp_char=2;
				result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
				
				cmMeter_get_data( EMMETER_ANALOG,&prv_ScmMeter_Analog);	
				_698_double_long((unsigned int)prv_ScmMeter_Analog.ulAcPwr,hplc_data);						
//				for(i=7;i>=0;i--){
//					if((i+1)%4==0){//总尖峰平谷，反向电压有尖峰平谷么？
//						temp_char=Data_double_long;//数字类型
//						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
//					}	
//							
//					temp_char=((prv_ScmMeter_Analog.ulAcPwr&(0xff000000>>((3-(i%4))*8)))>>((i%4)*8));//
//					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);				
//				}
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				result=-1;
			}			
			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  only supply  \n",__func__);
			break;		
	}	
	if(result == 0)	{//继续打包
		
	}	
	return result;		

}
/*

oi_parameter,//参变量类对象


*/

int oi_electrical_oib_sum(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	temp_char=(priv_698_state->oad_omd.oi[1]&OI1_MASK)>>4;
	switch (temp_char){//判别了两位，还要判断属性或者方法，很少，用if在里面判断
		case(0)://组合有功
			result=-1;
			rt_kprintf("[hplc]  (%s)   Combined  Active Power \n",__func__);			
			break;
		case(1)://正向有功
			rt_kprintf("[hplc]  (%s)   pasitive  Active Power \n",__func__);
			result=oi_electrical_pap(_698_frame_rev,priv_698_state,hplc_data);		
			break;
		case(2)://反向有功
			rt_kprintf("[hplc]  (%s)  Reverse Active Power    \n",__func__);
			result=oi_electrical_rap(_698_frame_rev,priv_698_state,hplc_data);			
			break;			
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  only supply communication addr \n",__func__);
			break;		
	}	
	if(result == 0)	{//继续打包		
	}	
	return result;		
}

/****
默认是成功，有错误直接return -1返回


***/
int oi_report_response_oib(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,len;
	unsigned char temp_char,*temp_array;

	rt_kprintf("[hplc]  (%s)   \n",__func__);
	switch (priv_698_state->oad_omd.oi[1]){//判别了两位，还要判断属性或者方法，很少，用if在里面判断

			
		case(0x12):
			//判断属性,处理方法
			if(priv_698_state->oad_omd.attribute_id==0x03){//下发充电计划
				if(_698_frame_rev->need_package==1){
					_698_frame_rev->need_package=0;
					rt_kprintf("[hplc]  (%s)   .need_package==1   \n",__func__);
					
				}else{

					if(_698_frame_rev->usrData_len>sizeof(_698_ONLINE_STATE_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len> array size  \n",__func__);
						return -1;					
					}

					_698_ONLINE_STATE=*_698_frame_rev;
					my_strcpy(_698_ONLINE_STATE_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_ONLINE_STATE.usrData=_698_ONLINE_STATE_data;						
					
					_698_ONLINE_STATE.time_flag_positon=_698_frame_rev->usrData_len;//最后一位，只给方法用时有效
		
					//保存启动功率
					//action_response_charge_strategy(&charge_strategy_ChgPlanIssue,_698_frame_rev);//存到里面等周调用
					report_response_strategy(&ONLINE_STATE_hplc,_698_frame_rev);
					strategy_event_send(Cmd_STAOnlineState);
//					CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,&ChgPlanIssue_rsp,1);//测试				
					
					return 2;//发送事件	
				}

				//发送信号让下面把充电计划单准备好
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}				
			break;		
							
					
		default:

			rt_kprintf("[hplc]  (%s)  only supply operation=%0x \n",__func__,priv_698_state->oad_omd.oi[1]);
			result=-1;
			break;		
	}	
	if(result == 0)	{//继续打包	
	}
	
	return result;		
	

}
/****
默认是成功，有错误直接return -1返回


***/
int oi_action_response_charge_oib(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,len;
	unsigned char temp_char,*temp_array;

	rt_kprintf("[hplc]  (%s)   \n",__func__);
	switch (priv_698_state->oad_omd.oi[1]){//判别了两位，还要判断属性或者方法，很少，用if在里面判断

//		case(0)://日期时间
//			rt_kprintf("[hplc]  (%s)  not the operation     \n",__func__);
//			result=-1;		
//			break;
		
		
		
		case(0x01):
			//判断属性,处理方法
			if(priv_698_state->oad_omd.attribute_id==0x7f){//下发充电计划
				if(_698_ChgPlanIssue.need_package==1){
					_698_ChgPlanIssue.need_package=0;
					rt_kprintf("[hplc]  (%s)   .need_package==1   \n",__func__);

					temp_char=ChgPlanIssue_rsp.cSucIdle;//DAR， 成功 （ 0），硬件失效 （ 1），其他 （255）					
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
					
					if((0<ChgPlanIssue_rsp.cRequestNO[0]<=sizeof(ChgPlanIssue_rsp.cRequestNO))&&
						(0<ChgPlanIssue_rsp.cAssetNO[0]<=sizeof(ChgPlanIssue_rsp.cAssetNO))){//DAR， 成功 （ 0）
						
						
						temp_char=01;// OPTIONAL=0 表示没有数据
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
						
						temp_char=Data_structure;//
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
						
						temp_char=3;//成员数量
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);							

						
						//充电申请单号 octet-string（SIZE(16)）				

						len=temp_char=(unsigned char )ChgPlanIssue_rsp.cRequestNO[0];//数量	
						if(len>sizeof(ChgPlanIssue_rsp.cRequestNO)){
							rt_kprintf("[hplc]  (%s) len> array size cRequestNO \n",__func__);
							return -1;
						}							
						temp_array=(unsigned char *)(ChgPlanIssue_rsp.cRequestNO+1);
						_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);


						//路由器资产编号  visible-string（SIZE(22)）

						len=temp_char=(unsigned char )ChgPlanIssue_rsp.cAssetNO[0];//数组数量
						if(len>sizeof(ChgPlanIssue_rsp.cAssetNO)){
							rt_kprintf("[hplc]  (%s) len> array size  ChgPlanIssue_rsp.cAssetNO \n",__func__);
							return -1;
						}	//changed next								
						temp_array=(unsigned char *)(ChgPlanIssue_rsp.cAssetNO+1);
						_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);					

						temp_char=Data_enum;//
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
						
						temp_char=ChgPlanIssue_rsp.GunNum;//枪号 需要改的地方
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);									
					}else{
						rt_kprintf("[hplc]  (%s) the data getted is not right!! \n",__func__);					
						temp_char=0;// OPTIONAL=0 表示没有数据
						result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);									
					}	
					
				}else{

//					if(1){//再处理一下用户上传的业务。
//						check_afair_from_botom(priv_698_state,hplc_data);
//					}
					
					if(_698_frame_rev->usrData_len>sizeof(_698_ChgPlanIssue_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len> array size  \n",__func__);
						return -1;					
					}

					_698_ChgPlanIssue=*_698_frame_rev;
					my_strcpy(_698_ChgPlanIssue_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_ChgPlanIssue.usrData=_698_ChgPlanIssue_data;						
					
					_698_ChgPlanIssue.time_flag_positon=_698_frame_rev->usrData_len;//最后一位，只给方法用时有效
		
					//保存启动功率
					action_response_charge_strategy(&charge_strategy_ChgPlanIssue,_698_frame_rev);//存到里面等周调用
					strategy_event_send(Cmd_ChgPlanIssue);
//					CtrlUnit_RecResp(Cmd_ChgPlanIssueAck,&ChgPlanIssue_rsp,1);//测试				
					
					return 2;//发送事件	
				}

				//发送信号让下面把充电计划单准备好
			}else if(priv_698_state->oad_omd.attribute_id==128){//变更充电计划
				if(_698_ChgPlanAdjust.need_package==1){
					_698_ChgPlanAdjust.need_package=0;
					
					temp_char=ChgPlanAdjust_rsp.cSucIdle;//DAR， 成功 （ 0），硬件失效 （ 1），其他 （255）
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
		
					temp_char=01;// OPTIONAL=0 表示没有数据
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
					
					temp_char=Data_structure;//00 —— Data OPTIONAL=0 表示没有数据
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
					
					temp_char=2;//成员数量
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);							
								
					//充电申请单号 octet-string（SIZE(16)）

					len=temp_char=ChgPlanAdjust_rsp.cRequestNO[0];//充电申请单号   octet-string（SIZE(16)）
					if(len>sizeof(ChgPlanAdjust_rsp.cRequestNO)){
						rt_kprintf("[hplc]  (%s) len> array size  ChgPlanAdjust_rsp.cRequestNO\n",__func__);
						return -1;
					}			
					temp_array=( unsigned char *) (ChgPlanAdjust_rsp.cRequestNO+1);
					_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);
	

				//路由器资产编号   visible-string（SIZE(22)）

					len=temp_char=ChgPlanAdjust_rsp.cAssetNO[0];
					if(len>sizeof(ChgPlanAdjust_rsp.cAssetNO)){
						rt_kprintf("[hplc]  (%s) len> array size ChgPlanAdjust_rsp.cAssetNO \n",__func__);
						return -1;
					}	//changed next			
					temp_array=( unsigned char *) (ChgPlanAdjust_rsp.cAssetNO+1);
					_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);	
				
				
				}else{
//					if((my_strategy_event_get()&ChgPlanAdjust_EVENT)!=0){//用户有没有完成上一次充电计划
//						rt_kprintf("[hplc]  (%s) ()&ChgPlanAdjust_EVENT)!=0     \n",__func__);
////						return 2;
//					};
//					if(1){//再处理一下用户上传的业务。
//						check_afair_from_botom(priv_698_state,hplc_data);
//					}
					
					if(_698_frame_rev->usrData_len>sizeof(_698_ChgPlanAdjust_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len> array size  \n",__func__);
						return -1;					
					}			

					_698_ChgPlanAdjust=*_698_frame_rev;
					my_strcpy(_698_ChgPlanAdjust_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_ChgPlanAdjust.usrData=_698_ChgPlanAdjust_data;						
					
					_698_ChgPlanAdjust.time_flag_positon=_698_ChgPlanAdjust.usrData_len;//最后一位，只给方法用时有效
					//保存充电计划单
					action_response_charge_strategy(&charge_strategy_ChgPlanAdjust,_698_frame_rev);
					strategy_event_send(Cmd_ChgPlanAdjust);
					return 2;//发送事件	
				}				
			
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}				
			break;		
			
		case(0x04)://充电服务

			//判断属性,处理方法
			if(priv_698_state->oad_omd.attribute_id==127){//启动（参数）
				if(_698_StartChg.need_package==1){
					_698_StartChg.need_package=0;
					
					temp_char=0;//DAR， 成功 （ 0），硬件失效 （ 1），其他 （255）
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
					temp_char=0;//Data OPTIONAL=0 表示没有数据
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);							
					result=0;
				
				}else{
//					if(1){//再处理一下用户上传的业务。
//						check_afair_from_botom(priv_698_state,hplc_data);
//					}

					if(_698_frame_rev->usrData_len>sizeof(_698_StartChg_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len> array size  \n",__func__);
						return -1;
					
					}			
					
					_698_StartChg=*_698_frame_rev;
					my_strcpy(_698_StartChg_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_StartChg.usrData=_698_StartChg_data;	
					
					_698_StartChg.time_flag_positon=_698_StartChg.usrData_len;//最后一位，只给方法用时有效

					rt_kprintf("[hplc]  (%s) start  rt_event_send \n",__func__);
					
					action_response_charge_StartStop(&CTL_CHARGE_START,_698_frame_rev);				
					strategy_event_send(Cmd_StartChg);
					return 2;//发送事件	
				}

				//发送信号让下面把充电计划单准备好
			}else if(priv_698_state->oad_omd.attribute_id==128){//停止（参数）
				if(_698_StopChg.need_package==1){
					_698_StopChg.need_package=0;
					
					temp_char=0;//DAR， 成功 （ 0），硬件失效 （ 1），其他 （255）
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

					temp_char=0;//Data OPTIONAL=0 表示没有数据
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
				
					result=0;
				
				}else{
;
//					if(1){//再处理一下用户上传的业务。
//						check_afair_from_botom(priv_698_state,hplc_data);
//					}					
									
					if(_698_frame_rev->usrData_len > sizeof(_698_StopChg_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len>array size  \n",__func__);
						return -1;
					
					}			
					_698_StopChg=*_698_frame_rev;
					my_strcpy(_698_StopChg_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_StopChg.usrData=_698_StopChg_data;	
					
					_698_StopChg.time_flag_positon=_698_StopChg.usrData_len;//最后一位，只给方法用时有效
					rt_kprintf("[hplc]  (%s) stop  rt_event_send \n",__func__);
					action_response_charge_StartStop(&CTL_CHARGE_STOP,_698_frame_rev);
					
					strategy_event_send(Cmd_StopChg);
					return 2;//发送事件	
				}
			} else if(priv_698_state->oad_omd.attribute_id==129){//功率调节（参数）
			
				if(_698_frame_rev->need_package==1){
					_698_frame_rev->need_package=0;
					rt_kprintf("[hplc]  (%s)   .need_package==1   \n",__func__);

					temp_char=CTL_CHARGE_Adj.cSucIdle;//DAR， 成功 （ 0），硬件失效 （ 1），其他 （255）					
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
					
					temp_char=0;// OPTIONAL=0 表示没有数据
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

				}else{

//					if(1){//再处理一下用户上传的业务。
//						check_afair_from_botom(priv_698_state,hplc_data);
//					}
					
					if(_698_frame_rev->usrData_len>sizeof(_698_CTL_CHARGE_Adj_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len> array size  \n",__func__);
						return -1;					
					}

					_698_CTL_CHARGE_Adj=*_698_frame_rev;//将接收698帧给
					my_strcpy(_698_CTL_CHARGE_Adj_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_CTL_CHARGE_Adj.usrData=_698_CTL_CHARGE_Adj_data;						
					
					_698_CTL_CHARGE_Adj.time_flag_positon=_698_frame_rev->usrData_len;//最后一位，只给方法用时有效
		

					action_response_power_adjust(&CTL_CHARGE_Adj,_698_frame_rev);					
					strategy_event_send(Cmd_PowerAdj);						
					return 2;//发送事件	
				}
		
			}else{
				rt_kprintf("[hplc]  (%s)  can not deal the affair  \n",__func__);
				return -1;
			}				
			break;					
					
		default:

			rt_kprintf("[hplc]  (%s)  only supply operation=%0x \n",__func__,priv_698_state->oad_omd.oi[1]);
			result=-1;
			break;		
	}	
	if(result == 0)	{//继续打包	
	}
	
	return result;		
	

}

int oi_esam_oib(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,cmd,i=0,priv_len=0,times=0;
	unsigned char temp_char;

	switch (priv_698_state->oad_omd.oi[1]){//判别了两位，还要判断属性或者方法，很少，用if在里面判断
		case(0):

			//判断属性,只处理属性2
			cmd=priv_698_state->oad_omd.attribute_id;
			rt_kprintf("[hplc]  (%s)  cmd==%d  \n",__func__,cmd);
			for(times=0;times<5;times++){
				hplc_ScmEsam_Comm.DataTx_len=0;
				if((cmd==2)||(cmd==4)){//esam 序列号   HOST_KEY_AGREE
					
					get_data_class(priv_698_state,hplc_data,Data_octet_string);
					ESAM_Communicattion(cmd,&hplc_ScmEsam_Comm);
					
					temp_char=(hplc_ScmEsam_Comm.DataRx_len-5);//数组数量，由上传者决定默认是一
				//	rt_kprintf("[hplc]  (%s)   result=%d hplc_ScmEsam_Comm.DataRx_len=%d\n",__func__,result,hplc_ScmEsam_Comm.DataRx_len);
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
					if(hplc_ScmEsam_Comm.DataRx_len<12){

						result=-1;//没读出数据
					}else{
						result=0;		
						result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+4),(hplc_ScmEsam_Comm.DataRx_len-5));
					}				
		
				}else if(cmd==7){//esam 序列号   HOST_KEY_AGREE
					get_data_class(priv_698_state,hplc_data,Data_structure);
					
					temp_char=4;//数组数量，由上传者决定默认是一
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);				


//					if(_698_frame_rev->addr.ca!=0){//
//						hplc_current_ESAM_CMD=1;
//						rt_kprintf("\n[hplc] addr.ca!=0 \n",__func__);
//					}else{
//						hplc_current_ESAM_CMD=cmd-1;
//						rt_kprintf("\n[hplc] addr.ca==0 \n",__func__);		
//					}					
					ESAM_Communicattion(cmd-1,&hplc_ScmEsam_Comm);


					
					if(hplc_ScmEsam_Comm.DataRx_len<12){

						result=-1;//没读出数据
					}else{
						result=0;	
						priv_len=	hplc_ScmEsam_Comm.DataRx_len-5;
						for(i=0;i<4;i++){
							
							temp_char=Data_double_long_unsigned;//数组数量，由上传者决定默认是一
							result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);						
							result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+(4*(i+1))),4);								
						}
						
//						for(i=0;i<1;i++){
//							
//							temp_char=Data_double_long_unsigned;//数组数量，由上传者决定默认是一
//							result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
//							
//							result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+(4*(i+1))),4);
//									
//						}
						
					}				
	
				}else{
					temp_char=0;//错误信息 [0]  DAR，
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);						
					
					temp_char=6;//对象不存在 （ 6），
					result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

					
					rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
					return -1;
				}
				if(result==0){
					return 0;
				}
				rt_thread_mdelay(200);	//不成功200毫秒再读
			}
			temp_char=0;//错误信息 [0]  DAR，
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);						
			
			temp_char=2;//暂时失效 （ 2），
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
			
			break;		

		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  only supply communication addr \n",__func__);
			break;		
	}	
	
	rt_kprintf("[hplc]  (%s)   result=%d\n",__func__,result);	
	if(result == 0)	{//继续打包	
		
	}
	
	return result;		
	

}


int oi_charge_oib(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;


	switch (priv_698_state->oad_omd.oi[1]){//判别了两位，还要判断属性或者方法，很少，用if在里面判断
		case(0)://日期时间
			result=-1;
//			printf("[hplc]  (%s)      \n",__func__);
			rt_kprintf("[hplc]  (%s)      \n",__func__);			
			break;
		case(0x01)://充电计划单
			if(priv_698_state->oad_omd.attribute_id==2){
				if(_698_frame_rev->need_package==1){					
					_698_frame_rev->need_package=0;
					get_data_class(priv_698_state,hplc_data,Data_array);
					
					result=oi_charge_strategy_package(_698_frame_rev,priv_698_state,hplc_data);
					//保存用户数据				
				}else{
//					if((my_strategy_event_get()&ChgPlanIssueGet_EVENT)!=0){//用户有没有完成上一次充电计划
//							rt_kprintf("[hplc]  (%s) ()&ChgPlanIssueGet_EVENT)!=0     \n",__func__);
////						return 2;
//						};
//						if(1){//再处理一下用户上传的业务。
//							check_afair_from_botom(priv_698_state,hplc_data);
//						}										
					
					if(_698_frame_rev->usrData_len>sizeof(_698_ChgPlanIssueGet_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len> array size  \n",__func__);
						return -1;
					
					}											
					_698_ChgPlanIssueGet=*_698_frame_rev;
					my_strcpy(_698_ChgPlanIssueGet_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_ChgPlanIssueGet.usrData=_698_ChgPlanIssueGet_data;	
						
//					strategy_event_send();//有问题体
					return 2;//发送事件	
				}
				//发送信号让下面把充电计划单准备好
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}				
			break;	

		case(0x03)://路由器工作状态
			//判断属性,只处理属性2
			rt_kprintf("[hplc]  (%s)   routerexestatus   \n",__func__);
			if(priv_698_state->oad_omd.attribute_id==2){
				if(_698_frame_rev->need_package==1){
					rt_kprintf("[hplc]  (%s)  need_package==1   \n",__func__);
					_698_frame_rev->need_package=0;
					get_data_class(priv_698_state,hplc_data,Data_array);
					result=oi_router_exe_state_package(_698_frame_rev,priv_698_state,hplc_data);
					//保存用户数据				
				}else{
//					if((my_strategy_event_get()&AskState_EVENT)!=0){//用户有没有完成上一次充电计划
//							rt_kprintf("[hplc]  (%s) ()&AskState_EVENT!=0     \n",__func__);
//					};
//					if(1){//再处理一下用户上传的业务。
//						check_afair_from_botom(priv_698_state,hplc_data);
//					}	

					if(_698_frame_rev->usrData_len>sizeof(_698_RouterExeState_data)){
						rt_kprintf("[hplc]  (%s)  usrData_len> array size  \n",__func__);
						return -1;
					
					}
					
					_698_RouterExeState=*_698_frame_rev;//将指针地址赋给了对方
					my_strcpy(_698_RouterExeState_data,_698_frame_rev->usrData,0,_698_frame_rev->usrData_len);//拷贝数组
					_698_RouterExeState.usrData=_698_RouterExeState_data;
					
					//清空 _698_frame_rev
					strategy_event_send(Cmd_RouterExeState);
					return 2;//发送事件	
				}
				//发送信号让下面把充电计划单准备好
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}				
			break;					

	case(0x08)://esam
			result=-1;
			//判断属性,只处理属性2
			if(priv_698_state->oad_omd.attribute_id==2){
	
					get_data_class(priv_698_state,hplc_data,Data_array);
//					hplc_ScmEsam_Comm.DataTx_len=0xff;//test
					result=oi_esam_info_package(_698_frame_rev,priv_698_state,hplc_data);
					//保存用户数据				
	
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}				
			break;		
			
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  only supply communication addr \n",__func__);
			break;		
	}	
	if(result == 0)	{//继续打包	
	}
	
	return result;		
	

}


/*

oi_parameter,//参变量类对象


*/

int oi_parameter_oib_general(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;


	switch (priv_698_state->oad_omd.oi[1]){//判别了两位，还要判断属性或者方法，很少，用if在里面判断
		case(0)://日期时间
			result=-1;
			if(priv_698_state->oad_omd.attribute_id==2){
				get_data_class(priv_698_state,hplc_data,Data_date_time_s);
				result=oi_parameter_get_time(_698_frame_rev,priv_698_state,hplc_data);
			
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}				
			break;
		case(1)://通信地址
			result=-1;
			//判断属性,只处理属性2
			if(priv_698_state->oad_omd.attribute_id==2){
				get_data_class(priv_698_state,hplc_data,Data_octet_string);

				result=oi_parameter_get_addr(_698_frame_rev,priv_698_state,hplc_data);
//				if(priv_698_state->meter_addr_send_ok==1){//可能设备重启了，而hplc却没有重启
					priv_698_state->meter_addr_send_ok=2;
//				}				
			}else{
				rt_kprintf("[hplc]  (%s)  only deal   attribute_id==2  \n",__func__);
				return -1;
			}				
			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  only supply communication addr \n",__func__);
			break;		
	}	
	if(result == 0)	{//继续打包	
	}	
	return result;		
}



/*
电能量类对象的处理
这层是过度作用，目前用处不大
*/

int get_response_variable_oia(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
		
	temp_char=priv_698_state->oad_omd.oi[0]&OI2_MASK;//获取对象标识
	switch (temp_char){//
		case(0)://计量
			result=-1;
			rt_kprintf("[hplc]  (%s)    meterage   \n",__func__);
			result=oi_variable_oib_meterage(_698_frame_rev,priv_698_state,hplc_data);	
			break;
		case(1)://统计
			result=-1;
			rt_kprintf("[hplc]  (%s)     \n",__func__);
			break;		
		case(2)://采集
			result=-1;
			rt_kprintf("[hplc]  (%s)     \n",__func__);
			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)   \n",__func__);
			break;		
	}	
		if(result == 0)	{//继续打包		
	}
	
	return result;		
}
/*
电能量类对象的处理
这层是过度作用，目前用处不大
*/
int get_response_electrical_oia(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
		
	temp_char=priv_698_state->oad_omd.oi[0]&OI2_MASK;//获取对象标识
	switch (temp_char){//
		case(0)://总
			result=-1;
			rt_kprintf("[hplc]  (%s)   sum   \n",__func__);

			result=oi_electrical_oib_sum(_698_frame_rev,priv_698_state,hplc_data);	

			break;
		case(1)://基波
			result=-1;
			rt_kprintf("[hplc]  (%s)   base component   \n",__func__);

			break;		
		case(2)://谐波
			result=-1;
			rt_kprintf("[hplc]  (%s)   harmonic component   \n",__func__);


			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)   \n",__func__);

			break;		
	}	
		if(result == 0)	{//继续打包
		
	}
	
	return result;		
}

int report_response_oia(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
//	rt_kprintf("[hplc]  (%s)  \n",__func__);
		
	temp_char=priv_698_state->oad_omd.oi[0]&OI2_MASK;//获取对象标识
	switch (temp_char){//
		case(0)://
			result=-1;
			rt_kprintf("[hplc]  (%s)   charge   \n",__func__);

			//result=oi_action_response_charge_oib(_698_frame_rev,priv_698_state,hplc_data);	
			result=oi_report_response_oib(_698_frame_rev,priv_698_state,hplc_data);
			break;
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)   \n",__func__);

			break;		
	}	
		if(result == 0)	{//继续打包
		
	}
	
	return result;		
}




int action_response_charge_oia(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
//	rt_kprintf("[hplc]  (%s)  \n",__func__);
		
	temp_char=priv_698_state->oad_omd.oi[0]&OI2_MASK;//获取对象标识
	switch (temp_char){//
		case(0)://
			result=-1;
			rt_kprintf("[hplc]  (%s)   charge   \n",__func__);

			result=oi_action_response_charge_oib(_698_frame_rev,priv_698_state,hplc_data);	

			break;
		case(1):////参变量类对象,
			result=-1;
			rt_kprintf("[hplc]  (%s)      \n",__func__);


			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)   \n",__func__);

			break;		
	}	
		if(result == 0)	{//继续打包
		
	}
	
	return result;		
}


int get_response_esam_oia(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
		
	temp_char=priv_698_state->oad_omd.oi[0]&OI2_MASK;//获取对象标识
	switch (temp_char){//
		case(0)://,
			result=-1;
			rt_kprintf("[hplc]  (%s)      \n",__func__);

			break;
		case(1):////参变量类对象,
			result=-1;

			rt_kprintf("[hplc]  (%s)   oi_esam_oib   \n",__func__);
			result=oi_esam_oib(_698_frame_rev,priv_698_state,hplc_data);	
//			rt_kprintf("[hplc]  (%s)   result=%d\n",__func__,result);
			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)   \n",__func__);

			break;		
	}	
		if(result == 0)	{//继续打包
		
	}
	
	return result;		
}




int get_response_charge_oia(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
		
	temp_char=priv_698_state->oad_omd.oi[0]&OI2_MASK;//获取对象标识
	switch (temp_char){//
		case(0)://,
			result=-1;
			rt_kprintf("[hplc]  (%s)   charge   \n",__func__);
			result=oi_charge_oib(_698_frame_rev,priv_698_state,hplc_data);	

			break;
		case(1):////参变量类对象,
			result=-1;
			rt_kprintf("[hplc]  (%s)      \n",__func__);


			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)   \n",__func__);

			break;		
	}	
		if(result == 0)	{//继续打包
		
	}
	
	return result;		
}

/*
参数类的处理
这层是过度作用，目前用处不大
*/
int get_response_parameter_oia(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
		
	temp_char=priv_698_state->oad_omd.oi[0]&OI2_MASK;//获取对象标识
	switch (temp_char){//
		case(0)://参变量类对象,
			result=-1;
			rt_kprintf("[hplc]  (%s)   case(0)   \n",__func__);

			result=oi_parameter_oib_general(_698_frame_rev,priv_698_state,hplc_data);	

			break;
		case(1):////参变量类对象,
			result=-1;
			rt_kprintf("[hplc]  (%s)   case(1)   \n",__func__);


			break;		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  default: \n",__func__);
			break;		
	}	
		if(result == 0)	{//继续打包		
	}	
	return result;		
}


/*
处理 对象型属性。
返回值：result==1，不需要发送

*/

int report_response_normal_omd(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	//保存oad
//	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=priv_698_state->oad_omd.oi[0];//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=priv_698_state->oad_omd.oi[1];//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=priv_698_state->oad_omd.attribute_id;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_698_state->oad_omd.attribute_index;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->oad_omd.oi[0]&OI1_MASK)>>4;//获取对象标识
	switch (temp_char){//
		
		case(6)://9 充电的类型,
			result=-1;
			rt_kprintf("[hplc]  (%s)     chargepile \n",__func__);
			//result=action_response_charge_oia(_698_frame_rev,priv_698_state,hplc_data);
			result=report_response_oia(_698_frame_rev,priv_698_state,hplc_data);
			break;
		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  not support  oia=%0x  \n",__func__,temp_char);
			break;		
	}	
		if(result == 0)	{//继续打包
		
	}	
	
		return result;
	
}


/*
处理 对象型属性。
返回值：result==1，不需要发送

*/

int action_response_normal_omd(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;
	unsigned char temp_char;
	//保存oad
//	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=priv_698_state->oad_omd.oi[0];//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=priv_698_state->oad_omd.oi[1];//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=priv_698_state->oad_omd.attribute_id;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_698_state->oad_omd.attribute_index;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->oad_omd.oi[0]&OI1_MASK)>>4;//获取对象标识
	switch (temp_char){//
		
		case(9)://9 充电的类型,
			result=-1;
			rt_kprintf("[hplc]  (%s)     chargepile \n",__func__);
			result=action_response_charge_oia(_698_frame_rev,priv_698_state,hplc_data);
		
			break;
		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  not support  oia=%0x  \n",__func__,temp_char);
			break;		
	}	
		if(result == 0)	{//继续打包
		
	}	
	
		return result;
	
}
/*
处理 对象型属性。
返回值：result==1，不需要发送

*/

int get_response_normal_oad(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0;
	unsigned char temp_char;
	//保存oad
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=priv_698_state->oad_omd.oi[0];//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	rt_kprintf("[hplc]  (%s)  priv_698_state->oad_omd.oi[0]=%0x\n",__func__,priv_698_state->oad_omd.oi[0]);	
	
	
	temp_char=priv_698_state->oad_omd.oi[1];//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=priv_698_state->oad_omd.attribute_id;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_698_state->oad_omd.attribute_index;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	
	temp_char=(priv_698_state->oad_omd.oi[0]&OI1_MASK)>>4;//获取对象标识
	switch (temp_char){//
		
		case(oi_electrical_energy)://0 电能量类对象
			result=-1;
			rt_kprintf("[hplc]  (%s)     oi_electrical_energy \n",__func__);
			result=get_response_electrical_oia(_698_frame_rev,priv_698_state,hplc_data);
			break;
		
		
		case(oi_variable)://2 变量类对象
			result=-1;
			rt_kprintf("[hplc]  (%s)     oi_variable \n",__func__);
			result=get_response_variable_oia(_698_frame_rev,priv_698_state,hplc_data);
			break;		
		
		
		
		case(oi_parameter)://4 参变量类对象,
			result=-1;
			rt_kprintf("[hplc]  (%s)     oi_parameter \n",__func__);
			result=get_response_parameter_oia(_698_frame_rev,priv_698_state,hplc_data);
			break;

		case(9)://9 充电的类型,
			result=-1;
			rt_kprintf("[hplc]  (%s)     chargepile \n",__func__);
			result=get_response_charge_oia(_698_frame_rev,priv_698_state,hplc_data);
			break;

		case(0xf)://获取esam信息
			result=-1;
			rt_kprintf("[hplc]  (%s)    esam \n",__func__);
//			for(i=0;i<0;i++){
							result=get_response_esam_oia(_698_frame_rev,priv_698_state,hplc_data);		
//			}


			break;		
		
		
		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)  not support  oia=%0x  \n",__func__,temp_char);

			break;		
	}	
		if(result == 0)	{//继续打包
	
	
	}	
	
		return result;
	
}

/*
  密文传输

*/
int unpatch_ScmEsam_Comm(struct CharPointDataManage * hplc_data,ScmEsam_Comm* l_stEsam_Comm){
	int result=0,i;
	int len_sid,len_a_data,len_data,len_mac,len_rn;
	int position_sid,position_a_data,position_data,position_mac,position_rn;
	unsigned char temp_char;

	rt_kprintf("[hplc]  (%s)  \n",__func__);
	
	if(hplc_data->_698_frame.usrData[1]!=1){//密文传输
		rt_kprintf("[hplc]  (%s)  [1]!=1\n",__func__);	
	}
	
	if((hplc_data->_698_frame.usrData[2]==0x81)||(hplc_data->_698_frame.usrData[2]==0x82)){//超过7f，在长度前面会加0x81
		rt_kprintf("[hplc]  (%s) length more then 1024   \n",__func__);
		len_data=hplc_data->_698_frame.usrData[3]*256+hplc_data->_698_frame.usrData[4];//data长度

		position_data=(4+1);//开始位置就是2+1=3   1是数据类型	
	}else{
		len_data=hplc_data->_698_frame.usrData[2];//data长度

		position_data=(2+1);//开始位置就是2+1=3   1是数据类型	
	}


	
	if(hplc_data->_698_frame.usrData[(position_data+len_data)]==0){
		//这个是类型，0是数据验证码 [0] SID_MAC，主站的；2是  [2]随机数+数据MAC  
		rt_kprintf("[hplc]  (%s)  [2+len_data]=%0x?=0!!!!  \n",__func__,hplc_data->_698_frame.usrData[(position_data+len_data)]);		
		len_sid=4;
		position_sid=position_data+len_data+1;//开始位置就是(3+len_data)+1(这个1是上面的类型)

		len_a_data=hplc_data->_698_frame.usrData[position_sid+len_sid];
		position_a_data=position_sid+len_sid+1;
		
		
	
		len_mac=hplc_data->_698_frame.usrData[position_a_data+len_a_data];
		position_mac=position_a_data+len_a_data+1;
		
		my_strcpy(l_stEsam_Comm->Tx_data,hplc_data->_698_frame.usrData,position_sid,len_sid);//
		l_stEsam_Comm->DataTx_len=len_sid;

		my_strcpy(l_stEsam_Comm->Tx_data+l_stEsam_Comm->DataTx_len,hplc_data->_698_frame.usrData,position_a_data,len_a_data);//	
		l_stEsam_Comm->DataTx_len+=len_a_data;
		
		my_strcpy(l_stEsam_Comm->Tx_data+l_stEsam_Comm->DataTx_len,hplc_data->_698_frame.usrData,position_data,len_data);//	
		l_stEsam_Comm->DataTx_len+=len_data;

		my_strcpy(l_stEsam_Comm->Tx_data+l_stEsam_Comm->DataTx_len,hplc_data->_698_frame.usrData,position_mac,len_mac);//	
		l_stEsam_Comm->DataTx_len+=len_mac;
	

		rt_kprintf("[hplc]  (%s)  len_a_data=%d len_mac=%d\n",__func__,len_a_data,len_mac);		


	}	else 	if(hplc_data->_698_frame.usrData[(position_data+len_data)]==2){
		//这个是类型，2是控制器的  [2]随机数+数据MAC  
		rt_kprintf("[hplc]  (%s)  [2+len_data]=%0x?=2!!!!  \n",__func__,hplc_data->_698_frame.usrData[(position_data+len_data)]);		

		len_rn=hplc_data->_698_frame.usrData[position_data+len_data+1];//可能是0
		position_rn=position_data+len_data+1+1;//开始位置

		
		len_mac=hplc_data->_698_frame.usrData[position_rn+len_rn];
		position_mac=position_rn+len_rn+1;
		

		my_strcpy(l_stEsam_Comm->Tx_data+l_stEsam_Comm->DataTx_len,hplc_data->_698_frame.usrData,position_data,len_data);//	
		l_stEsam_Comm->DataTx_len+=len_data;

		my_strcpy(l_stEsam_Comm->Tx_data+l_stEsam_Comm->DataTx_len,hplc_data->_698_frame.usrData,position_mac,len_mac);//	
		l_stEsam_Comm->DataTx_len+=len_mac;
	
	}	
	

		
	return result;	
}


/*
  密文传输

*/
int security_get_package(int security_style,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,len_data=0,len_mac=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;
	//return -1;//没有实现
	rt_kprintf("[hplc]  (%s) hplc_data->dataSize=%d \n",__func__,hplc_data->dataSize);
	//先解密

	hplc_ScmEsam_Comm.DataTx_len=hplc_data->_698_frame.usrData_len;
	my_strcpy(hplc_ScmEsam_Comm.Tx_data,hplc_data->_698_frame.usrData,0,hplc_data->_698_frame.usrData_len);
	security_style=1;
	if(security_style==1){//密文
		rt_kprintf("[hplc]  (%s) encode \n",__func__);
		hplc_current_ESAM_CMD=HOST_SESS_CALC_MAC_A7;
	}else{//明文
		rt_kprintf("[hplc]  (%s)  not coded \n",__func__);
		hplc_current_ESAM_CMD=HOST_SESS_CALC_MAC_11;
	}

	
	ESAM_Communicattion(hplc_current_ESAM_CMD,&hplc_ScmEsam_Comm);


	if((hplc_ScmEsam_Comm.Rx_data[2]*256+hplc_ScmEsam_Comm.Rx_data[3])==0){//esam高字节在前
		rt_kprintf("[hplc]  (%s) not get the right ack \n",__func__);
//		result =-1;
		return -1;
	}else{
		len_data=(hplc_ScmEsam_Comm.Rx_data[3]+hplc_ScmEsam_Comm.Rx_data[2]*256)-4;
		len_mac=4;//返回的mac的长度
		rt_kprintf("[hplc]  (%s)  len_mac=%d\n",__func__,len_mac);
	}

	hplc_data->dataSize=hplc_data->_698_frame.addr.s_addr_len+8;	//前面的长度
	
	rt_kprintf("[hplc]  (%s)  hplc_data->dataSize=%d\n",__func__,hplc_data->dataSize);
	
	
	temp_char=security_response;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=security_style;//密文
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	if((len_data)>0x7f){//
	temp_char=0x81;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	temp_char=len_data%256;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
	temp_char=len_data/256;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	
	}else if((len_data)>0xff){
		temp_char=0x82;//
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
			
		temp_char=len_data%256;//长度
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
			
		temp_char=len_data/256;//长度
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	}else{
		temp_char=len_data;//长度
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	}
	
	

	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_ScmEsam_Comm.Rx_data+4,len_data);	
		
	temp_char=1;//mac   数据验证信息 CHOICE OPTIONAL   OPTIONAL标识这个是可选项
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0;//choice
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=len_mac;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_ScmEsam_Comm.Rx_data+(len_data+len_mac),len_mac);	
	
	
	
	if(result == 0)	{//继续打包
		
		hplc_data->_698_frame.usrData_len=hplc_data->dataSize-priv_698_state->HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
		//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

		priv_698_state->FCS_position=hplc_data->dataSize;
		hplc_data->dataSize+=2;//加两字节的校验

			
		temp_char=hplc_data->_698_frame.end=0x16;
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么


	//给长度结构体赋值,这里判断是不是需要分针
		hplc_data->priveData[priv_698_state->len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

		hplc_data->priveData[priv_698_state->len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

	//校验头
		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->HCS_position);
		hplc_data->_698_frame.HCS0=hplc_data->priveData[priv_698_state->HCS_position];	
		hplc_data->_698_frame.HCS1=hplc_data->priveData[priv_698_state->HCS_position+1];

		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->FCS_position);
		
		hplc_data->_698_frame.FCS0=hplc_data->priveData[priv_698_state->FCS_position];
		hplc_data->_698_frame.FCS1=hplc_data->priveData[priv_698_state->FCS_position+1];		
	}	
	
	
	return result;	
}


/*
  响应读取一个记录型对象属性请求

*/
int get_response_package_record(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;
	//return -1;//没有实现
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=get_response;//可以不用hplc_698_link_response,但为了表意方便还是用了，而且登录用不了多少时间，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=_698_frame_rev->usrData[1];//具体类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=_698_frame_rev->usrData[2];//PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int time_flag=3;	
	
	//	往下处理oad,多个对象型的就这么处理
	oad_package(&priv_698_state->oad_omd,_698_frame_rev,3);	
	//oad_package(&priv_698_state->oad_omd,_698_frame_rev,3+4*i);//下一个oad	
	//result=get_response_record_oad(_698_frame_rev,priv_698_state,hplc_data);

	if(result==0){
		time_flag+=4;
		
		temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
		if(_698_frame_rev->usrData[time_flag]==0){//接收帧oad后面的那个是时间标识
			temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
		}else{//加时间标签
		
		}	
	}				
	return result;	
}



/*
通知用户

*/
int report_response_notice_user_Record(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state){
	int result=0,num=0,i=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char,*temp_array,CHARGE_APPLY_oad_array[4*17]={0x20, 0x22, 0x02, 0x00, 0x20, 0x1e, 0x02, 0x00, 
		    0x20, 0x22, 0x02, 0x00, 0x20, 0x24, 0x02, 0x00, 0x33, 0x00, 0x02, 0x00, 0x35, 0x05, 
				0x02, 0x06, 0x35, 0x05, 0x02, 0x07, 0x35, 0x05, 0x02, 0x08, 0x35, 0x05, 0x02, 0x09, 
		    0x35, 0x05, 0x02, 0x0a, 0x35, 0x05, 0x02, 0x0b, 0x35, 0x05, 0x02, 0x0c, 0x35, 0x05, 
				0x02, 0x0d, 0x35, 0x05, 0x02, 0x0e, 0x35, 0x05, 0x02, 0x0f, 0x35, 0x05, 0x02, 0x10, 
				0x35, 0x05, 0x02, 0x11};
	//	往下处理oad,多个对象型的就这么处理
//20 22 02 00//20 1e 02 00//20 22 02 00//20 24 02 00//33 00 02 00//35 05 02 06(第六个)
//         07//         08//         09//         0a//         0b//         0c
//         0d//         0e//         0f//         10//         11

	
	
	num=_698_frame_rev->usrData[3];//一个report_response  一个类型，一个piid  , 一个 长度
		
	if(num==0x11){//充电申请事件有应答
		for(i=0;i<num;i++){
			oad_package(&priv_698_state->oad_omd,_698_frame_rev,(4+4*i));//跟oad复用		
			if(priv_698_state->oad_omd.oi[0]==CHARGE_APPLY_oad_array[4*i]){
				if(priv_698_state->oad_omd.oi[1]==CHARGE_APPLY_oad_array[4*i+1]){
					if(priv_698_state->oad_omd.attribute_id==CHARGE_APPLY_oad_array[4*i+2]){
						if(priv_698_state->oad_omd.attribute_index==CHARGE_APPLY_oad_array[4*i+3]){
								rt_kprintf("[hplc]  (%s)   the %d oad is right \n",__func__,i);
							  if(i==(num-1)){
//								strategy_event_send();//通知周，上报的充电申请事件有应答										
								}						
						}else{
							break;
							rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
						}		
					}else{
						break;
						rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
					}	
				}else{
					break;
					rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
				}		
			}else{
				break;
				rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
			}		
		}
	}

//	if(num==0x11){//充电计划事件上报有应答
//		for(i=0;i<num;i++){
//			oad_package(&priv_698_state->oad_omd,_698_frame_rev,(4+4*i));//跟oad复用		
//			if(priv_698_state->oad_omd.oi[0]==[4*i]){
//				if(priv_698_state->oad_omd.oi[1]==[4*i+1]){
//					if(priv_698_state->oad_omd.attribute_id==[4*i+2]){
//						if(priv_698_state->oad_omd.attribute_index==[4*i+3]){
//								rt_kprintf("[hplc]  (%s)   the %d oad is right \n",__func__,i);
//							  if(i==(num-1)){
////								strategy_event_send();//通知周，上报的充电申请事件有应答										
//								}						
//						}else{
//							break;
//							rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
//						}		
//					}else{
//						break;
//						rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
//					}	
//				}else{
//					break;
//					rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
//				}		
//			}else{
//				break;
//				rt_kprintf("[hplc]  (%s)   not CHARGE_APPLY \n",__func__); 				
//			}		
//		}
//	}




	




	
	
	







	
	return result;	

}

/*
通知用户

*/
int report_response_notice_user_normal(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state){
	int result=0,num=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;

	oad_package(&priv_698_state->oad_omd,_698_frame_rev,3);//跟oad复用
	if(_698_frame_rev->need_package!=1){
		_698_frame_rev->time_flag_positon+=4;	//也用做数据的开始帧
	}	

//	90 02 7F 00  				— OMD（充电申请）
//	00 00 						— 成功且无附加数据

	if(priv_698_state->oad_omd.oi[0]==0x90){
		if(priv_698_state->oad_omd.oi[1]==0x0){
			if(priv_698_state->oad_omd.attribute_id==0x7F){
				if(priv_698_state->oad_omd.attribute_index==0x0){
					if(_698_frame_rev->usrData[7]==0x0){
//						strategy_event_send();//通知周，充电申请
						
					
					}	
				}	
			}	
		}	
	}
	
	
			
	return result;	

}



/*
通知用户

*/
int action_notice_user_normal(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state){
	int result=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;

	//	往下处理oad,多个对象型的就这么处理
	oad_package(&priv_698_state->oad_omd,_698_frame_rev,3);//跟oad复用
	if(_698_frame_rev->need_package!=1){
		_698_frame_rev->time_flag_positon+=4;	//也用做数据的开始帧
	}	

//	90 02 7F 00  				— OMD（充电申请）
//	00 00 						— 成功且无附加数据

	if(priv_698_state->oad_omd.oi[0]==0x90){
		if(priv_698_state->oad_omd.oi[1]==0x0){
			if(priv_698_state->oad_omd.attribute_id==0x7F){
				if(priv_698_state->oad_omd.attribute_index==0x0){
					if(_698_frame_rev->usrData[7]==0x0){
//						strategy_event_send();//通知周，充电申请
						
					
					}	
				}	
			}	
		}	
	}
	
	
			
	return result;	

}

/*
响应操作一个对象属性请求

*/
int report_response_package_normal(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=report_response;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=_698_frame_rev->usrData[1];//具体类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=_698_frame_rev->usrData[2];//PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=_698_frame_rev->usrData[3];//数组的长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	if(_698_frame_rev->need_package!=1){
		_698_frame_rev->time_flag_positon=3+1;	
	}
	//	往下处理oad,多个对象型的就这么处理
	oad_package(&priv_698_state->oad_omd,_698_frame_rev,4);//上报多了一位
	if(_698_frame_rev->need_package!=1){
		_698_frame_rev->time_flag_positon+=4;	//也用做数据的开始帧
	}	

	//result=action_response_normal_omd(_698_frame_rev,priv_698_state,hplc_data);
	result=report_response_normal_omd(_698_frame_rev,priv_698_state,hplc_data);
	if(result==0){		
		
//		temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
//		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
//		if(_698_frame_rev->usrData[_698_frame_rev->time_flag_positon]==0){//接收帧oad后面的那个是时间标识
			temp_char=0;//没有时间标签
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
//		}else{//加时间标签
//			rt_kprintf("[hplc]  (%s)  error need timeflag _698_frame_rev->time_flag_positon=%d\n",__func__,_698_frame_rev->time_flag_positon);
//			result=-1;
//		
//		}	
	}				
	return result;	
}


/*
响应操作一个对象属性请求

*/
int action_response_package_normal(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=action_response;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=_698_frame_rev->usrData[1];//具体类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=_698_frame_rev->usrData[2];//PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	if(_698_frame_rev->need_package!=1){
		_698_frame_rev->time_flag_positon=3;	
	}
	//	往下处理oad,多个对象型的就这么处理
	oad_package(&priv_698_state->oad_omd,_698_frame_rev,3);//跟oad复用
	if(_698_frame_rev->need_package!=1){
		_698_frame_rev->time_flag_positon+=4;	//也用做数据的开始帧
	}	
	//oad_package(&priv_698_state->oad_omd,_698_frame_rev,3+4*i);//下一个oad	
	result=action_response_normal_omd(_698_frame_rev,priv_698_state,hplc_data);

	if(result==0){		
		
		temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
//		if(_698_frame_rev->usrData[_698_frame_rev->time_flag_positon]==0){//接收帧oad后面的那个是时间标识
			temp_char=0;//没有时间标签
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
//		}else{//加时间标签
//			rt_kprintf("[hplc]  (%s)  error need timeflag _698_frame_rev->time_flag_positon=%d\n",__func__,_698_frame_rev->time_flag_positon);
//			result=-1;
//		
//		}	
	}				
	return result;	
}







/*
响应读取多个对象属性请求

*/


int get_response_package_normal_list(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,i=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=get_response;//可以不用hplc_698_link_response,但为了表意方便还是用了，而且登录用不了多少时间，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=_698_frame_rev->usrData[1];//具体类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=_698_frame_rev->usrData[2];//PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=_698_frame_rev->usrData[3];//oad个数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	int time_flag=4;
		
	
	//	往下处理oad,多个对象型的就这么处理
	for(i=0;i<_698_frame_rev->usrData[3];i++){
		oad_package(&priv_698_state->oad_omd,_698_frame_rev,(4+4*i));//下一个oad	
		result=get_response_normal_oad(_698_frame_rev,priv_698_state,hplc_data);

	}	


//	rt_kprintf("[hplc]  (%s)  error result=%d\n",__func__,result);//		
	
	if(result==0){
		time_flag+=4*_698_frame_rev->usrData[3];
		
		temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
//		if(_698_frame_rev->usrData[time_flag]==0){//接收帧oad后面的那个是时间标识
			temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
//		}else{//加时间标签		
//		
//		}	
	}				
	return result;	
}

/*
响应读取一个对象属性请求

*/


int get_response_package_normal(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	temp_char=get_response;//可以不用hplc_698_link_response,但为了表意方便还是用了，而且登录用不了多少时间，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=_698_frame_rev->usrData[1];//具体类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=_698_frame_rev->usrData[2];//PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int time_flag=3;	
	
	//	往下处理oad,多个对象型的就这么处理
	oad_package(&priv_698_state->oad_omd,_698_frame_rev,3);		
	result=get_response_normal_oad(_698_frame_rev,priv_698_state,hplc_data);

	if(result==0){
		time_flag+=4;
		
		temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
		if(_698_frame_rev->usrData[time_flag]==0){//接收帧oad后面的那个是时间标识
			temp_char=0;//FollowReport OPTIONAL=0 表示没有上报信息
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			
		}else{//加时间标签
		
		}	
	}				
	return result;	
}


/*
函数作用：对用户提交的
返回值：
*/

int report_response_notice_user(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state){

	int result=0;//不发送返回1,如果发送，result=0;

	unsigned char temp_char;	
                               	
	switch (_698_frame_rev->usrData[1]){//判断操作类型
		case(ReportNotificationList)://上报若干个对象属性的响应 [1] ReportResponseList，	
			rt_kprintf("[hplc]  (%s)     ReportNotificationList \n",__func__);
//				action_notice_user_normal(_698_frame_rev, priv_698_state);
			result=report_response_notice_user_normal(_698_frame_rev, priv_698_state);		
		
			break;
		
		case(ReportNotificationRecordList):// 上报若干个记录型对象属性的响应 [2] ReportResponseRecordList	
			rt_kprintf("[hplc]  (%s)    ReportNotificationRecordList \n",__func__);
			//result=action_response_package_List(_698_frame_rev, priv_698_state,hplc_data);
			result=report_response_notice_user_Record(_698_frame_rev, priv_698_state);		
		
			result=-1;
			break;		

		default:
			rt_kprintf("[hplc]  (%s)  not support type=%0x \n",__func__,_698_frame_rev->usrData[1]);
			result=-1;		
			break;		
	}
	
	
	return result;


}




/*
函数作用：对用户提交的
返回值：
*/

int action_response_notice_user(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state){

	int result=0;//不发送返回1,如果发送，result=0;

	unsigned char temp_char;	
                               	
	switch (_698_frame_rev->usrData[1]){//判断操作类型
		case(ActionRequest)://=1 操作一个对象方法请求 [1] ，	
			rt_kprintf("[hplc]  (%s)     ActionRequest \n",__func__);
			result=action_notice_user_normal(_698_frame_rev, priv_698_state);
			break;
		
		case(ActionRequestList):// 2//操作若干个对象方法请求 [2] ，		
			rt_kprintf("[hplc]  (%s)     ActionRequestList \n",__func__);
			//result=action_response_package_List(_698_frame_rev, priv_698_state,hplc_data);
			result=-1;
			break;		
		case(ActionThenGetRequestNormalList):// ActionThenGetRequestNormalList 3		
			rt_kprintf("[hplc]  (%s)     ActionThenGetRequestNormalList \n",__func__);
			//result=action_get_response_package_List(_698_frame_rev, priv_698_state,hplc_data);
			result=-1;
			break;		
		
		default:
			rt_kprintf("[hplc]  (%s)  not support type=%0x \n",__func__,_698_frame_rev->usrData[1]);
			result=-1;		
			break;		
	}
	
	
	return result;


}


/*
函数作用：
返回值：
*/

int Cmd_security_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * data_tx)
{
	int result=0;
//	struct  _698_FRAME  prv_struct=*_698_frame_rev;		
	if(_698_frame_rev->security_flag==1){//是安全请求,需要&&已经密钥协商过了？似乎只给密钥下载用
		//将用户数据整个地加，然后重新打包
		rt_kprintf("[hplc]  (%s)  security_flag==1  \n",__func__);
		result=security_get_package(_698_frame_rev->needEsam_flag,priv_698_state,data_tx);
		_698_frame_rev->security_flag=0;
		_698_frame_rev->needEsam_flag=0;

	}else{
		_698_frame_rev->security_flag=0;
		_698_frame_rev->needEsam_flag=0;	
		rt_kprintf("[hplc]  (%s)  no need to security  \n",__func__);
		result=0;
	}

	return result;
			

}

/*
函数作用：对用户提交的
返回值：
*/

int report_response_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data)
{


	int result=0;//不发送返回1,如果发送，result=0;

	unsigned char temp_char;	
	//结构体赋值，共同部分
  hplc_data->dataSize=0;

	if(_698_frame_rev->need_package==1){
		
		
	}//不加这个了，因为复用代码会产生很多问题。
	temp_char=hplc_data->_698_frame.head =_698_frame_rev->head;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	
	priv_698_state->len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的，长度

	temp_char=hplc_data->_698_frame.control=CON_STU_U|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	
	
	//拷贝主机地址
	hplc_data->_698_frame.addr.s_addr_len=priv_698_state->addr.s_addr_len;

	my_strcpy(hplc_data->_698_frame.addr.s_addr,priv_698_state->addr.s_addr,0,priv_698_state->addr.s_addr_len);//拷贝数组
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);//


	temp_char=hplc_data->_698_frame.addr.ca=_698_frame_rev->addr.ca;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	priv_698_state->HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位

	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);		                               
	
	switch (_698_frame_rev->usrData[1]){//判断操作类型
		case(ReportNotificationList)://=1 操作一个对象方法请求 [1] ，	
			rt_kprintf("[hplc]  (%s)     ReportNotificationList \n",__func__);
			result=-1;
			break;
		
		case(ReportNotificationRecordList):// 2//操作若干个对象方法请求 [2] ，		
			rt_kprintf("[hplc]  (%s)     ReportNotificationRecordList \n",__func__);
			//result=action_response_package_normal(_698_frame_rev, priv_698_state,hplc_data);
			result=report_response_package_normal(_698_frame_rev, priv_698_state,hplc_data);
			break;		
		
		default:
			rt_kprintf("[hplc]  (%s)  not support type=%0x \n",__func__,_698_frame_rev->usrData[1]);
			result=-1;		
			break;		
	}

	if(result == 0)	{//继续打包
		
		hplc_data->_698_frame.usrData_len=hplc_data->dataSize-priv_698_state->HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
		//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

		priv_698_state->FCS_position=hplc_data->dataSize;
		hplc_data->dataSize+=2;//加两字节的校验

			
		temp_char=hplc_data->_698_frame.end=0x16;
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么


	//给长度结构体赋值,这里判断是不是需要分针
		hplc_data->priveData[priv_698_state->len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

		hplc_data->priveData[priv_698_state->len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

	//校验头
		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->HCS_position);
		hplc_data->_698_frame.HCS0=hplc_data->priveData[priv_698_state->HCS_position];	
		hplc_data->_698_frame.HCS1=hplc_data->priveData[priv_698_state->HCS_position+1];

		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->FCS_position);
		
		hplc_data->_698_frame.FCS0=hplc_data->priveData[priv_698_state->FCS_position];
		hplc_data->_698_frame.FCS1=hplc_data->priveData[priv_698_state->FCS_position+1];		
	}
	
	
	return result;


}







/*
函数作用：操作一个对象方法
返回值：result==1，不需要发送
*/

int action_response_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;//不发送返回1,如果发送，result=0;

	unsigned char temp_char;	
	//结构体赋值，共同部分
  hplc_data->dataSize=0;

	if(_698_frame_rev->need_package==1){
		
		
	}//不加这个了，因为复用代码会产生很多问题。
	temp_char=hplc_data->_698_frame.head =_698_frame_rev->head;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	
	priv_698_state->len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的，长度

	temp_char=hplc_data->_698_frame.control=CON_STU_U|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	
	
	//拷贝主机地址
	hplc_data->_698_frame.addr.s_addr_len=priv_698_state->addr.s_addr_len;

	my_strcpy(hplc_data->_698_frame.addr.s_addr,priv_698_state->addr.s_addr,0,priv_698_state->addr.s_addr_len);//拷贝数组
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);//


	temp_char=hplc_data->_698_frame.addr.ca=_698_frame_rev->addr.ca;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	priv_698_state->HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位

	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);		                               
	
	switch (_698_frame_rev->usrData[1]){//判断操作类型
		case(ActionRequest)://=1 操作一个对象方法请求 [1] ，	
			rt_kprintf("[hplc]  (%s)     ActionRequest \n",__func__);
			result=action_response_package_normal(_698_frame_rev, priv_698_state,hplc_data);
			break;
		
		case(ActionRequestList):// 2//操作若干个对象方法请求 [2] ，		
			rt_kprintf("[hplc]  (%s)     ActionRequestList \n",__func__);
			//result=action_response_package_List(_698_frame_rev, priv_698_state,hplc_data);
			result=-1;
			break;		
		case(ActionThenGetRequestNormalList):// ActionThenGetRequestNormalList 3		
			rt_kprintf("[hplc]  (%s)     ActionThenGetRequestNormalList \n",__func__);
			//result=action_get_response_package_List(_698_frame_rev, priv_698_state,hplc_data);
			result=-1;
			break;		
		
		default:
			rt_kprintf("[hplc]  (%s)  not support type=%0x \n",__func__,_698_frame_rev->usrData[1]);
			result=-1;		
			break;		
	}

	if(result == 0)	{//继续打包
		
		hplc_data->_698_frame.usrData_len=hplc_data->dataSize-priv_698_state->HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
		//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

		priv_698_state->FCS_position=hplc_data->dataSize;
		hplc_data->dataSize+=2;//加两字节的校验

			
		temp_char=hplc_data->_698_frame.end=0x16;
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么


	//给长度结构体赋值,这里判断是不是需要分针
		hplc_data->priveData[priv_698_state->len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

		hplc_data->priveData[priv_698_state->len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

	//校验头
		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->HCS_position);
		hplc_data->_698_frame.HCS0=hplc_data->priveData[priv_698_state->HCS_position];	
		hplc_data->_698_frame.HCS1=hplc_data->priveData[priv_698_state->HCS_position+1];

		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->FCS_position);
		
		hplc_data->_698_frame.FCS0=hplc_data->priveData[priv_698_state->FCS_position];
		hplc_data->_698_frame.FCS1=hplc_data->priveData[priv_698_state->FCS_position+1];		
	}
	
	
	return result;
}

/*

返回值：result==1，不需要发送
*/

int get_response_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0;//不发送返回1,如果发送，result=0;

	unsigned char temp_char;	
	//结构体赋值，共同部分
  hplc_data->dataSize=0;

	temp_char=hplc_data->_698_frame.head =_698_frame_rev->head;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	
	priv_698_state->len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的，长度

	temp_char=hplc_data->_698_frame.control=CON_STU_U|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	
	
	//拷贝主机地址
	hplc_data->_698_frame.addr.s_addr_len=priv_698_state->addr.s_addr_len;

	my_strcpy(hplc_data->_698_frame.addr.s_addr,priv_698_state->addr.s_addr,0,priv_698_state->addr.s_addr_len);//拷贝数组
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);//


	temp_char=hplc_data->_698_frame.addr.ca=_698_frame_rev->addr.ca;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	priv_698_state->HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位

	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);		                               
	
	switch (_698_frame_rev->usrData[1]){//判断get类型
		case(GetRequestNormal)://=1 读取一个对象属性请求		
			rt_kprintf("[hplc]  (%s)     GetRequestNormal \n",__func__);
			result=get_response_package_normal(_698_frame_rev, priv_698_state,hplc_data);
			break;
		
	
		
		case(GetRequestNormalList)://=2 读取一个对象属性请求		
			rt_kprintf("[hplc]  (%s)     GetRequestNormalList \n",__func__);
			result=get_response_package_normal_list(_698_frame_rev, priv_698_state,hplc_data);
			break;		
		
		
		
		case(GetRequestRecord)://=3 读取一个记录型对象属性请求		
			rt_kprintf("[hplc]  (%s)     GetRequestRecord \n",__func__);
			//result=get_response_package_record(_698_frame_rev, priv_698_state,hplc_data);
			result=-1;
			break;		
		
		default:
			rt_kprintf("[hplc]  (%s)  not support type=%0x \n",__func__,_698_frame_rev->usrData[1]);
			result=-1;		
			break;		
	}
//	rt_kprintf("[hplc]  (%s)  error result=%d\n",__func__,result);//	
	if(result == 0)	{//继续打包
		
		hplc_data->_698_frame.usrData_len=hplc_data->dataSize-priv_698_state->HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
		//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

		priv_698_state->FCS_position=hplc_data->dataSize;
		hplc_data->dataSize+=2;//加两字节的校验

			
		temp_char=hplc_data->_698_frame.end=0x16;
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么


	//给长度结构体赋值,这里判断是不是需要分针
		hplc_data->priveData[priv_698_state->len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

		hplc_data->priveData[priv_698_state->len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

	//校验头
		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->HCS_position);
		hplc_data->_698_frame.HCS0=hplc_data->priveData[priv_698_state->HCS_position];	
		hplc_data->_698_frame.HCS1=hplc_data->priveData[priv_698_state->HCS_position+1];

		//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
		result=tryfcs16(hplc_data->priveData, priv_698_state->FCS_position);
		
		hplc_data->_698_frame.FCS0=hplc_data->priveData[priv_698_state->FCS_position];
		hplc_data->_698_frame.FCS1=hplc_data->priveData[priv_698_state->FCS_position+1];		
	}
	
	
	return result;
}

/*
函数作用：返回可用的data_tx
          服务器返回连接的响应

参数：
*/
int connect_response_package(struct  _698_FRAME  *_698_frame_rev,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int len_position=0,HCS_position=0,result=0;
	int i=0,len=0;
	struct _698_connect_response prive_struct;
	unsigned char temp_char;	
	unPackage_698_connect_request(priv_698_state,_698_frame_rev,&prive_struct);
	
	//结构体赋值，共同部分
  hplc_data->dataSize=0;

	temp_char=hplc_data->_698_frame.head =_698_frame_rev->head;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	
	len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的，长度

	temp_char=hplc_data->_698_frame.control=CON_STU_U|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	
	
	//拷贝主机地址
	hplc_data->_698_frame.addr.s_addr_len=priv_698_state->addr.s_addr_len;
//	if(hplc_data->_698_frame.addr.s_addr==RT_NULL){//先释放
		//rt_free(hplc_data->_698_frame.addr.s_addr);//不判断错误，顶多多吃点内存
		//hplc_data->_698_frame.addr.s_addr=(unsigned char *)rt_malloc(sizeof(unsigned char)*(priv_698_state->addr.s_addr_len));//分配空间	
//	}	

	my_strcpy(hplc_data->_698_frame.addr.s_addr,priv_698_state->addr.s_addr,0,priv_698_state->addr.s_addr_len);//拷贝数组
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);//


	temp_char=hplc_data->_698_frame.addr.ca=_698_frame_rev->addr.ca;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位

	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);	
	
	//if(hplc_data->_698_frame.usrData==RT_NULL){//先释放	
		//hplc_data->_698_frame.usrData=(unsigned char *)rt_malloc(sizeof(unsigned char)*(1024));//给用户分配空间	
		//_698_frame_rev->usrData_size=1024;//空间大小
	//}	
	temp_char=connect_response;//可以不用hplc_698_link_response,但为了表意方便还是用了，而且登录用不了多少时间，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);



	temp_char=prive_struct.piid_acd;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	result=save_char_point_data(hplc_data,hplc_data->dataSize,prive_struct.connect_res_fv.manufacturer_code,4);//厂商代码（ size(4) ） 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,prive_struct.connect_res_fv.soft_version,4);//软件版本号（ size(4) ） 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,prive_struct.connect_res_fv.soft_date,6);//+软件版本日期（ size(6) ）
	result=save_char_point_data(hplc_data,hplc_data->dataSize,prive_struct.connect_res_fv.hard_version,4);//+硬件版本号（ size(4) ） 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,prive_struct.connect_res_fv.hard_date,6);//硬件版本日期（ size(6) ） 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,prive_struct.connect_res_fv.manufacturer_ex_info,8);//+厂家扩展信息（ size(8) ）


	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+2),2);//期望的应用层协议版本号
	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+4),8);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+12),16);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+28),2);

	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+30),2);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+32),1);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+33),2);

	result=save_char_point_data(hplc_data,hplc_data->dataSize,(_698_frame_rev->usrData+35),4);//超时
	
//跟esam通信部分//

	//_698_frame_rev->usrData[40];//是SessionData1长度
	hplc_ScmEsam_Comm.DataTx_len=_698_frame_rev->usrData[40];
	my_strcpy(hplc_ScmEsam_Comm.Tx_data,_698_frame_rev->usrData,41,_698_frame_rev->usrData[40]);	
	
	
	//_698_frame_rev->usrData[41+_698_frame_rev->usrData[40]];//是ucOutSign长度
	hplc_ScmEsam_Comm.DataTx_len+=_698_frame_rev->usrData[41+_698_frame_rev->usrData[40]];

	my_strcpy(hplc_ScmEsam_Comm.Tx_data+_698_frame_rev->usrData[40],_698_frame_rev->usrData,(42+_698_frame_rev->usrData[40]),_698_frame_rev->usrData[41+_698_frame_rev->usrData[40]]);

	rt_kprintf("\n[hplc] hplc_ScmEsam_Comm.Tx_data \n",__func__);
	for(i=0;i<hplc_ScmEsam_Comm.DataTx_len;i++){
	
		rt_kprintf("%0x ",hplc_ScmEsam_Comm.Tx_data[i]);	
	}	
	rt_kprintf("over length=%d\n",hplc_ScmEsam_Comm.DataTx_len);	

	if(_698_frame_rev->addr.ca!=0){//
		hplc_current_ESAM_CMD=CON_KEY_AGREE;
		rt_kprintf("\n[hplc] addr.ca!=0 \n",__func__);
	}else{
		hplc_current_ESAM_CMD=HOST_KEY_AGREE;
		rt_kprintf("\n[hplc] addr.ca==0 \n",__func__);		
	}
	
	
	ESAM_Communicattion(hplc_current_ESAM_CMD,&hplc_ScmEsam_Comm);


	if(hplc_ScmEsam_Comm.DataRx_len<52){
		rt_kprintf("[hplc]  (%s)   result=%d hplc_ScmEsam_Comm.DataRx_len=%d\n",__func__,result,hplc_ScmEsam_Comm.DataRx_len);
		temp_char=0xff;//其他错误 （ 255）
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

		temp_char=0;//认证附加信息 SecurityData OPTIONAL
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
		 return -1;
	}else{
		temp_char=0;//认证结果  ConnectResult，
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

		temp_char=1;//认证附加信息 SecurityData OPTIONAL
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
		len=temp_char=(hplc_ScmEsam_Comm.Rx_data[2]*256+hplc_ScmEsam_Comm.Rx_data[3]-4);//+SessionData2	
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
		result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+4),len);
		
		temp_char=4;//MAC2
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
		result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+len+4),4);	

		
//		//未测试代码
//		if(priv_698_state->session_key_negotiation==1){
//			//基于这样的猜想，这是两个加密通道，根据不同的，而这两个通道没有区别，要是后来的那个要修改密钥，连个通道都被一个占了怎么办
//			priv_698_state->session_key_negotiation=0;
//		}else{
//			priv_698_state->session_key_negotiation=1;
//		}		
			
	}
	temp_char=0;//FollowReport
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0;//time_tag
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			


//	return -1;	
	
	hplc_data->_698_frame.usrData_len=hplc_data->dataSize-HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
	//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

	int FCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验

		
	temp_char=hplc_data->_698_frame.end=0x16;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么



//给长度结构体赋值,这里判断是不是需要分针
	hplc_data->priveData[len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

	hplc_data->priveData[len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	


//校验头
	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(hplc_data->priveData, HCS_position);
	hplc_data->_698_frame.HCS0=hplc_data->priveData[HCS_position];	
	hplc_data->_698_frame.HCS1=hplc_data->priveData[HCS_position+1];


	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(hplc_data->priveData, FCS_position);
	
	hplc_data->_698_frame.FCS0=hplc_data->priveData[FCS_position];
	hplc_data->_698_frame.FCS1=hplc_data->priveData[FCS_position+1];

	if(_698_frame_rev->addr.ca!=0){
		priv_698_state->addr.ca=_698_frame_rev->addr.ca;

	}




	return result;
	
}	
	



/*

函数作用：返回可用的data_tx

参数：size,返回组帧之后的帧长度。
注意：使用时，出去后，要将_698_frame.usrData指向hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);	
*/

int link_response_package(struct  _698_FRAME  *_698_frame_rev ,struct _698_FRAME  * _698_frame_send,struct _698_STATE  * priv_698_state,struct CharPointDataManage * data_tx){
	
	int i,len_position=0,HCS_position=0,result;
	struct _698_link_request hplc_698_link_request;
//	rt_kprintf("[hplc]  (%s)   link_response_package get inside \n",__func__); 	
//已定义静态变量   hplc_698_link_response
	
	unsigned char temp_char;//数组的不管了
  unPackage_698_link_request(_698_frame_rev,&hplc_698_link_request,&i);
	//结构体赋值，共同部分
  data_tx->dataSize=0;

	temp_char=_698_frame_send->head =_698_frame_rev->head;//起始帧头 = 0x68	
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);//这样打包好么,最后打包比较好


	
	len_position=data_tx->dataSize;
	data_tx->dataSize+=2;//加两字节的长度
	
	temp_char=_698_frame_send->control=CON_UTS_S|CON_LINK_MANAGE;   //控制域c,bit7,传输方向位
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);//这样打包好么

	
	temp_char=_698_frame_send->addr.sa=_698_frame_rev->addr.sa;
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);//这样打包好么


	//拷贝主机地址
	_698_frame_send->addr.s_addr_len=_698_frame_rev->addr.s_addr_len;
	//if(_698_frame_send->addr.s_addr==RT_NULL){//先释放
	//	_698_frame_send->addr.s_addr=(unsigned char *)rt_malloc(sizeof(unsigned char)*(_698_frame_send->addr.s_addr_len));//分配空间		
	//}

	my_strcpy(_698_frame_send->addr.s_addr,_698_frame_rev->addr.s_addr,0,_698_frame_send->addr.s_addr_len);//拷贝数组
	result=save_char_point_data(data_tx,data_tx->dataSize,_698_frame_send->addr.s_addr,_698_frame_send->addr.s_addr_len);//这样打包好么


	temp_char=_698_frame_send->addr.ca=_698_frame_rev->addr.ca;
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);

	HCS_position=data_tx->dataSize;
	data_tx->dataSize+=2;//加两字节的校验位



	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	_698_frame_send->usrData_len=0;//用户数据长度归零

	
	//if(_698_frame_send->usrData==RT_NULL){//先释放	
	//	_698_frame_send->usrData=(unsigned char *)rt_malloc(sizeof(unsigned char)*(1024));//给用户分配空间	
	//	_698_frame_rev->usrData_size=1024;//空间大小
	//}
	
	temp_char=hplc_698_link_response.type=link_response;//可以不用hplc_698_link_response,但为了表意方便还是用了，而且登录用不了多少时间，
																											//如果后期发现用的时间很多可以优化
	//temp_char=link_response;
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);
	
	temp_char=hplc_698_link_response.piid=hplc_698_link_request.piid_acd;
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);
	
	temp_char=hplc_698_link_response.result=0x80;//统统回复可信，成功。其他情况再说！！！！！
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);
	
	my_strcpy(hplc_698_link_response.date_time_ask.data,hplc_698_link_request.date_time.data,0,10);//10个数的请求时间
	result=save_char_point_data(data_tx,data_tx->dataSize,hplc_698_link_response.date_time_ask.data,10);//这样打包好么
		
	my_strcpy(hplc_698_link_response.date_time_rev.data,_698_frame_rev->rev_tx_frame_date_time.data,0,10);//10个数的获得帧的时间
	result=save_char_point_data(data_tx,data_tx->dataSize,hplc_698_link_response.date_time_rev.data,10);//这样打包好么
		
	get_current_time(hplc_698_link_response.date_time_response.data);
	result=save_char_point_data(data_tx,data_tx->dataSize,hplc_698_link_response.date_time_response.data,10);//这样打包好么
	rt_kprintf("[hplc]  (%s)  3link_response_package get inside _698_frame_rev->head=%0x data_tx->priveData[0]=%0x \n",__func__,_698_frame_rev->head,data_tx->priveData[0]);

	_698_frame_send->usrData_len=33;//用户数据总长度	,下面拷贝用户数据到usrData	
	//save_char_point_usrdata(_698_frame_send->usrData,&_698_frame_rev->usrData_size,data_tx->priveData,data_tx->dataSize-_698_frame_send->usrData_len,_698_frame_send->usrData_len);		


	int FCS_position=data_tx->dataSize;
	data_tx->dataSize+=2;//加两字节的校验

		
	temp_char=_698_frame_send->end=0x16;
	result=save_char_point_data(data_tx,data_tx->dataSize,&temp_char,1);//这样打包好么



//给长度结构体赋值,这里判断是不是需要分针
	data_tx->priveData[len_position]=_698_frame_send->length0=(data_tx->dataSize-2)%256;//data_tx->size<1024时

	data_tx->priveData[len_position+1]=_698_frame_send->length1=(data_tx->dataSize-2)/256;	


//校验头
	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(data_tx->priveData, HCS_position);
	_698_frame_send->HCS0=data_tx->priveData[HCS_position];	
	_698_frame_send->HCS1=data_tx->priveData[HCS_position+1];


	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(data_tx->priveData, FCS_position);
	
	_698_frame_send->FCS0=data_tx->priveData[FCS_position];
	_698_frame_send->FCS1=data_tx->priveData[FCS_position+1];
	
	return result;


/*	for(i=0;i<data_tx->dataSize;i++){
		rt_kprintf("[hplc]  (%s)  data_tx->priveData[%d]=%0x ",i,data_tx->priveData[i]); 
	
		if(i%4==0){
			rt_kprintf("[hplc]  (%s)   \n",__func__);
		}
		
	}
	rt_kprintf("[hplc]  (%s)   \n",__func__); 	
	
*/

}

/*
功能：对事务进行处理

要求：来到这里的必须是组帧完毕的


属于层：应用层，将数据给最上面的应用进程，应用进程组帧后就将数据发回给
参数：size,返回组帧之后的帧长度。

需要发送的返回0；

result==1，不需要发送
*/


int rev_698_del_affairs(struct _698_STATE  * priv_698_state,struct CharPointDataManage * data_tx,struct CharPointDataManage * data_rev){
	int i,result=0,usr_data_size=0;//不发送返回1,如果发送，result=0;
	unsigned char temp_char;
	int security_flag=0,needEsam_flag=0,position_data=0;
	data_rev->_698_frame.needEsam_flag=0;
	data_rev->_698_frame.security_flag=0;
	if(data_rev->_698_frame.usrData[0]==security_request){//是安全请求,需要&&已经密钥协商过了？似乎只给密钥下载用
		rt_kprintf("[hplc]  (%s)  security_request \n",__func__);
    //结构体赋值，共同部分
		if(priv_698_state->connect_flag==0){
			rt_kprintf("[hplc]  (%s)  not connect so can not ensecurity \n",__func__);
			return -1;
		}
		
		data_rev->_698_frame.security_flag=1;
		if(data_rev->_698_frame.usrData[1]==1){//密文

			data_rev->_698_frame.needEsam_flag=1;
			result=unpatch_ScmEsam_Comm(data_rev,&hplc_ScmEsam_Comm);
			
			hplc_current_ESAM_CMD=HOST_SESS_VERI_MAC;//主站会话解密验证MAC
			ESAM_Communicattion(hplc_current_ESAM_CMD,&hplc_ScmEsam_Comm);//spi会打印结果

			if(data_rev->_698_frame.usrData[2]!=(hplc_ScmEsam_Comm.Rx_data[2]*256+hplc_ScmEsam_Comm.Rx_data[3])){
				rt_kprintf("[hplc]  (%s)  usrData[2]!=(hplc_ScmEsam_Comm.Rx_data[2]*256 \n",__func__);		
			}

			data_rev->_698_frame.usrData_len=(hplc_ScmEsam_Comm.Rx_data[2]*256+hplc_ScmEsam_Comm.Rx_data[3]);
			rt_kprintf("[hplc]  (%s)  usrData_len=%d  Rx_data[2]=%d Rx_data[3]=%d\n",__func__,data_rev->_698_frame.usrData_len,hplc_ScmEsam_Comm.Rx_data[2],hplc_ScmEsam_Comm.Rx_data[3]);
			
			
			my_strcpy(data_rev->_698_frame.usrData,hplc_ScmEsam_Comm.Rx_data,4,data_rev->_698_frame.usrData_len);//	
			
			data_rev->_698_frame.needEsam_flag=1;
		}else if(data_rev->_698_frame.usrData[1]==0){//明文，是否需要进行mac验证

			if((data_rev->_698_frame.usrData_len==0x81)||(data_rev->_698_frame.usrData_len==0x82)){//超过7f，在长度前面会加0x81 抄255是82
				rt_kprintf("[hplc]  (%s) length more then 1024   \n",__func__);
				data_rev->_698_frame.usrData_len=data_rev->_698_frame.usrData[3]*256+data_rev->_698_frame.usrData[4];//data长度

				position_data=(4+1);//开始位置就是2+1=3   1是数据类型	
			}else{
				data_rev->_698_frame.usrData_len=data_rev->_698_frame.usrData[2];//data长度

				position_data=(2+1);//开始位置就是2+1=3   1是数据类型	
			}	
			//明文的还需要校验mac么
//			data_rev->_698_frame.usrData_len=(hplc_ScmEsam_Comm.Rx_data[2]*256+hplc_ScmEsam_Comm.Rx_data[1]);
			
			my_strcpy(data_rev->_698_frame.usrData,data_rev->_698_frame.usrData,position_data,data_rev->_698_frame.usrData_len);//		
		}

		rt_kprintf("[hplc]  (%s)  after unsecurity : \n",__func__);
		for(i=0;i<data_rev->_698_frame.usrData_len;i++){
			rt_kprintf("%0x ",data_rev->_698_frame.usrData[i]);
		}
		rt_kprintf("\n");	

	}

	switch (data_rev->_698_frame.usrData[0]){//来到这里的都必须是。
		case(link_request)://是服务器发出的，服务器不会收到		
			rt_kprintf("[hplc]  (%s)     link_request and do nothing \n",__func__);
			break;

		case(link_response)://服务器登录，客户机回应。是不是都是双向的都需要实现？？？？？
			rt_kprintf("[hplc]  (%s)  link_response  \n",__func__);
			if(data_rev->_698_frame.usrData[2]==0x80){//应答登录，应答可靠
				if(data_rev->_698_frame.usrData[2]==link_request_unload){	
					rt_kprintf("[hplc]  (%s)  link_request_unload \n",__func__);
					priv_698_state->link_flag=0;
					priv_698_state->connect_flag=0;//把链接也断开				
				}
				if(data_rev->_698_frame.usrData[2]==link_request_load){	
					rt_kprintf("[hplc]  (%s)  link_request_load \n",__func__);
					priv_698_state->link_flag=1;
					priv_698_state->connect_flag=1;//把链接置位				
				}				
				if(data_rev->_698_frame.usrData[2]==link_request_heart_beat){	
					rt_kprintf("[hplc]  (%s)  link_request_heart_beat and do nothing \n",__func__);										
				}								
			}else{
				rt_kprintf("[hplc]  (%s)  link_response result!=0x80  \n",__func__);
				return -1;
			}								
			break;		

		case(connect_request)://客户机发出请求，这里会收到
			rt_kprintf("[hplc]  (%s)  connect_request  \n",__func__);			
			result=connect_response_package(&data_rev->_698_frame,priv_698_state,data_tx);//应答connect_request
			if(result==0){
				priv_698_state->connect_flag=1;
			}else{
//				priv_698_state->connect_flag=0;//先不写这个
			}

			break;		

		case(connect_response)://服务器（电表）回应。永远都不会收到
			result=-1;
			rt_kprintf("[hplc]  (%s)  connect_response do nothing  \n",__func__);
			break;		

		case(release_request)://由客户机应用进程调用,收到后发送应答帧，置位断开连接标志，但不是退出登录
			rt_kprintf("[hplc]  (%s)  release_request \n",__func__);
			//result=release_response_package();
			result=-1;
			priv_698_state->connect_flag=0;
			break;				
		
		case(release_response)://本服务由服务器应用进程调用，
			result=-1;
			rt_kprintf("[hplc]  (%s)  release_response  do nothing \n",__func__);
			break;		


		case(release_notification):	//本服务由服务器应用进程调用，服务不需要客户机做任何响应，所以不会收到
			result=-1;
			rt_kprintf("[hplc]  (%s)  release_notification  do nothing \n",__func__);			
			break;		

		
		case(get_request)://由客户机应用进程调用
			rt_kprintf("[hplc]  (%s)  get_request \n",__func__);
			result=get_response_package(&data_rev->_698_frame,priv_698_state,data_tx);
			//返回时调用这个就可以了
			break;

		case(action_request)://由客户机应用进程调用,操作一个对象方法
			result=action_response_package(&data_rev->_698_frame,priv_698_state,data_tx);
			//返回时调用这个就可以了
			break;	

		case(action_response)://直接交给应用层就可以了，或者直接就不给
//			rt_kprintf("[hplc]  (%s)  action_request \n",__func__);
			result=action_response_notice_user(&data_rev->_698_frame,priv_698_state);

		
		
		case(report_response)://直接交给应用层就可以了，或者直接就不给
			rt_kprintf("[hplc]  (%s)  report_response priv_698_state->current_report=%d\n",__func__,priv_698_state->current_report);
//			result=report_response_notice_user(&data_rev->_698_frame,priv_698_state);
			strategy_event_send(priv_698_state->current_report);
			current_report_change(Cmd_Null);
			result=3;
			break;


		case(report_notification)://上报给我的只有离线或者在线事件//...........................
			rt_kprintf("[hplc]  (%s)  report_notification\n",__func__);
			result=report_response_package(&data_rev->_698_frame,priv_698_state,data_tx);
		
			//strategy_event_send(Cmd_STAOnlineState);//判断一下是否离线
		
			break;

		
		default:
			result=-1;
			rt_kprintf("[hplc]  (%s)   can not del the affair=%0x \n",__func__,data_rev->_698_frame.usrData[0]);
			break;		
	}
//	rt_kprintf("[hplc]  (%s)  error result=%d\n",__func__,result);//	
	
	
	if((data_rev->_698_frame.security_flag==1)&&(result==0)){//是安全请求,需要&&已经密钥协商过了？似乎只给密钥下载用
		//将用户数据整个地加密，然后重新打包
		rt_kprintf("[hplc]  (%s)  security_flag==1  \n",__func__);


		data_tx->dataSize=usr_data_size;//暂时没用
		result=security_get_package(data_rev->_698_frame.needEsam_flag,priv_698_state,data_tx);
		data_rev->_698_frame.security_flag=0;
		data_rev->_698_frame.needEsam_flag=0;

	}	
	
	return result;
}
/*
* 函数描述：Calculate a new fcs given the current fcs and the new data.
*/
unsigned short pppfcs16( unsigned short fcs, unsigned char *cp, int len){


	int i;
	while (len--){
		
		i=(fcs ^ *cp++) & 0xff;
		fcs=(fcs >> 8) ^ fcstab[i];
		/*if(len%10==0){
			rt_kprintf("[hplc]  (%s)  (fcs ^ *cp++) & 0xff %d  fcstab[(fcs ^ *cp++) & 0xff] %2x \n",__func__,i,fcstab[i]);
		
		}*/
	}
//	rt_kprintf("[hplc]  (%s)  fcs %2x PPPINITFCS16 %0x\n",__func__,fcs,PPPINITFCS16);
	return fcs;
}



/*
* How to use the fcs
需故需要在这里扩容,进来的都是临时文件才可以处理,
还是把策略都留在应用层比较好
*/
int tryfcs16(unsigned char *cp, int len){
	unsigned short trialfcs;
	// add on output 
	
	trialfcs=pppfcs16(PPPINITFCS16, cp+1,len-1);//不校验68
	trialfcs ^= 0xffff; // complement 
	cp[len]=(trialfcs & 0x00ff); // least significant byte first
	cp[len+1]=((trialfcs >> 8) & 0x00ff);
	//rt_kprintf("[hplc]  (%s)  cp[len]= %0x,cp[len+1]= %0x\n",__func__,cp[len],cp[len+1]);
	// check on input 
	trialfcs=pppfcs16(PPPINITFCS16, cp+1,len + 2-1);
	unsigned short tempcompare = PPPGOODFCS16;
	if ( trialfcs == tempcompare ){
		rt_kprintf("[hplc]  (%s)  Good FCS\n",__func__);
		return 0;
	}
	return -1;
	
}



/*
函数名：
函数参数：
返回值：

函数作用：
将结构体赋给static struct  _698_FRAME  _698_frame_rev
					调用响应函数,


*/
int _698_unPackage(unsigned char * data,struct  _698_FRAME  *_698_frame_rev,int size){
//	unsigned char * usrdata;
	rt_kprintf("[hplc]  (%s)  \n",__func__);
	//int i;
	//_698_frame_rev->control=data[3];//前面已经校验所以认为这一帧是对的
	_698_frame_rev->frame_apart=data[3]&CON_MORE_FRAME;//取分帧标志，先处理没有分针的,只标志分针，处理的在下一步

	//结构体赋值	
	_698_frame_rev->head = data[0];//起始帧头 = 0x68
	_698_frame_rev->length0=data[1];
	_698_frame_rev->length1=data[2];
	_698_frame_rev->length=((data[2]&0X3F)*256)+data[1];
	if(_698_frame_rev->length!=size-2){
		rt_kprintf("[hplc]  (%s)  _698_frame_rev->length!=size-2 \n",__func__);
		return -1;//长度解析不对
	}	
	//长度,两个字节,由bit0-bit13有效,是传输帧中除起始字符和结束字符之外的帧字节数。
	_698_frame_rev->control=data[3];   //控制域c,bit7,传输方向位
	
	_address_unPackage(data,&_698_frame_rev->addr,4);//地址域a
	_698_frame_rev->addr.s_addr_len=6;
	_698_frame_rev->HCS0=data[6+_698_frame_rev->addr.s_addr_len];
	_698_frame_rev->HCS1=data[7+_698_frame_rev->addr.s_addr_len];


	_698_frame_rev->usrData_len=size-11-_698_frame_rev->addr.s_addr_len;//计算用户数据长度

	if(_698_frame_rev->usrData_len<0||(_698_frame_rev->usrData_len>1024)){
		rt_kprintf("[hplc]  (%s)   _698_frame_rev->usrData_len<0 or >1024 \n",__func__);		
		return -1;//长度解析不对
	}else{
//		rt_kprintf("[hplc]  (%s)   _698_frame_rev->usrData_len=%d \n",__func__,_698_frame_rev->usrData_len);
	}

	//my_strcpy(_698_frame_rev->usrData,data,8+_698_frame_rev->addr.s_addr_len,_698_frame_rev->usrData_len);
	_698_frame_rev->usrData=data+(8+_698_frame_rev->addr.s_addr_len);
	
	_698_frame_rev->FCS0=data[size-3];
	_698_frame_rev->FCS1=data[size-2];
	_698_frame_rev->end=data[size-1];	
	
	return 0;
}
/*
	unsigned char sa;   //由bit0-bit3决定,对应1到16,就是需要加一    bit4-bit5是逻辑地址，bit6-bit是地址类型 
	unsigned char *s_add;//长度由sa决定
	unsigned char ca;//客户机地址
	int s_add_len;

*/
int _address_unPackage(unsigned char * data,struct _698_ADDR *_698_addr,int start_size){
//	unsigned char * s_addr;
	int i;
//	rt_kprintf("[hplc]  (%s)   \n",__func__);

	_698_addr->sa=data[start_size];	
	_698_addr->s_addr_len=(_698_addr->sa&0x0f)+1;

	if(_698_addr->s_addr_len<0||(_698_addr->s_addr_len>8)){
		rt_kprintf("[hplc]  (%s)   _698_addr->s_addr_len<0 or >8\n",__func__);		
		return -1;//长度解析不对
	}else{
		rt_kprintf("[hplc]  (%s)  s_addr_len=%d \n",__func__,_698_addr->s_addr_len);
	}
		
	for(i=0;i<_698_addr->s_addr_len;i++){
		_698_addr->s_addr[i]=data[start_size+1+i];
	}	
	_698_addr->ca=data[start_size+1+_698_addr->s_addr_len];
	return 0;	
}





/*

发送完后至少要33位的空闲间隔

*/
int _698_package(unsigned char * data,int size){

	return 0;
}
/*


*/

int _698_HCS(unsigned char *data, int start_size,int size,unsigned short HCS){

//	int i;
	unsigned short trialfcs;
	if (start_size<0){
		return -1;	
	}
	trialfcs=pppfcs16( PPPINITFCS16,data+start_size,size );
	unsigned short tempcompare = PPPGOODFCS16;
	if ( trialfcs == tempcompare ){
		rt_kprintf("[hplc]  (%s)  Good HCS\n",__func__);
		return 0;
	}else{
		rt_kprintf("[hplc]  (%s)  error HCS\n",__func__);
		return -1;
	}
}

/*
参数：给数据,并给,数据起点和字节长度,校验值默认是领

是对帧头部分除起始字符和HCS本身之外的所有字节的校验,start_size应该恒等于1
*/

int _698_FCS(unsigned char *data, int start_size,int size,unsigned short FCS){

	//int i;
	unsigned short trialfcs;
	if (start_size<0){
		return -1;	
	}
	trialfcs=pppfcs16( PPPINITFCS16,data+start_size,size );

	//rt_kprintf("[hplc]  (%s)  size %d  data[size-2]= %0x,data[len+1]= %0x\n",__func__,size,data[size-2],data[size-1]);	
	unsigned short tempcompare = PPPGOODFCS16;
	if ( trialfcs == tempcompare ){
		rt_kprintf("[hplc]  (%s)  Good FCS\n",__func__);
		return 0;
	}else{
		rt_kprintf("[hplc]  (%s)  error FCS \n",__func__);
		return -2;
	}
}


/*
	安全性由应用层负责。
*/
int my_strcpy_char(char *dst,char *src,int startSize,int size){
	for(int i=0;i<size;i++){
		dst[i]=src[startSize+i];		
		//if(startSize==0){//测试用
		//	rt_kprintf("[hplc]  (%s)  dst[%d]= %0x src[%d]= %0x\n",__func__,i,dst[i],startSize+i,src[startSize+i]);	
		//}
	}				
	return 0;
}
int my_strcpy(unsigned char *dst,unsigned char *src,int startSize,int size){
	for(int i=0;i<size;i++){
		dst[i]=src[startSize+i];		
		//if(startSize==0){//测试用
		//	rt_kprintf("[hplc]  (%s)  dst[%d]= %0x src[%d]= %0x\n",__func__,i,dst[i],startSize+i,src[startSize+i]);	
		//}
	}				
	return 0;
}


/*

说明辅助函数,用于数组扩容,默认增容1024
*/

int array_inflate(unsigned char *data, int size,int more_size){
//	int tempSzie=0;
//	int i;
	return -1;
/*	unsigned char *p=RT_NULL; 	
	if(more_size<0){		
		rt_kprintf("[hplc]  (%s)  Hello RT-Thread! \n",__func__);
		return -1;
	}else if(more_size==0){
		more_size=1024;	
	}
	
	tempSzie=size + more_size;
	rt_kprintf("[hplc]  (%s)  size= %d tempSzie= %d\n",__func__,size,tempSzie);	

	p=(unsigned char *)rt_malloc(sizeof(unsigned char)*(tempSzie));

	//rt_memset(p,0,tempSzie);//清空		
  if(size>0){
		for (i = 0; i < size; i++)
		{
			p[i] = data[i];
			//rt_kprintf("[hplc]  (%s)  data[%d] %0x\n",__func__,i,data[i]);
		}	
	
	}

	
	//rt_kprintf("[hplc]  (%s)  array_inflate will to  rt_free size=%d \n",__func__,size);
	if((data!=RT_NULL)&&(size!=0)){
		rt_free(data);	
	}
	rt_kprintf("[hplc]  (%s)  array_inflate over  rt_free size=%d \n",__func__,size);
	data = (unsigned char *)rt_malloc(sizeof(unsigned char)*(tempSzie));// data = p这样不行，可能p是因为是局域指针的原因
	if(p!=RT_NULL){
		rt_free(p);	
	}
	if(size>0){
		for (i = 0; i < size; i++)
		{
			 data[i]= p[i];
		}		
	}		
	if(p!=RT_NULL){
		rt_free(p);	
	}
	return -1;*/	
}
/*

说明辅助函数,用于数组兼容
*/

int array_deflate(unsigned char *data, int size,int de_size){
//	int tempSzie=0;
//	int i;
	return -1;
/*	
	if((size < de_size)||(de_size<0)){		
		rt_kprintf("[hplc]  (%s)  array_deflate  size < de_size\n",__func__);
		return -1;
	}else if((de_size==0)&&(size > 1024)){//减到初始大小
		size=1024;
	}
	tempSzie=size - de_size;
	
	
	unsigned char *p = (unsigned char *)rt_malloc(sizeof(unsigned char)*(tempSzie));

	for (i = 0; i < tempSzie; i++)
	{
		p[i] = data[i];
	}
	//array_free(a);
	rt_free(data);
	data = p;
	return -1;	*/
}


/*



*/

int save_meter_no_backward(struct CharPointDataManage *hplc_data,int position,unsigned char *Res,int size){
	int i=0;
	int result=0;	
	if(size<0){
		return -1;
	
	}	
	//rt_kprintf("[hplc]  (%s)  result=save_char_point_data  get inside hplc_data->size=%d !\n",__func__,hplc_data->size);
	
	if(hplc_data->size > (position+size)){//空间还有
		
		for(i=0;i<size;i++){
			hplc_data->priveData[position+i]=Res[size-1-i];
			
		}		
		hplc_data->dataSize+=size;//不一定等于position,用户多帧响应的情况
		//rt_kprintf("[hplc]  (%s)  result=save_char_point_data  hplc_data->dataSize = %d!\n",__func__,hplc_data->dataSize);
	}else{
		rt_kprintf("[hplc]  (%s)  hplc_data->size=%d < !position+size %d  and return -1\n",__func__,hplc_data->size,position+size);		
		return -1;

	}		
	
	return result;	
}


/*



*/

int save_char_point_data(struct CharPointDataManage *hplc_data,int position,unsigned char *Res,int size){
	int i=0;
	if(size<0){
		return -1;
	
	}	
	//rt_kprintf("[hplc]  (%s)  result=save_char_point_data  get inside hplc_data->size=%d !\n",__func__,hplc_data->size);
	
	if(hplc_data->size > (position+size)){//空间还有
		for(i=0;i<size;i++){
			hplc_data->priveData[position+i]=Res[i];
			
		}
		
		hplc_data->dataSize+=size;//不一定等于position,用户多帧响应的情况
		//rt_kprintf("[hplc]  (%s)  result=save_char_point_data  hplc_data->dataSize = %d!\n",__func__,hplc_data->dataSize);
	}else{
		rt_kprintf("[hplc]  (%s)  hplc_data->size=%d < !position+size %d  and return -1\n",__func__,hplc_data->size,position+size);		
		return -1;
/*		if(size<=1){
			if(array_inflate(hplc_data->priveData,hplc_data->size,1024)==0){
				hplc_data->size+=1024;
				rt_kprintf("[hplc]  (%s)  add 1024 size for hplc_data->size = %d!\n",__func__,hplc_data->size);
			}else{
				//rt_kprintf("[hplc]  (%s)  add 1024 size for hplc_data->size = %d   faild!\n",__func__,hplc_data->size);
				return -1;
			}		
		}else {
			if(array_inflate(hplc_data->priveData,hplc_data->size,size)==0){
				hplc_data->size+=size;
			}else{
				return -1;
			}						
		}		
		for(i=0;i<size;i++){
			hplc_data->priveData[position+i]=Res[i];	
			rt_kprintf("[hplc]  (%s)  hplc_data->priveData[position+i]=%0x Res[i]=%0x!\n",__func__,hplc_data->priveData[position+i],Res[i]);	
		}		
		hplc_data->dataSize+=size;//不一定等于position,用户多帧响应的情况			
*/
	}		
	
	return 0;	
}

/*


*/
int copy_ChgPlanIssue_rsp(CHARGE_STRATEGY_RSP *des,CHARGE_STRATEGY_RSP *src){
	my_strcpy_char(des->cRequestNO,src->cRequestNO,0,17);	
	my_strcpy_char(des->cAssetNO,src->cAssetNO,0,23);	//路由器资产编号  visible-string（SIZE(22)）
	des->cSucIdle=src->cSucIdle;
	return 0;
}

/*以防有用*/
int copy_charge_strategy(CHARGE_STRATEGY *des,CHARGE_STRATEGY *src){
//	int i;
	my_strcpy_char(des->cRequestNO,src->cRequestNO,0,17);
	//*des->cRequestNO=*src->cRequestNO;
	my_strcpy_char(des->cUserID,src->cUserID,0,65);
	//*des->cUserID=*src->cUserID;
	des->ucDecMaker=src->ucDecMaker;				//决策者  {主站（1）、控制器（2）}
	des->ucDecType=src->ucDecType; 					//决策类型{生成（1） 、调整（2）}
	STR_SYSTEM_TIME strDecTime;							//决策时间
	des->strDecTime=src->strDecTime;
	
	my_strcpy_char(des->cAssetNO,src->cAssetNO,0,23);	//路由器资产编号  visible-string（SIZE(22)）
	//*des->cAssetNO=*src->cAssetNO;
	des->ulChargeReqEle=src->ulChargeReqEle;					//充电需求电量（单位：kWh，换算：-2）
	des->ulChargeRatePow=src->ulChargeRatePow;				//充电额定功率 （单位：kW，换算：-4）
	des->ucChargeMode=src->ucChargeMode;							//充电模式 {正常（0），有序（1）}
	des->ucTimeSlotNum=src->ucTimeSlotNum;						//时间段数量
	//CHARGE_TIMESOLT strChargeTimeSolts[50];					//时间段内容，最大50段
	*des->strChargeTimeSolts=*src->strChargeTimeSolts;//是否可以用，待考证

	return 0;
}


/*
目标是从0开始的

*/


rt_uint8_t strategy_event_send(COMM_CMD_C cmd){
//	rt_uint32_t event;
//锁资源
	while(hplc_lock1==1){
		rt_kprintf("[hplc]  (%s)   lock1==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_lock1=1;
	while(hplc_lock2==1){
		rt_kprintf("[hplc]  (%s)   lock2==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_lock2=1;	
//  if(cmd<32){		
	rt_kprintf("[hplc]  (%s)   cmd=%d  \n",__func__,cmd);
	if(cmd>0)
		strategyEvent[0]|=(0x00000001<<cmd);
//}else{
//	strategyEvent[1]|=(0x00000001<<(cmd-32));

//}

	hplc_lock2=0;
	hplc_lock1=0;		
	return 0;
}

/*


*/

rt_uint32_t my_strategy_event_get(void){
	rt_uint32_t result=CTRL_NO_EVENT;
//	rt_uint32_t event;
//锁资源
	while(hplc_lock1==1){
		rt_kprintf("[hplc]  (%s)   lock1==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_lock1=1;
	while(hplc_lock2==1){
		rt_kprintf("[hplc]  (%s)   lock2==1  \n",__func__);		
		rt_thread_mdelay(20);
	}	
	hplc_lock2=1;	
//	rt_kprintf("[hplc]  (%s)   cmd=%d  \n",__func__,cmd);		

	result=strategyEvent[0];
	
	hplc_lock2=0;
	hplc_lock1=0;		
	return result;
}

int current_report_change(COMM_CMD_C cmd){
	rt_kprintf("[hplc]  (%s)   cmd==%x  \n",__func__,cmd);
	while(hplc_698_state.lock1==1){
		rt_kprintf("[hplc]  (%s)   lock1==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_698_state.lock1=1;
	while(hplc_698_state.lock2==1){
		rt_kprintf("[hplc]  (%s)   lock2==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_698_state.lock2=1;	
	
	hplc_698_state.current_report=cmd;//	
	
	
	hplc_698_state.lock2=0;	
	hplc_698_state.lock1=0;	
	return 0;	
}




/**
	根据命令
	1:将用户的数据拷贝过来，并置位事件

	2:或者将用户层来的数拷贝过来。


	注意：在解锁之前不能return。

*/

unsigned char * frome_user_tx;
rt_uint8_t CtrlUnit_RecResp(COMM_CMD_C cmd,void *STR_SetPara,int count){
	rt_uint8_t result=0;
	rt_uint32_t event,priv_event;
	//frome_user_tx=STR_SetPara;//每种指令长度一定？
	CHARGE_STRATEGY *prive_struct;
	CHARGE_STRATEGY_RSP * prive_struct_RSP;
	CHARGE_EXE_EVENT * prive_struct_EXE_STATE;
//锁资源
	while(hplc_698_state.lock1==1){
		rt_kprintf("[hplc]  (%s)   lock1==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_698_state.lock1=1;
	while(hplc_698_state.lock2==1){
		rt_kprintf("[hplc]  (%s)   lock2==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_698_state.lock2=1;	
	
	event=0x00000001<<cmd;
	
	
	if(count>=0){
		switch(cmd){	//可加策略
		
		case(Cmd_PowerAdj):	//将功率调节参数传给用户
			rt_kprintf("[hplc]  (%s)  Cmd_PowerAdj  \n",__func__);	
			*((CTL_CHARGE *)STR_SetPara)=CTL_CHARGE_Adj;								
			break;

		case(Cmd_PowerAdjAck)://功率调节应答,似乎没有参数
			rt_kprintf("[hplc]  (%s)   Cmd_PowerAdjAck  \n",__func__);
			CTL_CHARGE_Adj=*((CTL_CHARGE *)STR_SetPara);//我传给用户和用户传给我都用这一个CTL_CHARGE_Adj
			hplc_event=hplc_event|event;						
			break;				

		
		case(Cmd_StartChg)://启动充电参数下发,无参数传递
			rt_kprintf("[hplc]  (%s)   Cmd_StartChg  \n",__func__);
			*((CTL_CHARGE *)STR_SetPara)=CTL_CHARGE_START;
			break;
		
		case(Cmd_StartChgAck)://启动充电应答，应当无参数		
			rt_kprintf("[hplc]  (%s)   Cmd_StartChgAck cmd=%d \n",__func__,cmd);
			CTL_CHARGE_START=*((CTL_CHARGE *)STR_SetPara);
			hplc_event=hplc_event|event;
			result=0;											
			break;
		
		case(Cmd_StopChg)://停止充电参数下发	
			*((CTL_CHARGE *)STR_SetPara)=CTL_CHARGE_STOP;
			rt_kprintf("[hplc]  (%s)   Cmd_StopChg  \n",__func__);								
			break;
		
		case(Cmd_StopChgAck)://停止充电应答		
			rt_kprintf("[hplc]  (%s)   Cmd_StopChgAck   cmd=%d \n",__func__,cmd);
			CTL_CHARGE_STOP=*((CTL_CHARGE *)STR_SetPara);
			hplc_event=hplc_event|event;			
			break;		

		
		case(Cmd_ChgPlanIssue):	//将计划单传给用户
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanIssue  \n",__func__);	
			*((CHARGE_STRATEGY *)STR_SetPara)=charge_strategy_ChgPlanIssue;
		
			//拷贝给他,指针结构体直接赋值不成功！？									
			break;

		case(Cmd_ChgPlanIssueAck)://计划单完成，返回应答帧
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanIssueAck  \n",__func__);
			ChgPlanIssue_rsp=*((CHARGE_STRATEGY_RSP *)STR_SetPara);//可以用
			hplc_event=hplc_event|event;						
			break;				
	


		
		
 		case(Cmd_STAOnlineState)://STA监测自身及路由器在线状态
			rt_kprintf("[hplc]  (%s)   Cmd_STAOnlineState  \n",__func__);
			*((ONLINE_STATE *)STR_SetPara)=ONLINE_STATE_hplc;//拷贝给他
			break; 		


		case(Cmd_STAOnlineStateAck)://
			rt_kprintf("[hplc]  (%s) Cmd_STAOnlineStateAck  \n",__func__);	
			hplc_event=hplc_event|event;			
			break;
		



		
	
		
 		case(Cmd_ChgPlanAdjust)://变更充电计划,应用层得到数据，处理完后才下一步
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanAdjust  \n",__func__);
			*((CHARGE_STRATEGY *)STR_SetPara)=charge_strategy_ChgPlanAdjust;//拷贝给他

			break; 		


		case(Cmd_ChgPlanAdjustAck)://变更充电计划,以处理完
			rt_kprintf("[hplc]  (%s)  Cmd_ChgPlanAdjustAck  \n",__func__);
			prive_struct_RSP=(CHARGE_STRATEGY_RSP *)STR_SetPara;
			copy_ChgPlanIssue_rsp(&ChgPlanAdjust_rsp,prive_struct_RSP);		
			hplc_event=hplc_event|event;			
			break;


		

		
		case(Cmd_ChgPlanIssueGetAck)://不需要从我这里要数据，只执行就可以了
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanIssueGetAck  \n",__func__);		
			_698_charge_strategy.charge_strategy=(CHARGE_STRATEGY *)STR_SetPara;	
			//这个会有问题，如果变了，但是这个是我网上招的数据，我不动他也应该不动
			_698_charge_strategy.array_size=count;
			hplc_event=hplc_event|event;
			//是否还要判断是否运行成功，成功了之后才推出。
			result=0;	
			break;
		case(Cmd_RouterExeState)://路由器执行状态查询		
			rt_kprintf("[hplc]  (%s)   Cmd_RouterExeState  -1\n",__func__);		
			result=-1;				
			break;
		
	
	case(Cmd_RouterExeStateAck)://路由器执行状态应答	
			_698_charge_exe_state.charge_exe_state=(CHARGE_EXE_STATE *)STR_SetPara;
			rt_kprintf("[hplc]  (%s)  Cmd_RouterExeStateAck %0x \n",__func__,_698_charge_exe_state.charge_exe_state->exeState);	
			//这个会有问题，如果变了，但是这个是我往上招的数据，我不动他也应该不动
			_698_charge_exe_state.array_size=count;
//				_698_charge_exe_state.array_size=1;
				hplc_event=hplc_event|event;
			//是否还要判断是否运行成功，成功了之后才推出。
			break;				
		
/*
	***
	上送类事件		
	***
*/
		case(Cmd_ChgPlanExeState)://上送充电计划执行状态
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanExeState  \n",__func__);	
			hplc_698_state.current_report=Cmd_ChgPlanExeStateAck;
			hplc_CHARGE_EXE_EVENT=*((CHARGE_EXE_EVENT *)STR_SetPara);//可能赋值不上
			hplc_event=hplc_event|event;			
							
			break;
		
		
		
		case(Cmd_ChgPlanExeStateAck)://上送充电计划执行状态			
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanExeStateAck \n",__func__);								
			break;	

		case(Cmd_ChgRecord)://上送充电订单
			rt_kprintf("[hplc]  (%s)   Cmd_ChgRecord  \n",__func__);						
			hplc_698_state.current_report=Cmd_ChgRecordAck;			
			hplc_CHG_ORDER_EVENT=*((CHG_ORDER_EVENT *)STR_SetPara);//可能赋值不上
			hplc_event=hplc_event|event;	//测试		
					
		break;
		
		case(Cmd_ChgRecordAck)://上送充电订单				
			rt_kprintf("[hplc]  (%s)   Cmd_ChgRecordAck  \n",__func__);								
		break;			
				
	
	case(Cmd_ChgRequestReport)://充电申请事件上送
			rt_kprintf("[hplc]  (%s)   Cmd_ChgRequestReport  \n",__func__);		
			hplc_698_state.current_report=Cmd_ChgRequestReportAck;
			hplc_CHARGE_APPLY_EVENT=*((CHARGE_APPLY_EVENT *)STR_SetPara);
			hplc_event=hplc_event|event;
			break;		

	
	case(Cmd_ChgRequestReportAck):
			rt_kprintf("[hplc]  (%s)   Cmd_ChgRequestReportAck  \n",__func__);
			break;	
	
	case(Cmd_ChgPlanOffer)://上送充电计划单
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanOffer  \n",__func__);			
			hplc_698_state.current_report=Cmd_ChgPlanOfferAck;
			hplc_PLAN_OFFER_EVENT=*((PLAN_OFFER_EVENT *)STR_SetPara);		
			hplc_event=hplc_event|event;	//测试	
							
			break;		

	
	
		case(Cmd_DeviceFault)://上送路由器异常状态 
			rt_kprintf("[hplc]  (%s)   Cmd_DeviceFault  \n",__func__);
			hplc_698_state.current_report=Cmd_DeviceFaultAck;//少个应答
			_698_router_fail_event=*((ROUTER_FAULT_EVENT *)STR_SetPara);//不一定成功	
//			_698_router_fail_event.array_size=count;
			hplc_event=hplc_event|event;
			break;	

		
		case(Cmd_PileFault)://上送充电桩异常状态 
			rt_kprintf("[hplc]  (%s)   Cmd_PileFault  \n",__func__);
			hplc_698_state.current_report=Cmd_PileFaultAck;//少个应答		
			_698_pile_fail_event=*((ROUTER_FAULT_EVENT *)STR_SetPara);//不一定成功	
//			_698_pile_fail_event.array_size=count;
			hplc_event=hplc_event|event;
			break;	
	
	
	
	
	case(Cmd_ChgPlanOfferAck):
			rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanOfferAck  \n",__func__);								
			break;
	
		default:
			rt_kprintf("[hplc]  (%s)   not support \n",__func__);
			result=1;
			break;	
	}

	
	
	
	}
//  if(cmd<32){	


	strategyEvent[0]&=(~event);
	rt_kprintf("[hplc]  (%s)   event0=0x%4x  \n",__func__,event);
//	}else{
//		event=0x00000001<<(cmd-32);
//		strategyEvent[1]&=(~event);	
//		rt_kprintf("[hplc]  (%s)   event1=0x%4x  \n",__func__,event);

//	}

//解锁
	hplc_698_state.lock2=0;	
	hplc_698_state.lock1=0;	
	return result;

}


/*
回复全加密

*/
int check_afair_from_botom(struct _698_STATE  * priv_698_state,struct CharPointDataManage *data_tx){
	
	int result=0;
	int time_past;
//	rt_kprintf("[hplc]  (%s)  \n",__func__);
	while(hplc_698_state.lock1==1){
		rt_kprintf("[hplc]  (%s)   lock1==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_698_state.lock1=1;
	while(hplc_698_state.lock2==1){
		rt_kprintf("[hplc]  (%s)   lock2==1  \n",__func__);
		rt_thread_mdelay(20);
	}
	hplc_698_state.lock2=1;

	if((0)&&(hplc_698_state.report_esam_f==0)){
		
	
		time_past=((bcd_to_hex(System_Time_STR.Hour)*60)+bcd_to_hex(System_Time_STR.Minute))-hplc_698_state.time;//上来就发一次
		if(System_Time_STR.Hour==0){
			rt_kprintf("[hplc]  (%s)  Hour==0  \n",__func__);
			time_past=bcd_to_hex(System_Time_STR.Minute)+hplc_698_state.time-23*60;
			hplc_698_state.time=bcd_to_hex(System_Time_STR.Minute);			
		}

		rt_kprintf("[hplc]  (%s)  time_past=%d time= %d\n",__func__,time_past,hplc_698_state.time);
		if(time_past>=5){//调发送时间
			rt_kprintf("[hplc]  (%s)  time_past=%d >=40 \n",__func__,time_past);				
			time_past=0;
			hplc_698_state.time=((bcd_to_hex(System_Time_STR.Hour)*60)+bcd_to_hex(System_Time_STR.Minute));
		}
	}



	if(hplc_event&(0x1<<Cmd_PowerAdjAck)){	//功率调节应答	
		hplc_event&=(~(0x1<<Cmd_PowerAdjAck));	
		rt_kprintf("[hplc]  (%s)   Cmd_PowerAdjAck  \n",__func__);
		_698_CTL_CHARGE_Adj.need_package=1;
		//用小周的来填充我的_698_ChgPlanIssue，然后进行组应答帧		
		result=action_response_package(&_698_CTL_CHARGE_Adj,priv_698_state,data_tx);//发送

		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			if(Cmd_security_package(&_698_CTL_CHARGE_Adj,priv_698_state,data_tx)==0){
				hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
				printmy(&data_tx->_698_frame);
			}		
		}		
	}		
	
	
	

	if(hplc_event&(0x1<<Cmd_ChgPlanIssueAck)){	//充电计划下发应答	
		hplc_event&=(~(0x1<<Cmd_ChgPlanIssueAck));	
		rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanIssueAck  \n",__func__);
		_698_ChgPlanIssue.need_package=1;
		//用小周的来填充我的_698_ChgPlanIssue，然后进行组应答帧		
		result=action_response_package(&_698_ChgPlanIssue,priv_698_state,data_tx);//发送

		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			if(Cmd_security_package(&_698_ChgPlanIssue,priv_698_state,data_tx)==0){
				hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
				printmy(&data_tx->_698_frame);
			}		
		}		
	}		

	if(hplc_event&(0x1<<Cmd_ChgPlanAdjustAck)){	//充电计划调整应答	
		hplc_event&=(~(0x1<<Cmd_ChgPlanAdjustAck));		
		rt_kprintf("[hplc]  (%s)  Cmd_ChgPlanAdjustAck  \n",__func__);	
		_698_ChgPlanAdjust.need_package=1;
		result=action_response_package(&_698_ChgPlanAdjust,priv_698_state,data_tx);//发送	
		_698_ChgPlanAdjust.need_package=0;
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			if(Cmd_security_package(&_698_ChgPlanAdjust,priv_698_state,data_tx)==0){
				hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
				//printmy(&data_tx->_698_frame);
			}			
		}		
	}	
	
	
	if(hplc_event&(0x1<<Cmd_STAOnlineStateAck)){	//STA监测自身及路由器在线状态确认	
		hplc_event&=(~(0x1<<Cmd_STAOnlineStateAck));		
		rt_kprintf("[hplc]  (%s)  Cmd_STAOnlineStateAck  \n",__func__);	
		_698_ONLINE_STATE.need_package=1;
		result=report_response_package(&_698_ONLINE_STATE,priv_698_state,data_tx);//发送	
		_698_ONLINE_STATE.need_package=0;
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
			printmy(&data_tx->_698_frame);
		}		
	}		
	
	
	
	if(hplc_event&(0x1<<Cmd_ChgPlanIssueGetAck)){	
		hplc_event&=(~(0x1<<Cmd_ChgPlanIssueGetAck));		
		rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanIssueGetAck  \n",__func__);	
		_698_ChgPlanIssueGet.need_package=1;
		result=get_response_package(&_698_ChgPlanIssueGet,priv_698_state,data_tx);//发送
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			rt_kprintf("[hplc]  (%s)  print data_tx:\n",__func__);
			
			if(Cmd_security_package(&_698_ChgPlanIssueGet,priv_698_state,data_tx)==0){
				hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
				//printmy(&data_tx->_698_frame);
			}			
			
		}
	}	
	
	
	
	if(hplc_event&(0x1<<Cmd_RouterExeStateAck)){////路由器执行状态应答
		hplc_event&=(~(0x1<<Cmd_RouterExeStateAck));		
		rt_kprintf("[hplc]  (%s)   Cmd_RouterExeStateAck  \n",__func__);	
		_698_RouterExeState.need_package=1;
		
		result=get_response_package(&_698_RouterExeState,priv_698_state,data_tx);//发送
//		printmy(&data_tx->_698_frame);
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况	
			
			if(Cmd_security_package(&_698_RouterExeState,priv_698_state,data_tx)==0){
				rt_kprintf("[hplc]  (%s)  print data_tx:\n",__func__);
				hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据
				printmy(&data_tx->_698_frame);
			}
		}
	}		
	

	if(hplc_event&(0x1<<Cmd_StartChgAck)){	//启动充电应答
		hplc_event&=(~(0x1<<Cmd_StartChgAck));	
		rt_kprintf("[hplc]  (%s)   Cmd_StartChgAck  \n",__func__);

		_698_StartChg.need_package=1;
		result=action_response_package(&_698_StartChg,priv_698_state,data_tx);//发送
		rt_kprintf("[hplc]  (%s)    not security \n",__func__);//	
		printmy(&data_tx->_698_frame);
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			rt_kprintf("[hplc]  (%s)    ok \n",__func__);
			if(Cmd_security_package(&_698_StartChg,priv_698_state,data_tx)==0){
				printmy(&data_tx->_698_frame);
				hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
				
			}
		}		
	}

	if(hplc_event&(0x1<<Cmd_StopChgAck)){	//停止充电应答
		hplc_event&=(~(0x1<<Cmd_StopChgAck));	
		rt_kprintf("[hplc]  (%s)   Cmd_StopChgAck  \n",__func__);	
		_698_StopChg.need_package=1;
		result=action_response_package(&_698_StopChg,priv_698_state,data_tx);//发送

		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况

			if(Cmd_security_package(&_698_StopChg,priv_698_state,data_tx)==0){
					
				printmy(&data_tx->_698_frame);
				hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
				
			}else{
				rt_kprintf("[hplc]  (%s)    data_tx!=0 \n",__func__);//
			
			}
		}				
	}		
	
	
	if(hplc_event&(0x1<<Cmd_ChgRequestReport)){	//转发充电申请		
		hplc_event&=(~(0x1<<Cmd_ChgRequestReport));	//千万别忘了清理事件	
		rt_kprintf("[hplc]  (%s)   Cmd_ChgRequestReport  \n",__func__);
		result=report_notification_package(Cmd_ChgRequestReport,&hplc_CHARGE_APPLY_EVENT,data_tx,priv_698_state);

		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			printmy(&data_tx->_698_frame);
			hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
		}				
	}	



	if(hplc_event&(0x1<<Cmd_ChgPlanOffer)){	//充电计划事件上报	
		hplc_event&=(~(0x1<<Cmd_ChgPlanOffer));		
		rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanOffer  \n",__func__);	
		result=report_notification_package(Cmd_ChgPlanOffer,&hplc_PLAN_OFFER_EVENT,data_tx,priv_698_state);

		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{
			printmy(&data_tx->_698_frame);
			hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
		}				
	}		

	if(hplc_event&(0x1<<Cmd_ChgPlanExeState)){	//充电计划执行状态事件上报
		hplc_event&=(~(0x1<<Cmd_ChgPlanExeState));
		rt_kprintf("[hplc]  (%s)   Cmd_ChgPlanExeState  \n",__func__);	
		result=report_notification_package(Cmd_ChgPlanExeState,&hplc_CHARGE_EXE_EVENT,data_tx,priv_698_state);		
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			//rt_kprintf("[hplc]  (%s)  print data_tx:\n",__func__);	
			printmy(&data_tx->_698_frame);
			hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
		}													
	}
		
	if(hplc_event&(0x1<<Cmd_ChgRecord)){	//上送充电订单 REPORT  只有上报若干个对象属性，和上报若干个记录型对象属性两种
		hplc_event&=(~(0x1<<Cmd_ChgRecord));
		rt_kprintf("[hplc]  (%s)   Cmd_ChgRecord  \n",__func__);		
			result=report_notification_package(Cmd_ChgRecord,&hplc_CHG_ORDER_EVENT,data_tx,priv_698_state);		
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			//rt_kprintf("[hplc]  (%s)  print data_tx:\n",__func__);
			printmy(&data_tx->_698_frame);
			hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
		}				
	}	

	
	if(hplc_event&(0x1<<Cmd_DeviceFault)){	//上送路由器异常状态
		hplc_event&=(~(0x1<<Cmd_DeviceFault));
		rt_kprintf("[hplc]  (%s)   Cmd_DeviceFault  \n",__func__);	
		result=report_notification_package(Cmd_DeviceFault,&_698_router_fail_event,data_tx,priv_698_state);		
		
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			rt_kprintf("[hplc]  (%s)  print data_tx:\n",__func__);
			printmy(&data_tx->_698_frame);			
			result=hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	
		}					
	}	
	
	
	if(hplc_event&(0x1<<Cmd_PileFault)){	//上送充电桩异常状态
		hplc_event&=(~(0x1<<Cmd_PileFault));		
		rt_kprintf("[hplc]  (%s)   Cmd_DeviceFault  \n",__func__);
		report_notification_package(Cmd_PileFault,&_698_pile_fail_event,data_tx,priv_698_state);		
		if( result!=0){
				rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
		}else{//下面是需要回复的情况
			rt_kprintf("[hplc]  (%s)  print data_tx:\n",__func__);
			printmy(&data_tx->_698_frame);
			hplc_tx_frame(priv_698_state,hplc_serial,data_tx);//发送数据	

		}				
	}				
	hplc_698_state.lock2=0;
	hplc_698_state.lock1=0;

	return result;		
}



/*
 发送，调用发送单个帧，应答组帧时是不是需要等待hplc回复？按道理不要，是不是就没有组帧的需要，组帧还需要一发一确认么？


*/
int hplc_tx(struct CharPointDataManage *hplc_data){
	int result=0;
	//unsigned char rel_time[10];//9位时间，加一位，应该是时间差
	
	//    发送字符串 
  //rt_device_write(serial, 0, str, (sizeof(str) - 1));
	//获取实时时间,并计算时间差
	//开启发送的线程，不影响正常接收? 还是阻塞发送? 阻塞发送比较好实现，并且在没法发送情况下，收到也没用，不如放在缓存里，但是长帧怎么办
	//单一定要判断是不是工作着，因为有可能要转发其他用户的帧，这个指的是其他用户用hplc口发送数据，不是其他用户调用接口函数来组帧。
	//打包数据，并发送函数，发送完后，进行下一步，或者监听
	
	return result;	
}


int save_char_point_usrdata(unsigned char *data,int *length,unsigned char *Res,int position,int size){

	int i=0;
	if(size<0){//不考呗
		return -1;
	
	}
	
	if(*length > (size)){//空间还有
		for(i=0;i<size;i++){
			data[i]=Res[position+i];		
		}

		
	}else{
		rt_kprintf("[hplc]  (%s)  *usrdata_size=%d < (size=%d) mem is not enough!\n",__func__,*length,size);
		return -1;
		/*if(size<=1){//第一次扩大到这个程度
			if(array_inflate(data,*length,1024)==0){
				*length+=1024;
				rt_kprintf("[hplc]  (%s)  add 1024 size for   usrdata size*length =%d!\n",__func__,*length);//重要信息需要打印
			}else{
				return -1;
			}	
	
		}else {
			if(array_inflate(data,*length,size)==0){
				*length+=size;
			}else{
				return -1;
			}				
		
		}
		for(i=0;i<size;i++){
			data[i]=Res[position+i];		
		}*/		

	}
	return 0;	
}

int get_current_time(unsigned char * data){
	
	//从函数接口中获取实时时间

		
	
	
	for(int i=0 ;i<10; i++){
		data[i]=0xfe;	
	}
	return 0;

}
/*
功能：添加两帧之间的时间间隔，设备锁的策略
//添加两帧之间的时间间隔，设备锁的策略，																																															
//可以判断地址对不对（这个可提前，也可不做,暂时不做）
*/

int hplc_tx_frame(struct _698_STATE  * priv_698_state,rt_device_t serial,struct CharPointDataManage * data_tx){
	int i;
	rt_uint8_t buf[4]={0xfe,0xfe,0xfe,0xfe};
	//rt_kprintf("[hplc]  (%s)  data_tx->dataSize=%d\n",__func__,data_tx->dataSize);//重要信息需要打印
	rt_device_write(hplc_serial, 0, &buf, 4);
	rt_device_write(hplc_serial, 0, data_tx->priveData, data_tx->dataSize);//必须用dma的方式，要不然会消耗太多时间	
	rt_kprintf("[hplc]  (%s) :\n",__func__);
//	rt_kprintf("[hplc]  (%s)  time = %0x:%0x:%0x\n",__func__,System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second);
//	for(i=0;i<data_tx->dataSize;i++){
//		rt_kprintf("%02x ",data_tx->priveData[i]);	
//	}
//	rt_kprintf("\n");
	
	return 0;
}

int init_CharPointDataManage(struct CharPointDataManage *des){

//	des->priveData=RT_NULL;
//	rt_memset()
	des->size=HPLC_DATA_MAX_SIZE;
	des->dataSize=0;
	des->next=RT_NULL;
	des->list_no=0;//等待响应的任务队列，如果超时了就发送，如果有应答了就响应。
	init_698_FRAME(&des->_698_frame);
	des->_698_frame.usrData=des->priveData;
	return 0;	
}

int free_CharPointDataManage(struct CharPointDataManage *des){

//	free_698_FRAME(&des->_698_frame);
//	if(des->priveData!=RT_NULL){
//		rt_free(des->priveData)	;
//	}
//	rt_free(des);	
	return 0;	
}

int my_free(unsigned char  *des){
//	unsigned char  *p;
//	if(des!=RT_NULL){
//		p=des;
//		rt_free(p);
//		des=RT_NULL;
//	}
	return 0;
}


int init_698_FRAME(struct  _698_FRAME  * des){
	des->addr.ca=0;
	des->addr.sa=0;
	des->usrData_len=0;
	des->need_package=0;
	des->security_flag=0;
	des->needEsam_flag=0;
	des->frame_apart=0;
	des->frame_no=0;
//  des->addr.s_addr=(unsigned char *)rt_malloc(sizeof(unsigned char)*(8));;	
//	des->usrData_size=1024-8;	
//	des->usrData=(unsigned char *)rt_malloc(sizeof(unsigned char)*(des->usrData_size));//后面的策略是只要用了空间就，不动态释放了，就按最大的来
	return 0;
}
/*
清空所有指针

*/
int free_698_FRAME(struct  _698_FRAME  * des){

//	if(des->addr.s_addr!=RT_NULL){
//		rt_free(des->addr.s_addr);	
//	}
//	if(des->usrData!=RT_NULL){
//		rt_free(des->usrData);	
//	}
//	rt_free(des);	
	return 0;
}








/*
	判断来帧的表号
*/



int judge_meter_no(struct _698_STATE  * priv_698_state,struct CharPointDataManage *data_rev){
	int i;
	for(i=0;i<priv_698_state->addr.s_addr_len;i++){
		if((data_rev->_698_frame.addr.s_addr[i]==0xaa)||(priv_698_state->addr.s_addr[i]==data_rev->_698_frame.addr.s_addr[i])){

//			rt_kprintf("[hplc]  (%s) right meter No. \n",__func__);
			
		}else{
			rt_kprintf("[hplc]  (%s)  NOT right right meter No.  s_addr[%d]= %2X \n",__func__,i,priv_698_state->addr.s_addr[i]);
			return -1;
		}
	}
	return 0;		


}

int get_meter_addr(unsigned char * addr){//需要别人提供接口
unsigned char tmp_addr[6];
	int i=0,j=0;
//	addr[0]=0x15;
//	addr[1]=0x00;	
//	addr[2]=0x01;
//	addr[3]=0x03;
//	addr[4]=0x18;	
//	addr[5]=0x20;
	rt_kprintf("[hplc]  (%s)  \n",__func__);//重要信息需要打印		
	

//	
//	if(GetStorageData(Cmd_MeterNumRd,tmp_addr,13)==0){

//		for(i=0;i<13;i++){
//			j=i/2;
//			addr[j]=(RouterIfo.Addr[13-i]-0x30);
//			rt_kprintf("[hplc]  (%s)  RouterIfo.Addr[%d]=%0x \n",__func__,13-i,RouterIfo.Addr[13-i]);
//			i++;
//			addr[j]=(RouterIfo.Addr[13-i]-0x30)*10;
//			rt_kprintf("[hplc]  (%s)  RouterIfo.Addr[%d]=%0x \n",__func__,13-i,RouterIfo.Addr[13-i]);
//			
//			rt_kprintf("[hplc]  (%s)  addr[%d]=%0x \n",__func__,i, addr[i]);
//		}
		
		for(i=0;i<(RouterInfo.Addr[0]/2);i++){
			addr[i]=((RouterInfo.Addr[RouterInfo.Addr[0]-i*2-1]-0x30)<<4 | (RouterInfo.Addr[RouterInfo.Addr[0]-i*2]-0x30));
			rt_kprintf("[hplc]  (%s)  RouterInfo.Addr[%d]=%0x\n",__func__,i,RouterInfo.Addr[i]);//重要信息需要打印		
		}
//		addr[0]=0x11;//测试
		
		for(i=0;i<(RouterInfo.Addr[0]/2);i++){
			rt_kprintf("[hplc]  (%s)  addr[%d]=%0x\n",__func__,i,addr[i]);//重要信息需要打印		
		}

//		return 0;	

//	}else{
//		return -1;
//	}

	return 0;	
}
int init_698_state(struct _698_STATE  * priv_698_state){
	//int i;	
	//电表地址
	priv_698_state->piid=0;//PIID 是用于客户机 APDU（ Client-APDU）的各服务数据类型中，基本定
													//义如下，更具体应用约定应根据实际系统要求而定。
	priv_698_state->version[0]=0x18;
	priv_698_state->version[1]=0x19;	
	
	
	priv_698_state->addr.sa=05;
	priv_698_state->addr.s_addr_len=6;

	get_meter_addr(priv_698_state->addr.s_addr);
	

	priv_698_state->addr.ca=00;//客户机地址
			
	priv_698_state->heart_beat_time0=0x00;//太长,按道理应该只有两级，传过_698_frame来，或者再封装一个函数。
	priv_698_state->heart_beat_time1=0xb4;
	
	priv_698_state->FactoryVersion.manufacturer_code[0]=0x54;
	priv_698_state->FactoryVersion.manufacturer_code[1]=0x4F; 
	priv_698_state->FactoryVersion.manufacturer_code[2]=0x50;
	priv_698_state->FactoryVersion.manufacturer_code[3]=0x53;
	
	priv_698_state->FactoryVersion.soft_version[0]=0x30; 
	priv_698_state->FactoryVersion.soft_version[1]=0x31;
	priv_698_state->FactoryVersion.soft_version[2]=0x30; 
	priv_698_state->FactoryVersion.soft_version[3]=0x32; 
	
	priv_698_state->FactoryVersion.soft_date[0]=0x31;
	priv_698_state->FactoryVersion.soft_date[1]=0x36; 
	priv_698_state->FactoryVersion.soft_date[2]=0x30; 
	priv_698_state->FactoryVersion.soft_date[3]=0x37;  
	priv_698_state->FactoryVersion.soft_date[4]=0x33; 
	priv_698_state->FactoryVersion.soft_date[5]=0x31;
	
	priv_698_state->FactoryVersion.hard_version[0]=0x30; 
	priv_698_state->FactoryVersion.hard_version[1]=0x31;
	priv_698_state->FactoryVersion.hard_version[2]=0x30;
	priv_698_state->FactoryVersion.hard_version[3]=0x32; 
	
	priv_698_state->FactoryVersion.hard_date[0]=0x31; 
	priv_698_state->FactoryVersion.hard_date[1]=0x36;
	priv_698_state->FactoryVersion.hard_date[2]=0x30; 
	priv_698_state->FactoryVersion.hard_date[3]=0x37; 
	priv_698_state->FactoryVersion.hard_date[4]=0x33; 
	priv_698_state->FactoryVersion.hard_date[5]=0x31; 
	
	priv_698_state->FactoryVersion.manufacturer_ex_info[0]=0x00; 
	priv_698_state->FactoryVersion.manufacturer_ex_info[1]=0x00; 
	priv_698_state->FactoryVersion.manufacturer_ex_info[2]=0x00; 
	priv_698_state->FactoryVersion.manufacturer_ex_info[3]=0x00;
	priv_698_state->FactoryVersion.manufacturer_ex_info[4]=0x00; 
	priv_698_state->FactoryVersion.manufacturer_ex_info[5]=0x00; 
	priv_698_state->FactoryVersion.manufacturer_ex_info[6]=0x00;
	priv_698_state->FactoryVersion.manufacturer_ex_info[7]=0x00;
	
	get_current_time(priv_698_state->last_link_requset_time.data);//协商完也要一段时间
	//时间全为零，进去的第一次就超时，然后发送
	
	priv_698_state->try_link_type=-1;
	priv_698_state->meter_addr_send_ok=0;
	priv_698_state->len_left=0;
	priv_698_state->len_sa=0;
	priv_698_state->len_all=0;
	priv_698_state->FE_no=0;
	priv_698_state->lock1=0;
	priv_698_state->lock2=0;
	priv_698_state->link_flag=0;
	priv_698_state->connect_flag=0;
	priv_698_state->current_report=Cmd_Null;
	priv_698_state->session_key_negotiation=0;
	priv_698_state->time=0;
	priv_698_state->report_esam_f=0;
	
	
	return 0;
	
}
int date_time_s_to_STR_SYSTEM_TIME(STR_SYSTEM_TIME * SYSTEM_TIME,unsigned char * array){
	unsigned char year;
	year=((array[0]*256+array[1]))%100;

	Int_toBCD(&SYSTEM_TIME->Year,(rt_uint8_t*)&year,1);
	Int_toBCD(&SYSTEM_TIME->Month,(array+2),1);
	Int_toBCD(&SYSTEM_TIME->Day,array+3,1);
	Int_toBCD(&SYSTEM_TIME->Hour,array+4,1);
	Int_toBCD(&SYSTEM_TIME->Minute,array+5,1);
	Int_toBCD(&SYSTEM_TIME->Second,array+6,1);	

	return 0;
}
int STR_SYSTEM_TIME_to_date_time_s(STR_SYSTEM_TIME * SYSTEM_TIME,struct _698_date_time_s *date_time_s){

	
	
//	rt_kprintf("[hplc]  %2x \n",SYSTEM_TIME->Year); 
//	rt_kprintf("[hplc]  %2x \n",SYSTEM_TIME->Month); 
//	rt_kprintf("[hplc]  %2x \n",SYSTEM_TIME->Day); 
//	rt_kprintf("[hplc]  %2x \n",SYSTEM_TIME->Hour); 
//	rt_kprintf("[hplc]  %2x \n",SYSTEM_TIME->Minute); 





	if(SYSTEM_TIME->Year!=0){
		date_time_s->year[0]=date_time_s->data[0]=((2000&0xff00)>>8);//	
		date_time_s->year[1]=date_time_s->data[1]=(((SYSTEM_TIME->Year&0xf0)>>4)*10+(SYSTEM_TIME->Year&0x0f)+(2000&0x00ff));//年
	}
	else
	{
		date_time_s->year[0]=date_time_s->data[0]= 0;
		date_time_s->year[1]=date_time_s->data[1]= 0;
	}
	
		

	date_time_s->month=date_time_s->data[2]=(((SYSTEM_TIME->Month&0xf0)>>4)*10+(SYSTEM_TIME->Month&0x0f));//月
	date_time_s->day=date_time_s->data[3]=(((SYSTEM_TIME->Day&0xf0)>>4)*10+(SYSTEM_TIME->Day&0x0f));//日	
	date_time_s->hour=date_time_s->data[4]=(((SYSTEM_TIME->Hour&0xf0)>>4)*10+(SYSTEM_TIME->Hour&0x0f));//时
	date_time_s->minute=date_time_s->data[5]=(((SYSTEM_TIME->Minute&0xf0)>>4)*10+(SYSTEM_TIME->Minute&0x0f));//分	
	date_time_s->second=date_time_s->data[6]=(((SYSTEM_TIME->Second&0xf0)>>4)*10+(SYSTEM_TIME->Second&0x0f));//秒	
	
//	rt_kprintf("[hplc]  %2x \n",date_time_s->data[0]); 
//	rt_kprintf("[hplc]  %2x \n",date_time_s->data[1]); 
//	rt_kprintf("[hplc]  %2x \n",date_time_s->data[2]); 
//	rt_kprintf("[hplc]  %2x \n",date_time_s->data[3]); 
//	rt_kprintf("[hplc]  %2x \n",date_time_s->data[4]); 	
	
	
	
	return 0;


}

unsigned char bcd_to_hex(unsigned char data)
{
    unsigned char temp;
 
    temp = ((data>>4)*10 + (data&0x0f));
    return temp;
}




int unsigned_char_to_int(unsigned long *intNo,unsigned char * array){
	unsigned long temp=0;

	temp=array[0];
	temp=temp<<8;
	temp+=array[1];
	temp=temp<<8;
	temp+=array[2];
	temp=temp<<8;
	temp+=array[3];

	*intNo=temp;
	return 0;
}






int Report_Cmd_PileFault(struct CharPointDataManage *hplc_data,struct _698_STATE  * priv_698_state){
	int result=0;
	unsigned char temp_char;
	//结构体赋值，共同部分

	hplc_data->dataSize=0;	
	temp_char=hplc_data->_698_frame.head = 0x68;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	int len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的长度	
	
	temp_char=hplc_data->_698_frame.control=CON_STU_S|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa ;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//拷贝服务器地址
	hplc_data->_698_frame.addr=priv_698_state->addr;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);

	temp_char=hplc_data->_698_frame.addr.ca=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位	
	
	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);	                              
	
	
	temp_char=report_notification;//上报
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=ReportNotificationList;//上报类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x09;//自己定的PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	

	/**用户数据的结构体部分，参考读取一个记录型对象属性**/

	//SEQUENCE OF A-ResultNormal
	temp_char=0x01;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	//对象属性描述符 OAD		
	temp_char=0x34;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x06;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x06;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//Get-Result	
	temp_char=0x01;//数据 [1] Data
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	//	//	
//	plan_fail_event_package(_698_pile_fail_event.plan_fail_event,hplc_data);
	

	temp_char=0x00;// 没有时间标签
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		


	hplc_data->_698_frame.usrData_len=hplc_data->dataSize-HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
	//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

	
	
	
	int FCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验
		
	temp_char=hplc_data->_698_frame.end=0x16;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

//给长度结构体赋值,这里判断是不是需要分针
	if(hplc_data->dataSize>HPLC_DATA_MAX_SIZE){
			rt_kprintf("[hplc]  (%s)  >HPLC_DATA_MAX_SIZE too long   \n",__func__);
			return -1;	
	}
	
	hplc_data->priveData[len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

	hplc_data->priveData[len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

//校验头
	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(hplc_data->priveData, HCS_position);
	hplc_data->_698_frame.HCS0=hplc_data->priveData[HCS_position];	
	hplc_data->_698_frame.HCS1=hplc_data->priveData[HCS_position+1];

	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(hplc_data->priveData, FCS_position);
	
	hplc_data->_698_frame.FCS0=hplc_data->priveData[FCS_position];
	hplc_data->_698_frame.FCS1=hplc_data->priveData[FCS_position+1];		

  //还需处理的

	return result;//不发送
}



int Report_Cmd_DeviceFault(struct CharPointDataManage *hplc_data,struct _698_STATE  * priv_698_state){
	int result=0;
	unsigned char temp_char;
	//结构体赋值，共同部分

	hplc_data->dataSize=0;	
	temp_char=hplc_data->_698_frame.head = 0x68;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	int len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的长度	
	
	temp_char=hplc_data->_698_frame.control=CON_STU_S|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa ;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//拷贝服务器地址
	hplc_data->_698_frame.addr=priv_698_state->addr;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);

	temp_char=hplc_data->_698_frame.addr.ca=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位	
	
	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);	                              
	
	
	temp_char=report_notification;//上报
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=ReportNotificationList;//上报类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x09;//自己定的PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	

	/**用户数据的结构体部分，参考读取一个记录型对象属性**/

	//SEQUENCE OF A-ResultNormal
	temp_char=0x01;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	//对象属性描述符 OAD		
	temp_char=0x34;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x06;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x06;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//Get-Result	
	temp_char=0x01;//数据 [1] Data
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	//	//	
//	plan_fail_event_package(_698_router_fail_event.plan_fail_event,hplc_data);
	

	temp_char=0x00;// 没有时间标签
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		


	hplc_data->_698_frame.usrData_len=hplc_data->dataSize-HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
	//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		


	int FCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验
		
	temp_char=hplc_data->_698_frame.end=0x16;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

//给长度结构体赋值,这里判断是不是需要分针
	if(hplc_data->dataSize>HPLC_DATA_MAX_SIZE){
			rt_kprintf("[hplc]  (%s)  >HPLC_DATA_MAX_SIZE too long   \n",__func__);
			return -1;	
	}
	
	hplc_data->priveData[len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

	hplc_data->priveData[len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

//校验头
	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(hplc_data->priveData, HCS_position);
	hplc_data->_698_frame.HCS0=hplc_data->priveData[HCS_position];	
	hplc_data->_698_frame.HCS1=hplc_data->priveData[HCS_position+1];

	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(hplc_data->priveData, FCS_position);
	
	hplc_data->_698_frame.FCS0=hplc_data->priveData[FCS_position];
	hplc_data->_698_frame.FCS1=hplc_data->priveData[FCS_position+1];		


  //还需处理的



	return result;//不发送
}



/*
		上送路由器异常状态
*/
int report_PLAN_FAIL_EVENT_package_router(ROUTER_FAULT_EVENT *priv_EVENT,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,len=0,i=0,j=0;
	struct _698_date_time_s priv_date_time_s;
	unsigned char temp_char,*temp_array;
	CHARGE_TIMESOLT *priv_struct_TIMESOLT;

	
	rt_kprintf("[hplc]  (%s)  \n",__func__);

	temp_char=0x01;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x6012,0x0300,hplc_data);
	
//记录的 N 列属性描述符 RCSD，
	temp_char=02;//一个给  电表号  len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	temp_char=0x00;//CSD   [0] 代表OAD，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x202a,0x0200,hplc_data);	// 第1列OAD


	temp_char=0x01;//CSD 记录型对象属性描述符 [1] ROAD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	_698_oad_omd(0x3406,0x0200,hplc_data);	// 上报能源路由器异常事件

	temp_char=0x07;//oad个数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
//	
//	//保存列属性0ad

	_698_oad_omd(0x2022,0x0200,hplc_data);	// 事件记录序号  double-long-unsigned
	_698_oad_omd(0x201e,0x0200,hplc_data);	// 事件发生时间  date_time_s	
	_698_oad_omd(0x2020,0x0200,hplc_data);	// 事件结束时间  date_time_s，
	_698_oad_omd(0x2024,0x0200,hplc_data);	// 事件发生源    NULL
	_698_oad_omd(0x3300,0x0200,hplc_data);	// 事件上报状态  array 通道上报状态	
	_698_oad_omd(0x3507,0x0206,hplc_data);	// 故障状态 bit-string
	_698_oad_omd(0x3507,0x0207,hplc_data);	// 故障变位状态 bit-string

		
	temp_char=0x1;//响应数据 CHOICE	[1] SEQUENCE OF A-RecordRow
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=0x1;//表记录的长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	
	temp_char=Data_TSA;//数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=(priv_698_state->addr.s_addr_len+1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->addr.s_addr_len-1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	save_meter_no_backward(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);	
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);		

	temp_char=Data_array;//所有的oad看做是array
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0x07;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	event_no=priv_EVENT->OrderNum;//事件记录序号  double-long-unsigned
  _698_double_long_unsigned((unsigned int) event_no, hplc_data);//里面有类型


	temp_char=Data_date_time_s;//开始时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->StartTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	
	

	temp_char=Data_date_time_s;//结束时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->FinishTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	


	temp_char=0;//事件发生源    NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0;//事件上报状态  NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
//	temp_char=Data_array;//事件上报状态  array 通道上报状态
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=01;//长度
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=Data_structure;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=02;//项数
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=Data_OAD;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


//	_698_oad_omd(0xf209,0x0,hplc_data);	// //载波 微波  无线

//	temp_char=Data_unsigned;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

//	temp_char=0;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	temp_char=Data_bit_string;//故障状态 bit-string （15位，两个字节）
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=15;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Router_Fault.Total_Fau&(0xff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Router_Fault.Total_Fau&(0xff00)>>8);;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	

	temp_char=Data_bit_string;// 故障变位状态 bit-string （15位，两个字节）
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=15;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Router_Fault.Total_Fau&(0xff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Router_Fault.Total_Fau&(0xff00)>>8);;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	return result;//不发送

}




/*
		充电桩异常事件
*/
int report_PLAN_FAIL_EVENT_package_pile(ROUTER_FAULT_EVENT *priv_EVENT,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data){
	int result=0,len=0,i=0,j=0;
	struct _698_date_time_s priv_date_time_s;
	unsigned char temp_char,*temp_array;
	CHARGE_TIMESOLT *priv_struct_TIMESOLT;

	
	rt_kprintf("[hplc]  (%s)  \n",__func__);

	temp_char=0x01;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x6012,0x0300,hplc_data);
	
//记录的 N 列属性描述符 RCSD，
	temp_char=02;//一个给  电表号  len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	temp_char=0x00;//CSD   [0] 代表OAD，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x202a,0x0200,hplc_data);	// 第1列OAD


	temp_char=0x01;//CSD 记录型对象属性描述符 [1] ROAD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	_698_oad_omd(0x3407,0x0200,hplc_data);	// 上报充电桩异常事件

	temp_char=0x07;//oad个数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
//	
//	//保存列属性0ad

	_698_oad_omd(0x2022,0x0200,hplc_data);	// 事件记录序号  double-long-unsigned
	_698_oad_omd(0x201e,0x0200,hplc_data);	// 事件发生时间  date_time_s	
	_698_oad_omd(0x2020,0x0200,hplc_data);	// 事件结束时间  date_time_s，
	_698_oad_omd(0x2024,0x0200,hplc_data);	// 事件发生源    NULL
	_698_oad_omd(0x3300,0x0200,hplc_data);	// 事件上报状态  array 通道上报状态	
	_698_oad_omd(0x3507,0x0206,hplc_data);	// 故障状态 bit-string
	_698_oad_omd(0x3507,0x0207,hplc_data);	// 故障变位状态 bit-string

		
	temp_char=0x1;//响应数据 CHOICE	[1] SEQUENCE OF A-RecordRow
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=0x1;//表记录的长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	
	temp_char=Data_TSA;//数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=(priv_698_state->addr.s_addr_len+1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->addr.s_addr_len-1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	save_meter_no_backward(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);	
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);		

	temp_char=Data_array;//所有的oad看做是array
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0x07;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	event_no=priv_EVENT->OrderNum;//事件记录序号  double-long-unsigned
  _698_double_long_unsigned((unsigned int) event_no, hplc_data);//里面有类型


	temp_char=Data_date_time_s;//开始时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->StartTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	
	

	temp_char=Data_date_time_s;//结束时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->FinishTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	


	temp_char=0;//事件发生源    NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0;//事件上报状态  NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
//	temp_char=Data_array;//事件上报状态  array 通道上报状态
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=01;//长度
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=Data_structure;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=02;//项数
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=Data_OAD;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


//	_698_oad_omd(0xf209,0x0,hplc_data);	// //载波 微波  无线

//	temp_char=Data_unsigned;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

//	temp_char=0;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	temp_char=Data_bit_string;//故障状态 bit-string （27位，四个字节）
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=27;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Pile_Fault.Total_Fau&(0xff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=((priv_EVENT->Pile_Fault.Total_Fau&(0xff00))>>8);;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
		temp_char=((priv_EVENT->Pile_Fault.Total_Fau&(0xff0000))>>16);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=((priv_EVENT->Pile_Fault.Total_Fau&(0xff000000))>>24);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	

	temp_char=Data_bit_string;// 故障变位状态 bit-string （27位，四个字节）
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=27;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Pile_Fault.Total_Fau&(0xff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Pile_Fault.Total_Fau&(0xff00)>>8);;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
		temp_char=(priv_EVENT->Pile_Fault.Total_Fau&(0xff0000)>>16);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=(priv_EVENT->Pile_Fault.Total_Fau&(0xff000000)>>24);;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	return result;//不发送

}

/*
		充电订单事件记录单元
*/
int report_CHG_ORDER_package(CHG_ORDER_EVENT *priv_EVENT,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data)
{
	int result=0,len=0,i=0,j=0;
	struct _698_date_time_s priv_date_time_s;
	unsigned char temp_char,*temp_array;
	CHARGE_TIMESOLT *priv_struct_TIMESOLT;
	//结构体赋值，共同部分
	/**用户数据的结构体部分，参考读取一个记录型对象属性**/  
	//SEQUENCE OF A-ResultNormal
	rt_kprintf("[hplc]  (%s)  \n",__func__);

	temp_char=0x01;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x6012,0x0300,hplc_data);
	
//记录的 N 列属性描述符 RCSD，
	temp_char=02;//一个给  电表号  len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	temp_char=0x00;//CSD   [0] 代表OAD，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x202a,0x0200,hplc_data);	// 第1列OAD


	temp_char=0x01;//CSD 记录型对象属性描述符 [1] ROAD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	_698_oad_omd(0x3408,0x0200,hplc_data);	// 上报充电订单结算事件

	temp_char=0x13;//len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
//	
//	//保存列属性0ad

	_698_oad_omd(0x2022,0x0200,hplc_data);	// 事件记录序号  double-long-unsigned
	_698_oad_omd(0x201e,0x0200,hplc_data);	// 事件发生时间  date_time_s	
	_698_oad_omd(0x2020,0x0200,hplc_data);	// 事件结束时间  date_time_s，
	_698_oad_omd(0x2024,0x0200,hplc_data);	// 事件发生源    NULL
	_698_oad_omd(0x3300,0x0200,hplc_data);	// 事件上报状态  array 通道上报状态	
	_698_oad_omd(0x3508,0x0206,hplc_data);	// 充电申请单号  octet-string（SIZE(16)）
	_698_oad_omd(0x3508,0x0207,hplc_data);	// 
	_698_oad_omd(0x3508,0x0208,hplc_data);	// 
	_698_oad_omd(0x3508,0x0209,hplc_data);	// 
	_698_oad_omd(0x3508,0x020a,hplc_data);	// 
	_698_oad_omd(0x3508,0x020b,hplc_data);	// 
	_698_oad_omd(0x3508,0x020c,hplc_data);	// 
	_698_oad_omd(0x3508,0x020d,hplc_data);	// 
	_698_oad_omd(0x3508,0x020e,hplc_data);	//
	_698_oad_omd(0x3508,0x020f,hplc_data);	// 
	_698_oad_omd(0x3508,0x0210,hplc_data);	// 
	_698_oad_omd(0x3508,0x0211,hplc_data);	// 
	_698_oad_omd(0x3508,0x0212,hplc_data);	// 
	_698_oad_omd(0x3508,0x0213,hplc_data);	// 
		
	temp_char=0x1;//响应数据 CHOICE	[1] SEQUENCE OF A-RecordRow
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=0x1;//表记录的长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	
	temp_char=Data_TSA;//数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=(priv_698_state->addr.s_addr_len+1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->addr.s_addr_len-1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	save_meter_no_backward(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);	
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);		

	temp_char=Data_array;//所有的oad看做是array
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0x13;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	




	event_no=priv_EVENT->OrderNum;//事件记录序号  double-long-unsigned
  _698_double_long_unsigned((unsigned int) event_no, hplc_data);//里面有类型


	temp_char=Data_date_time_s;//开始时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->StartTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	
	

	temp_char=Data_date_time_s;//结束时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->FinishTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	


	temp_char=0;//事件发生源    NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0;//事件上报状态  array 通道上报状态
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
//	temp_char=Data_array;//事件上报状态  array 通道上报状态
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=01;//长度
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=Data_structure;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=02;//项数
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

//	temp_char=Data_OAD;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


//	_698_oad_omd(0xf209,0x0,hplc_data);	// //载波 微波  无线

//	temp_char=Data_unsigned;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

//	temp_char=0;//
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);





	len=temp_char=priv_EVENT->RequestNO[0];//充电申请单号   octet-string（SIZE(16)）
	if(len>sizeof(priv_EVENT->RequestNO)){
		rt_kprintf("[hplc]  (%s) len> array size  priv_EVENT->RequestNO\n",__func__);
		return -1;
	}				
	temp_array=( unsigned char *) (priv_EVENT->RequestNO+1);
	_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);
	


  len=temp_char=priv_EVENT->cUserID[0];//用户ID      visible-string（SIZE(64)），
	if(len>sizeof(priv_EVENT->cUserID)){
		rt_kprintf("[hplc]  (%s) len> array size  priv_EVENT->cUserID\n",__func__);
		return -1;
	}				
	temp_array=( unsigned char *) (priv_EVENT->cUserID+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);


	//路由器资产编号   visible-string（SIZE(22)）

  len=temp_char=priv_EVENT->AssetNO[0];
	if(len>sizeof(priv_EVENT->AssetNO)){
		rt_kprintf("[hplc]  (%s) len> array size  priv_EVENT->AssetNO\n",__func__);
		return -1;
	}				
	temp_array=( unsigned char *) (priv_EVENT->AssetNO+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);	
	
	temp_char=Data_enum;//枪序号	enum{A枪（1）、B枪（2）}，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->GunNum; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	//充电需求电量   double-long-unsigned（单位：kWh，换算：-2）

  len=priv_EVENT->ChargeReqEle;
  _698_double_long_unsigned((unsigned int) len, hplc_data);	



	temp_char=Data_date_time_s;//充电申请时间  date_time_s，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->RequestTimeStamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);




	temp_char=Data_date_time_s;//计划用车时间  date_time_s，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->PlanUnChg_TimeStamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);
	
	


	temp_char=Data_enum;	//充电模式      enum{正常（0），有序（1）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT-> ChargeMode; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	
	temp_char=Data_array;//充电启动时电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_EVENT->StartMeterValue[j];
		_698_double_long((unsigned int) len, hplc_data);		
	}
	
	
	
	temp_char=Data_array;//充电结束时电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_EVENT->StopMeterValue[j];
		_698_double_long((unsigned int) len, hplc_data);		
	}	

	
	temp_char=Data_date_time_s;//充电启动时间  date_time_s，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->ChgStartTime,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);
		
	
	temp_char=Data_date_time_s;//充电结束时间  date_time_s，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->ChgStopTime,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	
	
	

	
	temp_char=Data_array;//实际充电电量	array double-long-unsigned
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_EVENT->ucChargeEle[j];
		_698_double_long_unsigned((unsigned int) len, hplc_data);		
	}			
	
	//充电持续时间	double-long-unsigned单位：秒


  len=priv_EVENT->ucChargeTime;
	_698_double_long_unsigned((unsigned int) len, hplc_data);	


	return result;//不发送

}




/*
路由器工作状态  即 充电计划单执行状态

*/
int report_CHARGE_EXE_EVENT_package(CHARGE_EXE_EVENT *priv_EVENT,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data)
{
	int result=0,len=0,i=0,j=0;
	struct _698_date_time_s priv_date_time_s;
	unsigned char temp_char,*temp_array;
	CHARGE_TIMESOLT *priv_struct_TIMESOLT;
	//结构体赋值，共同部分
	/**用户数据的结构体部分，参考读取一个记录型对象属性**/  
	//SEQUENCE OF A-ResultNormal
	temp_char=0x01;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x6012,0x0300,hplc_data);
	
//记录的 N 列属性描述符 RCSD，
	temp_char=02;//一个给  电表号  len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	temp_char=0x00;//CSD   [0] 代表OAD，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x202a,0x0200,hplc_data);	// 第1列OAD


	temp_char=0x01;//CSD 记录型对象属性描述符 [1] ROAD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	_698_oad_omd(0x3404,0x0200,hplc_data);	// 上报充电计划执行事件

	temp_char=0x12;//len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
//	
//	//保存列属性0ad

	_698_oad_omd(0x2022,0x0200,hplc_data);	// 事件记录序号  double-long-unsigned
	_698_oad_omd(0x201e,0x0200,hplc_data);	// 事件发生时间  date_time_s	
	_698_oad_omd(0x2020,0x0200,hplc_data);	// 事件结束时间  date_time_s，
	_698_oad_omd(0x2024,0x0200,hplc_data);	// 事件发生源    NULL
	_698_oad_omd(0x3300,0x0200,hplc_data);	// 事件上报状态  array 通道上报状态	
	_698_oad_omd(0x3506,0x0206,hplc_data);	// 充电申请单号  octet-string（SIZE(16)）
	_698_oad_omd(0x3506,0x0207,hplc_data);	// 路由器资产编号  visible-string（SIZE(22)
	_698_oad_omd(0x3506,0x0208,hplc_data);	// 枪序号	enum{A枪（1）、B枪（2）}，
	_698_oad_omd(0x3506,0x0209,hplc_data);	// 执行状态	enum{正常执行（1）、执行结束（2）、执行失败（3）}
	_698_oad_omd(0x3506,0x020a,hplc_data);	// 电能示值底值	array double-long
	_698_oad_omd(0x3506,0x020b,hplc_data);	// 当前电能示值	array double-long
	_698_oad_omd(0x3506,0x020c,hplc_data);	// 已充电量	array double-long
	_698_oad_omd(0x3506,0x020d,hplc_data);	// 已充时间	double-long-unsigned单位：秒	
	_698_oad_omd(0x3506,0x020e,hplc_data);	// 计划充电功率	double-long（单位：W，换算：-1）
	_698_oad_omd(0x3506,0x020f,hplc_data);	// 当前充电功率	double-long（单位：W，换算：-1）
	_698_oad_omd(0x3506,0x0210,hplc_data);	// 电压	arraylong-unsigned，（单位：V，换算：-1）
	_698_oad_omd(0x3506,0x0211,hplc_data);	// 电流	arraydouble-long，（单位：A 换算：-3）
	_698_oad_omd(0x3506,0x0212,hplc_data);	// 充电桩状态	enum{待机（1）、工作（2）、故障（3）}
	
	temp_char=0x1;//响应数据 CHOICE	[1] SEQUENCE OF A-RecordRow
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=0x1;//表记录的长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	
	temp_char=Data_TSA;//数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=(priv_698_state->addr.s_addr_len+1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->addr.s_addr_len-1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	save_meter_no_backward(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);	
	

	temp_char=Data_array;//所有的oad看做是array
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=0x12;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	event_no=priv_EVENT->OrderNum;//事件记录序号  double-long-unsigned
  _698_double_long_unsigned((unsigned int) event_no, hplc_data);//里面有类型


	temp_char=Data_date_time_s;//开始时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->StartTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	
	



	temp_char=Data_date_time_s;//结束时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->FinishTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	


	temp_char=0;//事件发生源    NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	temp_char=Data_array;//事件上报状态  array 通道上报状态
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=01;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=Data_structure;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=02;//项数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=Data_OAD;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	_698_oad_omd(0xf209,0x0,hplc_data);	// //载波 微波  无线

	temp_char=Data_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);





  len=temp_char=priv_EVENT->Chg_ExeState.cRequestNO[0];//充电申请单号   octet-string（SIZE(16)）
	if(len>sizeof(priv_EVENT->Chg_ExeState.cRequestNO)){
		rt_kprintf("[hplc]  (%s) len=%d > array size  priv_EVENT->Chg_ExeState.cRequestNO=%d\n",__func__,len,sizeof(priv_EVENT->Chg_ExeState.cRequestNO));
		return -1;
	}				
	temp_array=( unsigned char *) (priv_EVENT->Chg_ExeState.cRequestNO+1);
	_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);
	
	
	//路由器资产编号   visible-string（SIZE(22)）

  len=temp_char=priv_EVENT->Chg_ExeState.cAssetNO[0];
	if(len>sizeof(priv_EVENT->Chg_ExeState.cAssetNO)){
		rt_kprintf("[hplc]  (%s) len> array size  priv_EVENT->Chg_ExeState.cAssetNO\n",__func__);
		return -1;
	}	//changed next			
	temp_array=( unsigned char *) (priv_EVENT->Chg_ExeState.cAssetNO+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);	
	

	temp_char=Data_enum;//枪序号	enum{A枪（1）、B枪（2）}，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->Chg_ExeState.GunNum; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	

	temp_char=Data_enum;//执行状态	enum{正常执行（1）、执行结束（2）、执行失败（3）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->Chg_ExeState.exeState; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	
	temp_char=Data_array;//电能示值底值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_EVENT->Chg_ExeState.ulEleBottomValue[j];			
		_698_double_long((unsigned int) len, hplc_data);		
	}
	
	
	temp_char=Data_array;//当前电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){

		len=priv_EVENT->Chg_ExeState.ulEleActualValue[j];			
		_698_double_long((unsigned int) len, hplc_data);		
	}
	
	
	temp_char=Data_array;//已充电量	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=5;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<5;j++){
		len=priv_EVENT->Chg_ExeState.ucChargeEle[j];			
		_698_double_long((unsigned int) len, hplc_data);		
	}	
	
	//已充时间	double-long-unsigned单位：秒

  len=priv_EVENT->Chg_ExeState.ucChargeTime;
  _698_double_long_unsigned((unsigned int) len, hplc_data);	
	
	
	//计划充电功率	double-long（单位：W，换算：-1）
  len=priv_EVENT->Chg_ExeState.ucPlanPower;
  _698_double_long((unsigned int) len, hplc_data);	
	
	//当前充电功率	double-long（单位：W，换算：-1）
  len=priv_EVENT->Chg_ExeState.ucActualPower;
  _698_double_long((unsigned int) len, hplc_data);
	
	//电压	array  long-unsigned，（单位：V，换算：-1）
	temp_char=Data_array;//当前电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=1;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<1;j++){
		len=priv_EVENT->Chg_ExeState.ucVoltage.A;
		_698_long_unsigned((unsigned int) len, hplc_data);		
	}	
	
	
	//电流	arraydouble-long，（单位：A 换算：-3）
	temp_char=Data_array;//当前电能示值	array double-long
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=1;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	for(j=0;j<1;j++){
		len=priv_EVENT->Chg_ExeState.ucCurrent.A;
		_698_double_long((unsigned int) len, hplc_data);		
	}	
	
	//充电桩状态	enum{待机（1）、工作（2）、故障（3）}
	temp_char=Data_enum;//执行状态	enum{正常执行（1）、执行结束（2）、执行失败（3）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->Chg_ExeState.ChgPileState; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	return result;//不发送

}

/*
充电计划上报记录单元

*/
int report_PLAN_OFFER_package(PLAN_OFFER_EVENT *priv_EVENT,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data)
{
	int result=0,len=0,i=0,j=0;
	struct _698_date_time_s priv_date_time_s;
	unsigned char temp_char,*temp_array;
	CHARGE_TIMESOLT *priv_struct_TIMESOLT;
	//结构体赋值，共同部分
	/**用户数据的结构体部分，参考读取一个记录型对象属性**/  
	//SEQUENCE OF A-ResultNormal
	temp_char=0x01;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x6012,0x0300,hplc_data);
	
//记录的 N 列属性描述符 RCSD，
	temp_char=02;//一个给  电表号  len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	temp_char=0x00;//CSD   [0] 代表OAD，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0x202a,0x0200,hplc_data);	// 第1列OAD


	temp_char=0x01;//CSD 记录型对象属性描述符 [1] ROAD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);


	_698_oad_omd(0x3402,0x0200,hplc_data);	// 充电计划上报事件

	temp_char=16;//len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
//	
//	//保存列属性0ad

	_698_oad_omd(0x2022,0x0200,hplc_data);	// 事件记录序号  double-long-unsigned
	_698_oad_omd(0x201e,0x0200,hplc_data);	// 事件发生时间  date_time_s	
	_698_oad_omd(0x2020,0x0200,hplc_data);	// 事件结束时间  date_time_s，
	_698_oad_omd(0x2024,0x0200,hplc_data);	// 事件发生源    NULL
	_698_oad_omd(0x3300,0x0200,hplc_data);	// 事件上报状态  array 通道上报状态	
	_698_oad_omd(0x3504,0x0206,hplc_data);	// 充电申请单号  octet-string（SIZE(16)）
	_698_oad_omd(0x3504,0x0207,hplc_data);	// 用户ID      visible-string（SIZE(64)）
	_698_oad_omd(0x3504,0x0208,hplc_data);	// 决策单元	enum{主站（1）、控制器（2）}	
	_698_oad_omd(0x3504,0x0209,hplc_data);	// 决策类型	enum{生成 、调整（2）}
	_698_oad_omd(0x3504,0x020a,hplc_data);	// 决策时间	date_time_s	
	_698_oad_omd(0x3504,0x020b,hplc_data);	// 路由器资产编号   visible-string（SIZE(22)），
	_698_oad_omd(0x3504,0x020c,hplc_data);	// 枪序号	enum{A枪（1）、B枪（2）}，
	_698_oad_omd(0x3504,0x020d,hplc_data);	// 充电需求电量  double-long-unsigned（单位：kWh，换算：-2）	
	_698_oad_omd(0x3504,0x020e,hplc_data);	// 充电额定功率  double-long（单位：kW，换算：-4）
	_698_oad_omd(0x3504,0x020f,hplc_data);	// 充电模式      enum{正常（0），有序（1）}
	_698_oad_omd(0x3504,0x0210,hplc_data);	// 充电时段  array时段充电功率	
	

	temp_char=0x1;//响应数据 CHOICE	[1] SEQUENCE OF A-RecordRow
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=0x1;//表记录的长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	
	temp_char=Data_TSA;//数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=(priv_698_state->addr.s_addr_len+1);//数据长度0
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->addr.s_addr_len-1);//数据长度1
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	save_meter_no_backward(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);	

	temp_char=Data_array;//所有的oad看做是array
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=16;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	event_no=priv_EVENT->OrderNum;
  _698_double_long_unsigned((unsigned int) event_no, hplc_data);//里面有类型


	temp_char=Data_date_time_s;//开始时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->StartTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	
	

	temp_char=Data_date_time_s;//结束时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->FinishTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	


	temp_char=0;//事件发生源    NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	temp_char=Data_array;//事件上报状态  array 通道上报状态
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=01;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=Data_structure;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=02;//项数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=Data_OAD;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	


	_698_oad_omd(0xf209,0x0,hplc_data);	// //载波 微波  无线

	temp_char=Data_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);




  len=temp_char=priv_EVENT->Chg_Strategy.cRequestNO[0];//申请单号
	if(len>sizeof(priv_EVENT->Chg_Strategy.cRequestNO)){
		rt_kprintf("[hplc]  (%s) len> array size priv_EVENT->Chg_Strategy.cRequestNO \n",__func__);
		return -1;
	}	//changed next			
	temp_array=( unsigned char *) (priv_EVENT->Chg_Strategy.cRequestNO+1);
	_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);

	//用户ID      visible-string（SIZE(64)）

	
  len=temp_char=priv_EVENT->Chg_Strategy.cUserID[0];
	if(len>sizeof(priv_EVENT->Chg_Strategy.cUserID)){
		rt_kprintf("[hplc]  (%s) len> array size  \n",__func__);
		return -1;
	}				
	temp_array=( unsigned char *) (priv_EVENT->Chg_Strategy.cUserID+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);



	temp_char=Data_enum;//决策单元	enum{主站（1）、控制器
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->Chg_Strategy.ucDecMaker; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=Data_enum;//决策类型	enum{生成 、调整（2）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->Chg_Strategy.ucDecMaker; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	



	temp_char=Data_date_time_s;//决策时间	date_time_s
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->Chg_Strategy.strDecTime,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	



	//路由器资产编号   visible-string（SIZE(22)）


  len=temp_char=priv_EVENT->Chg_Strategy.cAssetNO[0];
	if(len>sizeof(priv_EVENT->Chg_Strategy.cAssetNO)){
		rt_kprintf("[hplc]  (%s) len> array size  priv_EVENT->Chg_Strategy.cAssetNO\n",__func__);
		return -1;
	}	//changed next			
	temp_array=( unsigned char *) (priv_EVENT->Chg_Strategy.cAssetNO+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);


	temp_char=Data_enum;//枪序号	enum{A枪（1）、B枪（2）}，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->Chg_Strategy.GunNum; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	

	//充电需求电量   double-long-unsigned（单位：kWh，换算：-2）

	
  len=priv_EVENT->Chg_Strategy.ulChargeReqEle ;
  _698_double_long_unsigned((unsigned int) len, hplc_data);

	//充电额定功率  double-long（单位：kW，换算：-4），

  len=priv_EVENT->Chg_Strategy.ulChargeRatePow;
  _698_double_long((unsigned int) len, hplc_data);

	temp_char=Data_enum;//充电模式      enum{正常（0），有序（1）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->Chg_Strategy.ucChargeMode; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	



	temp_char=Data_array;//事件上报状态  array 通道上报状态
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	
	temp_char=priv_EVENT->Chg_Strategy.ucTimeSlotNum;//数组数量//测试
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	if(priv_EVENT->Chg_Strategy.ucTimeSlotNum==0){
		return 0;
	}
	
	for(j=0;j<priv_EVENT->Chg_Strategy.ucTimeSlotNum;j++){
		priv_struct_TIMESOLT=(CHARGE_TIMESOLT *)priv_EVENT->Chg_Strategy.strChargeTimeSolts+i;	
		temp_char=Data_structure;//结构体
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

		temp_char=3;//结构体成员数
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		//开始时间    date_time_s
		temp_char=Data_date_time_s;//
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

		STR_SYSTEM_TIME_to_date_time_s(&priv_struct_TIMESOLT->strDecStartTime,&priv_date_time_s);
		
		result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);					
		//结束时间    date_time_s，
		temp_char=Data_date_time_s;//
		result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
		
		STR_SYSTEM_TIME_to_date_time_s(&priv_struct_TIMESOLT->strDecStopTime,&priv_date_time_s);
		
		result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);		

//		充电功率    double-long（单位：kW，换算：-4）
//		Value=priv_struct_TIMESOLT->ulChargePow;

		len=priv_struct_TIMESOLT->ulChargePow;
		_698_double_long((unsigned int) len, hplc_data);
	
	}	
	return result;//不发送


}





/*
*充电申请事件记录单元

*/
int report_CHARGE_APPLY_package(CHARGE_APPLY_EVENT *priv_EVENT,struct _698_STATE  * priv_698_state,struct CharPointDataManage * hplc_data)
{
	int result=0,len=0,i=0;
	struct _698_date_time_s priv_date_time_s;
	unsigned char temp_char,*temp_array,oad_array[4*17]={0x20, 0x22, 0x02, 0x00, 0x20, 0x1e, 0x02, 0x00, 
		    0x20, 0x20, 0x02, 0x00, 0x20, 0x24, 0x02, 0x00, 0x33, 0x00, 0x02, 0x00, 0x35, 0x05, 
				0x02, 0x06, 0x35, 0x05, 0x02, 0x07, 0x35, 0x05, 0x02, 0x08, 0x35, 0x05, 0x02, 0x09, 
		    0x35, 0x05, 0x02, 0x0a, 0x35, 0x05, 0x02, 0x0b, 0x35, 0x05, 0x02, 0x0c, 0x35, 0x05, 
				0x02, 0x0d, 0x35, 0x05, 0x02, 0x0e, 0x35, 0x05, 0x02, 0x0f, 0x35, 0x05, 0x02, 0x10, 
				0x35, 0x05, 0x02, 0x11};
	//结构体赋值，共同部分
	/**用户数据的结构体部分，参考读取一个记录型对象属性**/  
	//SEQUENCE OF A-ResultNormal
	
	rt_kprintf("[hplc]  (%s) \n",__func__);		
	
	temp_char=0x01;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	//充电申请事件 OAD		属性2（事件记录表）∷=array 充电申请事件记录单元
	_698_oad_omd(0x6012,0x0300,hplc_data);	

	
//记录的 N 列属性描述符 RCSD，
	temp_char=02;//一个给  电表号  len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
		
	temp_char=0x00;//CSD   [0] 代表OAD，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	_698_oad_omd(0x202a,0x0200,hplc_data);// 第1列OAD	
	
	temp_char=0x01;//CSD 记录型对象属性描述符 [1] ROAD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	//充电申请事件记录单元
	_698_oad_omd(0x3403,0x0200,hplc_data);

	
	temp_char=0x11;//len
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//保存0ad
	result=save_char_point_data(hplc_data,hplc_data->dataSize,oad_array,4*17);		

	temp_char=0x1;//响应数据 CHOICE	[1] SEQUENCE OF A-RecordRow
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=0x1;//表记录的长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	
	temp_char=Data_TSA;//数据类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		

	temp_char=(priv_698_state->addr.s_addr_len+1);//数据长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=(priv_698_state->addr.s_addr_len-1);//地址长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	save_meter_no_backward(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);
//	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_698_state->addr.s_addr,priv_698_state->addr.s_addr_len);		

	temp_char=Data_array;//所有的oad看做是array
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=17;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	event_no=priv_EVENT->OrderNum;
  _698_double_long_unsigned((unsigned int) event_no, hplc_data);//里面有类型




	temp_char=Data_date_time_s;//开始时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->StartTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	




	temp_char=Data_date_time_s;//结束时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->FinishTimestamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	


	temp_char=0;//事件发生源    NULL
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	



	temp_char=Data_array;//事件上报状态  array 通道上报状态
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=01;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=Data_structure;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=02;//项数
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=Data_OAD;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	//充电申请事件记录单元
	temp_char=0xf2;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x09;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x00;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=Data_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);



	len=temp_char=priv_EVENT->RequestNO[0];//充电申请单号   octet-string（SIZE(16)）
	if(len>sizeof(priv_EVENT->RequestNO)){
		rt_kprintf("[hplc]  (%s) len=%d > array size priv_EVENT->RequestNO=%d \n",__func__,len,sizeof(priv_EVENT->RequestNO));
		return -1;
	}	//changed next			
	temp_array=( unsigned char *) (priv_EVENT->RequestNO+1);
	_698_visible_octet_string(Data_octet_string,len,temp_array,hplc_data);


	//路由器资产编号   visible-string（SIZE(22)）

  len=temp_char=priv_EVENT->AssetNO[0];
	if(len>sizeof(priv_EVENT->AssetNO)){
		rt_kprintf("[hplc]  (%s) len> array size priv_EVENT->AssetNO \n",__func__);
		return -1;
	}				
	temp_array=( unsigned char *) (priv_EVENT->AssetNO+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);	


	temp_char=Data_enum;//枪序号	enum{A枪（1）、B枪（2）}，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	temp_char=priv_EVENT->GunNum; // 
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	




	temp_char=Data_date_time_s;//充电申请时间
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->RequestTimeStamp,&priv_date_time_s);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);	

	//当前SOC  long-unsigned（单位：%，换算：-2）,

  len=priv_EVENT->actSOC;
  _698_long_unsigned((unsigned int)len , hplc_data);
	
	//目标SOC  long-unsigned（单位：%，换算：-2）,

  len=priv_EVENT->aimSOC;
  _698_long_unsigned((unsigned int)len , hplc_data);
	
	//电池容量 double-long-unsigned（单位：kWh，换算：-2），

  len=priv_EVENT->CellCapacity;
  _698_double_long_unsigned((unsigned int) len, hplc_data);
	
	//充电需求电量   double-long-unsigned（单位：kWh，换算：-2）

  len=priv_EVENT->ChargeReqEle;
  _698_double_long_unsigned((unsigned int) len, hplc_data);


	temp_char=Data_date_time_s;//计划用车时间  date_time_s，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	

	STR_SYSTEM_TIME_to_date_time_s(&priv_EVENT->PlanUnChg_TimeStamp,&priv_date_time_s);//计划用车时间  date_time_s，	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,priv_date_time_s.data,7);		


	temp_char=Data_enum;//充电方式      enum{正常（0），有序（1）}，
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=priv_EVENT->ChargeMode; //	充电模式 {正常（0），有序（1）}
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	

	//身份认证Token   visible-string（SIZE(38)），	

  len=temp_char=priv_EVENT->Token[0];
	if(len>sizeof(priv_EVENT->Token)){
		rt_kprintf("[hplc]  (%s) len> array size priv_EVENT->Token \n",__func__);
		return -1;
	}	//changed next			
	temp_array=( unsigned char *) (priv_EVENT->Token+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);


	//充电用户账号   visible-string（SIZE(9)），

  len=temp_char=priv_EVENT->UserAccount[0];
	if(len>sizeof(priv_EVENT->UserAccount)){
		rt_kprintf("[hplc]  (%s) len> array size priv_EVENT->UserAccount \n",__func__);
		return -1;
	}			
	temp_array=( unsigned char *) (priv_EVENT->UserAccount+1);
	_698_visible_octet_string(Data_visible_string,len,temp_array,hplc_data);

	return result;//不发送

}


/**
上报 esam_information

**/
	
int report_esam_information_package(COMM_CMD_C report_type,void *report_struct,struct CharPointDataManage * hplc_data,struct _698_STATE  * priv_698_state){


	int result=0;
	unsigned char temp_char;
	//结构体赋值，共同部分
		rt_kprintf("[hplc]  (%s)     \n",__func__);
	hplc_data->dataSize=0;	
	temp_char=hplc_data->_698_frame.head = 0x68;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	int len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的长度	
	
	temp_char=hplc_data->_698_frame.control=CON_STU_S|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa ;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//拷贝服务器地址
	hplc_data->_698_frame.addr=priv_698_state->addr;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);

	temp_char=hplc_data->_698_frame.addr.ca=0x00;//默认是上报给后台
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位	
	
	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);	                              
	
	
	temp_char=report_notification;//上报
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=ReportNotificationList;//上报类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x00;//自己定的PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	



//*********************************

	temp_char=0x03;//长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	_698_oad_omd(0xf102,0x0200,hplc_data);

	hplc_ScmEsam_Comm.DataTx_len=0;

	
	get_data_class(priv_698_state,hplc_data,Data_octet_string);
	ESAM_Communicattion(2,&hplc_ScmEsam_Comm);
	
	temp_char=(hplc_ScmEsam_Comm.DataRx_len-5);//长度

	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	if(hplc_ScmEsam_Comm.DataRx_len<12){
		rt_kprintf("[hplc]  (%s)  .DataRx_len<12 error and out  2  \n",__func__);
		return -1;//没读出数据
	}else{
	
		result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+4),(hplc_ScmEsam_Comm.DataRx_len-5));
	}				


	_698_oad_omd(0xf102,0x0400,hplc_data);
	
	get_data_class(priv_698_state,hplc_data,Data_octet_string);
	ESAM_Communicattion(4,&hplc_ScmEsam_Comm);
	
	temp_char=(hplc_ScmEsam_Comm.DataRx_len-5);//长度

	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	if(hplc_ScmEsam_Comm.DataRx_len<12){
		rt_kprintf("[hplc]  (%s)  .DataRx_len<12 error and out  4  \n",__func__);
		return -1;//没读出数据
	}else{
	
		result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+4),(hplc_ScmEsam_Comm.DataRx_len-5));
	}					
	
	
	_698_oad_omd(0xf102,0x0700,hplc_data);	

	get_data_class(priv_698_state,hplc_data,Data_structure);
	
	temp_char=4;//数组数量，由上传者决定默认是一
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);				

	ESAM_Communicattion(6,&hplc_ScmEsam_Comm);

	if(hplc_ScmEsam_Comm.DataRx_len<12){
		rt_kprintf("[hplc]  (%s)  .DataRx_len<12 error and out  7-1  \n",__func__);
		return -1;//没读出数据
	}else{
		int priv_len=	hplc_ScmEsam_Comm.DataRx_len-5;		
		for(int i=0;i<4;i++){
			
			temp_char=Data_double_long_unsigned;//数组数量，由上传者决定默认是一
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);						
			result=save_char_point_data(hplc_data,hplc_data->dataSize,(hplc_ScmEsam_Comm.Rx_data+(4*(i+1))),4);								
		}
							
	}				

//**********************************


	temp_char=0x00;// 无跟随
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=0x00;// 没有时间标签
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	hplc_data->_698_frame.usrData_len=hplc_data->dataSize-HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
	//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

	int FCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验
		
	temp_char=hplc_data->_698_frame.end=0x16;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

//给长度结构体赋值,这里判断是不是需要分针
	if(hplc_data->dataSize>HPLC_DATA_MAX_SIZE){
			rt_kprintf("[hplc]  (%s)  >HPLC_DATA_MAX_SIZE too long   \n",__func__);
			return -1;	
	}
	
	hplc_data->priveData[len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

	hplc_data->priveData[len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

//校验头
	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(hplc_data->priveData, HCS_position);
	if(result!=0){
		rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
		return -1;
	}

	hplc_data->_698_frame.HCS0=hplc_data->priveData[HCS_position];	
	hplc_data->_698_frame.HCS1=hplc_data->priveData[HCS_position+1];

	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(hplc_data->priveData, FCS_position);
	
	hplc_data->_698_frame.FCS0=hplc_data->priveData[FCS_position];
	hplc_data->_698_frame.FCS1=hplc_data->priveData[FCS_position+1];		

	
	
	if( result!=0){
		rt_kprintf("[hplc]  (%s)    error \n",__func__);//												
	}else{//下面是需要回复的情况
		rt_kprintf("[hplc]  (%s)  print data_tx:\n",__func__);
		printmy(&hplc_data->_698_frame);
		hplc_tx_frame(priv_698_state,hplc_serial,hplc_data);//发送数据	

	}			
	
}



/**
上报 report_notification 0x88

**/
	
int report_notification_package(COMM_CMD_C report_type,void *report_struct,struct CharPointDataManage * hplc_data,struct _698_STATE  * priv_698_state){
	
	int result=0;
	unsigned char temp_char;
	//结构体赋值，共同部分

	hplc_data->dataSize=0;	
	temp_char=hplc_data->_698_frame.head = 0x68;//起始帧头 = 0x68	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么,最后打包比较好
	int len_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的长度	
	
	temp_char=hplc_data->_698_frame.control=CON_STU_S|CON_U_DATA;   //控制域c,bit7,传输方向位
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么	

	temp_char=hplc_data->_698_frame.addr.sa=priv_698_state->addr.sa ;//& ADDR_SA_ADDR_LENGTH_MASK;//只取长度
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	//拷贝服务器地址
	hplc_data->_698_frame.addr=priv_698_state->addr;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,hplc_data->_698_frame.addr.s_addr,hplc_data->_698_frame.addr.s_addr_len);

	temp_char=priv_698_state->addr.ca;//控制器地址
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	int HCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验位	
	
	//下面的只处理数据，不打包到指针最后统一打包用户数据。
	hplc_data->_698_frame.usrData_len=0;//用户数据长度归零
	hplc_data->_698_frame.usrData=hplc_data->priveData+(8+hplc_data->_698_frame.addr.s_addr_len);	                              
	
	
	temp_char=report_notification;//上报
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	temp_char=ReportNotificationRecordList;//上报类型
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);

	temp_char=0x00;//自己定的PIID-ACD
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	//Get-Result
	if(report_type==Cmd_ChgRequestReport){   //充电申请事件上送
		rt_kprintf("[hplc]  (%s)  Cmd_CHARGE_APPLY   \n",__func__);
		result=report_CHARGE_APPLY_package((CHARGE_APPLY_EVENT *)report_struct,priv_698_state,hplc_data);	
		if(result!=0){
			rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
			return -1;
		}		
	}else if(report_type==Cmd_ChgPlanOffer){ //充电计划事件上报
		rt_kprintf("[hplc]  (%s)  Cmd_ChgPlanOffer   \n",__func__);
		result=report_PLAN_OFFER_package((PLAN_OFFER_EVENT *)report_struct,priv_698_state,hplc_data);
		if(result!=0){
			rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
			return -1;
		}		
	}else if(report_type==Cmd_ChgPlanExeState){ //上报充电计划执行事件
		rt_kprintf("[hplc]  (%s)  Cmd_ChgPlanExeState   \n",__func__);
		result=report_CHARGE_EXE_EVENT_package((CHARGE_EXE_EVENT *)report_struct,priv_698_state,hplc_data);	
		if(result!=0){
			rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
			return -1;
		}
	}else if(report_type==Cmd_ChgRecord){ //上送充电订单事件
		rt_kprintf("[hplc]  (%s)  Cmd_ChgRecord   \n",__func__);
		result=report_CHG_ORDER_package((CHG_ORDER_EVENT *)report_struct,priv_698_state,hplc_data);	
		if(result!=0){
			rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
			return -1;
		}	
	}else if(report_type==Cmd_DeviceFault){ //上送路由器异常状态
		rt_kprintf("[hplc]  (%s)  Cmd_DeviceFault   \n",__func__);
		result=report_PLAN_FAIL_EVENT_package_router((ROUTER_FAULT_EVENT *)report_struct,priv_698_state,hplc_data);	
		if(result!=0){
			rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
			return -1;
		}	
	}else if(report_type==Cmd_PileFault){ //上送充电桩异常状态
		rt_kprintf("[hplc]  (%s)  Cmd_DeviceFault   \n",__func__);
		result=report_PLAN_FAIL_EVENT_package_pile((ROUTER_FAULT_EVENT *)report_struct,priv_698_state,hplc_data);	
		if(result!=0){
			rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
			return -1;
		}	
	}else {
		rt_kprintf("[hplc]  (%s)  no such cmd   \n",__func__);
		return -1;
	}
	temp_char=0x00;// 无跟随
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=0x00;// 没有时间标签
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	hplc_data->_698_frame.usrData_len=hplc_data->dataSize-HCS_position-2;//用户数据总长度	,下面拷贝用户数据到usrData,这个式子还要试试。	
	//save_char_point_usrdata(hplc_data->_698_frame.usrData,&hplc_data->_698_frame.usrData_size,hplc_data->priveData,hplc_data->dataSize-hplc_data->_698_frame.usrData_len,hplc_data->_698_frame.usrData_len);		

	int FCS_position=hplc_data->dataSize;
	hplc_data->dataSize+=2;//加两字节的校验
		
	temp_char=hplc_data->_698_frame.end=0x16;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);//这样打包好么

//给长度结构体赋值,这里判断是不是需要分针
	if(hplc_data->dataSize>HPLC_DATA_MAX_SIZE){
			rt_kprintf("[hplc]  (%s)  >HPLC_DATA_MAX_SIZE too long   \n",__func__);
			return -1;	
	}
	
	hplc_data->priveData[len_position]=hplc_data->_698_frame.length0=(hplc_data->dataSize-2)%256;//hplc_data->size<1024时

	hplc_data->priveData[len_position+1]=hplc_data->_698_frame.length1=(hplc_data->dataSize-2)/256;	

//校验头
	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the HCS_positon=%d \n",__func__,HCS_position); 	
	result=tryfcs16(hplc_data->priveData, HCS_position);
	if(result!=0){
		rt_kprintf("[hplc]  (%s)  erro and out    \n",__func__);
		return -1;
	}

	hplc_data->_698_frame.HCS0=hplc_data->priveData[HCS_position];	
	hplc_data->_698_frame.HCS1=hplc_data->priveData[HCS_position+1];

	//rt_kprintf("[hplc]  (%s)   link_response_package calculate the FCS_position=%d \n",__func__,FCS_position); 	
	result=tryfcs16(hplc_data->priveData, FCS_position);
	
	hplc_data->_698_frame.FCS0=hplc_data->priveData[FCS_position];
	hplc_data->_698_frame.FCS1=hplc_data->priveData[FCS_position+1];		

	return result;//不发送

}



int _698_oad_omd(unsigned int oa_om,unsigned int ob_attribute,struct CharPointDataManage * hplc_data)
{
	
	unsigned char temp_char;
	int result=0;		
	temp_char=(oa_om&0x0000ff00)>>8;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	temp_char=oa_om&0x000000ff;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	temp_char=(ob_attribute&0x0000ff00)>>8;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	temp_char=(ob_attribute&0x000000ff);//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);
	
	return result;	
}




int _698_double_long(unsigned int num,struct CharPointDataManage * hplc_data)
{
	unsigned char temp_char;
	int result=0;		
	temp_char=Data_double_long;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=((num &0xff000000)>>24);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=(( num&0x00ff0000)>>16);//超了是不是溢出
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((num&0x0000ff00)>>8);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=((num&0x000000ff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	return result;
}


/*int _698_double_long_unsigned(unsigned char *priv_char)
{
	
	int result=0;		
	temp_char=Data_double_long_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=((num &0xff000000)>>24);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=(( num&0x00ff0000)>>16);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((num&0x0000ff00)>>8);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=((num&0x000000ff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	return result;
}
*/
int _698_double_long_unsigned(unsigned int num,struct CharPointDataManage * hplc_data)
{
	unsigned char temp_char;
	int result=0;		
	temp_char=Data_double_long_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=((num &0xff000000)>>24);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=(( num&0x00ff0000)>>16);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);			

	temp_char=((num&0x0000ff00)>>8);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=((num&0x000000ff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	return result;
}

int _698_long_unsigned(unsigned int num,struct CharPointDataManage * hplc_data){
	unsigned char temp_char;
	int result=0;		
	temp_char=Data_long_unsigned;//
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);		
	
	temp_char=((num&0x0000ff00)>>8);
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	

	temp_char=((num&0x000000ff));
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&temp_char,1);	
	
	return result;
}


int _698_visible_octet_string_to_ascill(unsigned char data_type,int len,unsigned char * array,struct CharPointDataManage * hplc_data){
	
	unsigned char tempchar=0,length=(unsigned char) len,i,result=0;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&data_type,1);	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&length,1);
	
	if(data_type==Data_visible_string){
		for(i=0;i<len;i++){
			tempchar=(array[i]);	
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&tempchar,1);
		}		
	}	else{
		for(i=0;i<len;i++){	
			tempchar=array[i];	
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&tempchar,1);
		}	
	}	
}


int _698_visible_octet_string(unsigned char data_type,int len,unsigned char * array,struct CharPointDataManage * hplc_data){
	int result=0;	
	unsigned char tempchar=0,length=(unsigned char) len,i;
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&data_type,1);	
	result=save_char_point_data(hplc_data,hplc_data->dataSize,&length,1);
	
	if(data_type==Data_visible_string){
		for(i=0;i<len;i++){			
			tempchar=(array[i]);	
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&tempchar,1);
		}		
	}	else{
		for(i=0;i<len;i++){	
			tempchar=array[i];	
			result=save_char_point_data(hplc_data,hplc_data->dataSize,&tempchar,1);
		}	
	}	
	return result;
}


int hplc_thread_init(void)
{
	rt_err_t res;
	int result;
	rt_kprintf("[hplc]  (%s) \n",__func__);
	
	result=rt_event_init(&PowerCtrlEvent,"PowerCtrlEvent",RT_IPC_FLAG_FIFO);
	if(result!=RT_EOK){		
			rt_kprintf("[hplc]  (%s)  rt_event PowerCtrlEvent faild! \n",__func__);
	}else{
	
	}	
	res=rt_thread_init(&hplc,
											"hplc",
											hplc_thread_entry,
											RT_NULL,//parameter
											hplc_stack,//stack_start
											THREAD_HPLC_STACK_SIZE,
											THREAD_HPLC_PRIORITY,
											THREAD_HPLC_TIMESLICE);
	if (res == RT_EOK) 
	{
			rt_thread_startup(&hplc);
	}
	return res;
}

#if defined (RT_HPLC_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
	INIT_APP_EXPORT(hplc_thread_init);
#endif
MSH_CMD_EXPORT(hplc_thread_init, hplc thread run);

/**********测试用数据**********/
//获取esam信息
unsigned char esam_data[1024]={
0x68, 0x4f, 0x0, 0x43, 0x05, 0x11, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x84, 0x53, 0x10, 0x01, 0x30, 0x7d, 0x01, 0x4b, 0x86, 0xc0, 0x9f, 0x52, 0xf9,
0x8e, 0x91, 0x3b, 0x86, 0xd8, 0x69, 0x79, 0x77, 0x9f, 0x65, 0xa9, 0xd3, 0x65, 0xc2, 0x57, 0xd9, 0xc3, 0xf1, 0xdc, 0x95, 0x8b, 0xd8, 0xf7, 
0x2c, 0x2a, 0x07, 0x36, 0xba, 0xb7, 0xa2, 0xd4, 0x58, 0x2d, 0xbb, 0x34, 0xf6, 0x52, 0x20, 0xcd, 0xe3, 0x00, 0x81, 0x1c, 0x33, 0x10, 0x02, 
0x00, 0x34, 0x04, 0x29, 0x18, 0xa7, 0xb2, 0x69, 0xca, 0x16
};