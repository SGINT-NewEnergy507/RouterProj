#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>   //中断函数头文件 rt_hw_interrupt_disable rt_hw_interrupt_enable
#include <string.h>
#include <stdio.h>
#include <global.h>
#include <storage.h>
#include <meter.h>
#include "strategy.h"
#include "global.h"

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#include <dfs_posix.h>
#include <uffs/uffs_fd.h>
#endif

#define THREAD_STORAGE_PRIORITY     19
#define THREAD_STORAGE_STACK_SIZE   1024*4
#define THREAD_STORAGE_TIMESLICE    5

#define WRITE    1
#define READ     2

static char* get_name(char* strtemp,rt_uint8_t*fpname,rt_uint8_t *fpcnt);
static char* get_pvalue(char* strtemp,rt_uint32_t*fpvalue,rt_uint8_t cnt);
static rt_uint8_t str2num(rt_uint8_t*src,rt_uint32_t *dest);
static rt_uint8_t str2nnum(rt_uint8_t*src,rt_uint8_t *dest);
static rt_uint32_t pow_df(rt_uint8_t m,rt_uint8_t n);

static struct rt_thread storage;
static rt_uint8_t storage_stack[THREAD_STORAGE_STACK_SIZE];//线程堆栈
static rt_mutex_t storage_ReWr_mutex = RT_NULL;
#define MAX_DIR_NUM    16

#define MAX_MALLOC_NUM    1024

///* 定义邮箱控制块 */
//rt_mailbox_t m_save_mail = RT_NULL;

const char* _dir_name[MAX_DIR_NUM]={
	"/SysPara",
	"/ChargePilePara",
	"/Strategy",
	"/Strategy/OrderCharge",
	"/Strategy/PlanOffer",
	"/Strategy/PlanFail",
	"/Strategy/OnlineState",
	"/Strategy/ChgExecute",
	"/Strategy/ChgRequest",
	"/Meter",
	"/HistoryRecord",
	"/ChargeRecord",
	"/PERecord",
	"/AlarmRecord",
	"/EventRecord",
	"/LOG",
};

//ROUTER_IFO_UNIT RouterIfo;

__align(4) char *Para_Buff = NULL; //固化参数buff
const char* METER_POWER_PATH=(const char*)"/Meter";
const char* LOG_PATH=(const char*)"/LOG";
const char* CHARGE_RECORD_PATH=(const char*)"/ChargeRecord";
const char* HISTORY_RECORD_PATH=(const char*)"/HistoryRecord";
const char* ORDER_CHARGE_PATH=(const char*)"/Strategy/OrderCharge";
const char* PLAN_OFFER_PATH=(const char*)"/Strategy/PlanOffer";
const char* PLAN_FAIL_PATH=(const char*)"/Strategy/PlanFail";
const char* ONLINE_STATE_PATH=(const char*)"/Strategy/OnlineState";
const char* CHG_EXECUTE_PATH=(const char*)"/Strategy/ChgExecute";
const char* CHG_REQUEST_PATH=(const char*)"/Strategy/ChgRequest";

const char* NAND_LOG_PATH_FILE=(const char*)"/LOG/log.txt";
const char* METER_ANALOG_PATH_FILE=(const char*)"/Meter/analog.txt";
const char* ROUTER_PARA_PATH_FILE=(const char*)"/SysPara/RouterPata.txt";
const char* METER_GJFMode_PATH_FILE=(const char*)"/Meter/GJFMode.txt";
const char* METER_HALF_POWER_PATH_FILE=(const char*)"/Meter/MeterHalfPower.txt";

static int Meter_Anolag_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd);
static int Router_Para_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd);
static int Meter_Power_Storage(const char *file,void *Storage_Para,rt_uint32_t YMD,rt_uint32_t cmd);
static int Meter_GJFMode_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd);
static int Meter_HalfPower_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd);
static int Charge_Record_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd);

static int Order_Charge_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd);
static int Plan_Offer_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd);	
static int Plan_Fail_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd);
static int Online_State_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd);
static int Chg_Execute_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd);
static int Chg_Request_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd);




static  int Log_Process(void);
static  int NAND_LogWrite(char* Pnand_Log,rt_uint32_t *nand_LogLen);
static  int LOG_BackUP(void);
static  int Catalogue_Del_Oldest(const char *PATH);
static  int Find_path_file(const char *PATH,int number,char *pathfile,rt_uint32_t *filenum);

static void storage_thread_entry(void *parameter)
{
	
	rt_thread_delay(2000);
	char i,para[9];
	char array[9];
//	
//	rt_uint8_t test;
	
//	test = 0;
//	
	if(GetStorageData(Cmd_MeterNumRd,&RouterInfo,sizeof(RouterInfo)) < 0)//读取文件错误 设置为默认值
	{
		RouterInfo.AssetNum[0] = 0x16;
		for(i = 0; i < (RouterInfo.AssetNum[0]-1);i++)
		{
			RouterInfo.AssetNum[i+1] = 0x30;
		}
		RouterInfo.AssetNum[RouterInfo.AssetNum[0]] = 0x31;
		
		RouterInfo.Addr[0] = 0x0C;
		for(i = 0; i < (RouterInfo.Addr[0]-1);i++)
		{
			RouterInfo.Addr[i+1] = 0x30;
		}
		RouterInfo.Addr[RouterInfo.Addr[0]] = 0x31;
		
		rt_kprintf("[Storage]:Read meter number err!\r\n");
	}
//	if(test)
//	{
//		memset(RouterInfo.AssetNum,0,sizeof(RouterInfo.AssetNum));
//		RouterInfo.AssetNum[0] = 0x0C;
//		for(i = 0; i < (RouterInfo.AssetNum[0]-1);i++)
//		{
//			RouterInfo.AssetNum[i+1] = 0x30;
//		}
////		RouterIfo.AssetNum[RouterIfo.AssetNum[0]-1] = 0x31;
//		RouterInfo.AssetNum[RouterInfo.AssetNum[0]] = 0x31;
//		
//		RouterInfo.Addr[0] = 0x0C;
//		for(i = 0; i < (RouterInfo.Addr[0]-1);i++)
//		{
//			RouterInfo.Addr[i+1] = 0x30;
//		}
//		RouterInfo.Addr[RouterInfo.Addr[0]] = 0x31;
//	}
	
	while (1)
	{
//		Log_Process();// 根据log大小存储LOG本地
		
//		df("/");
//		extern void list_mem(void);
//		list_mem();		
//		static struct statfs freespace;
//		int rval = statfs((const char*)("/"),&freespace);
//		if(RT_EOK == rval)
//		{
//			rt_lprintf("Storage:freespace.f_bfree=%u\n",freespace.f_bfree); /* free blocks in file system */
//			if(freespace.f_bfree < 1024/16)//剩余空间小于1/16=8MB时，删除LOG文件夹最老的日志  1024/16
//			{
//				Catalogue_Del_Oldest(LOG_PATH);
//			}
//		}
//		else
//		{
//			rt_lprintf("Storage:获取NANDFLASH剩余空间失败=%d\n",rval);	
//		}

		rt_thread_mdelay(1000);
	}
}
/*********************************************************************************************************
** Function name:		Log_Process
** Descriptions:		LOG函数
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/


extern rt_uint32_t LogBufferLen;
extern char LogBuffer[4096];

static int Log_Process(void)
{
	if(LogBufferLen > sizeof(LogBuffer)/2)	
	{	
		//存储到FLASH
		NAND_LogWrite(LogBuffer,&LogBufferLen);	
	}
	else
	{
//		rt_lprintf("[storage]:log未满\n");
	}
	
    return 0;	
}
/*********************************************************************************************************
** Function name:		NAND_LogWrite
** Descriptions:		LOG函数
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int NAND_LogWrite(char* Pnand_Log,rt_uint32_t *nand_LogLen)   //函数添加锁 为了在写文件时候防止丢log
{	   	   
	rt_uint8_t rval=0;			//返回值	
	
	/* 只写并在末尾添加打开 */
	int fd= open(NAND_LOG_PATH_FILE,O_WRONLY | O_CREAT | O_APPEND);
	if(fd >= 0)
	{
		rt_lprintf("[Storage]:%s文件打开成功\n",NAND_LOG_PATH_FILE);
	}
	else
	{
		strcpy((char*)Pnand_Log,"");//清空LogBuffer之后，strlen(LogBuffer) = 0;
		rt_lprintf("[Storage]:%s文件打开失败 fd=%d\n",NAND_LOG_PATH_FILE,fd);
		rval = 1;
		return rval;
	}
	
	static struct stat logstat;
	rval = fstat(fd,&logstat);
	if(rval == RT_EOK)
	{
		rt_lprintf("[Storage]:log字节数=%u\n",logstat.st_size);
	}
	else
	{
		rt_lprintf("[Storage]:log文件状态错误=%d\n",rval);	
	}
	
	int writelen = write(fd,Pnand_Log,*nand_LogLen);//写入首部   返回值0：成功	
	strcpy((char*)Pnand_Log,"");//清空USBLogBuffer之后，strlen(USBLogBuffer) = 0;
	if(writelen > 0)
	{
		rt_lprintf("[Storage]:log文件写入成功 writelen=%d\n",writelen);
	}
	else
	{
		extern rt_thread_t rt_current_thread;
		rt_lprintf("[Storage]:log文件写入失败 writelen=%d,error=%d\n",writelen,rt_current_thread->error);
		rval = 2;
	}
	
	if(close(fd) != UENOERR)
	{
		rt_lprintf("[Storage]:log文件关闭失败\n");
		rval = 3;
		return rval; 			
	}
	else
	{
		rt_lprintf("[Storage]:log文件关闭成功\n");
	}
		
	if(logstat.st_size > 3*1024*1024)//3MB
	{
		rt_lprintf("[Storage]:LOG文件>3MB\n");
		LOG_BackUP();
	}
	
	return rval;
}
/*
*********************************************************************************************************
*	函 数 名: LOG_BakeUP
*	功能说明: 将LOG文件移动到同目录下时间命令文件
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static int LOG_BackUP(void)
{
	rt_uint8_t rval=0;			//返回值	  
	char timestamp[32];
	char logBack_file[64];

	sprintf((char*)logBack_file,"%s","/LOG");	
	sprintf((char*)timestamp,"/%02X%02X%02X%02X%02X%02X.TXT",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
															 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
	strcat((char*)logBack_file,(const char*)timestamp);
	rval = unlink((const char*)logBack_file);//目标文件夹存在该文件,先删除
	if(rval == UENOERR)
	{
		rt_lprintf("%s 文件删除成功 fd=%d\n",logBack_file,rval);
	}
	else
	{
		rt_lprintf("%s 删除文件不存在 fd=%d\n",logBack_file,rval);
	}

	rval = rename((const char*)NAND_LOG_PATH_FILE,(const char*)logBack_file);//从源文件夹移动到目的文件夹  名字保持不变
	if(rval == UENOERR)
	{
		rt_lprintf("LOG_BackUP:%s to %s成功\n",NAND_LOG_PATH_FILE,logBack_file);
	}
	else
	{
		rt_lprintf("LOG_BackUP:%s to %s失败,rval=%u\n",NAND_LOG_PATH_FILE,logBack_file,rval);
	}			
	
	return rval;
}
/*********************************************************************************************************
** Function name:		Catalogue_Del_Oldest
** Descriptions:		
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Catalogue_Del_Oldest(const char *PATH)
{
	int result;
	DIR *dirp;
	struct dirent *fd;
	rt_uint32_t filenum = 0;
	char oldest_file[] = "9999-991231080808.TXT";//默认最老的文件
	char oldest_path_file[] = "/AAAAAAAAAAAA/BBBBBBBBBBBA/9999-991231080808.TXT";	
	
	/* 打开PATH 目录 */
	dirp = opendir(PATH);
	if(dirp == RT_NULL)
	{
		rt_lprintf("opendir %s error\n",PATH);
		result = -1;
	}
	else
	{
		rt_lprintf("opendir %s ok\n",PATH);
	}

	/* 读取目录 */
	while ((fd = readdir(dirp)) != RT_NULL)
	{
		rt_lprintf("found %s\n", fd->d_name);
		if(strcmp((const char*)oldest_file,(const char*)fd->d_name) > 0)
		{
			strcpy((char*)oldest_file,(const char*)fd->d_name);
		}
		rt_lprintf("oldest_file:%s\n",(const char*)oldest_file);
		filenum++;	
	}
	rt_lprintf("%s has %d files\n",PATH,filenum);

	sprintf((char*)oldest_path_file,"%s/",PATH);
	strcat((char*)oldest_path_file,(const char*)oldest_file);	
	result = unlink((const char*)oldest_path_file); //删除oldest_file 必须完整路径
	if(UENOERR == result)
	{
		rt_lprintf("%s 文件删除成功\n",oldest_path_file);
	}
	else
	{
		rt_lprintf("%s 删除文件不存在 result=%d\n",oldest_path_file,result);
	}
//	/* 获取目录流的读取位置 */
//	off_t offset = telldir(dirp);	
//	seekdir(dirp,offset);	
//	rt_lprintf("current offset=%d\n",offset);
//	/* 重设读取目录为开头位置 */
//	rewinddir(dirp);		

	/* 关闭目录 */
	result = closedir(dirp);	
	if(result == RT_NULL)
	{
		rt_lprintf("closedir %s ok\n",PATH);
	}
	else
	{
		rt_lprintf("closedir %s error\n",PATH);
		result = -2;
	}	
	return result;
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//	int result;
//	struct dirent *fd;
//	int filenum = 0;
//	char oldest_path_file[64];
//	char history_log[40][32]={0};

//	/* 打开目录 */
//	DIR *dirp = opendir(PATH);
//	if(dirp == RT_NULL)
//	{
//		rt_lprintf("opendir %s error\n",PATH);
//		result = -1;
//		return result;
//	}
//	else
//	{
//		rt_lprintf("opendir %s ok\n",PATH);
//	}

//	/* 读取目录 */
//	while ((fd = readdir(dirp)) != RT_NULL)
//	{
//		rt_lprintf("found %s\n", fd->d_name);
//		strcpy((char*)history_log[filenum],(const char*)fd->d_name);
//		rt_lprintf("history_log[%d]=%s\n",filenum,history_log[filenum]);
//		filenum++;
//	}
//	
//	rt_lprintf("filenum=%d\n",filenum);
//	char temp_file[] = "999999999999.TXT";//临时文件
//	for(int i=0;i<filenum-1;i++)
//	{
//		for(int j=0;j<filenum-i-1;j++)
//		{
//			if(strcmp((char*)history_log[j],(char*)history_log[j+1]) > 0) //寻找最老文件名在最前面
//			{
//				strcpy((char*)temp_file,(char*)history_log[j]);
//				strcpy((char*)(char*)history_log[j],(char*)history_log[j+1]);
//				strcpy((char*)(char*)history_log[j+1],(char*)temp_file);
//			}
//		}
//	}

//	for(int i=0;i<filenum;i++)
//	{
//		rt_lprintf("history_log[%d]=%s\n",i,history_log[i]);
//	}
//	
//	/* 更新需要的目标文件绝对路径 */
//	sprintf((char*)oldest_path_file,"%s/",PATH);
//	strcat((char*)oldest_path_file,(const char*)history_log[0]);
//	result = unlink((const char*)oldest_path_file); //删除oldest_file 必须完整路径
//	if(UENOERR == result)
//	{
//		rt_lprintf("%s 文件删除成功\n",oldest_path_file);
//	}
//	else
//	{
//		rt_lprintf("%s 删除文件不存在 result=%d\n",oldest_path_file,result);
//	}
//	
//	/* 关闭目录 */
//	result = closedir(dirp);	
//	if(result == RT_NULL)
//	{
//		rt_lprintf("closedir %s ok\n",PATH);
//	}
//	else
//	{
//		rt_lprintf("closedir %s error\n",PATH);
//		result = -2;
//	}	
//	return result;
}
/*********************************************************************************************************
** Function name:		Find_path_file
** Descriptions:		
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#define MAX_History_num_10    10
static int Find_path_file(const char *PATH,int number,char *pathfile,rt_uint32_t *filenum)
{
	int result;
	struct dirent *fd;
	*filenum = 0;
	
	char history_record[MAX_History_num_10][24]={0};	
	
	/* 打开目录 */
	DIR *dirp = opendir(PATH);
	if(dirp == RT_NULL)
	{
		rt_lprintf("opendir %s error\n",PATH);
		result = -1;
		return result;
	}
	else
	{
		rt_lprintf("opendir %s ok\n",PATH);
	}
	/* 读取目录文件个数 */
	while ((fd = readdir(dirp)) != RT_NULL)
	{
		(*filenum)++;
	}
	rt_lprintf("current filenum=%d\n",*filenum);
	/* 删除过多文件 */
	if(*filenum > MAX_History_num_10)
	{
		/* 关闭目录 */
		result = closedir(dirp);	
		if(result == RT_NULL)
		{
			rt_lprintf("closedir %s ok\n",PATH);
		}
		else
		{
			rt_lprintf("closedir %s error\n",PATH);
			result = -2;
			return result;
		}
		
		rt_lprintf("错误: too many filenum=%d\n",*filenum);
		int i;
		for(int i=0;i<*filenum - MAX_History_num_10;i++)
		{
			Catalogue_Del_Oldest(PATH);
		}
		/* 二次打开目录 */
		dirp = opendir(PATH);
		if(dirp == RT_NULL)
		{
			rt_lprintf("opendir %s error\n",PATH);
			result = -1;
			return result;
		}
		else
		{
			rt_lprintf("opendir %s ok\n",PATH);
		}		
	}
	else
	{
		/* 重设读取目录为开头位置 */
		rewinddir(dirp);
	}
	
	/* 清除filenum，读取目录 */
	(*filenum) = 0;
	while ((fd = readdir(dirp)) != RT_NULL)
	{
		rt_lprintf("found %s\n", fd->d_name);
		if(*filenum < MAX_History_num_10)
		{
			strcpy((char*)history_record[*filenum],(const char*)fd->d_name);
			rt_lprintf("history_record[%d]=%s\n",*filenum,history_record[*filenum]);
		}
		(*filenum)++;
	}
	/* 关闭目录 */
	result = closedir(dirp);	
	if(result == RT_NULL)
	{
		rt_lprintf("closedir %s ok\n",PATH);
	}
	else
	{
		rt_lprintf("closedir %s error\n",PATH);
		result = -2;
		return result;
	}	
	
	/* 冒泡排序法 */
	RT_ASSERT(*filenum <= MAX_History_num_10);
	char temp_file[] = "9999-999999999999.TXT";//临时文件
	for(int i=0;i<(*filenum)-1;i++)
	{
		for(int j=0;j<(*filenum)-i-1;j++)
		{
			if(strcmp((char*)history_record[j],(char*)history_record[j+1]) < 0) //寻找最新文件名在最前面
			{
				strcpy((char*)temp_file,(char*)history_record[j+1]);
				strcpy((char*)(char*)history_record[j+1],(char*)history_record[j]);
				strcpy((char*)(char*)history_record[j],(char*)temp_file);
			}
		}
	}
	/* 列出历史记录 */
	for(int i=0;i<(*filenum);i++)
	{
		rt_lprintf("history_record[%d]=%s\n",i,history_record[i]);
	}
	
	/* 更新返回的目标文件绝对路径 */
	sprintf((char*)pathfile,"%s/",PATH);
	strcat((char*)pathfile,(const char*)history_record[number]);
		
	return result;
}
/*********************************************************************************************************
** Function name:		Meter_Anolag_Storage
** Descriptions:		
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Meter_Anolag_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd)
{
	int fd,writelen = 0,readlen = 0;
	char buffer[64];
	ScmMeter_Analog* pMeter_Anolag = (ScmMeter_Analog*)Storage_Para;
	
	if(cmd == WRITE)//保存到本地write
	{
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"ulVol=%u\n",(rt_uint32_t)pMeter_Anolag->ulVol);// 交流电压
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulCur=%u\n",(rt_uint32_t)pMeter_Anolag->ulCur);// 交流电流
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulAcPwr=%u\n",(rt_uint32_t)pMeter_Anolag->ulAcPwr);// 瞬时有功功率
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulMeterTotal=%u\n",(rt_uint32_t)pMeter_Anolag->ulMeterTotal);//有功总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPwrFactor=%u\n",(rt_uint32_t)pMeter_Anolag->ulPwrFactor);//功率因数
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulFrequency=%u\n",(rt_uint32_t)pMeter_Anolag->ulFrequency);//频率
		strcat((char*)Para_Buff,(const char*)buffer);		
		
		if(strlen((const char*)Para_Buff)>MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			return -1;
		}
		else
		{
//			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}
		/* Opens the file, if it is existing. If not, a new file is created. */
		fd= open(file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("%s文件打开成功 fd=%d\n",file,fd);
		}
		else
		{
			rt_lprintf("%s文件打开失败 fd=%d\n",file,fd);
		}
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:文件写入成功 writelen=%d\n",writelen);
		}
		else
		{
			rt_lprintf("[storage]:文件写入失败 writelen=%d\n",writelen);
		}				
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:文件关闭失败\n");
		}			
	}
	else if(cmd == READ) //从本地read
	{
		fd= open(file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("%s文件打开成功\n",file);
		}
		else
		{
			rt_lprintf("%s文件打开失败 fd=%d\n",file,fd);
		}
		readlen=read(fd,Para_Buff,strlen(Para_Buff));	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:文件读取成功 readlen=%d\n",readlen);
		}
		else
		{
			rt_lprintf("[storage]:文件读取失败 readlen=%d\n",readlen);
		}
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:文件关闭失败\n");
		}
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen < RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:文件读取内容\n%s\n",Para_Buff);
		}
	
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0};        //最多记录32个字节	

////////////////////交流电压//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulVol")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)pMeter_Anolag->ulVol,1);//返回当前文件读指针
				rt_lprintf("ulVol=%u;\n",pMeter_Anolag->ulVol);
			}			
			else
			{
				rt_lprintf("文件名不符合 ulVol=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		} 			
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////交流电流////////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulCur")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)pMeter_Anolag->ulCur,1);//返回当前文件读指针
				rt_lprintf("ulCur=%u;\n",pMeter_Anolag->ulCur);
			}			
			else
			{
				rt_lprintf("文件名不符合 ulCur=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////瞬时有功功率////////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulAcPwr")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)pMeter_Anolag->ulAcPwr,1);//返回当前文件读指针
				rt_lprintf("ulAcPwr=%u;\n",pMeter_Anolag->ulAcPwr);
			}			
			else
			{
				rt_lprintf("文件名不符合 ulAcPwr=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////有功总电量////////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulMeterTotal")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)pMeter_Anolag->ulMeterTotal,1);//返回当前文件读指针
				rt_lprintf("ulMeterTotal=%u;\n",pMeter_Anolag->ulMeterTotal);
			}			
			else
			{
				rt_lprintf("文件名不符合 ulMeterTotal=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////功率因数////////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPwrFactor")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)pMeter_Anolag->ulPwrFactor,1);//返回当前文件读指针
				rt_lprintf("ulPwrFactor=%u;\n",pMeter_Anolag->ulPwrFactor);
			}			
			else
			{
				rt_lprintf("文件名不符合 ulPwrFactor=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////频率///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulFrequency")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)pMeter_Anolag->ulFrequency,1);//返回当前文件读指针
				rt_lprintf("ulFrequency=%u;\n",pMeter_Anolag->ulFrequency);
			}			
			else
			{
				rt_lprintf("[storage]:文件名不符合 ulFrequency=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("[storage]:namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:文件无效命令\n");
	}
	
	return 0;
}
/*********************************************************************************************************
** Function name:		Router_Para_Storage
** Descriptions:		
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Router_Para_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd)	
{
	int fd,writelen = 0,readlen = 0;
	char buffer[128];
	
	sprintf(buffer,"");
	
	if(cmd == WRITE)//保存到本地write
	{
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		strcat((char*)Para_Buff,(const char*)"cAssetNum=");
		for(int i=0;i<(sizeof(RouterInfo.AssetNum)-1);i++)//资产编号 22位
		{
			sprintf((char*)buffer,"%02X",*((char*)Storage_Para+i));// 表号
			strcat((char*)Para_Buff,(const char*)buffer);
		}
		strcat((char*)Para_Buff,(const char*)"\n");	

		strcat((char*)Para_Buff,(const char*)"cAddr=");
		for(int i=0;i<(sizeof(RouterInfo.Addr)-1);i++)//通讯地址12位
		{
			sprintf((char*)buffer,"%02X",*((char*)Storage_Para+i+sizeof(RouterInfo.AssetNum)));// 表号
			strcat((char*)Para_Buff,(const char*)buffer);
		}
		strcat((char*)Para_Buff,(const char*)"\n");			
		
		if(strlen((const char*)Para_Buff)>MAX_MALLOC_NUM)
		{
			rt_kprintf("[storage]: Para_Buff overflow\n");
			
			return -1;
		}
		else
		{
			rt_kprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}
		
/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd= open(file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_kprintf("[storage]:%s文件打开成功\n",file);
		}
		else
		{
			rt_kprintf("[storage]:%s文件打开失败 fd=%d\n",file,fd);

			return -2; 			
		}		
/************************************************************************************************/			
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_kprintf("[storage]:文件写入成功 writelen=%d\n",writelen);
		}
		else
		{
			rt_kprintf("[storage]:文件写入失败 writelen=%d\n",writelen);
		}				
		if(close(fd) != UENOERR)
		{
			rt_kprintf("[storage]:文件关闭失败\n");
		}
		return datalen;		
	}
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		fd= open(file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_kprintf("[storage]:%s文件打开成功\n",file);
		}
		else
		{
			rt_kprintf("[storage]:%s文件打开失败 fd=%d\n",file,fd);
		}
        /***************************************************************************************/
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_kprintf("[storage]:文件读取成功 readlen=%d\n",readlen);
		}
		else
		{
			rt_kprintf("[storage]:文件读取失败 readlen=%d\n",readlen);
		}
		if(close(fd) != UENOERR)
		{
			rt_kprintf("[storage]:文件关闭失败\n");
		}
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_kprintf("[storage]:文件读取内容\n%s\n",Para_Buff);
		}
	
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0};        //最多记录32个字节	

////////////////////资产编号//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"cAssetNum")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)(&RouterInfo.AssetNum),sizeof(RouterInfo.AssetNum)-1);//返回当前文件读指针
				for(int i=0;i<sizeof(RouterInfo.AssetNum);i++)
				{
					rt_kprintf((char*)buffer,"0x%02X",*((char*)RouterInfo.AssetNum+i));// 表号
				}
				rt_kprintf("\n");
			}			
			else
			{
				rt_kprintf("[storage]:文件名不符合 ulVol=%s\n",fpname); 
			}
			namelen=0;		
		}
		else
		{
            rt_kprintf("[storage]:namelen=0\n");
		}
		
		////////////////////通讯地址//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"cAddr")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)(&RouterInfo.Addr),(sizeof(RouterInfo.Addr)-1));//返回当前文件读指针
				for(int i=0;i<sizeof(RouterInfo.Addr);i++)
				{
					rt_kprintf((char*)buffer,"0x%02X",*((char*)RouterInfo.Addr));// 表号
				}
				rt_kprintf("\n");
			}			
			else
			{
				rt_kprintf("[storage]:文件名不符合 ulVol=%s\n",fpname); 
			}
			namelen=0;		
		}
		else
		{
            rt_kprintf("[storage]:namelen=0\n");
		}
		return readlen/2;	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_kprintf("[storage]:文件无效命令\n");
		return 0;	
	}	
}
/*********************************************************************************************************
** Function name:		Meter_GJFMode_Storage
** Descriptions:		LOG函数
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Meter_GJFMode_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd)	
{
	int fd,writelen = 0,readlen = 0;
	char buffadd[32];
	int i = 0;
	ScmMeter_PriceModle* pMeter_GJFMode = (ScmMeter_PriceModle*)Storage_Para;

	rt_lprintf("Meter_GJFMode_Storage:file = %s\n",(char*)file);	
	
	if(cmd == WRITE)//保存到本地write
	{
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffadd,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",\
								System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
								System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffadd);

		/****************************************************************************************/	
			//计费模型ID
			sprintf((char*)buffadd,"uiJFmodID=%02X%02X%02X%02X%02X%02X%02X%02X\n",\
									pMeter_GJFMode->uiJFmodID[0],pMeter_GJFMode->uiJFmodID[1],\
									pMeter_GJFMode->uiJFmodID[2],pMeter_GJFMode->uiJFmodID[3],
									pMeter_GJFMode->uiJFmodID[4],pMeter_GJFMode->uiJFmodID[5],\
									pMeter_GJFMode->uiJFmodID[6],pMeter_GJFMode->uiJFmodID[7]); 
			strcat((char*)Para_Buff,(const char*)buffadd);
		/****************************************************************************************/
			//生效时间	
			sprintf((char*)buffadd,"EffectiveTime=%02X%02X%02X%02X%02X%02X\n",\
									pMeter_GJFMode->EffectiveTime.Second,\
									pMeter_GJFMode->EffectiveTime.Minute,\
									pMeter_GJFMode->EffectiveTime.Hour,\
									pMeter_GJFMode->EffectiveTime.Day,\
									pMeter_GJFMode->EffectiveTime.Month,\
									pMeter_GJFMode->EffectiveTime.Year); 
			strcat((char*)Para_Buff,(const char*)buffadd);				
		/****************************************************************************************/	
		/****************************************************************************************/
			//失效时间	
			sprintf((char*)buffadd,"unEffectiveTime=%02X%02X%02X%02X%02X%02X\n",\
									pMeter_GJFMode->unEffectiveTime.Second,\
									pMeter_GJFMode->unEffectiveTime.Minute,\
									pMeter_GJFMode->unEffectiveTime.Hour,\
									pMeter_GJFMode->unEffectiveTime.Day,\
									pMeter_GJFMode->unEffectiveTime.Month,\
									pMeter_GJFMode->unEffectiveTime.Year); 
			strcat((char*)Para_Buff,(const char*)buffadd);		
		/****************************************************************************************/
		/****************************************************************************************/	
			//执行状态
			sprintf((char*)buffadd,"state=%u\n",pMeter_GJFMode->state);
			strcat((char*)Para_Buff,(const char*)buffadd);
		/****************************************************************************************/
			//计量类型
			sprintf((char*)buffadd,"style=%u\n",pMeter_GJFMode->style);
			strcat((char*)Para_Buff,(const char*)buffadd);		
		/****************************************************************************************/			
			//48个费率号 0:00~24:00
			for(i=0;i<48;i++)
			{
				sprintf((char*)buffadd,"ulTimeNo[%u]=%u\n",i,pMeter_GJFMode->ulTimeNo[i]); 
				strcat((char*)Para_Buff,(const char*)buffadd);			
			}
		/****************************************************************************************/
			//有效费率数
			sprintf((char*)buffadd,"count=%u\n",pMeter_GJFMode->count); //存储计费模型费率个数
			strcat((char*)Para_Buff,(const char*)buffadd);
		/****************************************************************************************/	
			// count个电价
			for(i=0;i<pMeter_GJFMode->count;i++)
			{
				sprintf((char*)buffadd,"ulPriceNo[%u]=%u\n",i,(rt_uint32_t)pMeter_GJFMode->ulPriceNo[i]); 
				strcat((char*)Para_Buff,(const char*)buffadd);		
				
			}
		/****************************************************************************************/
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)>MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",file,fd);
			return -2;		
		}
/************************************************************************************************/		
		struct dfs_fd *fp= fd_get(fd);//与fd_put配套使用否则打开文件过多
		fd_put(fp);
		rt_lprintf("Storage:fp->pos=%d,fp->size=%d\n",fp->pos,fp->size);
/************************************************************************************************/			
/************************************************************************************************/				
		if(fp->size <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:文件写内容\n%s\n",Para_Buff);
		}
		
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:文件关闭失败\n");
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		fd= open(file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",file,fd);
			return -2;
		}
        /***************************************************************************************/	
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:计费模型文件读取成功 readlen=%d\n",readlen);
		}
		else
		{
			rt_lprintf("[storage]:计费模型文件读取失败 readlen=%d\n",readlen);
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:文件关闭失败\n");
		}
		
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:文件读取内容\n%s\n",Para_Buff);
		}

		/************************************************************************************/		
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////
////////////////////需要升级uiJFmodID//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);
		if(namelen)
		{
			sprintf((char*)fpnameRd,"uiJFmodID"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->uiJFmodID,8);
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pMeter_GJFMode->uiJFmodID[0],pMeter_GJFMode->uiJFmodID[1],pMeter_GJFMode->uiJFmodID[2],pMeter_GJFMode->uiJFmodID[3],\
							pMeter_GJFMode->uiJFmodID[4],pMeter_GJFMode->uiJFmodID[5],pMeter_GJFMode->uiJFmodID[6],pMeter_GJFMode->uiJFmodID[7]);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");	
		}
//////////////////生效时间//////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"EffectiveTime"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->EffectiveTime.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pMeter_GJFMode->EffectiveTime.Year,\
							pMeter_GJFMode->EffectiveTime.Month,\
							pMeter_GJFMode->EffectiveTime.Day,\
							pMeter_GJFMode->EffectiveTime.Hour,\
							pMeter_GJFMode->EffectiveTime.Minute,\
							pMeter_GJFMode->EffectiveTime.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
//////////////////失效时间//////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"unEffectiveTime"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->unEffectiveTime.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pMeter_GJFMode->unEffectiveTime.Year,\
							pMeter_GJFMode->unEffectiveTime.Month,\
							pMeter_GJFMode->unEffectiveTime.Day,\
							pMeter_GJFMode->unEffectiveTime.Hour,\
							pMeter_GJFMode->unEffectiveTime.Minute,\
							pMeter_GJFMode->unEffectiveTime.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
////////////////////////////////////////////////////////////////		
///////////////////执行状态/////////////////////////////////////////////////			
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"state"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)			
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->state,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pMeter_GJFMode->state);
			}			
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
///////////////////计量类型/////////////////////////////////////////////////			
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"style"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->style,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pMeter_GJFMode->style);
			}			
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}		
////////////////////计费模型/////////////////////////////////////////////////////////////
		for(i=0;i<48;i++)
		{
			sprintf((char*)fpnameRd,"ulTimeNo[%u]",i); 
		    fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->ulTimeNo[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pMeter_GJFMode->ulTimeNo[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}
		}
///////////////////计费模型费率个数/////////////////////////////////////////////////			
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"count"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->count,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pMeter_GJFMode->count);
			}			
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		} 		
/////////////////////////////////////////////////////////////////////////////////////////
////////////////////计费模型费率价格//////////////////////////////////////////////////////////
		for(i=0;i<pMeter_GJFMode->count;i++)//
		{
			sprintf((char*)fpnameRd,"ulPriceNo[%u]",i); 

		    fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_GJFMode->ulPriceNo[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pMeter_GJFMode->ulPriceNo[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}
		}
        rt_lprintf("pMeter_GJFMode file read OK\n");		
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:文件无效命令\n");
	}

	return 0;	
	
}
/*********************************************************************************************************
** Function name:		Meter_HalfPower_Storage
** Descriptions:		Meter_HalfPower_Storage函数
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Meter_HalfPower_Storage(const char *file,void *Storage_Para,rt_uint32_t datalen,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffadd[32];
	int i = 0;
	rt_uint32_t* ulMeter_Half = (rt_uint32_t*)Storage_Para;

	rt_lprintf("Meter_HalfPower_Storage:file = %s\n",(char*)file);	
	
	if(cmd == WRITE)//保存到本地write
	{
		int writelen = 0;
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffadd,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",\
								System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
								System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffadd);

		/****************************************************************************************/			
		//48个HalfPower 0:00~24:00
		for(i=0;i<48;i++)
		{
			sprintf((char*)buffadd,"ulMeter_Half[%u]=%u\n",i,ulMeter_Half[i]); 
			strcat((char*)Para_Buff,(const char*)buffadd);			
		}
		/****************************************************************************************/
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)>MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",file,fd);

			return -2; 			
		}
/************************************************************************************************/		
		struct dfs_fd *fp= fd_get(fd);//与fd_put配套使用否则打开文件过多
		fd_put(fp);
		rt_lprintf("Storage:fp->pos=%d,fp->size=%d\n",fp->pos,fp->size);
/************************************************************************************************/			
/************************************************************************************************/
		if(fp->size <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:文件写内容\n%s\n",Para_Buff);
		}
		
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:文件关闭失败\n");
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		fd= open(file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",file,fd);
			return -2; 
		}
        /***************************************************************************************/	
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:半小时电量文件读取成功 readlen=%d\n",readlen);
		}
		else
		{
			rt_lprintf("[storage]:半小时电量文件读取失败 readlen=%d\n",readlen);
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:文件关闭失败\n");
		}
		
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:半小时电量文件读取内容\n%s\n",Para_Buff);
		}

		/************************************************************************************/		
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////半小时电量////////////////////////////////////////////////////////////
		for(i=0;i<48;i++)
		{
			sprintf((char*)fpnameRd,"ulMeter_Half[%u]",i); 
		    fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&ulMeter_Half[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,ulMeter_Half[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}	

		}
        rt_lprintf("ulMeter_Half file read OK\n");		
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:文件无效命令\n");
	}

	return 0;	
	
}
/*********************************************************************************************************
** Function name:		Meter_Power_Storage
** Descriptions:		LOG函数
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Meter_Power_Storage(const char *file,void *Storage_Para,rt_uint32_t YMD,rt_uint32_t cmd)	
{
	int fd,writelen = 0,readlen = 0;
	char buffer[256]; 
	char path[14];
	char path_file[64];
	ScmMeter_HisData* pMeter_HisData = (ScmMeter_HisData*)Storage_Para;

	sprintf((char*)path_file,"%s",METER_POWER_PATH);	
	sprintf((char*)path,"/%06X.txt",YMD>>8);
	strcat((char*)path_file,(const char*)path);
	rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);	
	
	if(cmd == WRITE)//保存到本地write
	{
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"%08X\n",YMD);// 日-20190801
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"ulPowerTday=%06u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Day.ulPowerT);// 日-总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerJday=%06u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Day.ulPowerJ);// 日-尖
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerFday=%06u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Day.ulPowerF);// 日-峰
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerPday=%06u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Day.ulPowerP);// 日-平
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerGday=%06u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Day.ulPowerG);// 日-谷
		strcat((char*)Para_Buff,(const char*)buffer);
        /****************************************************************************************/
		sprintf((char*)buffer,"ulPowerTmon=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Month.ulPowerT);// 月-总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerJmon=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Month.ulPowerJ);// 月-总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerFmon=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Month.ulPowerF);// 月-总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerPmon=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Month.ulPowerP);// 月-总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerGmon=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Month.ulPowerG);// 月-总电量
		strcat((char*)Para_Buff,(const char*)buffer);	
		
		        /****************************************************************************************///wyg191112
		sprintf((char*)buffer,"ulPowerTotal=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Total.ulPowerT);// 总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerJTotal=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Total.ulPowerJ);// 总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerFTotal=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Total.ulPowerF);// 总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerPTotal=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Total.ulPowerP);// 总电量
		strcat((char*)Para_Buff,(const char*)buffer);

		sprintf((char*)buffer,"ulPowerGTotal=%08u\n",(rt_uint32_t)pMeter_HisData->ulMeter_Total.ulPowerG);// 总电量
		strcat((char*)Para_Buff,(const char*)buffer);
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_RDWR | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
/************************************************************************************************/		
		struct dfs_fd *fp= fd_get(fd);//与fd_put配套使用否则打开文件过多
		fd_put(fp);
		rt_lprintf("Storage:fp->pos=%d,fp->size=%d\n",fp->pos,fp->size);
/************************************************************************************************/
		if(fp->size > 0)
		{
			char* Para_Bufftemp = rt_malloc(fp->size);//申请内存
			if(Para_Bufftemp == NULL)
			{
				rt_lprintf("[storage]:Para_Bufftemp 分配内存失败\n");
				if(close(fd) != UENOERR)
				{
					rt_lprintf("[storage]:%s文件关闭失败\n",file);
				}			
				return -3;
			}		
			readlen=read(fd,Para_Bufftemp,fp->size);	//读出txt里面的内容	
			if(readlen > 0) //0
			{
				rt_lprintf("[storage]:电表电量文件读取成功 readlen=%d\n",readlen);
			}
			else
			{
				rt_lprintf("[storage]:电表电量文件读取为空 readlen=%d\n",readlen);
			}
			/******文件不为空才进行查找哪天电量，否则直接写当天电量****************************/
			if(readlen > 0) //0
			{
				char ymd[] ="20180808";
				sprintf((char*)ymd,"%08X",YMD);// 日-20190801	
				char *addr = strstr(Para_Bufftemp,ymd);
				if(addr == RT_NULL)
				{
					rt_lprintf("[storage]:%s未找到\n",ymd);		
				}
				else
				{
					rt_lprintf("[storage]:%s找到\n",ymd);
					rt_uint32_t addr_offset = 0x00;
					addr_offset = addr - Para_Bufftemp;
					rt_lprintf("[storage]:addr_offset=%d\n",addr_offset);			
					lseek(fd,addr_offset-23,SEEK_SET);//偏移到指定位置 23：标识更新最开始的时间戳		
				}
				if(readlen <= RT_CONSOLEBUF_SIZE)
				{
					rt_lprintf("[storage]:文件读内容\n%s\n",Para_Bufftemp);
				}		
				rt_free(Para_Bufftemp);Para_Bufftemp = NULL;
			}
		}		
/************************************************************************************/				
/**********更新写当天电量**************************************************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/
/************************************************************************************************/		
		struct dfs_fd *fp= fd_get(fd);//与fd_put配套使用否则打开文件过多
		fd_put(fp);
		rt_lprintf("Storage:fp->pos=%d,fp->size=%d\n",fp->pos,fp->size);
		if(fp->size == NULL)
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件为空,直接返回\n",path_file);
			return -1;
		}		
/************************************************************************************************/		
		char* Para_Bufftemp = rt_malloc(fp->size);//申请内存
		if(Para_Bufftemp == NULL)
		{
			rt_lprintf("[storage]:Para_Bufftemp 分配内存失败\n");
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",file);
			}	
			return -2;
		}		
		readlen=read(fd,Para_Bufftemp,fp->size);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",file);
			}
			rt_free(Para_Bufftemp);Para_Bufftemp = NULL;
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/******文件不为空才进行查找哪天电量,否则直接写当天电量****************************/
		char ymd[] ="20180808";
		sprintf((char*)ymd,"%08X",YMD);// 日-20190801	
		char *addr = strstr(Para_Bufftemp,ymd);
		if(addr == RT_NULL)
		{
			rt_lprintf("[storage]:%s未找到\n",ymd);	
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",file);
			}
			rt_free(Para_Bufftemp);Para_Bufftemp = NULL;
			return -4;				
		}
		else
		{
			rt_lprintf("[storage]:%s找到\n",ymd);
			rt_uint32_t addr_offset = 0x00;
			addr_offset = addr - Para_Bufftemp;
			rt_lprintf("[storage]:addr_offset=%d\n",addr_offset);			
			lseek(fd,addr_offset-23,SEEK_SET);//偏移到指定位置 23：标识更新最开始的时间戳		
		}
		*(Para_Bufftemp+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:文件写内容\n%s\n",Para_Bufftemp);
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = addr;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0};        //最多记录32个字节	
////////////////////日--总电量//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerTday")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Day.ulPowerT,1);//返回当前文件读指针
				rt_lprintf("ulPowerTday=%u;\n",pMeter_HisData->ulMeter_Day.ulPowerT);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerTday=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
			rt_lprintf("namelen=0\n");
		} 			
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////日--尖电量////////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerJday")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Day.ulPowerJ,1);//返回当前文件读指针
				rt_lprintf("ulPowerJday=%u;\n",pMeter_HisData->ulMeter_Day.ulPowerJ);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerJday=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
			rt_lprintf("namelen=0\n");
		} 
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////日--峰电量////////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerFday")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Day.ulPowerF,1);//返回当前文件读指针
				rt_lprintf("ulPowerFday=%u;\n",pMeter_HisData->ulMeter_Day.ulPowerF);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerFday=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
			rt_lprintf("namelen=0\n");
		} 
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////日--平电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerPday")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Day.ulPowerP,1);//返回当前文件读指针
				rt_lprintf("ulPowerPday=%u;\n",pMeter_HisData->ulMeter_Day.ulPowerP);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerPday=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
			rt_lprintf("namelen=0\n");
		} 
/////////////////////////////////////////////////////////////////////////////////////////
////////////////////日--谷电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerGday")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Day.ulPowerG,1);//返回当前文件读指针
				rt_lprintf("ulPowerGday=%u;\n",pMeter_HisData->ulMeter_Day.ulPowerG);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerGday=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
			rt_lprintf("namelen=0\n");
		} 
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////月--总电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerTmon")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Month.ulPowerT,1);//返回当前文件读指针
				rt_lprintf("ulPowerTmon=%u;\n",pMeter_HisData->ulMeter_Month.ulPowerT);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerTmon=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////月--尖电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerJmon")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Month.ulPowerJ,1);//返回当前文件读指针
				rt_lprintf("ulPowerJmon=%u;\n",pMeter_HisData->ulMeter_Month.ulPowerJ);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerJmon=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////月--峰电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerFmon")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Month.ulPowerF,1);//返回当前文件读指针
				rt_lprintf("ulPowerFmon=%u;\n",pMeter_HisData->ulMeter_Month.ulPowerF);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerFmon=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////月--平电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerPmon")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Month.ulPowerP,1);//返回当前文件读指针
				rt_lprintf("ulPowerPmon=%u;\n",pMeter_HisData->ulMeter_Month.ulPowerP);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerPmon=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////月--谷电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerGmon")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Month.ulPowerG,1);//返回当前文件读指针
				rt_lprintf("ulPowerGmon=%u;\n",pMeter_HisData->ulMeter_Month.ulPowerG);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerGmon=%s\n",fpname); 
			}
			namelen=0;
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////总电量/////////////////////////////////////////////////////////////////wyg191112
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerTotal")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Total.ulPowerT,1);//返回当前文件读指针
				rt_lprintf("ulPowerTotal=%u;\n",pMeter_HisData->ulMeter_Total.ulPowerT);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerTotal=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////尖总电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerJTotal")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Total.ulPowerJ,1);//返回当前文件读指针
				rt_lprintf("ulPowerJTotal=%u;\n",pMeter_HisData->ulMeter_Total.ulPowerJ);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerJTotal=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////峰总电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerFTotal")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Total.ulPowerF,1);//返回当前文件读指针
				rt_lprintf("ulPowerFTotal=%u;\n",pMeter_HisData->ulMeter_Total.ulPowerF);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerFTotal=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////平总电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerPTotal")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Total.ulPowerP,1);//返回当前文件读指针
				rt_lprintf("ulPowerPTotal=%u;\n",pMeter_HisData->ulMeter_Total.ulPowerP);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerPTotal=%s\n",fpname); 
			}
			namelen=0;			
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////谷总电量///////////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)"ulPowerGTotal")==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pMeter_HisData->ulMeter_Total.ulPowerG,1);//返回当前文件读指针
				rt_lprintf("ulPowerGTotal=%u;\n",pMeter_HisData->ulMeter_Total.ulPowerG);
			}			
			else        
			{
				rt_lprintf("文件名不符合 ulPowerGTotal=%s\n",fpname); 
			}
			namelen=0;
		}
		else
		{
            rt_lprintf("namelen=0\n");
		}
		rt_free(Para_Bufftemp);Para_Bufftemp = NULL;		
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:文件无效命令\n");
	}

	return 0;	
}
/*********************************************************************************************************
** Function name:		Charge_Record_Storage
** Descriptions:		LOG函数
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Charge_Record_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffer[48]; 
	char path_file[64];
	CHG_ORDER_EVENT* pCharge_Record = (CHG_ORDER_EVENT*)Storage_Para;
		
	if(cmd == WRITE)//保存到本地write
	{
		// 创建文件名
		int writelen = 0;
		char path[18];
		sprintf((char*)path_file,"%s",PATH);
		sprintf((char*)path,"/%04u-%02X%02X%02X%02X%02X%02X.txt",(rt_uint32_t)pCharge_Record->OrderNum,\
																			  pCharge_Record->StartTimestamp.Second,\
																			  pCharge_Record->StartTimestamp.Minute,\
																			  pCharge_Record->StartTimestamp.Hour,\
																			  pCharge_Record->StartTimestamp.Day,\
																			  pCharge_Record->StartTimestamp.Month,\
																			  pCharge_Record->StartTimestamp.Year);
		strcat((char*)path_file,(const char*)path);
		rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);		
		
		// 准备写的内容
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"OrderNum=%04u\n",(rt_uint32_t)pCharge_Record->OrderNum);//	事件记录序号
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"StartTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pCharge_Record->StartTimestamp.Second,\
																		  (rt_uint32_t)pCharge_Record->StartTimestamp.Minute,\
																		  (rt_uint32_t)pCharge_Record->StartTimestamp.Hour,\
																		  (rt_uint32_t)pCharge_Record->StartTimestamp.Day,\
																		  (rt_uint32_t)pCharge_Record->StartTimestamp.Month,\
																		  (rt_uint32_t)pCharge_Record->StartTimestamp.Year);//  事件发生时间
		strcat((char*)Para_Buff,(const char*)buffer);		
		
		sprintf((char*)buffer,"FinishTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pCharge_Record->FinishTimestamp.Second,\
																		   (rt_uint32_t)pCharge_Record->FinishTimestamp.Minute,\
																		   (rt_uint32_t)pCharge_Record->FinishTimestamp.Hour,\
																		   (rt_uint32_t)pCharge_Record->FinishTimestamp.Day,\
																		   (rt_uint32_t)pCharge_Record->FinishTimestamp.Month,\
																		   (rt_uint32_t)pCharge_Record->FinishTimestamp.Year);//  事件结束时间
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件发生原因
		sprintf((char*)buffer,"OccurSource=%03u\n",(rt_uint32_t)pCharge_Record->OccurSource); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件上报状态 = 通道上报状态
		sprintf((char*)buffer,"ChannelState=%03u\n",(rt_uint32_t)pCharge_Record->ChannelState);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 用户id  visible-string（SIZE(64)）
		sprintf((char*)buffer,"cUserID=");
		char bytebuf[] = "FF";
		for(int i=0;i<sizeof(pCharge_Record->cUserID);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pCharge_Record->cUserID[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");		
		
		// 充电申请单号（SIZE(16)）
		sprintf((char*)buffer,"RequestNO=");
		for(int i=0;i<sizeof(pCharge_Record->RequestNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pCharge_Record->RequestNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");
	
		//	路由器资产编号 visible-string（SIZE(22)）
		sprintf((char*)buffer,"AssetNO=");
		for(int i=0;i<sizeof(pCharge_Record->AssetNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pCharge_Record->AssetNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");

		// 枪序号{A枪（1）、B枪（2）}
		sprintf((char*)buffer,"GunNum=%03u\n",(rt_uint32_t)pCharge_Record->GunNum); 
		strcat((char*)Para_Buff,(const char*)buffer);

		//	充电需求电量（单位：kWh，换算：-2）
		sprintf((char*)buffer,"ChargeReqEle=%08u\n",(rt_uint32_t)pCharge_Record->ChargeReqEle);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		//	充电申请时间
		sprintf((char*)buffer,"RequestTimeStamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pCharge_Record->RequestTimeStamp.Second,\
																		    (rt_uint32_t)pCharge_Record->RequestTimeStamp.Minute,\
																		    (rt_uint32_t)pCharge_Record->RequestTimeStamp.Hour,\
																		    (rt_uint32_t)pCharge_Record->RequestTimeStamp.Day,\
																		    (rt_uint32_t)pCharge_Record->RequestTimeStamp.Month,\
																		    (rt_uint32_t)pCharge_Record->RequestTimeStamp.Year);//	充电申请时间
		strcat((char*)Para_Buff,(const char*)buffer);	
		//	计划用车时间
		sprintf((char*)buffer,"PlanUnChg_TimeStamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pCharge_Record->PlanUnChg_TimeStamp.Second,\
																		       (rt_uint32_t)pCharge_Record->PlanUnChg_TimeStamp.Minute,\
																		       (rt_uint32_t)pCharge_Record->PlanUnChg_TimeStamp.Hour,\
																		       (rt_uint32_t)pCharge_Record->PlanUnChg_TimeStamp.Day,\
																		       (rt_uint32_t)pCharge_Record->PlanUnChg_TimeStamp.Month,\
																		       (rt_uint32_t)pCharge_Record->PlanUnChg_TimeStamp.Year);//	计划用车时间
		strcat((char*)Para_Buff,(const char*)buffer);		

		//	充电模式 {正常（0），有序（1）}
		sprintf((char*)buffer,"ChargeMode=%08u\n",(rt_uint32_t)pCharge_Record->ChargeMode);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		//	启动时电表表底值
		for(int i=0;i<sizeof(pCharge_Record->StartMeterValue);i++)
		{
			sprintf((char*)buffer,"StartMeterValue[%d]=",i);
			sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Record->StartMeterValue[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
	
		
		//	停止时电表表底值
		for(int i=0;i<sizeof(pCharge_Record->StopMeterValue);i++)
		{
			sprintf((char*)buffer,"StopMeterValue[%d]=",i);
			sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Record->StopMeterValue[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		
		//	充电启动时间
		sprintf((char*)buffer,"ChgStartTime=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pCharge_Record->ChgStartTime.Second,\
																		(rt_uint32_t)pCharge_Record->ChgStartTime.Minute,\
																		(rt_uint32_t)pCharge_Record->ChgStartTime.Hour,\
																		(rt_uint32_t)pCharge_Record->ChgStartTime.Day,\
																		(rt_uint32_t)pCharge_Record->ChgStartTime.Month,\
																		(rt_uint32_t)pCharge_Record->ChgStartTime.Year);	
		//	充电停止时间
		sprintf((char*)buffer,"ChgStopTime=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pCharge_Record->ChgStopTime.Second,\
																	   (rt_uint32_t)pCharge_Record->ChgStopTime.Minute,\
																	   (rt_uint32_t)pCharge_Record->ChgStopTime.Hour,\
																	   (rt_uint32_t)pCharge_Record->ChgStopTime.Day,\
																	   (rt_uint32_t)pCharge_Record->ChgStopTime.Month,\
																	   (rt_uint32_t)pCharge_Record->ChgStopTime.Year);	

		//	已充电量（单位：kWh，换算：-2）
		for(int i=0;i<sizeof(pCharge_Record->ucChargeEle);i++)
		{
			sprintf((char*)buffer,"ucChargeEle[%d]=",i);
			sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Record->ucChargeEle[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		
		//	已充时间（单位：s）	
		sprintf((char*)buffer,"ucChargeTime=%08u\n",(rt_uint32_t)pCharge_Record->ucChargeTime);
		strcat((char*)Para_Buff,(const char*)buffer);		
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
		/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}		
		/************************************************************************************/				
		/**********更新充电订单事件记录单元**************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		rt_uint32_t file_num;
		Find_path_file(PATH,ordernum,path_file,&file_num);

		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/			
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/***************************************************************************************/
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:%s文件读内容\n%s\n",path_file,Para_Buff);
		}		
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0,};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0,};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件记录序号/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OrderNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->OrderNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->OrderNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////事件发生时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"StartTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->StartTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Record->StartTimestamp.Year,\
							pCharge_Record->StartTimestamp.Month,\
							pCharge_Record->StartTimestamp.Day,\
							pCharge_Record->StartTimestamp.Hour,\
							pCharge_Record->StartTimestamp.Minute,\
							pCharge_Record->StartTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件结束时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"FinishTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->FinishTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Record->FinishTimestamp.Year,\
							pCharge_Record->FinishTimestamp.Month,\
							pCharge_Record->FinishTimestamp.Day,\
							pCharge_Record->FinishTimestamp.Hour,\
							pCharge_Record->FinishTimestamp.Minute,\
							pCharge_Record->FinishTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
////////////////////事件发生原因/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OccurSource"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->OccurSource,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->OccurSource);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////事件上报状态 = 通道上报状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChannelState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->ChannelState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->ChannelState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////用户id  visible-string（SIZE(64)）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"cUserID"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->cUserID,sizeof(pCharge_Record->cUserID));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pCharge_Record->cUserID);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////充电申请单号 SIZE(16)/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"RequestNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->RequestNO,sizeof(pCharge_Record->RequestNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pCharge_Record->RequestNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////路由器资产编号 visible-string（SIZE(22)）////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"AssetNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->AssetNO,sizeof(pCharge_Record->AssetNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pCharge_Record->AssetNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////枪序号	{A枪（1）、B枪（2）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"GunNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->GunNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->GunNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////充电需求电量（单位：kWh，换算：-2）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChargeReqEle"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->ChargeReqEle,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->ChargeReqEle);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////充电申请时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"RequestTimeStamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->RequestTimeStamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Record->RequestTimeStamp.Year,\
							pCharge_Record->RequestTimeStamp.Month,\
							pCharge_Record->RequestTimeStamp.Day,\
							pCharge_Record->RequestTimeStamp.Hour,\
							pCharge_Record->RequestTimeStamp.Minute,\
							pCharge_Record->RequestTimeStamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
////////////////////计划用车时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"PlanUnChg_TimeStamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->PlanUnChg_TimeStamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Record->PlanUnChg_TimeStamp.Year,\
							pCharge_Record->PlanUnChg_TimeStamp.Month,\
							pCharge_Record->PlanUnChg_TimeStamp.Day,\
							pCharge_Record->PlanUnChg_TimeStamp.Hour,\
							pCharge_Record->PlanUnChg_TimeStamp.Minute,\
							pCharge_Record->PlanUnChg_TimeStamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
/////////充电模式 {正常（0），有序（1）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChargeMode"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->ChargeMode,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->ChargeMode);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////启动时电表表底值////////////////////////////////////////////////////////////////////////////////
		for(int i=0;i<sizeof(pCharge_Record->StartMeterValue);i++)
		{
			sprintf((char*)fpnameRd,"StartMeterValue[%d]=",i);
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->StartMeterValue[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->StartMeterValue[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}			
		}		
/////////停止时电表表底值////////////////////////////////////////////////////////////////////////////////
		for(int i=0;i<sizeof(pCharge_Record->StopMeterValue);i++)
		{
			sprintf((char*)fpnameRd,"StopMeterValue[%d]=",i);
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->StopMeterValue[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->StopMeterValue[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}			
		}
////////////////////充电启动时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"ChgStartTime"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->ChgStartTime.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Record->ChgStartTime.Year,\
							pCharge_Record->ChgStartTime.Month,\
							pCharge_Record->ChgStartTime.Day,\
							pCharge_Record->ChgStartTime.Hour,\
							pCharge_Record->ChgStartTime.Minute,\
							pCharge_Record->ChgStartTime.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
////////////////////充电停止时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"ChgStopTime"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->ChgStopTime.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Record->ChgStopTime.Year,\
							pCharge_Record->ChgStopTime.Month,\
							pCharge_Record->ChgStopTime.Day,\
							pCharge_Record->ChgStopTime.Hour,\
							pCharge_Record->ChgStopTime.Minute,\
							pCharge_Record->ChgStopTime.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
/////////已充电量（单位：kWh，换算：-2）////////////////////////////////////////////////////////////////////////////////	
		for(int i=0;i<sizeof(pCharge_Record->ucChargeEle);i++)
		{
			sprintf((char*)fpnameRd,"ucChargeEle[%d]=",i);
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->ucChargeEle[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->ucChargeEle[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}			
		}
////////已充时间（单位：s）	///////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucChargeTime"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Record->ucChargeTime,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Record->ucChargeTime);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
		rt_lprintf("[storage]:%s文件写成功\n",path_file);
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:%s文件无效命令\n",path_file);
	}

	return 0;	
}
/*********************************************************************************************************
** Function name:		Order_Charge_Storage
** Descriptions:		
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Order_Charge_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffer[48]; 
	char path_file[64];
	ORDER_CHG_EVENT* pOrder_Charge = (ORDER_CHG_EVENT*)Storage_Para;
	
	if(cmd == WRITE)//保存到本地write
	{
		// 创建文件名
		int writelen = 0;
		char path[18];
		sprintf((char*)path_file,"%s",PATH);
		sprintf((char*)path,"/%04u-%02X%02X%02X%02X%02X%02X.txt",(rt_uint32_t)pOrder_Charge->OrderNum,\
																			  pOrder_Charge->StartTimestamp.Second,\
																			  pOrder_Charge->StartTimestamp.Minute,\
																			  pOrder_Charge->StartTimestamp.Hour,\
																			  pOrder_Charge->StartTimestamp.Day,\
																			  pOrder_Charge->StartTimestamp.Month,\
																			  pOrder_Charge->StartTimestamp.Year);
		strcat((char*)path_file,(const char*)path);
		rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);		
		
		// 准备写的内容
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"OrderNum=%04u\n",(rt_uint32_t)pOrder_Charge->OrderNum);//	事件记录序号
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"StartTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pOrder_Charge->StartTimestamp.Second,\
																		  (rt_uint32_t)pOrder_Charge->StartTimestamp.Minute,\
																		  (rt_uint32_t)pOrder_Charge->StartTimestamp.Hour,\
																		  (rt_uint32_t)pOrder_Charge->StartTimestamp.Day,\
																		  (rt_uint32_t)pOrder_Charge->StartTimestamp.Month,\
																		  (rt_uint32_t)pOrder_Charge->StartTimestamp.Year);//  事件发生时间
		strcat((char*)Para_Buff,(const char*)buffer);		
		
		sprintf((char*)buffer,"FinishTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pOrder_Charge->FinishTimestamp.Second,\
																		   (rt_uint32_t)pOrder_Charge->FinishTimestamp.Minute,\
																		   (rt_uint32_t)pOrder_Charge->FinishTimestamp.Hour,\
																		   (rt_uint32_t)pOrder_Charge->FinishTimestamp.Day,\
																		   (rt_uint32_t)pOrder_Charge->FinishTimestamp.Month,\
																		   (rt_uint32_t)pOrder_Charge->FinishTimestamp.Year);//  事件结束时间
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件发生源    NULL
		sprintf((char*)buffer,"OccurSource=%03u\n",(rt_uint32_t)pOrder_Charge->OccurSource);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件上报状态 = 通道上报状态
		sprintf((char*)buffer,"ChannelState=%03u\n",(rt_uint32_t)pOrder_Charge->ChannelState);
		strcat((char*)Para_Buff,(const char*)buffer);

		// 总故障
		sprintf((char*)buffer,"TotalFault=%08u\n",(rt_uint32_t)pOrder_Charge->TotalFault);
		strcat((char*)Para_Buff,(const char*)buffer);	
		
		// 各项故障状态
		sprintf((char*)buffer,"Fau=");
		char bytebuf[] = "FF";
		for(int i=0;i<sizeof(pOrder_Charge->Fau);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pOrder_Charge->Fau[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");	
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
		/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}		
		/************************************************************************************/				
		/**********更新写当天电量************************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		rt_uint32_t file_num;	
		Find_path_file(PATH,ordernum,path_file,&file_num);

		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/			
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/***************************************************************************************/
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:%s文件读内容\n%s\n",path_file,Para_Buff);
		}		
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0,};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0,};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件记录序号/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OrderNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOrder_Charge->OrderNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOrder_Charge->OrderNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////事件发生时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"StartTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOrder_Charge->StartTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pOrder_Charge->StartTimestamp.Year,\
							pOrder_Charge->StartTimestamp.Month,\
							pOrder_Charge->StartTimestamp.Day,\
							pOrder_Charge->StartTimestamp.Hour,\
							pOrder_Charge->StartTimestamp.Minute,\
							pOrder_Charge->StartTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件结束时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"FinishTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOrder_Charge->FinishTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pOrder_Charge->FinishTimestamp.Year,\
							pOrder_Charge->FinishTimestamp.Month,\
							pOrder_Charge->FinishTimestamp.Day,\
							pOrder_Charge->FinishTimestamp.Hour,\
							pOrder_Charge->FinishTimestamp.Minute,\
							pOrder_Charge->FinishTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
////////////////////事件发生源/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OccurSource"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOrder_Charge->OccurSource,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOrder_Charge->OccurSource);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////事件上报状态 = 通道上报状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChannelState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOrder_Charge->ChannelState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOrder_Charge->ChannelState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////总故障///////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"TotalFault"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOrder_Charge->TotalFault,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOrder_Charge->TotalFault);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
		rt_lprintf("[storage]:%s文件写成功\n",path_file);		
/////////各项故障状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"Fau"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOrder_Charge->Fau,sizeof(pOrder_Charge->Fau));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pOrder_Charge->Fau);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:%s文件无效命令\n",path_file);
	}

	return 0;	
}
/*********************************************************************************************************
** Function name:		Plan_Offer_Storage
** Descriptions:		
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Plan_Offer_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffer[256]; //wyg191101
	char path_file[64];
	PLAN_OFFER_EVENT* pPlan_Offer = (PLAN_OFFER_EVENT*)Storage_Para;
	
	if(cmd == WRITE)//保存到本地write
	{
		// 创建文件名
		int writelen = 0;
		char path[18];
		sprintf((char*)path_file,"%s",PATH);
		sprintf((char*)path,"/%04u-%02X%02X%02X%02X%02X%02X.txt",(rt_uint32_t)pPlan_Offer->OrderNum,\
																			  pPlan_Offer->StartTimestamp.Second,\
																			  pPlan_Offer->StartTimestamp.Minute,\
																			  pPlan_Offer->StartTimestamp.Hour,\
																			  pPlan_Offer->StartTimestamp.Day,\
																			  pPlan_Offer->StartTimestamp.Month,\
																			  pPlan_Offer->StartTimestamp.Year);
		strcat((char*)path_file,(const char*)path);
		rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);		
		
		// 准备写的内容
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"OrderNum=%04u\n",(rt_uint32_t)pPlan_Offer->OrderNum);//	事件记录序号
		strcat((char*)Para_Buff,(const char*)buffer);
		
		sprintf((char*)buffer,"StartTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pPlan_Offer->StartTimestamp.Second,\
																		  (rt_uint32_t)pPlan_Offer->StartTimestamp.Minute,\
																		  (rt_uint32_t)pPlan_Offer->StartTimestamp.Hour,\
																		  (rt_uint32_t)pPlan_Offer->StartTimestamp.Day,\
																		  (rt_uint32_t)pPlan_Offer->StartTimestamp.Month,\
																		  (rt_uint32_t)pPlan_Offer->StartTimestamp.Year);//  事件发生时间
		strcat((char*)Para_Buff,(const char*)buffer);		
		
		sprintf((char*)buffer,"FinishTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pPlan_Offer->FinishTimestamp.Second,\
																		   (rt_uint32_t)pPlan_Offer->FinishTimestamp.Minute,\
																		   (rt_uint32_t)pPlan_Offer->FinishTimestamp.Hour,\
																		   (rt_uint32_t)pPlan_Offer->FinishTimestamp.Day,\
																		   (rt_uint32_t)pPlan_Offer->FinishTimestamp.Month,\
																		   (rt_uint32_t)pPlan_Offer->FinishTimestamp.Year);//  事件结束时间
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件发生源
		sprintf((char*)buffer,"OccurSource=%03u\n",(rt_uint32_t)pPlan_Offer->OccurSource); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件上报状态 = 通道上报状态
		sprintf((char*)buffer,"ChannelState=%03u\n",(rt_uint32_t)pPlan_Offer->ChannelState);
		strcat((char*)Para_Buff,(const char*)buffer);

		// 充电申请单号（SIZE(16)）
		sprintf((char*)buffer,"cRequestNO=");
		char bytebuf[] = "FF";
		for(int i=0;i<sizeof(pPlan_Offer->Chg_Strategy.cRequestNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pPlan_Offer->Chg_Strategy.cRequestNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");
		
		//	路由器资产编号 visible-string（SIZE(22)）
		sprintf((char*)buffer,"cAssetNO=");
		for(int i=0;i<sizeof(pPlan_Offer->Chg_Strategy.cAssetNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pPlan_Offer->Chg_Strategy.cAssetNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");		
		
		// 枪序号{A枪（1）、B枪（2）}
		sprintf((char*)buffer,"GunNum=%03u\n",(rt_uint8_t)pPlan_Offer->Chg_Strategy.GunNum); 
		strcat((char*)Para_Buff,(const char*)buffer);		
		
		
		// 用户id  visible-string（SIZE(64)）
		sprintf((char*)buffer,"cUserID=");
		for(int i=0;i<sizeof(pPlan_Offer->Chg_Strategy.cUserID);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pPlan_Offer->Chg_Strategy.cUserID[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");	
	
		//决策者  {主站（1）、控制器（2）}
		sprintf((char*)buffer,"ucDecMaker=%03u\n",(rt_uint8_t)pPlan_Offer->Chg_Strategy.ucDecMaker); 
		strcat((char*)Para_Buff,(const char*)buffer);

		//决策类型{生成（1） 、调整（2）}
		sprintf((char*)buffer,"ucDecType=%03u\n",(rt_uint8_t)pPlan_Offer->Chg_Strategy.ucDecType); 
		strcat((char*)Para_Buff,(const char*)buffer);
		
		//决策时间
		sprintf((char*)buffer,"strDecTime=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pPlan_Offer->Chg_Strategy.strDecTime.Second,\
																				   (rt_uint32_t)pPlan_Offer->Chg_Strategy.strDecTime.Minute,\
																				   (rt_uint32_t)pPlan_Offer->Chg_Strategy.strDecTime.Hour,\
																				   (rt_uint32_t)pPlan_Offer->Chg_Strategy.strDecTime.Day,\
																				   (rt_uint32_t)pPlan_Offer->Chg_Strategy.strDecTime.Month,\
																				   (rt_uint32_t)pPlan_Offer->Chg_Strategy.strDecTime.Year);
		strcat((char*)Para_Buff,(const char*)buffer);

		//	充电需求电量（单位：kWh，换算：-2）
		sprintf((char*)buffer,"ulChargeReqEle=%08u\n",(rt_uint32_t)pPlan_Offer->Chg_Strategy.ulChargeReqEle);
		strcat((char*)Para_Buff,(const char*)buffer);	
			
		//	充电额定功率 （单位：kW，换算：-4）
		sprintf((char*)buffer,"ulChargeRatePow=%08u\n",(rt_uint32_t)pPlan_Offer->Chg_Strategy.ulChargeRatePow);
		strcat((char*)Para_Buff,(const char*)buffer);

		//	充电模式 {正常（0），有序（1）}
		sprintf((char*)buffer,"ucChargeMode=%03u\n",(rt_uint8_t)pPlan_Offer->Chg_Strategy.ucChargeMode);
		strcat((char*)Para_Buff,(const char*)buffer);
	
		//	时间段数量
		sprintf((char*)buffer,"ucTimeSlotNum=%03u\n",(rt_uint8_t)pPlan_Offer->Chg_Strategy.ucTimeSlotNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		//	时间段内容，最大50段
		for(int i=0;i<sizeof(pPlan_Offer->Chg_Strategy.ucTimeSlotNum);i++)
		{
			//起始时间
			sprintf((char*)buffer,"strChargeTimeSolts[%d].strDecStartTime=%02X%02X%02X%02X%02X%02X\n",i,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Second,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Minute,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Hour,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Day,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Month,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Year);
			strcat((char*)Para_Buff,(const char*)buffer);
			//结束时间
			sprintf((char*)buffer,"strChargeTimeSolts[%d].strDecStopTime=%02X%02X%02X%02X%02X%02X\n",i,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Second,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Minute,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Hour,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Day,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Month,\
									(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Year);
			strcat((char*)Para_Buff,(const char*)buffer);
			//充电功率设定值（单位：kW，换算：-4）
			sprintf((char*)buffer,"strChargeTimeSolts[%d].ulChargePow=%08u\n",i,(rt_uint8_t)pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].ulChargePow);
			strcat((char*)Para_Buff,(const char*)buffer);			
		}			
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
		/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}		
		/************************************************************************************/				
		/**********更新写当天电量************************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		rt_uint32_t file_num;	
		Find_path_file(PATH,ordernum,path_file,&file_num);

		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/			
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/***************************************************************************************/
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:%s文件读内容\n%s\n",path_file,Para_Buff);
		}		
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0,};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0,};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件记录序号/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OrderNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->OrderNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->OrderNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////事件发生时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"StartTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->StartTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pPlan_Offer->StartTimestamp.Year,\
							pPlan_Offer->StartTimestamp.Month,\
							pPlan_Offer->StartTimestamp.Day,\
							pPlan_Offer->StartTimestamp.Hour,\
							pPlan_Offer->StartTimestamp.Minute,\
							pPlan_Offer->StartTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件结束时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"FinishTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->FinishTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pPlan_Offer->FinishTimestamp.Year,\
							pPlan_Offer->FinishTimestamp.Month,\
							pPlan_Offer->FinishTimestamp.Day,\
							pPlan_Offer->FinishTimestamp.Hour,\
							pPlan_Offer->FinishTimestamp.Minute,\
							pPlan_Offer->FinishTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
////////////////////事件发生源/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OccurSource"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->OccurSource,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->OccurSource);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////事件上报状态 = 通道上报状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChannelState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->ChannelState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->ChannelState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////		
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////充电申请单号 SIZE(16)/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"cRequestNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.cRequestNO,sizeof(pPlan_Offer->Chg_Strategy.cRequestNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pPlan_Offer->Chg_Strategy.cRequestNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////路由器资产编号 visible-string（SIZE(22)）////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"cAssetNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.cAssetNO,sizeof(pPlan_Offer->Chg_Strategy.cAssetNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pPlan_Offer->Chg_Strategy.cAssetNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////枪序号	{A枪（1）、B枪（2）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"GunNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.GunNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.GunNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////用户id  visible-string（SIZE(64)）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"cUserID");
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.cUserID,sizeof(pPlan_Offer->Chg_Strategy.cUserID));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pPlan_Offer->Chg_Strategy.cUserID);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////决策者  {主站（1）、控制器（2）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucDecMaker"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.ucDecMaker,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.ucDecMaker);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////决策类型{生成（1） 、调整（2）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucDecType"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.ucDecType,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.ucDecType);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////决策时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"strDecTime"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.strDecTime.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pPlan_Offer->Chg_Strategy.strDecTime.Year,\
							pPlan_Offer->Chg_Strategy.strDecTime.Month,\
							pPlan_Offer->Chg_Strategy.strDecTime.Day,\
							pPlan_Offer->Chg_Strategy.strDecTime.Hour,\
							pPlan_Offer->Chg_Strategy.strDecTime.Minute,\
							pPlan_Offer->Chg_Strategy.strDecTime.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
/////////充电需求电量（单位：kWh，换算：-2）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ulChargeReqEle"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.ulChargeReqEle,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.ulChargeReqEle);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////充电额定功率 （单位：kW，换算：-4）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ulChargeRatePow"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.ulChargeRatePow,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.ulChargeRatePow);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////充电模式 {正常（0），有序（1）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucChargeMode"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.ucChargeMode,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.ucChargeMode);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////时间段数量/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucTimeSlotNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.ucTimeSlotNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.ucTimeSlotNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////时间段内容，最大50段/////////////////////////////////////////////////////////////////////////////////
		for(int i=0;i<sizeof(pPlan_Offer->Chg_Strategy.ucTimeSlotNum);i++)
		{
			////////////////////起始时间//////////////////////////////////////////////////////////
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				sprintf((char*)fpnameRd,"strChargeTimeSolts[%d].strDecStartTime",i); 
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Second,6);//返回当前文件读指针
					rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Year,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Month,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Day,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Hour,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Minute,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStartTime.Second);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
				}
				namelen=0;							
			}
			else
			{
				rt_lprintf("namelen=0\n");			
			}				
			////////////////////结束时间//////////////////////////////////////////////////////////
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				sprintf((char*)fpnameRd,"strDecStopTime[%d].strDecStartTime",i); 
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Second,6);//返回当前文件读指针
					rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Year,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Month,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Day,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Hour,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Minute,\
								pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].strDecStopTime.Second);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
				}
				namelen=0;							
			}
			else
			{
				rt_lprintf("namelen=0\n");			
			}			
			/////////充电功率设定值（单位：kW，换算：-4）/////////////////////////////////////////////////////////////////////////////////
			sprintf((char*)fpnameRd,"strChargeTimeSolts[%d].ulChargePow",i); 
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].ulChargePow,1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Offer->Chg_Strategy.strChargeTimeSolts[i].ulChargePow);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}
		}
/////////////////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:%s文件无效命令\n",path_file);
	}

	return 0;	
}
/*********************************************************************************************************
** Function name:		Plan_Fail_Storage
** Descriptions:		
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Plan_Fail_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffer[48]; 
	char path_file[64];
	PLAN_FAIL_EVENT* pPlan_Fail = (PLAN_FAIL_EVENT*)Storage_Para;
	
	if(cmd == WRITE)//保存到本地write
	{
		// 创建文件名
		int writelen = 0;
		char path[18];
		sprintf((char*)path_file,"%s",PATH);
		sprintf((char*)path,"/%04u-%02X%02X%02X%02X%02X%02X.txt",(rt_uint32_t)pPlan_Fail->OrderNum,\
																			  pPlan_Fail->StartTimestamp.Second,\
																			  pPlan_Fail->StartTimestamp.Minute,\
																			  pPlan_Fail->StartTimestamp.Hour,\
																			  pPlan_Fail->StartTimestamp.Day,\
																			  pPlan_Fail->StartTimestamp.Month,\
																			  pPlan_Fail->StartTimestamp.Year);
		strcat((char*)path_file,(const char*)path);
		rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);		
		
		// 准备写的内容
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		//记录序号
		sprintf((char*)buffer,"OrderNum=%04u\n",(rt_uint32_t)pPlan_Fail->OrderNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件发生时间
		sprintf((char*)buffer,"StartTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pPlan_Fail->StartTimestamp.Second,\
																		  (rt_uint32_t)pPlan_Fail->StartTimestamp.Minute,\
																		  (rt_uint32_t)pPlan_Fail->StartTimestamp.Hour,\
																		  (rt_uint32_t)pPlan_Fail->StartTimestamp.Day,\
																		  (rt_uint32_t)pPlan_Fail->StartTimestamp.Month,\
																		  (rt_uint32_t)pPlan_Fail->StartTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);		
		//事件结束时间 
		sprintf((char*)buffer,"FinishTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint32_t)pPlan_Fail->FinishTimestamp.Second,\
																		   (rt_uint32_t)pPlan_Fail->FinishTimestamp.Minute,\
																		   (rt_uint32_t)pPlan_Fail->FinishTimestamp.Hour,\
																		   (rt_uint32_t)pPlan_Fail->FinishTimestamp.Day,\
																		   (rt_uint32_t)pPlan_Fail->FinishTimestamp.Month,\
																		   (rt_uint32_t)pPlan_Fail->FinishTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件发生源    NULL
		sprintf((char*)buffer,"OccurSource=%03u\n",(rt_uint32_t)pPlan_Fail->OccurSource);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 事件上报状态 = 通道上报状态
		sprintf((char*)buffer,"ChannelState=%03u\n",(rt_uint32_t)pPlan_Fail->ChannelState);
		strcat((char*)Para_Buff,(const char*)buffer);

		// 充电申请单号（SIZE(16)）
		sprintf((char*)buffer,"RequestNO=");
		char bytebuf[] = "FF";
		for(int i=0;i<sizeof(pPlan_Fail->RequestNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pPlan_Fail->RequestNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");
	
		//	路由器资产编号 visible-string（SIZE(22)）
		sprintf((char*)buffer,"AssetNO=");
		for(int i=0;i<sizeof(pPlan_Fail->AssetNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pPlan_Fail->AssetNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");	
		// 枪序号	{A枪（1）、B枪（2）}
		sprintf((char*)buffer,"GunNum=%03u\n",(rt_uint32_t)pPlan_Fail->GunNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
		/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}		
		/************************************************************************************/				
		/**********更新写当天电量************************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		rt_uint32_t file_num;	
		Find_path_file(PATH,ordernum,path_file,&file_num);

		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/			
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/***************************************************************************************/
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:%s文件读内容\n%s\n",path_file,Para_Buff);
		}		
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0,};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0,};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件记录序号/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OrderNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->OrderNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Fail->OrderNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////事件发生时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"StartTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->StartTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pPlan_Fail->StartTimestamp.Year,\
							pPlan_Fail->StartTimestamp.Month,\
							pPlan_Fail->StartTimestamp.Day,\
							pPlan_Fail->StartTimestamp.Hour,\
							pPlan_Fail->StartTimestamp.Minute,\
							pPlan_Fail->StartTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件结束时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"FinishTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->FinishTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pPlan_Fail->FinishTimestamp.Year,\
							pPlan_Fail->FinishTimestamp.Month,\
							pPlan_Fail->FinishTimestamp.Day,\
							pPlan_Fail->FinishTimestamp.Hour,\
							pPlan_Fail->FinishTimestamp.Minute,\
							pPlan_Fail->FinishTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
////////////////////事件发生源/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OccurSource"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->OccurSource,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Fail->OccurSource);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////事件上报状态 = 通道上报状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChannelState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->ChannelState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Fail->ChannelState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////充电申请单号 SIZE(16)/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"RequestNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->RequestNO,sizeof(pPlan_Fail->RequestNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pPlan_Fail->RequestNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////路由器资产编号 visible-string（SIZE(22)）////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"AssetNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->AssetNO,sizeof(pPlan_Fail->AssetNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pPlan_Fail->AssetNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////枪序号	{A枪（1）、B枪（2）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"GunNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pPlan_Fail->GunNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pPlan_Fail->GunNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////////////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:%s文件无效命令\n",path_file);
	}

	return 0;	
}
/*********************************************************************************************************
** Function name:		Online_State_Storage
** Descriptions:		
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Online_State_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffer[48]; 
	char path_file[64];
	ONLINE_STATE* pOnline_State = (ONLINE_STATE*)Storage_Para;
	
	if(cmd == WRITE)//保存到本地write
	{
		// 创建文件名
		int writelen = 0;
		char path[18];
		sprintf((char*)path_file,"%s",PATH);
		sprintf((char*)path,"/%04u-%02X%02X%02X%02X%02X%02X.txt",(rt_uint32_t)pOnline_State->OrderNum,\
																			  pOnline_State->OnlineTimestamp.Second,\
																			  pOnline_State->OnlineTimestamp.Minute,\
																			  pOnline_State->OnlineTimestamp.Hour,\
																			  pOnline_State->OnlineTimestamp.Day,\
																			  pOnline_State->OnlineTimestamp.Month,\
																			  pOnline_State->OnlineTimestamp.Year);
		strcat((char*)path_file,(const char*)path);
		rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);		
		
		// 准备写的内容
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件记录序号
		sprintf((char*)buffer,"OrderNum=%04u\n",(rt_uint32_t)pOnline_State->OrderNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		//上线时间
		sprintf((char*)buffer,"StartTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pOnline_State->OnlineTimestamp.Second,\
																		  (rt_uint8_t)pOnline_State->OnlineTimestamp.Minute,\
																		  (rt_uint8_t)pOnline_State->OnlineTimestamp.Hour,\
																		  (rt_uint8_t)pOnline_State->OnlineTimestamp.Day,\
																		  (rt_uint8_t)pOnline_State->OnlineTimestamp.Month,\
																		  (rt_uint8_t)pOnline_State->OnlineTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);		
		//离线时间
		sprintf((char*)buffer,"FinishTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pOnline_State->OfflineTimestamp.Second,\
																		   (rt_uint8_t)pOnline_State->OfflineTimestamp.Minute,\
																		   (rt_uint8_t)pOnline_State->OfflineTimestamp.Hour,\
																		   (rt_uint8_t)pOnline_State->OfflineTimestamp.Day,\
																		   (rt_uint8_t)pOnline_State->OfflineTimestamp.Month,\
																		   (rt_uint8_t)pOnline_State->OfflineTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件发生源
		sprintf((char*)buffer,"OccurSource=%03u\n",(rt_uint32_t)pOnline_State->OccurSource);
		strcat((char*)Para_Buff,(const char*)buffer);		
		// 通道状态
		sprintf((char*)buffer,"ChannelState=%03u\n",(rt_uint8_t)pOnline_State->ChannelState);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		//状态变化 {上线（0）， 离线（1）}
		sprintf((char*)buffer,"AutualState=%03u\n",(rt_uint8_t)pOnline_State->AutualState);
		strcat((char*)Para_Buff,(const char*)buffer);
		//离线信息---//本次离线时长（单位：秒）
		sprintf((char*)buffer,"OfflinePeriod=%03u\n",(rt_uint8_t)pOnline_State->OfflineIfo.OfflinePeriod);
		strcat((char*)Para_Buff,(const char*)buffer);
		//离线信息---//离线原因 {未知（0），停电（1），信道变化（2）}
		sprintf((char*)buffer,"OfflineReason=%03u\n",(rt_uint8_t)pOnline_State->OfflineIfo.OfflineReason);
		strcat((char*)Para_Buff,(const char*)buffer);		
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
		/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}		
		/************************************************************************************/				
		/**********更新写当天电量************************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		rt_uint32_t file_num;	
		Find_path_file(PATH,ordernum,path_file,&file_num);

		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/			
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/***************************************************************************************/
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:%s文件读内容\n%s\n",path_file,Para_Buff);
		}		
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0,};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0,};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件记录序号/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OrderNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->OrderNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOnline_State->OrderNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////上线时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"OnlineTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->OnlineTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pOnline_State->OnlineTimestamp.Year,\
							pOnline_State->OnlineTimestamp.Month,\
							pOnline_State->OnlineTimestamp.Day,\
							pOnline_State->OnlineTimestamp.Hour,\
							pOnline_State->OnlineTimestamp.Minute,\
							pOnline_State->OnlineTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////离线时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"OfflineTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->OfflineTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pOnline_State->OfflineTimestamp.Year,\
							pOnline_State->OfflineTimestamp.Month,\
							pOnline_State->OfflineTimestamp.Day,\
							pOnline_State->OfflineTimestamp.Hour,\
							pOnline_State->OfflineTimestamp.Minute,\
							pOnline_State->OfflineTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
/////////事件发生源/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OccurSource"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->OccurSource,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOnline_State->OccurSource);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////事件上报状态 = 通道上报状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChannelState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->ChannelState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOnline_State->ChannelState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////状态变化 {上线（0）， 离线（1）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"AutualState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->AutualState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOnline_State->AutualState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////离线信息---//本次离线时长（单位：秒）///////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OfflinePeriod"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->OfflineIfo.OfflinePeriod,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOnline_State->OfflineIfo.OfflinePeriod);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////离线信息---//离线原因 {未知（0），停电（1），信道变化（2）}////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OfflineReason"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pOnline_State->OfflineIfo.OfflineReason,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pOnline_State->OfflineIfo.OfflineReason);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:%s文件无效命令\n",path_file);
	}

	return 0;	
}
/*********************************************************************************************************
** Function name:		Chg_Execute_Storage
** Descriptions:		
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Chg_Execute_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffer[48]; 
	char path_file[64];
	CHARGE_EXE_EVENT* pCharge_Exe = (CHARGE_EXE_EVENT*)Storage_Para;	
	
	if(cmd == WRITE)//保存到本地write
	{
		// 创建文件名
		int writelen = 0;
		char path[18];
		sprintf((char*)path_file,"%s",PATH);
		sprintf((char*)path,"/%04u-%02X%02X%02X%02X%02X%02X.txt",(rt_uint32_t)pCharge_Exe->OrderNum,\
																			  pCharge_Exe->StartTimestamp.Second,\
																			  pCharge_Exe->StartTimestamp.Minute,\
																			  pCharge_Exe->StartTimestamp.Hour,\
																			  pCharge_Exe->StartTimestamp.Day,\
																			  pCharge_Exe->StartTimestamp.Month,\
																			  pCharge_Exe->StartTimestamp.Year);
		strcat((char*)path_file,(const char*)path);
		rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);		
		
		// 准备写的内容
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件记录序号
		sprintf((char*)buffer,"OrderNum=%04u\n",(rt_uint32_t)pCharge_Exe->OrderNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件发生时间
		sprintf((char*)buffer,"StartTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pCharge_Exe->StartTimestamp.Second,\
																		  (rt_uint8_t)pCharge_Exe->StartTimestamp.Minute,\
																		  (rt_uint8_t)pCharge_Exe->StartTimestamp.Hour,\
																		  (rt_uint8_t)pCharge_Exe->StartTimestamp.Day,\
																		  (rt_uint8_t)pCharge_Exe->StartTimestamp.Month,\
																		  (rt_uint8_t)pCharge_Exe->StartTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);		
		//事件结束时间
		sprintf((char*)buffer,"FinishTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pCharge_Exe->FinishTimestamp.Second,\
																		   (rt_uint8_t)pCharge_Exe->FinishTimestamp.Minute,\
																		   (rt_uint8_t)pCharge_Exe->FinishTimestamp.Hour,\
																		   (rt_uint8_t)pCharge_Exe->FinishTimestamp.Day,\
																		   (rt_uint8_t)pCharge_Exe->FinishTimestamp.Month,\
																		   (rt_uint8_t)pCharge_Exe->FinishTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件发生源
		sprintf((char*)buffer,"OccurSource=%03u\n",(rt_uint32_t)pCharge_Exe->OccurSource);
		strcat((char*)Para_Buff,(const char*)buffer);		
		// 通道状态
		sprintf((char*)buffer,"ChannelState=%03u\n",(rt_uint8_t)pCharge_Exe->ChannelState);
		strcat((char*)Para_Buff,(const char*)buffer);
/////////////////////////////////////////////////////////////////////////////////////////////////		
		// 充电申请单号
		sprintf((char*)buffer,"cRequestNO=");
		char bytebuf[] = "FF";
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.cRequestNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint8_t)pCharge_Exe->Chg_ExeState.cRequestNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");
	
		//	路由器资产编号
		sprintf((char*)buffer,"cAssetNO=");
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.cAssetNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint8_t)pCharge_Exe->Chg_ExeState.cAssetNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");	
		// 枪序号	{A枪（1）、B枪（2）}
		sprintf((char*)buffer,"GunNum=%03u\n",(rt_uint8_t)pCharge_Exe->Chg_ExeState.GunNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		//执行状态
		sprintf((char*)buffer,"exeState=%03u\n",(rt_uint8_t)pCharge_Exe->Chg_ExeState.exeState);
		strcat((char*)Para_Buff,(const char*)buffer);
		//时间段数量
		sprintf((char*)buffer,"ucTimeSlotNum=%03u\n",(rt_uint8_t)pCharge_Exe->Chg_ExeState.ucTimeSlotNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		//电能示值底值
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.ulEleBottomValue);i++)
		{
			sprintf((char*)buffer,"ulEleBottomValue[%d]=",i);
			sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ulEleBottomValue[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		//当前电能示值
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.ulEleActualValue);i++)
		{
			sprintf((char*)buffer,"ulEleActualValue[%d]=",i);
			sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ulEleActualValue[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}		
		//已充电量
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.ucChargeEle);i++)
		{
			sprintf((char*)buffer,"ucChargeEle[%d]=",i);
			sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucChargeEle[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		//已充时间
		sprintf((char*)buffer,"ucChargeTime=%03u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucChargeTime);
		strcat((char*)Para_Buff,(const char*)buffer);
		//计划充电功率
		sprintf((char*)buffer,"ucPlanPower=%03u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucPlanPower);
		strcat((char*)Para_Buff,(const char*)buffer);		
		//当前充电功率
		sprintf((char*)buffer,"ucActualPower=%03u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucActualPower);
		strcat((char*)Para_Buff,(const char*)buffer);		
		
		//当前充电电压
		sprintf((char*)buffer,"ucVoltage.A=");
		sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucVoltage.A);
		strcat((char*)buffer,(const char*)bytebuf);	
		sprintf((char*)buffer,"ucVoltage.B=");
		sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucVoltage.B);
		strcat((char*)buffer,(const char*)bytebuf);			
		sprintf((char*)buffer,"ucVoltage.C=");
		sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucVoltage.C);
		strcat((char*)buffer,(const char*)bytebuf);			
		//当前充电电流
		sprintf((char*)buffer,"ucCurrent.A=");
		sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucCurrent.A);
		strcat((char*)buffer,(const char*)bytebuf);	
		sprintf((char*)buffer,"ucCurrent.B=");
		sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucCurrent.B);
		strcat((char*)buffer,(const char*)bytebuf);			
		sprintf((char*)buffer,"ucCurrent.C=");
		sprintf((char*)bytebuf,"%08u\n",(rt_uint32_t)pCharge_Exe->Chg_ExeState.ucCurrent.C);
		strcat((char*)buffer,(const char*)bytebuf);	
		//充电桩状态	
		sprintf((char*)buffer,"ChgPileState=%03u\n",(rt_uint8_t)pCharge_Exe->Chg_ExeState.ChgPileState);
		strcat((char*)Para_Buff,(const char*)buffer);
/////////////////////////////////////////////////////////////////////////////////////////////////		
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
		/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}		
		/************************************************************************************/				
		/**********更新写当天电量************************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		rt_uint32_t file_num;	
		Find_path_file(PATH,ordernum,path_file,&file_num);

		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/			
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/***************************************************************************************/
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:%s文件读内容\n%s\n",path_file,Para_Buff);
		}		
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0,};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0,};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件记录序号/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OrderNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->OrderNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->OrderNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////事件发生时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"StartTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->StartTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Exe->StartTimestamp.Year,\
							pCharge_Exe->StartTimestamp.Month,\
							pCharge_Exe->StartTimestamp.Day,\
							pCharge_Exe->StartTimestamp.Hour,\
							pCharge_Exe->StartTimestamp.Minute,\
							pCharge_Exe->StartTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件结束时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"FinishTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->FinishTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pCharge_Exe->FinishTimestamp.Year,\
							pCharge_Exe->FinishTimestamp.Month,\
							pCharge_Exe->FinishTimestamp.Day,\
							pCharge_Exe->FinishTimestamp.Hour,\
							pCharge_Exe->FinishTimestamp.Minute,\
							pCharge_Exe->FinishTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
/////////事件发生源/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OccurSource"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->OccurSource,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->OccurSource);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////事件上报状态 = 通道上报状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChannelState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->ChannelState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->ChannelState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////充电申请单号 SIZE(16)/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"cRequestNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.cRequestNO,sizeof(pCharge_Exe->Chg_ExeState.cRequestNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pCharge_Exe->Chg_ExeState.cRequestNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////路由器资产编号 visible-string（SIZE(22)）////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"cAssetNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.cAssetNO,sizeof(pCharge_Exe->Chg_ExeState.cAssetNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pCharge_Exe->Chg_ExeState.cAssetNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////枪序号	{A枪（1）、B枪（2）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"GunNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.GunNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.GunNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////执行状态//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"exeState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.exeState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.exeState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////时间段数量//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucTimeSlotNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucTimeSlotNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucTimeSlotNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////电能示值底值//////////////////////////////////////////////////////////
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.ulEleBottomValue);i++)
		{
			sprintf((char*)fpnameRd,"ulEleBottomValue[%d]=",i);
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ulEleBottomValue[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ulEleBottomValue[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}
		}	
////////////////////当前电能示值//////////////////////////////////////////////////////////
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.ulEleActualValue);i++)
		{
			sprintf((char*)fpnameRd,"ulEleActualValue[%d]=",i);
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ulEleActualValue[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ulEleActualValue[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}
		}	
////////////////////已充电量//////////////////////////////////////////////////////////
		for(int i=0;i<sizeof(pCharge_Exe->Chg_ExeState.ucChargeEle);i++)
		{
			sprintf((char*)fpnameRd,"ucChargeEle[%d]=",i);
			fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
			if(namelen)//接收完了
			{
				if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
				{
					fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucChargeEle[i],1);//返回当前文件读指针
					rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucChargeEle[i]);
				}
				else
				{
					rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
				}
				namelen=0;
			}
			else
			{
				rt_lprintf("namelen=0\n");
			}
		}		
////////////////////已充时间//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucChargeTime"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucChargeTime,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucChargeTime);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////计划充电功率//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucPlanPower"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucPlanPower,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucPlanPower);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
////////////////////当前充电功率//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucActualPower"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucActualPower,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucActualPower);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
////////////////////当前充电电压//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucVoltage.A"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucVoltage.A,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucVoltage.A);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
		sprintf((char*)fpnameRd,"ucVoltage.B"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucVoltage.B,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucVoltage.B);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
		sprintf((char*)fpnameRd,"ucVoltage.C"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucVoltage.C,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucVoltage.C);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////当前充电电流//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ucCurrent.A"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucCurrent.A,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucCurrent.A);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
		sprintf((char*)fpnameRd,"ucCurrent.B"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucCurrent.B,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucCurrent.B);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
		sprintf((char*)fpnameRd,"ucCurrent.C"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ucCurrent.C,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ucCurrent.C);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////充电桩状态//////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChgPileState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pCharge_Exe->Chg_ExeState.ChgPileState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pCharge_Exe->Chg_ExeState.ChgPileState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:%s文件无效命令\n",path_file);
	}

	return 0;
}
/*********************************************************************************************************
** Function name:		Chg_Request_Storage
** Descriptions:		
** input parameters:	YMD:20190731 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static int Chg_Request_Storage(const char *PATH,void *Storage_Para,rt_uint32_t ordernum,rt_uint32_t cmd)	
{
	int fd = 0;
	char buffer[256]; 
	char path_file[64];
	CHARGE_APPLY_EVENT* pChg_Request = (CHARGE_APPLY_EVENT*)Storage_Para;	
	
	if(cmd == WRITE)//保存到本地write
	{
		// 创建文件名
		int writelen = 0;
		char path[18];
		sprintf((char*)path_file,"%s",PATH);
		sprintf((char*)path,"/%04u-%02X%02X%02X%02X%02X%02X.txt",(rt_uint32_t)pChg_Request->OrderNum,\
																			  pChg_Request->StartTimestamp.Second,\
																			  pChg_Request->StartTimestamp.Minute,\
																			  pChg_Request->StartTimestamp.Hour,\
																			  pChg_Request->StartTimestamp.Day,\
																			  pChg_Request->StartTimestamp.Month,\
																			  pChg_Request->StartTimestamp.Year);
		strcat((char*)path_file,(const char*)path);
		rt_lprintf("%s:path_file = %s\n",__FUNCTION__,(char*)path_file);		
		
		// 准备写的内容
		strcpy((char*)Para_Buff,"");
		sprintf((char*)buffer,"Time=%02X-%02X-%02X-%02X-%02X-%02X\n",System_Time_STR.Year,System_Time_STR.Month,System_Time_STR.Day,\
																	 System_Time_STR.Hour,System_Time_STR.Minute,System_Time_STR.Second); 
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件记录序号
		sprintf((char*)buffer,"OrderNum=%04u\n",(rt_uint32_t)pChg_Request->OrderNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件发生时间
		sprintf((char*)buffer,"StartTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pChg_Request->StartTimestamp.Second,\
																		  (rt_uint8_t)pChg_Request->StartTimestamp.Minute,\
																		  (rt_uint8_t)pChg_Request->StartTimestamp.Hour,\
																		  (rt_uint8_t)pChg_Request->StartTimestamp.Day,\
																		  (rt_uint8_t)pChg_Request->StartTimestamp.Month,\
																		  (rt_uint8_t)pChg_Request->StartTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);		
		//事件结束时间
		sprintf((char*)buffer,"FinishTimestamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pChg_Request->FinishTimestamp.Second,\
																		   (rt_uint8_t)pChg_Request->FinishTimestamp.Minute,\
																		   (rt_uint8_t)pChg_Request->FinishTimestamp.Hour,\
																		   (rt_uint8_t)pChg_Request->FinishTimestamp.Day,\
																		   (rt_uint8_t)pChg_Request->FinishTimestamp.Month,\
																		   (rt_uint8_t)pChg_Request->FinishTimestamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);
		//事件发生源
		sprintf((char*)buffer,"OccurSource=%03u\n",(rt_uint32_t)pChg_Request->OccurSource);
		strcat((char*)Para_Buff,(const char*)buffer);		
		// 通道状态
		sprintf((char*)buffer,"ChannelState=%03u\n",(rt_uint8_t)pChg_Request->ChannelState);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		// 充电申请单号（SIZE(16)）
		sprintf((char*)buffer,"RequestNO=");
		char bytebuf[] = "FF";
		for(int i=0;i<sizeof(pChg_Request->RequestNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pChg_Request->RequestNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");
	
		//	路由器资产编号 visible-string（SIZE(22)）
		sprintf((char*)buffer,"AssetNO=");
		for(int i=0;i<sizeof(pChg_Request->AssetNO);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pChg_Request->AssetNO[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");	
		// 枪序号	{A枪（1）、B枪（2）}
		sprintf((char*)buffer,"GunNum=%03u\n",(rt_uint32_t)pChg_Request->GunNum);
		strcat((char*)Para_Buff,(const char*)buffer);
		//充电申请时间
		sprintf((char*)buffer,"RequestTimeStamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pChg_Request->RequestTimeStamp.Second,\
																		    (rt_uint8_t)pChg_Request->RequestTimeStamp.Minute,\
																		    (rt_uint8_t)pChg_Request->RequestTimeStamp.Hour,\
																		    (rt_uint8_t)pChg_Request->RequestTimeStamp.Day,\
																		    (rt_uint8_t)pChg_Request->RequestTimeStamp.Month,\
																		    (rt_uint8_t)pChg_Request->RequestTimeStamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);
		//当前SOC（单位：%，换算：-2）
		sprintf((char*)buffer,"actSOC=%03u\n",(rt_uint32_t)pChg_Request->actSOC);
		strcat((char*)Para_Buff,(const char*)buffer);		

		//目标SOC（单位：%，换算：-2）
		sprintf((char*)buffer,"aimSOC=%03u\n",(rt_uint32_t)pChg_Request->aimSOC);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		//电池容量（单位：kWh，换算：-2）
		sprintf((char*)buffer,"CellCapacity=%03u\n",(rt_uint32_t)pChg_Request->CellCapacity);
		strcat((char*)Para_Buff,(const char*)buffer);
		
		//充电需求电量（单位：kWh，换算：-2）		
		sprintf((char*)buffer,"ChargeReqEle=%03u\n",(rt_uint32_t)pChg_Request->ChargeReqEle);
		strcat((char*)Para_Buff,(const char*)buffer);	
		//计划用车时间
		sprintf((char*)buffer,"PlanUnChg_TimeStamp=%02X%02X%02X%02X%02X%02X\n",(rt_uint8_t)pChg_Request->PlanUnChg_TimeStamp.Second,\
																		       (rt_uint8_t)pChg_Request->PlanUnChg_TimeStamp.Minute,\
																		       (rt_uint8_t)pChg_Request->PlanUnChg_TimeStamp.Hour,\
																		       (rt_uint8_t)pChg_Request->PlanUnChg_TimeStamp.Day,\
																		       (rt_uint8_t)pChg_Request->PlanUnChg_TimeStamp.Month,\
																		       (rt_uint8_t)pChg_Request->PlanUnChg_TimeStamp.Year);
		strcat((char*)Para_Buff,(const char*)buffer);
		//充电模式 {正常（0），有序（1）}		
		sprintf((char*)buffer,"ChargeMode=%03u\n",(rt_uint32_t)pChg_Request->ChargeMode);
		strcat((char*)Para_Buff,(const char*)buffer);
		//用户登录令牌  visible-string（SIZE(38)）
		sprintf((char*)buffer,"Token=");
		for(int i=0;i<sizeof(pChg_Request->Token);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pChg_Request->Token[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");
		//充电用户账号  visible-string（SIZE(9)）
		sprintf((char*)buffer,"UserAccount=");
		for(int i=0;i<sizeof(pChg_Request->UserAccount);i++)
		{
			sprintf((char*)bytebuf,"%02X",(rt_uint32_t)pChg_Request->UserAccount[i]);
			strcat((char*)buffer,(const char*)bytebuf);
		}
		strcat((char*)buffer,(const char*)"\n");		
		/****************************************************************************************/
		if(strlen((const char*)Para_Buff)> MAX_MALLOC_NUM)
		{
			rt_lprintf("[storage]: Para_Buff overflow=%d\n",strlen((const char*)Para_Buff));
			
			return -1;
		}
		else
		{
			rt_lprintf("[storage]:strlen(Para_Buff)=%d\n",strlen(Para_Buff));
		}	
		/************************************************************************************************/			
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */
		int fd = open(path_file,O_WRONLY | O_CREAT);
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}		
		/************************************************************************************/				
		/**********更新写当天电量************************************************************/				
		writelen = write(fd,Para_Buff,strlen((const char*)Para_Buff));//写入首部   返回值0：成功	
		if(writelen > 0)
		{
			rt_lprintf("[storage]:%s文件写入成功 writelen=%d\n",(char*)path_file,writelen);
		}
		else
		{
			rt_lprintf("[storage]:%s文件写入失败 writelen=%d\n",(char*)path_file,writelen);
		}
		/*****************************************************************************************/
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}			
	}
/************************************************************************************************/
/************************************************************************************************/
/************************************************************************************************/
	else if(cmd == READ) //从本地read
	{
		int readlen = 0;
		rt_uint32_t file_num;	
		Find_path_file(PATH,ordernum,path_file,&file_num);

		fd= open(path_file,O_RDONLY);//打开文件。如果文件不存在，则打开失败。
		if(fd >= 0)
		{
			rt_lprintf("[storage]:%s文件打开成功\n",path_file);
		}
		else
		{
			rt_lprintf("[storage]:%s文件打开失败 fd=%d\n",path_file,fd);
			return -2;
		}
        /***************************************************************************************/			
		readlen=read(fd,Para_Buff,MAX_MALLOC_NUM);	//读出txt里面的内容	
		if(readlen > 0) //0
		{
			rt_lprintf("[storage]:%s文件读取成功 readlen=%d\n",path_file,readlen);
		}
		else
		{
			if(close(fd) != UENOERR)
			{
				rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
			}
			rt_lprintf("[storage]:%s文件读取为空 readlen=%d\n",path_file,readlen);
			return -3;
		}
		/***************************************************************************************/	
		if(close(fd) != UENOERR)
		{
			rt_lprintf("[storage]:%s文件关闭失败\n",path_file);
		}
		/***************************************************************************************/
		*(Para_Buff+readlen) = '\0';//需要追加结束符
		if(readlen <= RT_CONSOLEBUF_SIZE)
		{
			rt_lprintf("[storage]:%s文件读内容\n%s\n",path_file,Para_Buff);
		}		
		/************************************************************************************/			
		/************************************************************************************/		
		char *fpoint = Para_Buff;//参数
		rt_uint8_t namelen = 0;
		rt_uint8_t fpname[32] = {0,};        //最多记录32个字节
		rt_uint8_t fpnameRd[32] = {0,};      //最多记录32个字节	
		strcpy((char*)fpnameRd,"");
		strcpy((char*)fpname,"");
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件记录序号/////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OrderNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->OrderNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->OrderNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////事件发生时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"StartTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->StartTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pChg_Request->StartTimestamp.Year,\
							pChg_Request->StartTimestamp.Month,\
							pChg_Request->StartTimestamp.Day,\
							pChg_Request->StartTimestamp.Hour,\
							pChg_Request->StartTimestamp.Minute,\
							pChg_Request->StartTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////////////////////////////////////////////////////////////////////////////////////		
////////////////////事件结束时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"FinishTimestamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->FinishTimestamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pChg_Request->FinishTimestamp.Year,\
							pChg_Request->FinishTimestamp.Month,\
							pChg_Request->FinishTimestamp.Day,\
							pChg_Request->FinishTimestamp.Hour,\
							pChg_Request->FinishTimestamp.Minute,\
							pChg_Request->FinishTimestamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
/////////事件发生源/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"OccurSource"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->OccurSource,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->OccurSource);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
/////////事件上报状态 = 通道上报状态/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChannelState"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->ChannelState,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->ChannelState);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////充电申请单号 SIZE(16)/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"RequestNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->RequestNO,sizeof(pChg_Request->RequestNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pChg_Request->RequestNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////路由器资产编号 visible-string（SIZE(22)）////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"AssetNO"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->AssetNO,sizeof(pChg_Request->AssetNO));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pChg_Request->AssetNO);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////枪序号	{A枪（1）、B枪（2）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"GunNum"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->GunNum,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->GunNum);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
////////////////////充电申请时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"RequestTimeStamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->RequestTimeStamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pChg_Request->RequestTimeStamp.Year,\
							pChg_Request->RequestTimeStamp.Month,\
							pChg_Request->RequestTimeStamp.Day,\
							pChg_Request->RequestTimeStamp.Hour,\
							pChg_Request->RequestTimeStamp.Minute,\
							pChg_Request->RequestTimeStamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}
/////////当前SOC（单位：%，换算：-2）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"actSOC"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->actSOC,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->actSOC);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////目标SOC（单位：%，换算：-2）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"aimSOC"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->aimSOC,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->aimSOC);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////电池容量（单位：kWh，换算：-2）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"CellCapacity"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->CellCapacity,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->CellCapacity);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////充电需求电量（单位：kWh，换算：-2）/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChargeReqEle"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->ChargeReqEle,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->ChargeReqEle);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}	
////////////////////计划用车时间//////////////////////////////////////////////////////////
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			sprintf((char*)fpnameRd,"PlanUnChg_TimeStamp"); 
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->PlanUnChg_TimeStamp.Second,6);//返回当前文件读指针
				rt_lprintf("%s=%02X%02X%02X%02X%02X%02X;\n",fpname,\
							pChg_Request->PlanUnChg_TimeStamp.Year,\
							pChg_Request->PlanUnChg_TimeStamp.Month,\
							pChg_Request->PlanUnChg_TimeStamp.Day,\
							pChg_Request->PlanUnChg_TimeStamp.Hour,\
							pChg_Request->PlanUnChg_TimeStamp.Minute,\
							pChg_Request->PlanUnChg_TimeStamp.Second);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;							
		}
		else
		{
			rt_lprintf("namelen=0\n");			
		}		
/////////充电模式 {正常（0），有序（1）}/////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"ChargeMode"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->ChargeMode,1);//返回当前文件读指针
				rt_lprintf("%s=%u;\n",fpnameRd,pChg_Request->ChargeMode);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname); 
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
////////用户登录令牌////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"Token"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->Token,sizeof(pChg_Request->Token));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pChg_Request->Token);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}		
////////充电用户账号////////////////////////////////////////////////////////////////////////////////
		sprintf((char*)fpnameRd,"UserAccount"); 
		fpoint = get_name(fpoint,fpname,&namelen);//返回当前文件读指针
		if(namelen)//接收完了
		{
			if(strcmp((const char*)fpname,(const char*)fpnameRd)==0)
			{
				fpoint = get_pvalue(fpoint,(rt_uint32_t*)&pChg_Request->UserAccount,sizeof(pChg_Request->UserAccount));//返回当前文件读指针
				rt_lprintf("%s=%s;\n",fpnameRd,pChg_Request->UserAccount);
			}
			else
			{
				rt_lprintf("%s变量名不符合fpname=%s\n",fpnameRd,fpname);
			}
			namelen=0;
		}
		else
		{
			rt_lprintf("namelen=0\n");
		}
/////////////////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////			
	}
	else
	{
		rt_lprintf("[storage]:%s文件无效命令\n",path_file);
	}

	return 0;	
}
/*********************************************************************************************************
** Function name:		Log_Process
** Descriptions:		LOG函数
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//从str中得到变量名
static char* get_name(char* strtemp,rt_uint8_t*fpname,rt_uint8_t *fpcnt)
{
	while(*strtemp!='\0')//没有结束 文件第一行为标题 第二行为变量开始
	{
		if((*strtemp=='\r')||(*strtemp=='\n'))//最多记录50个字符
		{	
			break;
		}
		strtemp++; 
		
	}
	while(*strtemp!='\0')//没有结束
	{
		if((*strtemp!=' ')&&(*strtemp!='=')&&(*strtemp!='\r')&&(*strtemp!='\n')&&\
			(*strtemp!='/')&&(*fpcnt)<50)//最多记录50个字符
		{	
			fpname[(*fpcnt)++]=*strtemp;//fpcnt必须加括号，++优先级高于*
			strtemp++; 
		}
		else if(*fpcnt) break;//存储之后第一次打断退出
		else strtemp++; 
		
	}
	if(*fpcnt)//接收完了
	{
		fpname[*fpcnt]='\0';//加入结束符
	}
	
  return strtemp; 	
	
}
/*********************************************************************************************************
** Function name:		Log_Process
** Descriptions:		LOG函数
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//从str中得到变量值   最大位数16位
static char* get_pvalue(char* strtemp,rt_uint32_t*fpvalue,rt_uint8_t cnt)
{
	rt_uint16_t i,j = 0;
	rt_uint8_t valueArry[48] = {0};
	rt_uint8_t arryadd[72] = {0};
	rt_uint8_t Arrycnt = 0;
	
	while(*strtemp!='\0')//没有结束
	{	
		if(((*strtemp!=' ')&&(*strtemp!='=')&&(*strtemp!=';')&& \
			(*strtemp<='9')&&(*strtemp>='0')&&(Arrycnt<48))||((*strtemp>='A')&&(*strtemp<='F')))//最多记录48个字符
		{			
			valueArry[Arrycnt++]=*strtemp;//
			strtemp++; 
		}
		else if(Arrycnt) break;//存储之后第一次打断退出
		else strtemp++; 
	}
	if(Arrycnt)//接收完了
	{
		valueArry[Arrycnt++]='\0';//加入结束符
	} 
    if(cnt == 1) //一个变量值
	{		
	    str2num(valueArry,fpvalue);
	}
	else  //多个变量值
	{
		for(i=0,j=0;i<cnt*2; )  //在cnt个变量之间添加分解符 '\0'
		{
		    arryadd[j++] = valueArry[i++];
			arryadd[j++] = valueArry[i++];
			arryadd[j++] = '\0';
		}		

		for(i=0;i<cnt;i++)
		{
		    str2nnum(arryadd+3*i,((rt_uint8_t*)fpvalue)+i);//16进制+空格=3字符
		}
		
	}
	
  return strtemp; 	
}
/*********************************************************************************************************
** Function name:		Log_Process
** Descriptions:		LOG函数
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
int storage_thread_init(void)
{
//	extern void nerase_all(void);
//	nerase_all();//格式化nand文件系统
	
	if(dfs_mount("nand0", "/", "uffs", 0, 0) == RT_EOK)
	{	
		rt_lprintf("[storage]:nand flash mount to / success\n");
	}
	else
	{
		rt_lprintf("[storage]:nand flash mount to / fail\n");
	}
	
	for(rt_uint8_t i = 0; i< MAX_DIR_NUM; i++)
	{
		int result = mkdir(_dir_name[i],0x777);//创建_dir_name包含的文件目录
		if(result == UENOERR)
		{
			rt_lprintf("[storage]:make %s dir ok\n",_dir_name[i]);
		}
		else if(result == (-EEXIST))
		{
			rt_lprintf("[storage]:%s dir is already exist\n",_dir_name[i]);
		}
		else
		{
			rt_lprintf("[storage]:make %s dir fail,error num is %d\n",_dir_name[i],result);
		}
	}
/**********************************************************************************************/	
/**********************************************************************************************/
/**********************************************************************************************/
		/*O_CREAT: Opens the file, if it is existing. If not, a new file is created. */
		/*O_TRUNC: Creates a new file. If the file is existing, it is truncated and overwritten. */
		/*O_EXCL: Creates a new file. The function fails if the file is already existing. */

//		int fd = unlink(METER_GJFMode_PATH_FILE);
//		if(fd >= UENOERR)
//		{
//			rt_lprintf("%s文件删除成功 fd=%d\n",METER_GJFMode_PATH_FILE,fd);
//		}
//		else
//		{
//			rt_lprintf("%s删除文件不存在 fd=%d\n",METER_GJFMode_PATH_FILE,fd);
//		}

//		int fd = rmdir("/LOG");//必须为空目录
//		if(fd >= UENOERR)
//		{
//			rt_lprintf("%s目录删除成功 fd=%d\n","/LOG",fd);
//		}
//		else
//		{
//			rt_lprintf("%s删除目录不存在 fd=%d\n","/LOG",fd);
//		}	
	
//		int fd = open(NAND_LOG_PATH_FILE,O_CREAT);//此处创建文件必须用 O_CREAT
//		if(fd >= 0)
//		{
//			rt_lprintf("%s文件创建成功 fd=%d\n",NAND_LOG_PATH_FILE,fd);
//		}
//		else
//		{
//			rt_lprintf("%s文件创建失败 fd=%d\n",NAND_LOG_PATH_FILE,fd);
//		}

//		
//		if(close(fd) != UENOERR)
//		{
//			rt_lprintf("[storage]:%s文件关闭失败\n",NAND_LOG_PATH_FILE);
//		}
//		else
//		{
//			rt_lprintf("[storage]:%s文件关闭成功\n",NAND_LOG_PATH_FILE);
//		}	
/**********************************************************************************************/	
/**********************************************************************************************/
/**********************************************************************************************/	
//	RouterIfo.AssetNum[0] = 22;
//	memcpy(&RouterIfo.AssetNum[1], "0011223344000000000011", sizeof(RouterIfo.AssetNum)-1);// 表号
//    rt_lprintf("[storage]:电表资产号：%s\n",RouterIfo.AssetNum);	
/**********************************************************************************************/	
/**********************************************************************************************/
/**********************************************************************************************/	
	/* 创建互斥锁 */
	storage_ReWr_mutex = rt_mutex_create("storage_ReWr_mutex", RT_IPC_FLAG_FIFO);
	if (storage_ReWr_mutex == RT_NULL)
	{
		rt_lprintf("[storage]:storage_ReWr_mutex创建互斥锁失败\n");
		return 0;
	}	
/**********************************************************************************************/	
/**********************************************************************************************/
/**********************************************************************************************/	
	rt_err_t res = rt_thread_init(&storage,
											"storage",
											storage_thread_entry,
											RT_NULL,
											storage_stack,
											THREAD_STORAGE_STACK_SIZE,
											THREAD_STORAGE_PRIORITY,
											THREAD_STORAGE_TIMESLICE);
	if (res == RT_EOK) 
	{
			rt_thread_startup(&storage);
	}
	return res;
}


#if defined (RT_STORAGE_AUTORUN) && defined(RT_USING_COMPONENTS_INIT)
	INIT_APP_EXPORT(storage_thread_init);
#endif
MSH_CMD_EXPORT(storage_thread_init, storage thread run);
/*********************************************************************************************************
** Function name:		Log_Process
** Descriptions:		LOG函数
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
int GetStorageData(STORAGE_CMD_ENUM cmd,void *Storage_Para,rt_uint32_t datalen)
{
	if(Para_Buff == NULL)
	{
	    Para_Buff = rt_malloc(MAX_MALLOC_NUM);//申请内存
		if(Para_Buff == NULL)
		{
			rt_kprintf("[storage]:Para_Buff 分配内存失败\n");
			return -1;
		}
	}
	rt_err_t ret = 0;	
	rt_err_t result = rt_mutex_take(storage_ReWr_mutex, RT_WAITING_FOREVER);
	switch(cmd)
	{
		case Cmd_MeterNumRd://
			ret = Router_Para_Storage(ROUTER_PARA_PATH_FILE,Storage_Para,datalen,READ);
		
			break;
		case Cmd_MeterPowerRd://
			ret = Meter_Power_Storage(ROUTER_PARA_PATH_FILE,Storage_Para,datalen,READ);

			break;
		case Cmd_MeterGJFModeRd://
			ret = Meter_GJFMode_Storage(METER_GJFMode_PATH_FILE,Storage_Para,datalen,READ);	
			
			break;			
		case Cmd_MeterHalfPowerRd://
			ret = Meter_HalfPower_Storage(METER_HALF_POWER_PATH_FILE,Storage_Para,datalen,READ);	
			
			break;
		case Cmd_MeterAnalogRd://
			ret = Meter_Anolag_Storage(METER_ANALOG_PATH_FILE,Storage_Para,datalen,READ);	
			
			break;
		case Cmd_HistoryRecordRd://
			ret = Charge_Record_Storage(HISTORY_RECORD_PATH,Storage_Para,datalen,READ);	
			
			break;
		case Cmd_OrderChargeRd:/*有序充电事件记录单元*/
			ret = Order_Charge_Storage(ORDER_CHARGE_PATH,Storage_Para,datalen,READ);	
			
			break;	
		case Cmd_PlanOfferRd:/*充电计划上报记录单元*/
			ret = Plan_Offer_Storage(PLAN_OFFER_PATH,Storage_Para,datalen,READ);	
			
			break;
		case Cmd_PlanFailRd:/*充电计划生成失败记录单元*/
			ret = Plan_Fail_Storage(PLAN_FAIL_PATH,Storage_Para,datalen,READ);	
			
			break;
		case Cmd_OnlineStateRd:/*表计在线状态*/
			ret = Online_State_Storage(ONLINE_STATE_PATH,Storage_Para,datalen,READ);	
			
			break;
		case Cmd_ChgExecuteRd:/*充电执行事件记录单元*/
			ret = Chg_Execute_Storage(CHG_EXECUTE_PATH,Storage_Para,datalen,READ);	
			
			break;              			
		case Cmd_ChgRequestRd:/*充电申请事件记录单元*/
			ret = Chg_Request_Storage(CHG_REQUEST_PATH,Storage_Para,datalen,READ);	
			
			break;		
		default:
			rt_kprintf("[storage]:Waring：%s收到未定义指令%u\r\n",__FUNCTION__,cmd);
		    ret = -1;
			break;
	}	
	
	rt_free(Para_Buff);Para_Buff = NULL;
	
	if (result == RT_EOK)
	{
		/* 释放互斥锁 */
		rt_mutex_release(storage_ReWr_mutex);
	}
	else
	{
		rt_kprintf("[storage]:storage_ReWr_mutex 释放互斥锁失败\n");
	}
	
	return ret;
}
int SetStorageData(STORAGE_CMD_ENUM cmd,void *Storage_Para,rt_uint32_t datalen)
{
	if(Para_Buff == NULL)
	{
	  Para_Buff = rt_malloc(MAX_MALLOC_NUM);//申请内存
		if(Para_Buff == NULL)
		{
			rt_lprintf("[storage]:Para_Buff 分配内存失败\n");
			return -1;
		}
		rt_lprintf("[storage]:Para_Buff rt_mallocOK\n");
	}
	rt_err_t ret = 0;	
	rt_err_t result = rt_mutex_take(storage_ReWr_mutex, RT_WAITING_FOREVER);

	
	switch(cmd)
	{
		case Cmd_MeterNumWr://
			ret = Router_Para_Storage(ROUTER_PARA_PATH_FILE,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_MeterPowerWr://
			ret = Meter_Power_Storage(ROUTER_PARA_PATH_FILE,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_MeterGJFModeWr://
			ret = Meter_GJFMode_Storage(METER_GJFMode_PATH_FILE,Storage_Para,datalen,WRITE);	
			
			break;		
		case Cmd_MeterHalfPowerWr://
			ret = Meter_HalfPower_Storage(METER_HALF_POWER_PATH_FILE,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_MeterAnalogWr://
			ret = Meter_Anolag_Storage(METER_ANALOG_PATH_FILE,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_HistoryRecordWr:/*充电订单事件记录单元*/
			ret = Charge_Record_Storage(HISTORY_RECORD_PATH,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_OrderChargeWr:/*有序充电事件记录单元*/
			ret = Order_Charge_Storage(ORDER_CHARGE_PATH,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_PlanOfferWr:/*充电计划上报记录单元*/
			ret = Plan_Offer_Storage(PLAN_OFFER_PATH,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_PlanFailWr:/*充电计划生成失败记录单元*/
			ret = Plan_Fail_Storage(PLAN_FAIL_PATH,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_OnlineStateWr:/*表计在线状态*/
			ret = Online_State_Storage(ONLINE_STATE_PATH,Storage_Para,datalen,WRITE);	
			
			break;
		case Cmd_ChgExecuteWr:/*充电执行事件记录单元*/
			ret = Chg_Execute_Storage(CHG_EXECUTE_PATH,Storage_Para,datalen,WRITE);	
			
			break;              			
		case Cmd_ChgRequestWr:/*充电申请事件记录单元*/
			ret = Chg_Request_Storage(CHG_REQUEST_PATH,Storage_Para,datalen,WRITE);	
			
			break;		
		default:
			rt_lprintf("[storage]:Waring：%s收到未定义指令%u\r\n",__FUNCTION__,cmd);
		    ret = -1;
			break;	
	}
	
	rt_free(Para_Buff);Para_Buff = NULL;
	
	if (result == RT_EOK)
	{
		/* 释放互斥锁 */
		rt_mutex_release(storage_ReWr_mutex);
	}
	else
	{
		rt_lprintf("[storage]:storage_ReWr_mutex 释放互斥锁失败\n");
	}	
	
	return ret;
}
/**************************************************************
 * 函数名称: rt_uint8_t str2num(rt_uint8_t*src,rt_uint32_t *dest)
 * 参    数: 无
 * 返 回 值: 无
 * 描    述: 把字符串转为数字
 **************************************************************/	
//支持16进制转换,格式为以0X或者0x开头的.
//不支持负数 
//*src:数字字符串指针
//*dest:转换完的结果存放地址.
//返回值:0，成功转换完成.其他,错误代码.
//1,数据格式错误.2,16进制位数为0.3,起始格式错误.4,十进制位数为0.
static rt_uint8_t str2num(rt_uint8_t*src,rt_uint32_t *dest)
{
	rt_uint32_t t;
	rt_uint8_t bnum=0;	//数字的位数
	rt_uint8_t *p;		  
	rt_uint8_t hexdec=10;//默认为十进制数据
	p=src;
	*dest=0;//清零.
	while(1)
	{
		if((*p<='9'&&*p>='0')||(*p<='F'&&*p>='A')||(*p=='X'&&bnum==1))//参数合法
		{
			if(*p>='A') hexdec=16;	//字符串中存在字母,为16进制格式.
			bnum++;					//位数增加.
		}
		else if(*p=='\0')
		    break;	                //碰到结束符,退出.
		else 
			return 1;				//不全是十进制或者16进制数据.
		p++; 
	} 
	p=src;			    //重新定位到字符串开始的地址.
	if(hexdec==16)		//16进制数据
	{
		if(bnum<3) return 2;			//位数小于3，直接退出.因为0X就占了2个,如果0X后面不跟数据,则该数据非法.
		if(*p=='0'&&((*(p+1)=='X')||(*(p+1)=='x')))//必须以'0X'或'0x'开头.
		{
			p+=2;	//偏移到数据起始地址.
			bnum-=2;//减去偏移量	 
		}
		else 
			return 3;//起始头的格式不对
	}
	else if(bnum==0)
		return 4;//位数为0，直接退出.	  
	while(1)
	{
		if(bnum) bnum--;
		if(*p<='9'&&*p>='0') t=*p-'0';	//得到数字的值
		else t=*p-'A'+10;				//得到A~F对应的值	    
		*dest += t*pow_df(hexdec,bnum);		   
		p++;
		if(*p=='\0')break;//数据都查完了.	
	}
	return 0;//成功转换
}
/**************************************************************
 * 函数名称: rt_uint8_t str2nnum(rt_uint8_t*src,rt_uint8_t *dest)
 * 参    数: 无
 * 返 回 值: 无
 * 描    述: 把字符串转为数字
 **************************************************************/	
//支持16进制转换,但是16进制字母必须是大写的,且格式为以0X开头的.
//不支持负数 
//*src:数字字符串指针
//*dest:转换完的结果存放地址.
//返回值:0，成功转换完成.其他,错误代码.
//1,数据格式错误.2,16进制位数为0.3,起始格式错误.4,十进制位数为0.
static rt_uint8_t str2nnum(rt_uint8_t*src,rt_uint8_t *dest)
{
	rt_uint32_t t;
	rt_uint8_t bnum=0;	//数字的位数
	rt_uint8_t *p;		  
	rt_uint8_t hexdec=10;//默认为十进制数据
	p=src;
	*dest=0;//清零.
	while(1)
	{
		if((*p<='9'&&*p>='0')||(*p<='F'&&*p>='A')||(*p=='X'&&bnum==1))//参数合法
		{
			if(*p>='A') hexdec=16;	//字符串中存在字母,为16进制格式.
			bnum++;					//位数增加.
		}
		else if(*p=='\0')
		    break;	                //碰到结束符,退出.
		else 
			return 1;				//不全是十进制或者16进制数据.
		p++; 
	} 
	p=src;			    //重新定位到字符串开始的地址.
	hexdec=16;		    //16进制数据
	  
	while(1)
	{
		if(bnum) bnum--;
		if(*p<='9'&&*p>='0') t=*p-'0';	//得到数字的值
		else t=*p-'A'+10;				//得到A~F对应的值	    
		*dest += t*pow_df(hexdec,bnum);		   
		p++;
		if(*p=='\0') break;//数据都查完了.	
	}
	return 0;//成功转换
}
/*********************************************************************************************************
** Function name:		Log_Process
** Descriptions:		LOG函数
** input parameters:	 
** 						
** return value:		
** Created by:			LCF		  
** Created Date:		20170511	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//m^n函数
//返回值:m^n次方
static rt_uint32_t pow_df(rt_uint8_t m,rt_uint8_t n)
{
	rt_uint32_t result=1;
	while(n--)result*=m;
	return result;
}
//对于nptr指向的字符串，其开头和结尾处的空格被忽视，字符串中间的空格被视为非法字符。
//D:\tc2>test myarg1 myarg2 
//这个时候，argc的值是3，argc[0]的值是”test”，argc[1]的值是”myarg1”，argc[2]的值是”myarg2”。
//argc 的值 这个赋值过程是编译器完成的，我们只需要读出数据就可以了
void WrAssetNum(int argc, char**argv)//WrAssetNum 191000000035 zichan 191000000035
{
	char buf[24];
	if(argc != 2)
	{
		rt_kprintf("Please input single string e.g.\nWrAssetNum *******\n");
		return;	
	}
	rt_uint32_t level = rt_hw_interrupt_disable();
	rt_size_t len = strlen(argv[1]);
	rt_kprintf("len:%d\n",len);
//	char *buf = (char*)rt_malloc(len);
	buf[0] = len;
	memcpy(&buf[1],argv[1],buf[0]);
	for(int i = 0;i < len+1;i++)
	{
		rt_kprintf("buf[%u]=%02X\n",i,buf[i]);
	}
	
	if(GetStorageData(Cmd_MeterNumRd,&RouterInfo,sizeof(RouterInfo))<0)
	{
		memset(&RouterInfo,0,sizeof(RouterInfo));
	}
	
	strcpy(RouterInfo.AssetNum,buf);
	
	
	
	SetStorageData(Cmd_MeterNumWr,&RouterInfo,sizeof(RouterInfo));
	rt_kprintf("WrAssetNum success\n");
//	rt_free(buf);
	rt_hw_interrupt_enable(level);
}
MSH_CMD_EXPORT(WrAssetNum, set assetnum);
//对于nptr指向的字符串，其开头和结尾处的空格被忽视，字符串中间的空格被视为非法字符。
//D:\tc2>test myarg1 myarg2 
//这个时候，argc的值是3，argc[0]的值是”test”，argc[1]的值是”myarg1”，argc[2]的值是”myarg2”。
//argc 的值 这个赋值过程是编译器完成的，我们只需要读出数据就可以了
void RdAssetNum(int argc, char**argv)
{
	char asserr[13];
	if(argc != 1)
	{
		rt_kprintf("Please input single cmd e.g.\nWrAssetNum\n");
		return;	
	}
	rt_uint32_t level = rt_hw_interrupt_disable();
	rt_size_t len = 13;
//	char *buf = (char*)rt_malloc(len);
	GetStorageData(Cmd_MeterNumRd,&RouterInfo,sizeof(RouterInfo));	
	for(int i = 0;i < len;i++)
	{
		rt_kprintf("asserr[%u]=%02X\n",i,RouterInfo.AssetNum[i]);
	}
//	rt_free(buf);
	rt_hw_interrupt_enable(level);
}
MSH_CMD_EXPORT(RdAssetNum, read AssetNum);



//对于nptr指向的字符串，其开头和结尾处的空格被忽视，字符串中间的空格被视为非法字符。
//D:\tc2>test myarg1 myarg2 
//这个时候，argc的值是3，argc[0]的值是”test”，argc[1]的值是”myarg1”，argc[2]的值是”myarg2”。
//argc 的值 这个赋值过程是编译器完成的，我们只需要读出数据就可以了
void WrAddr(int argc, char**argv)//WrAddr 191000000035 zichan 191000000035
{
	char buf[13];
	if(argc != 2)
	{
		rt_kprintf("Please input single string e.g.\nWrAssetNum *******\n");
		return;	
	}
	rt_uint32_t level = rt_hw_interrupt_disable();
	rt_size_t len = strlen(argv[1]);
	rt_kprintf("len:%d\n",len);
//	char *buf = (char*)rt_malloc(len);
	buf[0] = len;
	memcpy(&buf[1],argv[1],buf[0]);
	for(int i = 0;i < len+1;i++)
	{
		rt_kprintf("buf[%u]=%02X\n",i,buf[i]);
	}
	
	if(GetStorageData(Cmd_MeterNumRd,&RouterInfo,sizeof(RouterInfo))<0)
	{
		memset(&RouterInfo,0,sizeof(RouterInfo));
	}
	
	strcpy(RouterInfo.Addr,buf);
	
	SetStorageData(Cmd_MeterNumWr,&RouterInfo,sizeof(RouterInfo));
	rt_kprintf("WrAddr success\n");
//	rt_free(buf);
	rt_hw_interrupt_enable(level);
}
MSH_CMD_EXPORT(WrAddr, set addr);
//对于nptr指向的字符串，其开头和结尾处的空格被忽视，字符串中间的空格被视为非法字符。
//D:\tc2>test myarg1 myarg2 
//这个时候，argc的值是3，argc[0]的值是”test”，argc[1]的值是”myarg1”，argc[2]的值是”myarg2”。
//argc 的值 这个赋值过程是编译器完成的，我们只需要读出数据就可以了
void RdAddr(int argc, char**argv)
{
	char asserr[13];
	if(argc != 1)
	{
		rt_kprintf("Please input single cmd e.g.\nWrAssetNum\n");
		return;	
	}
	rt_uint32_t level = rt_hw_interrupt_disable();
	rt_size_t len = 13;
//	char *buf = (char*)rt_malloc(len);
	GetStorageData(Cmd_MeterNumRd,&RouterInfo,sizeof(RouterInfo));	
	for(int i = 0;i < sizeof(RouterInfo.Addr);i++)
	{
		rt_kprintf("asserr[%u]=%02X\n",i,RouterInfo.Addr[i]);
	}
//	rt_free(buf);
	rt_hw_interrupt_enable(level);
}
MSH_CMD_EXPORT(RdAddr, read addr);









/*
// C prototype : void StrToHex(BYTE *pbDest, BYTE *pbSrc, int nLen)
// parameter(s): [OUT] pbDest - 输出缓冲区
//	[IN] pbSrc - 字符串
//	[IN] nLen - 16进制数的字节数(字符串的长度/2)
// return value: 
// remarks : 将字符串转化为16进制数
*/
#include "ctype.h"
void StrToHex(char *pbDest, char *pbSrc, int nLen)
{
	char h1,h2;
	char s1,s2;
	int i;

	for (i=0; i<nLen; i++)
	{
		h1 = pbSrc[2*i];
		h2 = pbSrc[2*i+1];

		s1 = toupper(h1) - 0x30; //toupper将字符转换为大写英文字母
		if (s1 > 9) 
		s1 -= 7;

		s2 = toupper(h2) - 0x30;
		if (s2 > 9) 
		s2 -= 7;

		pbDest[i] = s1*16 + s2;
	}
}
/*
// C prototype : void HexToStr(BYTE *pbDest, BYTE *pbSrc, int nLen)
// parameter(s): [OUT] pbDest - 存放目标字符串
//	[IN] pbSrc - 输入16进制数的起始地址
//	[IN] nLen - 16进制数的字节数
// return value: 
// remarks : 将16进制数转化为字符串
*/
void HexToStr(char *pbDest, char *pbSrc, int nLen)
{
	char ddl,ddh;
	int i;

	for (i=0; i<nLen; i++)
	{
		ddh = 48 + pbSrc[i] / 16;
		ddl = 48 + pbSrc[i] % 16;
		if (ddh > 57) ddh = ddh + 7;
		if (ddl > 57) ddl = ddl + 7;
		pbDest[i*2] = ddh;
		pbDest[i*2+1] = ddl;
	}

	pbDest[nLen*2] = '\0';
}
