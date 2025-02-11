#include <rtthread.h>
#include <global.h>
#include <stdio.h>



#define ENUM_CHIP_TYPE_CASE(x)   case x: return(#x);
//////////////////////////////////////////////////////////////////////////////////	 
//********************************************************************************
//修改说明
//无
////////////////////////////////////// 全局变量定义区 //////////////////////////////////////////// 


const char ProgramVersion[8] = {"V1.0.06"}; // 版本号 //故意给数组预留一个空位存放结束符 '\0'
CCMRAM char Printf_Buffer[1024];
CCMRAM char Sprintf_Buffer[1024];
//rt_mq_t storage_mq;
ROUTER_INFO_UNIT RouterInfo;
PILE_INFO_UNIT PileInfo;
//ROUTER_FAULT Fault;

CCMRAM ROUTER_WORK_STATE Router_WorkState;
CCMRAM CTRL_CHARGE_EVENT CtrlCharge_Event;

STR_SYSTEM_TIME System_Time_STR;


static rt_mutex_t my_printf_mutex = RT_NULL;//wyg  191025  printf 锁
/////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char DEBUG_MSH = 1;

//const unsigned short CRC16_CCITT_Table[256]={      //CRC16_CCITT余式表        
//    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
//    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
//    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
//    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
//    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
//    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
//    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
//    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
//    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
//    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
//    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
//    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
//    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
//    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
//    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
//    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
//    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
//    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
//    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
//    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
//    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
//    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
//    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
//    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
//    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
//    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
//    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
//    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
//    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
//    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
//    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
//    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
//};

const unsigned char USCRCLo[256]=
{
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
};

const unsigned char USCRCHi[256]=
{
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 
	0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 
	0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 
	0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 
	0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
	0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 
	0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 
	0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 
	0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 
	0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
	0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 
	0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 
	0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 
	0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 
	0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 
	0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 
	0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 
	0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 
	0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 
	0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 
	0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 
	0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 
	0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 
	0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 
	0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 
	0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 
};
/**************************************************************
 * 函数名称:CRC-CCITT   
 * 描    述: SD卡,ISO HDLC, ITU X.25, V.34/V.41/V.42, PPP-FCS  CRC-CCITT16 多项式 x^16 + x^12 + x^5 + 1 (0x1021) 初始值 0x0000 
* 参    数: IN:*ptr
 * 返 回 值: 
 **************************************************************/
// unsigned short CRC16_CCITT_ISO(unsigned char *ptr, unsigned int count)
//{
//    unsigned int  crc16 = 0;
//    unsigned char crcregister;

//		if(ptr == NULL||count<=0||count>0xfffffffC) return 0xff;
//	  while(count--)
//		{
//        crcregister = (unsigned int)(crc16 >> 8);
//        crc16 <<= 8;
//        crc16 ^= CRC16_CCITT_Table[crcregister ^ *ptr];
//        ptr++;
//     }
//    return(crc16&0x0000FFFF);
//}
/**************************************************************
 * 函数名称:CRC-CCITT   
 * 描    述: SD卡,ISO HDLC, ITU X.25, V.34/V.41/V.42, PPP-FCS  CRC-CCITT16 多项式 x^16 + x^12 + x^5 + 1 (0x1021) 初始值 0x0000 
 * 参    数: data本次要参与CRC计算的数据, crc16上次数据计算的CRC结果
 * 返 回 值: 
 **************************************************************/
//unsigned short CRC_CCITT_CalcByte(unsigned char data, unsigned short crc16)
//{
//	unsigned char crcregister;
//    crcregister = (unsigned int)crc16 >> 8;
//    crc16 <<= 8;
//    crc16 ^= CRC16_CCITT_Table[crcregister ^ data];
//    data++;
//    return(crc16&0x0000FFFF);	
//}

/**************************************************************
 * 函数名称:CRC_16校验
 * 参    数: 
 * 返 回 值: 
 * 描    述: 
 **************************************************************/
unsigned int CRC_16(unsigned char *pcData,unsigned int DataLen)
{
	unsigned int iTemp=0;
	unsigned char cCRCHi = 0xFF; // Initial High Byte
	unsigned char cCRCLo = 0xFF; // Initial Low Byte
	unsigned char cIndex ; 

	while (DataLen--) 
	{
		cIndex = cCRCLo ^ *pcData++; // Calculate CRC
		cCRCLo = cCRCHi ^ USCRCLo[cIndex];
		cCRCHi = USCRCHi[cIndex];
	}
	iTemp+=cCRCHi;
	iTemp<<=8;
	iTemp+=cCRCLo;
	return (iTemp);
}
/**************************************************************
 * 函数名称:CRC7   
 * 描    述: SD卡,多项式 x^7 + x^3 + 1 () 初始值 0x0000 
* 参    数: IN:*ptr要参与CRC计算的数据,count数据长度字节数
 * 返 回 值: 
 **************************************************************/
unsigned char CRC7(unsigned char *ptr,unsigned int count) 
{
		unsigned int i,j;
		unsigned char crc=0,Data; 
	  if(ptr == NULL) return 0xff;
		for(i=0;i<count;i++)	
		{
				Data=ptr[i];
				for(j=0;j<8;j++)
				{
					crc<<=1;
					if((Data&0x80)^(crc&0x80)) crc ^=0x09;
					Data<<=1;
				}

		}	
		crc=(crc<<1)|1;
		return crc;
}

//////////////////////////////////////////////////////////////////////////////////
const unsigned char Month_Length[12]={31,28,31,30,31,30,31,31,30,31,30,31};

//年份表
const unsigned int gYearTable[100] = {
    0,  366,  731, 1096, 1461, 1827, 2192, 2557, 2922, 3288,
  3653, 4018, 4383, 4749, 5114, 5479, 5844, 6210, 6575, 6940,
  7305, 7671, 8036, 8401, 8766, 9132, 9497, 9862,10227,10593,
  10958,11323,11688,12054,12419,12784,13149,13515,13880,14245,
  14610,14976,15341,15706,16071,16437,16802,17167,17532,17898,
  18263,18628,18993,19359,19724,20089,20454,20820,21185,21550,
  21915,22281,22646,23011,23376,23742,24107,24472,24837,25203,
  25568,25933,26298,26664,27029,27394,27759,28125,28490,28855,
  29220,29586,29951,30316,30681,31047,31412,31777,32142,32508,
  32873,33238,33603,33969,34334,34699,35064,35430,35795,36160
};

//非闰年
const unsigned int gMonthTable1[12] = 
{
 0,31,59,90,120,151,181,212,243,273,304,334
};
//闰年
const unsigned int gMonthTable2[12] = 
{
 0,31,60,91,121,152,182,213,244,274,305,335
};
////////////////////////////////////////////////////////////////////////////////// 
unsigned long timebin2long(unsigned char *buf)
{
    rt_uint32_t tlong;
    rt_uint8_t ch,ch1;
    ch = (buf[5]/16)*10+buf[5]%16;//年
    tlong = (uint32_t)((uint16_t)gYearTable[ch]);
    ch1 = (buf[4]/16)*10+buf[4]%16;
    if(ch%4 == 0)//月
        tlong += (uint32_t)((uint16_t)gMonthTable2[ch1-1]);
    else
        tlong += (uint32_t)((uint16_t)gMonthTable1[ch1-1]);
    ch = (buf[3]/16)*10+buf[3]%16;//日
    tlong += (uint32_t)((uint8_t)(ch-1));
    tlong *= 24;
    ch = (buf[2]/16)*10+buf[2]%16;//时
    tlong += (uint32_t)((uint8_t)ch);
    tlong *= 60;
    ch = (buf[1]/16)*10+buf[1]%16;//分
    tlong += (uint32_t)((uint8_t)ch);
    tlong *= 60;
    ch = (buf[0]/16)*10+buf[0]%16;//秒
    tlong += (uint32_t)((uint8_t)ch);
    return tlong;
}


/**************************************************************
 * 函数名称: u8 str2bcd(u8*src,u8 *dest)
 * 参    数: 无
 * 返 回 值: 无
 * 描    述: 把字符串转为BCD码
 **************************************************************/	
//支持16进制转换,但是16进制字母必须是大写的,且格式为以0X开头的.
//不支持负数 
//*src:数字字符串指针
//*dest:转换完的结果存放地址.
//返回值:0，成功转换完成.其他,错误代码.
//1,数据格式错误.2,16进制位数为0.3,起始格式错误.4,十进制位数为0.
unsigned char str2bcd(char*src,unsigned char *dest)
{
	unsigned char i,t,bnum=0;	//数字的位数
	char *p;		  
	p=src;
	*dest=0;//清零.
	while(1)
	{
		if((*p<='9'&&*p>='0')||(*p<='F'&&*p>='A'))//参数合法
		{
			bnum++;					//位数增加.
		}
		else if(*p=='\0')
		    break;	                //碰到结束符,退出.
		else 
			return 1;				//不全是十进制或者16进制数据.
		p++; 
	} 
	p=src;			    //重新定位到字符串开始的地址.
	
	for(i=0;i<(bnum/2);i++)
	{
		if((*(p+2*i)<='9'&&*(p+2*i)>='0')&&(*(p+2*i+1)<='9'&&*(p+2*i+1)>='0'))
		{
			t = *(p+2*i)-'0';	//得到数字的值
			t = (((t<<4)&0xf0)|((*(p+2*i+1)-'0')&0x0f));
			*(dest+i) = t;			
		}
		else if((*(p+2*i)<='9'&&*(p+2*i)>='0')&&(*(p+2*i+1)<='F'&&*(p+2*i+1)>='A'))
		{
			t = *(p+2*i)-'0';	//得到数字的值
			t = (((t<<4)&0xf0)|((*(p+2*i+1)-'7')&0x0f));
			*(dest+i) = t;
		}
		else if((*(p+2*i)<='F'&&*(p+2*i)>='A')&&(*(p+2*i+1)<='9'&&*(p+2*i+1)>='0'))
		{
			t = *(p+2*i)-'7';	//得到数字的值
			t = (((t<<4)&0xf0)|((*(p+2*i+1)-'0')&0x0f));
			*(dest+i) = t;
		}
		else
		{
			t = *(p+2*i)-'7';	//得到数字的值
			t = (((t<<4)&0xf0)|((*(p+2*i+1)-'7')&0x0f));
			*(dest+i) = t;
		}
	}
	return 0;//成功转换
}

/**************************************************************
 * 函数名称: u8 str2bcd(u8*src,u8 *dest)
 * 参    数: 无
 * 返 回 值: 无
 * 描    述: 把字符串转为BCD码
 **************************************************************/	
//支持16进制转换,但是16进制字母必须是大写的,且格式为以0X开头的.
//不支持负数 
//*src:数字字符串指针
//*dest:转换完的结果存放地址.
//返回值:0，成功转换完成.其他,错误代码.
//1,数据格式错误.2,16进制位数为0.3,起始格式错误.4,十进制位数为0.
unsigned char bcd2str(unsigned char *src,char *dest,unsigned char count)
{
	unsigned char i,t1,t2;	//数字的位数
	unsigned char *p;

	*dest='0';//清零.
	p=src;			    //重新定位到字符串开始的地址.
	
	for(i=0;i<count;i++)
	{
		t1 = (unsigned char)((*(p+i)>>4)&0x0f);
		t2 = (unsigned char)(*(p+i)&0x0f);		
		if((t1<=0x09)&&(t2<=0x09))
		{
			*(dest+2*i) = (*(p+i)>>4&0x0f)+'0';
			*(dest+2*i+1) = (*(p+i)&0x0f)+'0';
		}
		else if((t1<=0x09)&&(t2>=0x0A)&&(t2<=0x0F))
		{
			*(dest+2*i) = (*(p+i)>>4&0x0f)+'0';
			*(dest+2*i+1) = (*(p+i)&0x0f)+'7';
		}
		else if((t1>=0x0A)&&(t1<=0x0F)&&(t2<=0x09))
		{
			*(dest+2*i) = (*(p+i)>>4&0x0f)+'7';
			*(dest+2*i+1) = (*(p+i)&0x0f)+'0';
		}
		else
		{
			*(dest+2*i) = (*(p+i)>>4&0x0f)+'7';
			*(dest+2*i+1) = (*(p+i)&0x0f)+'7';
		}
	}
	*(dest+2*count) = '\0';
	return 0;//成功转换
}



/**************************************************************
 * 函数名称: void BCD_toInt(uint8_t *data1,uint8_t *data2,uint8_t len)
 * 参    数: 
 * 返 回 值: 
 * 描    述: BCD码转换成十进制
 **************************************************************	
 */
void BCD_toInt(unsigned char *data1,unsigned char *data2,unsigned char len)
{
	unsigned char i;

	for(i=0;i<len;i++)
	{
		data1[i]= (unsigned char)((data2[i]&0xff) - (((data2[i]>>4)&0x0f)*6));
	}
//	return tmp;	
}

/**************************************************************
 * 函数名称: void BCD_toInt(uint8_t *data1,uint8_t *data2,uint8_t len)
 * 参    数: 
 * 返 回 值: 
 * 描    述: 十进制转换成BCD码
 **************************************************************	
 */
void Int_toBCD(unsigned char *data1,unsigned char *data2,unsigned char len)
{
	uint8_t i;

	for(i=0;i<len;i++)
	{
		data1[i]= data2[i] + (data2[i]/10)*6;
	}
//	return tmp;	
}

/**************************************************************
 * 函数名称: uint8_t XOR_Check(uint8_t *pData, uint8_t Len) 
 * 参    数: 
 * 返 回 值: 
 * 描    述: 累加和校验
 **************************************************************/
unsigned char XOR_Check(unsigned char *pData, unsigned int Len)
{
	unsigned int i;
	unsigned char Check_Data;
	
	Check_Data = 0;

	for(i=0;i<Len;i++)
	{
		Check_Data^=*pData;
		pData++;	
	}
	return Check_Data;	
}

void my_printf(char* buf,rt_uint32_t datalenth,rt_uint8_t type,rt_uint8_t cmd,char* head,char* function,char* name)
{
	rt_uint32_t i;	
	
	rt_mutex_take(my_printf_mutex, RT_WAITING_FOREVER);
	
	strcpy((char*)Printf_Buffer,"");
	strcpy((char*)Sprintf_Buffer,"");
	switch(type)
	{
		case MY_HEX:
		{
			for(i = 0; i <datalenth; i++)
			{			 
				sprintf((char*)Sprintf_Buffer,"%02X ",*(buf+i)); 
				strcat((char*)Printf_Buffer,(const char*)Sprintf_Buffer);								
			}
			if(cmd)
				rt_kprintf("%s (%s) %s %s\n",head,function,name,Printf_Buffer);
			else
				rt_lprintf("%s (%s) %s %s\n",head,function,name,Printf_Buffer);
			break;
		}
		case MY_CHAR:
		{
			if(cmd)
				rt_kprintf("%s (%s) %s %s\n",head,function,name,buf);
			else
				rt_lprintf("%s (%s) %s %s\n",head,function,name,buf);
			break;
		}
		default:
			break;
	}
	rt_mutex_release(my_printf_mutex);
}

char* comm_cmdtype_to_string(COMM_CMD_C cmd)
{
	switch(cmd)
	{
		ENUM_CHIP_TYPE_CASE(Cmd_Null);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgRequest);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgRequestAck);
		
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanIssue);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanIssueAck);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanOffer);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanOfferAck);
		
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanAdjust);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanAdjustAck);

		ENUM_CHIP_TYPE_CASE(Cmd_ChgRequestReport);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgRequestReportAck);

		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanExeState);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanExeStateAck);
	//	Cmd_ChgRequestConfirm,					//充电申请确认（通知蓝牙）
		
		ENUM_CHIP_TYPE_CASE(Cmd_StartChg);
		ENUM_CHIP_TYPE_CASE(Cmd_StartChgAck);
		ENUM_CHIP_TYPE_CASE(Cmd_StopChg);
		ENUM_CHIP_TYPE_CASE(Cmd_StopChgAck);
		ENUM_CHIP_TYPE_CASE(Cmd_PowerAdj);
		ENUM_CHIP_TYPE_CASE(Cmd_PowerAdjAck);

		ENUM_CHIP_TYPE_CASE(Cmd_ChgRecord);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgRecordAck);
		ENUM_CHIP_TYPE_CASE(Cmd_DeviceFault);
		ENUM_CHIP_TYPE_CASE(Cmd_DeviceFaultAck);
		ENUM_CHIP_TYPE_CASE(Cmd_PileFault);
		ENUM_CHIP_TYPE_CASE(Cmd_PileFaultAck);
		ENUM_CHIP_TYPE_CASE(Cmd_ChgPlanIssueGetAck);
		
		ENUM_CHIP_TYPE_CASE(Cmd_RouterExeState);
		ENUM_CHIP_TYPE_CASE(Cmd_RouterExeStateAck);
		
		ENUM_CHIP_TYPE_CASE(Cmd_STAOnlineState);
		ENUM_CHIP_TYPE_CASE(Cmd_STAOnlineStateAck);
	}
	return "no cmd";
}




/**************************************************************
 * 函数名称:  
 * 参    数: 
 * 返 回 值: 
 * 描    述: 
 **************************************************************/
 char *itoa(int num,char *str,int radix) 
{  
	/* 索引表 */ 
	char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; 
	unsigned unum; /* 中间变量 */ 
	int i=0,j,k; 
	char temp;
	str[i]='\0';

	/* 确定unum的值 */ 
	if(radix==10&&num<0) /* 十进制负数 */ 
	{ 
		unum=(unsigned)-num; 
		str[i++]='-'; 
	} 
	else 
		unum=(unsigned)num; /* 其它情况 */ 
	/* 逆序 */ 
	do  
	{ 
		str[i++]=index[unum%(unsigned)radix]; 
		unum/=radix; 
	}while(unum); 

	 
	/* 转换 */ 
	if(str[0]=='-') k=1; /* 十进制负数 */ 
	else k=0; 
	/* 将原来的“/2”改为“/2.0”，保证当num在16~255之间，radix等于16时，也能得到正确结果 */  
	for(j=k;j<=(i-k-1)/2.0;j++) 
	{ 
		temp=str[j]; 
		str[j]=str[i-j-1]; 
		str[i-j-1]=temp; 
	} 
	return str; 
}

void my_printf_mutex_init(void)
{
	my_printf_mutex = rt_mutex_create("my_printf_mutex", RT_IPC_FLAG_FIFO);
}
INIT_APP_EXPORT(my_printf_mutex_init);


