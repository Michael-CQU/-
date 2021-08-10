#include "stdafx.h"

#include "Comm.h"
#include"GLOBALDEFINE.H"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

extern volatile bool sys_exit;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CComm::CComm()
{
//	hMappingPLC = CreateFileMapping((HANDLE)0xFFFFFFFFFFFFFFFF, NULL, PAGE_READWRITE, 0, 256, _T("PLC_COM"));  
	//创建内存映射，把文件映射到内存，最后一个参数为内存映射对象指定名字
	hMappingPLC = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, 256, _T("PLC_COM"));//  INVALID_HANDLE_VALUE代表创建一个与物理文件无关的内存映射，注意256代表字节
	if(hMappingPLC == NULL)  
	{                                                                                                                                                          
		AfxMessageBox(_T("Create PLC_COM failed."));  
		exit(1);  
	}
	//映射对象的一个视图（把内存中的文件映射到进程地址空间上），得到指向共享内存的指针
	LPData = (byte*)MapViewOfFile(hMappingPLC, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	if(LPData == NULL)  
	{  
		AfxMessageBox(_T("Map PLC_COM failed."));  
		exit(1); //表示出错退出 
	}

	LPDataIn = LPData;//测量站的输入数据
	LPDataOut = LPData + 128;//测量站的输出数据
	LPMeasureResult = LPData + 200;

//	hMappingResult = CreateFileMapping((HANDLE)0xFFFFFFFFFFFFFFFF, NULL, PAGE_READWRITE, 0, 0xff, _T("gluecheck"));  
	hMappingResult = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, 0x400, _T("gluecheck"));  
	if(hMappingResult == NULL)
	{
		AfxMessageBox(_T("Create gluecheck failed."));
		exit(1);
	}
	LPDataResult = (byte*)MapViewOfFile(hMappingResult, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	if(LPDataResult == NULL)
	{
		AfxMessageBox(_T("Map gluecheck failed."));
		exit(1);
	}
}
//其他程序要用我建立的共享内存时候，先用OpenFileMapping打开我建立的共享内存，比如（"PLC_COM"），打开后再用MapViewOfFile
CComm::~CComm()
{
	//接触文件映射，关闭内存映射文件对象句柄
	UnmapViewOfFile(LPData);  //释放映射内存
	CloseHandle(hMappingPLC);
	UnmapViewOfFile(LPDataResult);  
	CloseHandle(hMappingResult);
}

void CComm::SetBit(BYTE *lpdata, int index_byte, int index_bit, bool bEnable)
{
	BYTE val;
	val = (0x01 << index_bit);
	if (bEnable)//置位
	{
		*(lpdata + index_byte) = *(lpdata + index_byte) | val;
	}	
	else //清零
	{
		val = ~val;
        *(lpdata + index_byte) = *(lpdata + index_byte) & val;
	}	
}

bool CComm::GetBit(BYTE *lpdata, int index_byte, int index_bit)
{	
	BYTE val = 0x01 << index_bit;
	if (*(lpdata + index_byte) & val)
	{
		return true;
    } 
    else
    {
		return false;
    } 
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////获得PLC输入的信号/////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CComm::GetRobMeasStart(int RobotNum)//测量开始(对应与机器人1通讯输入的0.1)//出发拍照
{
	switch (RobotNum)
	{
	case 1:
		return GetBit(LPDataIn, 0, 1);
		break;
	default:
		return GetBit(LPDataIn, 0, 1);
	}
}
bool CComm::GetTrajChange(int RobotNum)//请求更换车型和轨迹0.0
{
	switch (RobotNum)
	{
	case 1:
		return GetBit(LPDataIn, 0, 0);
		break;
	default:
		return GetBit(LPDataIn, 0, 0);
	}
}
bool CComm::GetRobMeaOverAck(int RobotNum)//测量结束应答(对应与机器人1通讯输入的0.1)
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		return GetBit(LPDataIn,0, 1);
// 		break;
// 	default:
// 		return GetBit(LPDataIn,0, 1);
//  	 }
	return 0;
}

bool CComm::GetMeasResAck(int RobNum)//机器人发回的收到测量结果应答(对应Input0.2)
 {
	 return GetBit(LPDataIn,0, 2);
 }

bool CComm::GetTrajMode(int RobotNum)// 测量模式，正常还是示教
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		return GetBit(LPDataIn,0, 3);
// 		break;
// 	default:
// 		return GetBit(LPDataIn,0,3);
//	}
	return 0;
}

bool CComm::GetTrajStart(int RobNum)//轨迹开始(对应Input0.5)
{
	return GetBit(LPDataIn,0, 5);
}

bool CComm::GetSegmentStart(int RobNum)//段开始(对应Input0.7)////20201229加
{
	return GetBit(LPDataIn, 0, 7);
}

bool CComm::GetTrajOver(int RobotNum)//轨迹结束(0.4)////和原来定义不同，改为0.6位
 {
	 switch (RobotNum)
	 {
	 case 1:
         return GetBit(LPDataIn,0, 6);
      default:
		 return GetBit(LPDataIn,0, 6);
	 }
 }

bool CComm::GetEnableMzr(int RobNum)//使能测量计算机(对应Input0.0)
{
	//return GetBit(LPDataIn,1, 1);
	return 0;
}

bool CComm::GetEmergencyStatus(int RobotNum)//急停应答
{
	//return GetBit(LPDataIn, 1, 2);
	return 0;
}

bool CComm::GetTrajReset(int RobNum)//reset(对应Input0.3)
{
	return GetBit(LPDataIn,0,3);
}

int  CComm::GetRobPointID(int RobotNum)
{
	int rtn1, offset;
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		rtn1 = *(LPDataIn + 2);
// 		break;
// 	default:
// 		rtn1 = *(LPDataIn + 2);
// 	}
	rtn1=*(LPDataIn + 3);
	return rtn1;
 }

int CComm::GetRobTrajID(int RobotNum)//轨迹号(对应与机器人1通讯输入的2个字节和第3个字节)
{	
	int rtn1, offset;
	//rtn1 = *(LPDataIn + 1);
	rtn1 = (*(LPDataIn + 2))*256+*(LPDataIn + 1);//11.24改成上面形式，轨迹号和点号反了
	return rtn1;
}

CString CComm::GetVIN()
{
	char tem[18];
	for (int i=0;i<17;i++)
	{
		tem[i]=*(LPDataIn + 4 + i);
	}
	tem[17] = 0;
	CString rtn;
	rtn = "";
	rtn = rtn + tem;
	return rtn;
}


//点号(对应与机器人1通讯输入的2.0-3.7)
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////输出给PLC/////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CComm::SetRobMovAllow( int RobotNum, bool b)//测量完成，收到机器人应答后，发送机器人运动许可(对应输出给机器人的0.0)
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		SetBit(LPDataOut, 0, 0, b);
// 		break;
// 	default:
// 		SetBit(LPDataOut, 0, 0, b);
// 		break;
// 	}
}
void CComm::SetTrajChangeOver(int RobotNum,bool b )//车型更换完成0.1
{
	switch (RobotNum)
	{
	case 1:
		SetBit(LPDataOut, 0, 1, b);
		break;
	default:
		SetBit(LPDataOut, 0, 1, b);
		break;
	}
}
void CComm::SetRobMeaACK( int RobotNum, bool b)//是否将检测结果发送给机器人(对应输出给机器人的0.2，20210408新加的)
{ 
	 switch (RobotNum)
	 {
	 case 1:
         SetBit(LPDataOut, 0, 2, b);
		 break;
      default:
		  SetBit(LPDataOut, 0, 2, b);
		 break;
	 }
 }

void CComm::SetMeaResEnable( int RobotNum, bool b)//表示测量结果输出(对应输出给机器人的0.3)
{ 
	switch (RobotNum)
	{
	case 1:
		SetBit(LPDataOut, 0, 3, b);
		break;
	default:
		SetBit(LPDataOut, 0, 3, b);
		break;
	}
}

void CComm::SetRobTrajsStartAck( int RobotNum, bool b)//机器人轨迹开始应答(对应输出给机器人的0.5)
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		SetBit(LPDataOut, 0, 5, b);
// 		break;
// 	default:
// 		SetBit(LPDataOut, 0, 5, b);
// 		break;
// 	 }
}

void CComm::SetRobTrajOverAck( int RobotNum, bool b)//轨迹结束应答(对应输出给机器人的0.6)
{
	switch (RobotNum)
	{
	case 1:
		SetBit(LPDataOut, 0, 6, b);
		break;
	default:
		SetBit(LPDataOut, 0, 6, b);
		break;
	}
}

void CComm::SetMesSysOK(int RobotNum, bool b)//系统无故障0.0
{
	switch (RobotNum)
	{
	case 1:
		SetBit(LPDataOut, 0, 0, b);
		break;
	default:
		SetBit(LPDataOut, 0, 0, b);
		break;
	}
}

void CComm::SetMesSysReady(int RobotNum, bool b)//系统准备就绪0.7
{
	switch (RobotNum)
	{
	case 1:
		SetBit(LPDataOut, 0, 7, b);
		break;
	default:
		SetBit(LPDataOut, 0, 7, b);
		break;
	}
}

void CComm::SetMesSysEmergency(int RobotNum, bool b)//系统请求急停
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		SetBit(LPDataOut, 1, 2, b);
// 		break;
// 	default:
// 		SetBit(LPDataOut, 1, 2, b);
// 		break;
// 	}
}
void CComm::SetMesSysResetAck(int RobotNum, bool b)//系统无故障
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		SetBit(LPDataOut, 1, 7, b);
// 		break;
// 	default:
// 		SetBit(LPDataOut, 1, 7, b);
// 		break;
// 	}
}

void CComm:: SetRobPointIdBack(int RobotNum, int Num)//复位(对应输出给机器人的0.7)
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		*(LPDataOut + 2) = Num;
// 		break;
// 	default:
// 		*(LPDataOut + 2) = Num;
// 	}
}

void CComm:: SetRobTrajIdBack(int RobotNum, int Num)//复位(对应输出给机器人的0.7)
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		*(LPDataOut + 3) = Num;
// 		break;
// 	default:
// 		*(LPDataOut + 3) = Num;
// 	}
}

void CComm:: SetRobMeaResult(int RobotNum, bool b)//复位(对应输出给机器人的0.4)
{
// 	switch (RobotNum)
// 	{
// 	case 1:
// 		*(LPDataOut + 3) = Num;
// 		break;
// 	default:
// 		*(LPDataOut + 3) = Num;
// 	 }
	switch (RobotNum)
	{
	case 1:
		SetBit(LPDataOut, 0, 4, b);
		break;
	default:
		SetBit(LPDataOut, 0, 4, b);
		break;
	}
}
 void CComm::SetResetOver(int RobotNum, bool b)///复位完成
 {
	 switch (RobotNum)
	 {
	 case 1:
		 SetBit(LPDataOut, 0, 5, b);
		 break;
	 default:
		 SetBit(LPDataOut, 0, 5, b);
		 break;
	 }
 }
void CComm::SetByte(int byte_index, byte* data, int nums)
{
	if (nums>0)
	{
		for (int i=0;i<nums;i++)
		{
			*(LPDataOut+byte_index+i) =  *(data+i);
		}
	}
}

void CComm::SetBits(int byte_index, byte bits_enable, byte data)
{
	if (bits_enable != 0)
	{
		int j, k, m, n;
		j = 1;
		for (int i=0;i<8;i++)
		{
			k = j<<i;
			m = bits_enable&k;
			if (m>0)
			{
				n = data&k;
				if (n == 0)////置0
				{
					*(LPDataOut+ byte_index) = *(LPDataOut+ byte_index)&(~k);
				}
				else////置1
				{
					*(LPDataOut+ byte_index)=*(LPDataOut+ byte_index)|k;
				}
			}
		}
	}
}

void CComm::SetRobMeaResult1(int RobotNum, bool b)//OKNG信号（20210408新加的）
{
	// 	switch (RobotNum)
	// 	{
	// 	case 1:
	// 		*(LPDataOut + 3) = Num;
	// 		break;
	// 	default:
	// 		*(LPDataOut + 3) = Num;
	// 	 }
	switch (RobotNum)
	{
	case 1:
		SetBit(LPDataOut, 1, 1, b);
		break;
	default:
		SetBit(LPDataOut, 1, 1, b);
		break;
	}
}

