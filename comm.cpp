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
	//�����ڴ�ӳ�䣬���ļ�ӳ�䵽�ڴ棬���һ������Ϊ�ڴ�ӳ�����ָ������
	hMappingPLC = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, 256, _T("PLC_COM"));//  INVALID_HANDLE_VALUE������һ���������ļ��޹ص��ڴ�ӳ�䣬ע��256�����ֽ�
	if(hMappingPLC == NULL)  
	{                                                                                                                                                          
		AfxMessageBox(_T("Create PLC_COM failed."));  
		exit(1);  
	}
	//ӳ������һ����ͼ�����ڴ��е��ļ�ӳ�䵽���̵�ַ�ռ��ϣ����õ�ָ�����ڴ��ָ��
	LPData = (byte*)MapViewOfFile(hMappingPLC, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	if(LPData == NULL)  
	{  
		AfxMessageBox(_T("Map PLC_COM failed."));  
		exit(1); //��ʾ�����˳� 
	}

	LPDataIn = LPData;//����վ����������
	LPDataOut = LPData + 128;//����վ���������
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
//��������Ҫ���ҽ����Ĺ����ڴ�ʱ������OpenFileMapping���ҽ����Ĺ����ڴ棬���磨"PLC_COM"�����򿪺�����MapViewOfFile
CComm::~CComm()
{
	//�Ӵ��ļ�ӳ�䣬�ر��ڴ�ӳ���ļ�������
	UnmapViewOfFile(LPData);  //�ͷ�ӳ���ڴ�
	CloseHandle(hMappingPLC);
	UnmapViewOfFile(LPDataResult);  
	CloseHandle(hMappingResult);
}

void CComm::SetBit(BYTE *lpdata, int index_byte, int index_bit, bool bEnable)
{
	BYTE val;
	val = (0x01 << index_bit);
	if (bEnable)//��λ
	{
		*(lpdata + index_byte) = *(lpdata + index_byte) | val;
	}	
	else //����
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
//////////////////////////////////////////���PLC������ź�/////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool CComm::GetRobMeasStart(int RobotNum)//������ʼ(��Ӧ�������1ͨѶ�����0.1)//��������
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
bool CComm::GetTrajChange(int RobotNum)//����������ͺ͹켣0.0
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
bool CComm::GetRobMeaOverAck(int RobotNum)//��������Ӧ��(��Ӧ�������1ͨѶ�����0.1)
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

bool CComm::GetMeasResAck(int RobNum)//�����˷��ص��յ��������Ӧ��(��ӦInput0.2)
 {
	 return GetBit(LPDataIn,0, 2);
 }

bool CComm::GetTrajMode(int RobotNum)// ����ģʽ����������ʾ��
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

bool CComm::GetTrajStart(int RobNum)//�켣��ʼ(��ӦInput0.5)
{
	return GetBit(LPDataIn,0, 5);
}

bool CComm::GetSegmentStart(int RobNum)//�ο�ʼ(��ӦInput0.7)////20201229��
{
	return GetBit(LPDataIn, 0, 7);
}

bool CComm::GetTrajOver(int RobotNum)//�켣����(0.4)////��ԭ�����岻ͬ����Ϊ0.6λ
 {
	 switch (RobotNum)
	 {
	 case 1:
         return GetBit(LPDataIn,0, 6);
      default:
		 return GetBit(LPDataIn,0, 6);
	 }
 }

bool CComm::GetEnableMzr(int RobNum)//ʹ�ܲ��������(��ӦInput0.0)
{
	//return GetBit(LPDataIn,1, 1);
	return 0;
}

bool CComm::GetEmergencyStatus(int RobotNum)//��ͣӦ��
{
	//return GetBit(LPDataIn, 1, 2);
	return 0;
}

bool CComm::GetTrajReset(int RobNum)//reset(��ӦInput0.3)
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

int CComm::GetRobTrajID(int RobotNum)//�켣��(��Ӧ�������1ͨѶ�����2���ֽں͵�3���ֽ�)
{	
	int rtn1, offset;
	//rtn1 = *(LPDataIn + 1);
	rtn1 = (*(LPDataIn + 2))*256+*(LPDataIn + 1);//11.24�ĳ�������ʽ���켣�ź͵�ŷ���
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


//���(��Ӧ�������1ͨѶ�����2.0-3.7)
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////�����PLC/////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CComm::SetRobMovAllow( int RobotNum, bool b)//������ɣ��յ�������Ӧ��󣬷��ͻ������˶����(��Ӧ����������˵�0.0)
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
void CComm::SetTrajChangeOver(int RobotNum,bool b )//���͸������0.1
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
void CComm::SetRobMeaACK( int RobotNum, bool b)//�Ƿ񽫼�������͸�������(��Ӧ����������˵�0.2��20210408�¼ӵ�)
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

void CComm::SetMeaResEnable( int RobotNum, bool b)//��ʾ����������(��Ӧ����������˵�0.3)
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

void CComm::SetRobTrajsStartAck( int RobotNum, bool b)//�����˹켣��ʼӦ��(��Ӧ����������˵�0.5)
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

void CComm::SetRobTrajOverAck( int RobotNum, bool b)//�켣����Ӧ��(��Ӧ����������˵�0.6)
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

void CComm::SetMesSysOK(int RobotNum, bool b)//ϵͳ�޹���0.0
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

void CComm::SetMesSysReady(int RobotNum, bool b)//ϵͳ׼������0.7
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

void CComm::SetMesSysEmergency(int RobotNum, bool b)//ϵͳ����ͣ
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
void CComm::SetMesSysResetAck(int RobotNum, bool b)//ϵͳ�޹���
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

void CComm:: SetRobPointIdBack(int RobotNum, int Num)//��λ(��Ӧ����������˵�0.7)
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

void CComm:: SetRobTrajIdBack(int RobotNum, int Num)//��λ(��Ӧ����������˵�0.7)
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

void CComm:: SetRobMeaResult(int RobotNum, bool b)//��λ(��Ӧ����������˵�0.4)
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
 void CComm::SetResetOver(int RobotNum, bool b)///��λ���
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
				if (n == 0)////��0
				{
					*(LPDataOut+ byte_index) = *(LPDataOut+ byte_index)&(~k);
				}
				else////��1
				{
					*(LPDataOut+ byte_index)=*(LPDataOut+ byte_index)|k;
				}
			}
		}
	}
}

void CComm::SetRobMeaResult1(int RobotNum, bool b)//OKNG�źţ�20210408�¼ӵģ�
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

