#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "GlobalDefine.h"

class CComm  
{
public:
	CComm();
	virtual ~CComm();

public:
	byte* LPData;
	byte* LPDataIn;
	byte* LPDataOut;
	byte* LPMeasureResult;

	HANDLE hMappingPLC;	
	HANDLE hMappingResult;
	byte* LPDataResult;
	
	void SetBit(BYTE *lpdata, int index_byte, int index_bit, bool bEnable);
	bool GetBit(BYTE *lpdata, int index_byte, int index_bit);
	//获取输入信号

	//获取成员变量的值
	bool GetTrajChange(int RobotNum);//请求更换轨迹或车型
	bool GetRobMeasStart(int RobotNum);//测量开始(对应与机器人1通讯输入的0.0)
	bool GetRobMeaOverAck(int RobotNum);//测量结束应答(对应与机器人1通讯输入的0.1)
	bool GetMeasResAck(int RobotNum);//收到测量结果后应答(对应Input0.2)
	bool GetTrajMode(int RobotNum);//轨迹结束(对应Input0.3)
	bool GetTrajStart(int RobotNum);
	bool GetSegmentStart(int RobNum);
	//轨迹开始(对应Input0.5)
	bool GetTrajOver(int RobotNum);//轨迹结束(对应Input0.6)
	bool GetEnableMzr(int RobotNum);//使能测量计算机(对应Input1.1)
	bool GetEmergencyStatus(int RobotNum);//机器人收到急停后发送急停应答，急停信号没有后清除(对应Input1.2)
	bool GetTrajReset(int RobotNum);//请求复位(对应Input1.7)
	int GetRobPointID(int RobotNum);//点号(对应与机器人1通讯输入的2.0-2.7)
	int GetRobTrajID(int RobotNum);//轨迹号(对应与机器人1通讯输入的3.0-3.7)
	CString GetVIN();//获取车身编号
	
	void SetTrajChangeOver(int RobotNum,bool b);//车型更换完成0.1
	void SetRobMovAllow( int RobotNum, bool b);//测量程序测量开始(对应输出给机器人的0.0)
    void SetRobMeaACK( int RobotNum, bool b);//测量请求应答，测量开始(对应输出给机器人的0.2)
  	void SetRobTrajsStartAck( int RobotNum, bool b);//测量轨迹簇OK，可以进入轨迹up了(对应输出给机器人的0.5)
	void SetRobTrajOverAck( int RobotNum, bool b);//示教模式(对应输出给机器人的0.6)
	void SetMesSysOK(int RobotNum, bool b);//系统无故障(对应Output1.0)
	void SetMesSysReady(int RobotNum, bool b);//系统准备就绪(对应Output1.1)
	void SetMesSysEmergency(int RobotNum, bool b);//系统请求急停(对应Output1.2)
	void SetMesSysResetAck( int RobotNum, bool b);//复位应答(对应输出给机器人的0.6)
	void SetRobTrajIdBack(int RobotNum, int Num);

	void SetRobPointIdBack(int RobotNum, int Num);
	void SetMeaResEnable( int RobotNum, bool b);///是否将检测结果发给机器人0.2
	void SetRobMeaResult(int RobotNum, bool b);///第3个字节

	void SetResetOver(int RobotNum, bool b);///复位完成
	void LedOn(int RobotNum, bool b);///2D检测位置设为1灯板亮，0关闭灯板
	void LaserOn(int RobotNum, bool b);///3D检测位置设为1激光器亮，0关闭激光器

	void SetRobMeaResult1(int RobotNum, bool b); 

public:
	void SetByte(int byte_index, byte* data, int nums);
	void SetBits(int byte_index, byte bits_enable, byte data);

	//输入信号
	//输入信号(PLC)
	bool m_enableMzr;//使能测量计算机(对应Input0.0)
	bool m_chooseMzr;//选择测量计算机(对应Input0.1)
 	//输入信号(机器人)
    bool m_rob_MeasStart[ROBOTSUM];//测量开始(对应与机器人1通讯输入的0.0)
	bool m_rob_MeaEndAck[ROBOTSUM];//测量结束应答(对应与机器人1通讯输入的0.1)
	bool m_rob_ModEnd[ROBOTSUM];//车型结束(对应与机器人1通讯输入的0.2)
	bool m_rob_WaitStart[ROBOTSUM];//等待开始(对应与机器人1通讯输入的0.3)
	bool m_rob_Mixedmode[ROBOTSUM];//混合模式(对应与机器人1通讯输入的0.4)
	bool m_rob_Reset[ROBOTSUM];//复位(对应与机器人1通讯输入的0.7)
	BYTE m_rob_TrajID[ROBOTSUM];///轨迹号2.0—2.7
	BYTE m_rob_PointID[ROBOTSUM];//点号(对应与机器人1通讯输入的3.0-3.7)
};
