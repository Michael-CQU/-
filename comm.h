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
	//��ȡ�����ź�

	//��ȡ��Ա������ֵ
	bool GetTrajChange(int RobotNum);//��������켣����
	bool GetRobMeasStart(int RobotNum);//������ʼ(��Ӧ�������1ͨѶ�����0.0)
	bool GetRobMeaOverAck(int RobotNum);//��������Ӧ��(��Ӧ�������1ͨѶ�����0.1)
	bool GetMeasResAck(int RobotNum);//�յ����������Ӧ��(��ӦInput0.2)
	bool GetTrajMode(int RobotNum);//�켣����(��ӦInput0.3)
	bool GetTrajStart(int RobotNum);
	bool GetSegmentStart(int RobNum);
	//�켣��ʼ(��ӦInput0.5)
	bool GetTrajOver(int RobotNum);//�켣����(��ӦInput0.6)
	bool GetEnableMzr(int RobotNum);//ʹ�ܲ��������(��ӦInput1.1)
	bool GetEmergencyStatus(int RobotNum);//�������յ���ͣ���ͼ�ͣӦ�𣬼�ͣ�ź�û�к����(��ӦInput1.2)
	bool GetTrajReset(int RobotNum);//����λ(��ӦInput1.7)
	int GetRobPointID(int RobotNum);//���(��Ӧ�������1ͨѶ�����2.0-2.7)
	int GetRobTrajID(int RobotNum);//�켣��(��Ӧ�������1ͨѶ�����3.0-3.7)
	CString GetVIN();//��ȡ������
	
	void SetTrajChangeOver(int RobotNum,bool b);//���͸������0.1
	void SetRobMovAllow( int RobotNum, bool b);//�������������ʼ(��Ӧ����������˵�0.0)
    void SetRobMeaACK( int RobotNum, bool b);//��������Ӧ�𣬲�����ʼ(��Ӧ����������˵�0.2)
  	void SetRobTrajsStartAck( int RobotNum, bool b);//�����켣��OK�����Խ���켣up��(��Ӧ����������˵�0.5)
	void SetRobTrajOverAck( int RobotNum, bool b);//ʾ��ģʽ(��Ӧ����������˵�0.6)
	void SetMesSysOK(int RobotNum, bool b);//ϵͳ�޹���(��ӦOutput1.0)
	void SetMesSysReady(int RobotNum, bool b);//ϵͳ׼������(��ӦOutput1.1)
	void SetMesSysEmergency(int RobotNum, bool b);//ϵͳ����ͣ(��ӦOutput1.2)
	void SetMesSysResetAck( int RobotNum, bool b);//��λӦ��(��Ӧ����������˵�0.6)
	void SetRobTrajIdBack(int RobotNum, int Num);

	void SetRobPointIdBack(int RobotNum, int Num);
	void SetMeaResEnable( int RobotNum, bool b);///�Ƿ񽫼��������������0.2
	void SetRobMeaResult(int RobotNum, bool b);///��3���ֽ�

	void SetResetOver(int RobotNum, bool b);///��λ���
	void LedOn(int RobotNum, bool b);///2D���λ����Ϊ1�ư�����0�رյư�
	void LaserOn(int RobotNum, bool b);///3D���λ����Ϊ1����������0�رռ�����

	void SetRobMeaResult1(int RobotNum, bool b); 

public:
	void SetByte(int byte_index, byte* data, int nums);
	void SetBits(int byte_index, byte bits_enable, byte data);

	//�����ź�
	//�����ź�(PLC)
	bool m_enableMzr;//ʹ�ܲ��������(��ӦInput0.0)
	bool m_chooseMzr;//ѡ����������(��ӦInput0.1)
 	//�����ź�(������)
    bool m_rob_MeasStart[ROBOTSUM];//������ʼ(��Ӧ�������1ͨѶ�����0.0)
	bool m_rob_MeaEndAck[ROBOTSUM];//��������Ӧ��(��Ӧ�������1ͨѶ�����0.1)
	bool m_rob_ModEnd[ROBOTSUM];//���ͽ���(��Ӧ�������1ͨѶ�����0.2)
	bool m_rob_WaitStart[ROBOTSUM];//�ȴ���ʼ(��Ӧ�������1ͨѶ�����0.3)
	bool m_rob_Mixedmode[ROBOTSUM];//���ģʽ(��Ӧ�������1ͨѶ�����0.4)
	bool m_rob_Reset[ROBOTSUM];//��λ(��Ӧ�������1ͨѶ�����0.7)
	BYTE m_rob_TrajID[ROBOTSUM];///�켣��2.0��2.7
	BYTE m_rob_PointID[ROBOTSUM];//���(��Ӧ�������1ͨѶ�����3.0-3.7)
};
