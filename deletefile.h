#include <time.h>

class cyclethread {
//��ȡ�ļ��д���ʱ��;

//��ȡ�ļ�����޸�����
bool GetFileModifyDate(CString filePathName, SYSTEMTIME &modDate);

//ת��ʱ������
time_t TimeConvertToSec(int year, int month, int day);

//�������������������
int DaysOffset(int fYear, int fMonth, int fDay, int tYear, int tMonth, int tDay);

////�������������������(fΪ�ļ�ʱ�䣬tΪϵͳʱ��)
//int MonthOffset(int fMonth, int tMonth)
//{
//	if (tMonth - fMonth < 0)
//		tMonth + 12;
//	return tMonth - fMonth;
//}
int MonthOffset(int fYear, int tYear, int fMonth, int tMonth);

//ɾ��Ŀ¼
bool DeleteDirectory(CString directory_path);

//ɾ��ָ���ļ���Ŀ¼��ȫ���ļ�(�����ļ���) ��������ɾ����
void DeleteAllFile(CString strDir, int days);

//ɾ��ָ���ļ���Ŀ¼��ȫ���ļ�(�����ļ���) (���·�ɾ��)
void DeleteAllFileMonth(CString strDir, int Month);
};