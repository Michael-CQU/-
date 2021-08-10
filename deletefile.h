#include <time.h>

class cyclethread {
//获取文件夹创建时间;

//获取文件最后修改日期
bool GetFileModifyDate(CString filePathName, SYSTEMTIME &modDate);

//转换时间秒数
time_t TimeConvertToSec(int year, int month, int day);

//计算两个日期相差天数
int DaysOffset(int fYear, int fMonth, int fDay, int tYear, int tMonth, int tDay);

////计算两个日期相差月数(f为文件时间，t为系统时间)
//int MonthOffset(int fMonth, int tMonth)
//{
//	if (tMonth - fMonth < 0)
//		tMonth + 12;
//	return tMonth - fMonth;
//}
int MonthOffset(int fYear, int tYear, int fMonth, int tMonth);

//删除目录
bool DeleteDirectory(CString directory_path);

//删除指定文件夹目录中全部文件(包含文件夹) （按日期删除）
void DeleteAllFile(CString strDir, int days);

//删除指定文件夹目录中全部文件(包含文件夹) (按月份删除)
void DeleteAllFileMonth(CString strDir, int Month);
};