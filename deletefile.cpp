#include"deletefile"

class cyclethread {
//获取文件夹创建时间
SYSTEMTIME GetFolderCreateTime(CString sFolder)
{
	SYSTEMTIME screatetime;
	HANDLE hDir;
	hDir = CreateFile(sFolder, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_DELETE, NULL,
		OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, NULL);//打开现存目录 只读方式打开即可

	FILETIME lpCreateTime, lpAccessTime, lpWriteTime;
	if (GetFileTime(hDir, &lpCreateTime, &lpAccessTime, &lpWriteTime))
	{
		FILETIME fcreatetime;
		FileTimeToLocalFileTime(&lpCreateTime, &fcreatetime);//转换为本地时间
		FileTimeToSystemTime(&fcreatetime, &screatetime);//转换为系统时间
														 //cout << &lpCreateTime << &fcreatetime << &screatetime << endl;
	}

	CloseHandle(hDir);//关闭文件句柄

	return screatetime;
}

//获取文件最后修改日期
bool GetFileModifyDate(CString filePathName, SYSTEMTIME &modDate)
{
	HANDLE   hFile;
	WIN32_FIND_DATA   wfd;
	SYSTEMTIME   systime;
	FILETIME   localtime;

	memset(&wfd, 0, sizeof(wfd));
	if ((hFile = FindFirstFile(filePathName, &wfd)) == INVALID_HANDLE_VALUE)
		return false;
	//转换时间  
	FileTimeToLocalFileTime(&wfd.ftLastWriteTime, &localtime);
	FileTimeToSystemTime(&localtime, &systime);

	modDate = systime;
	return true;
}

//转换时间秒数
time_t TimeConvertToSec(int year, int month, int day)
{
	tm info = { 0 };
	info.tm_year = year - 1900;
	info.tm_mon = month - 1;
	info.tm_mday = day;
	return mktime(&info);
}

//计算两个日期相差天数
int DaysOffset(int fYear, int fMonth, int fDay, int tYear, int tMonth, int tDay)
{
	int fromSecond = (int)TimeConvertToSec(fYear, fMonth, fDay);
	int toSecond = (int)TimeConvertToSec(tYear, tMonth, tDay);
	return (toSecond - fromSecond) / 24 / 3600;
}

////计算两个日期相差月数(f为文件时间，t为系统时间)
//int MonthOffset(int fMonth, int tMonth)
//{
//	if (tMonth - fMonth < 0)
//		tMonth + 12;
//	return tMonth - fMonth;
//}
int MonthOffset(int fYear, int tYear, int fMonth, int tMonth)
{
	int Year_difference = tYear - fYear;
	//int Month_difference = tMonth - fMonth;
	if (Year_difference > 0)
		tMonth = tMonth + 12 * Year_difference;
	return tMonth - fMonth;
}

//删除目录
bool DeleteDirectory(CString directory_path)
{
	CFileFind finder;
	CString path;
	path.Format(_T("%s\\*.*"), directory_path);
	BOOL bWorking = finder.FindFile(path);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())//处理文件夹
		{
			//递归删除文件夹
			DeleteDirectory(finder.GetFilePath());
			RemoveDirectory(finder.GetFilePath());

		}
		else//删除文件
		{
			DeleteFile(finder.GetFilePath());
		}
	}
	RemoveDirectory(directory_path);

	return true;
}

//删除指定文件夹目录中全部文件(包含文件夹) （按日期删除）
void DeleteAllFile(CString strDir, int days)
{
	if (days < 0)
		days = 30;
	SYSTEMTIME curDate;
	GetLocalTime(&curDate); //获取当前时间
	CFileFind   finder;
	BOOL   bFound = finder.FindFile(strDir + L"\\*", 0);
	while (bFound)
	{
		bFound = finder.FindNextFile();
		if (finder.GetFileName() == "." || finder.GetFileName() == "..")
			continue;
		//   去掉文件(夹)只读等属性
		SetFileAttributes(finder.GetFilePath(), FILE_ATTRIBUTE_NORMAL);
		if (finder.IsDirectory())
		{
			SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFilePath());  //获取文件最后修改时间
																		   //SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFileName());  //获取文件最后修改时间
			int dayOffset = DaysOffset(fDate.wYear, fDate.wMonth, fDate.wDay,
				curDate.wYear, curDate.wMonth, curDate.wDay);
			if (dayOffset > days)
			{
				DeleteDirectory(finder.GetFilePath());
			}
		}
		else
		{
			SYSTEMTIME fDate;
			if (GetFileModifyDate(finder.GetFilePath(), fDate))  //获取文件最后修改时间
			{
				int dayOffset = DaysOffset(fDate.wYear, fDate.wMonth, fDate.wDay,
					curDate.wYear, curDate.wMonth, curDate.wDay);
				if (dayOffset > days)
					DeleteFile(finder.GetFilePath());
			}
		}
	}
	finder.Close();
	//  然后删除该文件夹
	//RemoveDirectory(strDir);
}

//删除指定文件夹目录中全部文件(包含文件夹) (按月份删除)
void DeleteAllFileMonth(CString strDir, int Month)
{
	if (Month < 0)
		Month = 36;
	SYSTEMTIME curDate;
	GetLocalTime(&curDate); //获取当前时间
	CFileFind   finder;
	BOOL   bFound = finder.FindFile(strDir + L"\\*", 0);
	while (bFound)
	{
		bFound = finder.FindNextFile();
		if (finder.GetFileName() == "." || finder.GetFileName() == "..")
			continue;
		//   去掉文件(夹)只读等属性
		SetFileAttributes(finder.GetFilePath(), FILE_ATTRIBUTE_NORMAL);
		if (finder.IsDirectory())
		{
			SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFilePath());  //获取文件最后修改时间
			//SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFileName());  //获取文件最后修改时间
			//int monthOffset = MonthOffset(fDate.wMonth, curDate.wMonth);
			int monthOffset = MonthOffset(fDate.wYear, curDate.wYear, fDate.wMonth, curDate.wMonth);
			if (monthOffset > Month)
			{
				DeleteDirectory(finder.GetFilePath());
			}
		}
		else
		{
			SYSTEMTIME fDate;
			if (GetFileModifyDate(finder.GetFilePath(), fDate))  //获取文件最后修改时间
			{
				//int monthOffset = MonthOffset(fDate.wMonth, curDate.wMonth);
				int monthOffset = MonthOffset(fDate.wYear, curDate.wYear, fDate.wMonth, curDate.wMonth);
				if (monthOffset > Month)
				{
					DeleteDirectory(finder.GetFilePath());
				}
			}
		}
	}
	finder.Close();
	//  然后删除该文件夹
	//RemoveDirectory(strDir);
}
};