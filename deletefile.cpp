#include"deletefile"

class cyclethread {
//��ȡ�ļ��д���ʱ��
SYSTEMTIME GetFolderCreateTime(CString sFolder)
{
	SYSTEMTIME screatetime;
	HANDLE hDir;
	hDir = CreateFile(sFolder, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_DELETE, NULL,
		OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, NULL);//���ִ�Ŀ¼ ֻ����ʽ�򿪼���

	FILETIME lpCreateTime, lpAccessTime, lpWriteTime;
	if (GetFileTime(hDir, &lpCreateTime, &lpAccessTime, &lpWriteTime))
	{
		FILETIME fcreatetime;
		FileTimeToLocalFileTime(&lpCreateTime, &fcreatetime);//ת��Ϊ����ʱ��
		FileTimeToSystemTime(&fcreatetime, &screatetime);//ת��Ϊϵͳʱ��
														 //cout << &lpCreateTime << &fcreatetime << &screatetime << endl;
	}

	CloseHandle(hDir);//�ر��ļ����

	return screatetime;
}

//��ȡ�ļ�����޸�����
bool GetFileModifyDate(CString filePathName, SYSTEMTIME &modDate)
{
	HANDLE   hFile;
	WIN32_FIND_DATA   wfd;
	SYSTEMTIME   systime;
	FILETIME   localtime;

	memset(&wfd, 0, sizeof(wfd));
	if ((hFile = FindFirstFile(filePathName, &wfd)) == INVALID_HANDLE_VALUE)
		return false;
	//ת��ʱ��  
	FileTimeToLocalFileTime(&wfd.ftLastWriteTime, &localtime);
	FileTimeToSystemTime(&localtime, &systime);

	modDate = systime;
	return true;
}

//ת��ʱ������
time_t TimeConvertToSec(int year, int month, int day)
{
	tm info = { 0 };
	info.tm_year = year - 1900;
	info.tm_mon = month - 1;
	info.tm_mday = day;
	return mktime(&info);
}

//�������������������
int DaysOffset(int fYear, int fMonth, int fDay, int tYear, int tMonth, int tDay)
{
	int fromSecond = (int)TimeConvertToSec(fYear, fMonth, fDay);
	int toSecond = (int)TimeConvertToSec(tYear, tMonth, tDay);
	return (toSecond - fromSecond) / 24 / 3600;
}

////�������������������(fΪ�ļ�ʱ�䣬tΪϵͳʱ��)
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

//ɾ��Ŀ¼
bool DeleteDirectory(CString directory_path)
{
	CFileFind finder;
	CString path;
	path.Format(_T("%s\\*.*"), directory_path);
	BOOL bWorking = finder.FindFile(path);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())//�����ļ���
		{
			//�ݹ�ɾ���ļ���
			DeleteDirectory(finder.GetFilePath());
			RemoveDirectory(finder.GetFilePath());

		}
		else//ɾ���ļ�
		{
			DeleteFile(finder.GetFilePath());
		}
	}
	RemoveDirectory(directory_path);

	return true;
}

//ɾ��ָ���ļ���Ŀ¼��ȫ���ļ�(�����ļ���) ��������ɾ����
void DeleteAllFile(CString strDir, int days)
{
	if (days < 0)
		days = 30;
	SYSTEMTIME curDate;
	GetLocalTime(&curDate); //��ȡ��ǰʱ��
	CFileFind   finder;
	BOOL   bFound = finder.FindFile(strDir + L"\\*", 0);
	while (bFound)
	{
		bFound = finder.FindNextFile();
		if (finder.GetFileName() == "." || finder.GetFileName() == "..")
			continue;
		//   ȥ���ļ�(��)ֻ��������
		SetFileAttributes(finder.GetFilePath(), FILE_ATTRIBUTE_NORMAL);
		if (finder.IsDirectory())
		{
			SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFilePath());  //��ȡ�ļ�����޸�ʱ��
																		   //SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFileName());  //��ȡ�ļ�����޸�ʱ��
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
			if (GetFileModifyDate(finder.GetFilePath(), fDate))  //��ȡ�ļ�����޸�ʱ��
			{
				int dayOffset = DaysOffset(fDate.wYear, fDate.wMonth, fDate.wDay,
					curDate.wYear, curDate.wMonth, curDate.wDay);
				if (dayOffset > days)
					DeleteFile(finder.GetFilePath());
			}
		}
	}
	finder.Close();
	//  Ȼ��ɾ�����ļ���
	//RemoveDirectory(strDir);
}

//ɾ��ָ���ļ���Ŀ¼��ȫ���ļ�(�����ļ���) (���·�ɾ��)
void DeleteAllFileMonth(CString strDir, int Month)
{
	if (Month < 0)
		Month = 36;
	SYSTEMTIME curDate;
	GetLocalTime(&curDate); //��ȡ��ǰʱ��
	CFileFind   finder;
	BOOL   bFound = finder.FindFile(strDir + L"\\*", 0);
	while (bFound)
	{
		bFound = finder.FindNextFile();
		if (finder.GetFileName() == "." || finder.GetFileName() == "..")
			continue;
		//   ȥ���ļ�(��)ֻ��������
		SetFileAttributes(finder.GetFilePath(), FILE_ATTRIBUTE_NORMAL);
		if (finder.IsDirectory())
		{
			SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFilePath());  //��ȡ�ļ�����޸�ʱ��
			//SYSTEMTIME fDate = GetFolderCreateTime(finder.GetFileName());  //��ȡ�ļ�����޸�ʱ��
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
			if (GetFileModifyDate(finder.GetFilePath(), fDate))  //��ȡ�ļ�����޸�ʱ��
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
	//  Ȼ��ɾ�����ļ���
	//RemoveDirectory(strDir);
}
};