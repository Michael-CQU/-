#include "cyclethread.h"

class cyclethread {
public:
	void cycle_start_process() {
	PylonInitialize();
	int len;
	char tem[21];
	sync_signal_counter = 1;
	ofstream count_figure_number("采图模式记录表.txt", ofstream::app);//记录采图的段号和图片数
	count_figure_number << "段起始图片张数： ";

	theApp.bemeasuring = true;
	theApp.measureovered = false;
	//////////////////////////此处添加关闭测量结果指示灯
	if (theApp.meamode == 5)
	{
		if (robot[1].currenttrajid == 0)
		{
			robot[1].currenttrajid = 1;
		}
	}
	
	theApp.body_type_ml = Comm.GetRobTrajID(1);
	if (theApp.body_type_ml != 0)
	{
		theApp.body_type = theApp.BodyType[theApp.body_type_ml];
	}
	else
	{
		theApp.body_type = 1;/////////////如果始终是0的话，就假定为1号轨迹
	}
	robot[1].currenttrajid = theApp.body_type;
	//cout << "轨迹的值 " << robot[1].currenttrajid << endl;
	for (int tem = 0;tem < theApp.MeasurePointSumRobot[1][robot[1].currenttrajid]; tem++)
	{
		if (robot[1].traj[robot[1].currenttrajid].pointseq[tem][1] < MEASUREPOINTSUM)
		{
			measurepoint[robot[1].traj[robot[1].currenttrajid].pointseq[tem][1]].measured = false;
		}
	}
	theApp.last_point_ml=0;
	int cnt=0;

	Comm.SetRobMeaACK(1,false);
	theApp.rob_cycle_end[1]=false;
	robot[1].currenttrajid=theApp.body_type;

	theApp.current_point=0;

	int robotid;
	theApp.cycle_start=true;

	theApp.cycle_over=false;
	theApp.data_saved_in_cyc_body=false;

	Comm.SetRobTrajOverAck(1,false);
	Comm.SetMeaResEnable(1,false);
	Comm.SetRobMeaResult(1,false);
	Comm.SetRobMeaResult1(1, 0);

	Comm.SetRobTrajOverAck(1,false);
	theApp.rob_cycle_end[1]=false;
	theApp.measure_over[1]=false;

	///////////初始化所有要保存的测量点的点号
	for (int j=0;j<MAXPOINTSPERCYC;j++)
	{
		theApp.MeasuredPoint[1][j]=0;
	}
	theApp.MeasuredPointIndex[1]=0;
	robot[1].pointid=0;
	theApp.image_index = 0;
	theApp.ImageProcessResult[IMAGESOFAREA] = { 0 };
	theApp.Result_Connection[IMAGESOFAREA] = { 0 };
	theApp.counter_in_segment = 1;
	theApp.counter_in_segment_succed = 1;
	theApp.cycle_over=false;
	theApp.FMKIndex=0;
	theApp.traj_to_save[1][0]=65535;
	theApp.traj_to_save[1][1]=65535;
	////////////////////初始化存储参数
	theApp.defective_image_counter = 0;
	theApp.current_imagecolor_index = 0;
	theApp.current_image_index = 0;
	///////////////产生 测量时间和图像保存目录
	theApp.robot_out_home = false;
	CString MeasureTime1, MeasureTime2, MeasureTime3;
	char TemStr[30];
	SYSTEMTIME m_time;
	MeasureTime1 = "'";
	MeasureTime2 = "";
	MeasureTime3 = "";
	GetLocalTime(&m_time);
	sprintf(TemStr, "%04d", m_time.wYear);
	MeasureTime1 += TemStr;
	MeasureTime2 += TemStr;
	*(Comm.LPDataResult + 27) = TemStr[0];
	*(Comm.LPDataResult + 28) = TemStr[1];
	*(Comm.LPDataResult + 29) = TemStr[2];
	*(Comm.LPDataResult + 30) = TemStr[3];
	//cout << TemStr[0] <<" " <<TemStr[1] << " " <<TemStr[2] << " " <<TemStr[3] << " " <<endl;
	MeasureTime1 += "-";
	theApp.stryyyymm = MeasureTime2;
	sprintf(TemStr, "%02d", m_time.wMonth);
	MeasureTime1 += TemStr;
	MeasureTime2 += TemStr;
	MeasureTime1 += "-";
	*(Comm.LPDataResult + 31) = TemStr[0];
	*(Comm.LPDataResult + 32) = TemStr[1];
	//cout << TemStr[0] << " " << TemStr[1] << endl;
	_itoa(m_time.wMonth, TemStr, 10);
	theApp.stryyyymm += TemStr;

	theApp.ImageDir_image_ng = "image_ng";
	theApp.ImageDir_image = "image";
	if (theApp.ImageDir_root == "")
	{
		theApp.ImageDir_image_ng = ".\\" + theApp.ImageDir_image_ng;
		theApp.ImageDir_image = ".\\" + theApp.ImageDir_image;
	}
	else
	{
		theApp.ImageDir_image_ng = theApp.ImageDir_root + "\\" + theApp.ImageDir_image_ng;
		theApp.ImageDir_image = theApp.ImageDir_root + "\\" + theApp.ImageDir_image;
	}

	theApp.ImageDir_image_ny = theApp.ImageDir_image + "\\" + MeasureTime2;//所有图片—年月
	theApp.ImageDir_image_ng_ny = theApp.ImageDir_image_ng + "\\" + MeasureTime2;//错误图像—年月

	sprintf(TemStr, "%02d", m_time.wDay);
	MeasureTime1 += TemStr;
	MeasureTime2 += TemStr;
	MeasureTime1 += " ";
	*(Comm.LPDataResult + 33) = TemStr[0];
	*(Comm.LPDataResult + 34) = TemStr[1];

	theApp.ImageDir_image_r = theApp.ImageDir_image_ny + "\\" + TemStr;//所有图片—日
	theApp.ImageDir_image_ng_r = theApp.ImageDir_image_ng_ny + "\\" + TemStr;//错误图片—日

	sprintf(TemStr, "%02d", m_time.wHour);
	MeasureTime1 += TemStr;
	MeasureTime2 += TemStr;
	MeasureTime3 += TemStr;
	MeasureTime1 += ":";
	*(Comm.LPDataResult + 35) = TemStr[0];
	*(Comm.LPDataResult + 36) = TemStr[1];
	sprintf(TemStr, "%02d", m_time.wMinute);
	MeasureTime1 += TemStr;
	MeasureTime2 += TemStr;
	MeasureTime3 += TemStr;
	MeasureTime1 += ":";
	*(Comm.LPDataResult + 37) = TemStr[0];
	*(Comm.LPDataResult + 38) = TemStr[1];
	sprintf(TemStr, "%02d", m_time.wSecond);
	MeasureTime1 += TemStr;
	MeasureTime2 += TemStr;
	MeasureTime3 += TemStr;
	*(Comm.LPDataResult + 39) = TemStr[0];
	*(Comm.LPDataResult + 40) = TemStr[1];
	MeasureTime1 += "'";
	theApp.MeasureTime = MeasureTime1;

	theApp.bodyid = Comm.GetVIN();

	theApp.ImageDir_image_ng_time = theApp.ImageDir_image_ng_r + "\\" + MeasureTime2 + "_" + theApp.bodyid;//所有图片—年月日时分秒_VIN码
	theApp.ImageDir_image_time = theApp.ImageDir_image_r + "\\" + MeasureTime2 + "_" + theApp.bodyid;//错误图片—年月日时分秒_VIN码

	theApp.ImageDir = theApp.ImageDir_image_time;
	theApp.ImageDir_p = theApp.ImageDir_image_time + "_p";/////保存处理后图像
	//theApp.ImageDir_Ng = theApp.ImageDir_image_ng_time + "_ng";
	theApp.ImageDir_Ng = theApp.ImageDir_image_ng_time + "_p";

	theApp.bodyno = CString(theApp.bodyid);
	*(Comm.LPDataResult + 50) = 1;   //robot id
	*(Comm.LPDataResult + 51) = robot[1].currenttrajid;   //traj  id
	//cout << robot[1].currenttrajid << endl;
	for (int i = 61; i < 256; i++)////clear the result;
	{
		*(Comm.LPDataResult + i) = 0;
	}

	if (theApp.meamode != 4)
	{
		CreateDirectory(theApp.ImageDir_image.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir_image_ng.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir_image_ny.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir_image_ng_ny.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir_image_r.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir_image_ng_r.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir_p.GetBuffer(256), NULL);
		CreateDirectory(theApp.ImageDir_Ng.GetBuffer(256), NULL);
	}
	theApp.ImageDir.ReleaseBuffer();
	theApp.ImageDir_p.ReleaseBuffer();
	theApp.ImageDir_Ng.ReleaseBuffer();

	theApp.image_all_processed = false;
	theApp.IsCompleteCycle = false;

	CString strnp;
	strnp.Format(_T("开始新循环: external=%d, saveall=%d, Traj no=%d"), theApp.m_external_trigger, theApp.ImageSave,robot[1].currenttrajid);
	AppendMessage(strnp, pEditCtrl);
}
	void measureover_process() {
	AppendMessage(_T("measureover_process: start"), pEditCtrl);

	while (!theApp.image_all_processed)
	{
		Sleep(200);
	}

	if (theApp.IsCompleteCycle)
	{
		AppendMessage(_T("measureover_process: CompleteCycle"), pEditCtrl);
		
		int res = 0x80;
		for (int i = 0;i < theApp.MeasuredPointIndex[1];i++)
		{	
			if (measurepoint[theApp.MeasuredPoint[1][i]].MeasureResult != 0)
			{
				res = res | measurepoint[theApp.MeasuredPoint[1][i]].MeasureResult;
			}
		}

		if (res != 0x80)
		{
			res &= 0x7F;
		}

		*(Comm.LPDataResult + 54) = res;//将测量结果给客户端程序
		//cout << res << endl;
		theApp.cheshen_res = res;

		//亮柱灯
		if ((((res & 0x08) != 0) && (theApp.m_duanjiao_enable)) ||
			(((res & 0x04) != 0) && (theApp.m_guokuan_enable)) ||
			(((res & 0x02) != 0) && (theApp.m_guozhai_enable)) ||
			(((res & 0x10) != 0) && (theApp.m_weizhicuowu_enable)))
		{
			*(Comm.LPMeasureResult) = 48;//断胶红灯亮
			Comm.SetRobMeaResult(1, false);
			Comm.SetRobMeaResult1(1, 1);
			

		} 
		else if (res == 0x80)
		{
			*(Comm.LPMeasureResult) = 1;//其它情况绿灯亮
			Comm.SetRobMeaResult(1, true);
			//Comm.SetRobMeaResult1(1, 0);

		}
		else
		{
			*(Comm.LPMeasureResult) = 1;//其它情况绿灯亮
			Comm.SetRobMeaResult(1, true);
			//Comm.SetRobMeaResult1(1, 0);

		}

		Comm.SetMeaResEnable(1, true);

		if (theApp.MeasuredPointIndex[1] > 0 )
		{
			if (theApp.meamode != 4)  ////////////空运行不保存数据
			{
				//采图程序注释以下括号内代码
				if (DETECTION_MODE == true)
				{AppendMessage(_T("measureover_process: WriteDatabase"), pEditCtrl);
				theApp.stryyyymm_save=theApp.stryyyymm;
				theApp.bodyno_save=theApp.bodyno;		//车身流水号
				theApp.MeasureTime_save=theApp.MeasureTime;///测量时间
				theApp.hThreadWriteDatabase = CreateThread(NULL,
					0,
					(LPTHREAD_START_ROUTINE)WriteDatabaseThread,
					NULL,
					0,
					&theApp.ThreadWriteDatabase);

				theApp.data_saved_in_cyc_body=true;
				}
			}
		}
		
		/////////////读取 机器人或plc是否收到结果的信号，等待机器人收到后，清除输出信号。
	DWORD dwTickStart, dwTimeEclipse;
		dwTickStart = GetTickCount();
		while (1)
		{
			dwTimeEclipse = GetTickCount() - dwTickStart;
			if ((!Comm.GetMeasResAck(0)) && (dwTimeEclipse < 3000))///最多等3秒
			{
				Sleep(50);
				continue;
			}
			break;
		}

		CString strnp;
		strnp.Format(_T("reset the signal result enable"));
		AppendMessage(strnp, pEditCtrl);

		//Comm.SetMeaResEnable(1, 1);
		Comm.SetRobMeaResult(1, false);
		//Comm.SetRobMeaResult1(1, 0);

		///////////////////////////////////////////

		CString ImageDir, stem, ImgFileName;
		char ctem[20];

		EnterCriticalSection(&theApp.Critical);

		if (theApp.m_save_defectiveimage)
		{
			if (theApp.defective_image_counter > 0)
			{
				ImageDir = theApp.ImageDir_Ng;
				for (int i = 0;i < theApp.current_imagecolor_index;i++)
				{
					if (((theApp.imagecolor_save_index[i][6] & 0x1F) & theApp.SaveImage_Ng_Type) != 0)
					{
						_itoa(theApp.imagecolor_save_index[i][0], ctem, 10);
						stem = CString(ctem) + "_";
						_itoa(theApp.imagecolor_save_index[i][1], ctem, 10);
						stem = stem + CString(ctem) + "_";
						_itoa(theApp.imagecolor_save_index[i][2], ctem, 10);
						stem = stem + CString(ctem);
						ImgFileName = ImageDir + "\\";
						ImgFileName += stem;
						ImgFileName += ".jpg";
						MbufExport(ImgFileName.GetBuffer(256), M_JPEG_LOSSY, theApp.imagecolor_save[theApp.imagecolor_save_index[i][5]]);
						ImgFileName.ReleaseBuffer();
					}				
				}
			}
		}

		LeaveCriticalSection(&theApp.Critical);
	}
}
	void cycle_over_process() {
	AppendMessage(_T("cycle over!"), pEditCtrl);

	PylonTerminate();

	char diryue[1024] = "";
	char dirimage[1024] = "";
	char dirimage_ng[1024] = "";
#ifdef _WIN32
	_fullpath(diryue, theApp.ImageDir_image_ny, 1024);
	_fullpath(dirimage, theApp.ImageDir_image, 1024);
	_fullpath(dirimage_ng, theApp.ImageDir_image_ng, 1024);
#else
	realpath(theApp.ImageDir_image_ny, diryue);
	realpath(theApp.ImageDir_image, dirimage);
	realpath(theApp.ImageDir_image_ng, dirimage_ng);
#endif
	string::size_type posyue = 0;
	string::size_type postoor = 0;
	string::size_type postng = 0;
	string stringyue(diryue);
	string stringimage(dirimage);
	string stringimage_ng(dirimage_ng);
	//cout << test << endl;
	while ((posyue = stringyue.find_first_of('\\', posyue)) != string::npos)
	{
		stringyue.insert(posyue, "\\");
		posyue = posyue + 2;
	}
	//cout << stringyue << endl;
	while ((postoor = stringimage.find_first_of('\\', postoor)) != string::npos)
	{
		stringimage.insert(postoor, "\\");
		postoor = postoor + 2;
	}
	//cout << stringimage << endl;
	while ((postng = stringimage.find_first_of('\\', postng)) != string::npos)
	{
		stringimage_ng.insert(postng, "\\");
		postng = postng + 2;
	}
	//cout << stringimage_ng << endl;

	//DeleteAllFile(stringyue.c_str(), 2);//删除image/image/年月份目录文件夹下n天前的文件
	//DeleteAllFile(stringimage.c_str(), 2);//删除image/image目录文件夹下n天前的文件
	//DeleteAllFileMonth(stringimage_ng.c_str(), 3);//删除image/image_ng目录文件夹下n月前的文件
	std::thread eight(DeleteAllFile, stringyue.c_str(), theApp.delete_all);
	eight.detach();
	std::thread ninth(DeleteAllFile, stringimage.c_str(), theApp.delete_all);
	ninth.detach();
	std::thread tenth(DeleteAllFileMonth, stringimage_ng.c_str(), theApp.delete_all);
	tenth.detach();

	sync_signal_counter = 1;
	sync_parameter == false;
	Change_parameters();//刷新第一个调用的相机采图参数
	//cout << "调用刷新1参数" << endl;

	ofstream count_figure_number("采图模式记录表.txt", ofstream::app);//记录采图的段号和图片数
	count_figure_number << endl;//关闭段起始txt记录表
	count_figure_number.close();

	EnterCriticalSection(&theApp.Critical);

	theApp.bemeasuring=false;
	theApp.cycle_over=true;
	theApp.cycle_start=false;
	theApp.data_saved=false;

	theApp.IsCompleteCycle = false;

	LeaveCriticalSection(&theApp.Critical);

	CString ssttrr;
	ssttrr.Format(_T("cycle over: CameraMask=%x"), theApp.m_CameraMask);
	AppendMessage(ssttrr, pEditCtrl);
	}
};