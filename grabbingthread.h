void Grabbingthread()
{
	try
	{ 
	 	if (robot[1].currenttrajid == 0)
	 	{
	 		robot[1].currenttrajid = 1;
	 	}
		int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
	// 	int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
	// 	int last_s_id = measurepoint[last_pointindex].sensorid;
	// 	int last_segment_num = measurepoint[last_pointindex].segmentnum;

	////�ڶ�������в���֮ǰ����֤��־λ��ȷ���ϴγ������к�����ͷŸɾ�
	PylonInitialize();
	PylonTerminate();
	////int exitCode = 0;
	PylonInitialize();
	CTlFactory& tlFactory = CTlFactory::GetInstance();
	DeviceInfoList_t devices;
	CString str;
	str.Format(_T("�������Ϊ: %d"), tlFactory.EnumerateDevices(devices));
	AppendMessage(str, pEditCtrl);
	//���������Ϊ0����ǿ��return�޷����в�ͼ
	if (tlFactory.EnumerateDevices(devices) == 0)
	{
		AppendMessage(_T("δ��⵽���������������ߣ�"), pEditCtrl);
		return;
	}
	newcameranumbers = tlFactory.EnumerateDevices(devices);
	//�����������Ϊ4����ǿ��return�޷����в�ͼ
	//if (tlFactory.EnumerateDevices(devices) != 4)
	//{
	//	AppendMessage(_T("�����ȫ������������ߣ�"), pEditCtrl);
	//	return;
	//}
	////-------------------�¼����� �������к�ʵ�������20210407�������ԣ�---------------------
	////CBaslerUniversalInstantCamera camera1(tlFactory.CreateDevice(devices[0]));
	////CBaslerUniversalInstantCamera camera2(tlFactory.CreateDevice(devices[1]));
	////CBaslerUniversalInstantCamera camera3(tlFactory.CreateDevice(devices[2]));
	////CBaslerUniversalInstantCamera camera4(tlFactory.CreateDevice(devices[3]));
	//CBaslerUniversalInstantCamera camera1;
	//CBaslerUniversalInstantCamera camera2;
	//CBaslerUniversalInstantCamera camera3;
	//CBaslerUniversalInstantCamera camera4;

	for (int i = 0; i < tlFactory.EnumerateDevices(devices); i++)
	{
		using namespace Pylon;

		String_t serialNumber = devices[i].GetSerialNumber();

		if (sensor[1].serial_number == serialNumber)
		{
			camera1.Attach(tlFactory.CreateDevice(devices[i]));
		}
		if (sensor[2].serial_number == serialNumber)
		{
			camera2.Attach(tlFactory.CreateDevice(devices[i]));
		}
		if (sensor[3].serial_number == serialNumber)
		{
			camera3.Attach(tlFactory.CreateDevice(devices[i]));
		}
		if (sensor[4].serial_number == serialNumber)
		{
			camera4.Attach(tlFactory.CreateDevice(devices[i]));
		}
	}
	//-----------------------------------------------------------------------------
	
	//ע��ص�����2021.4.15
	//camera1.RegisterImageEventHandler(new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
	camera1.RegisterImageEventHandler(new CSampleImageEventHandler1, RegistrationMode_Append, Cleanup_Delete);
	camera2.RegisterImageEventHandler(new CSampleImageEventHandler2, RegistrationMode_Append, Cleanup_Delete);
	camera3.RegisterImageEventHandler(new CSampleImageEventHandler3, RegistrationMode_Append, Cleanup_Delete);
	camera4.RegisterImageEventHandler(new CSampleImageEventHandler4, RegistrationMode_Append, Cleanup_Delete);

	switch (tlFactory.EnumerateDevices(devices))
	{
	defult:
		AppendMessage(_T("�����Ϊ0������������ߣ�.\r\n"), pEditCtrl);
		break;
	case 1:
		camera1.Open();
		break;
	case 2:
		camera1.Open();
		camera2.Open();
		break;
	case 3:
		camera1.Open();
		camera2.Open();
		camera3.Open();
		break;
	case 4:
		camera1.Open();
		camera2.Open();
		camera3.Open();
		camera4.Open();
		break;
	}
	//camera1.GrabCameraEvents = true;
	//camera2.GrabCameraEvents = true;
	//camera3.GrabCameraEvents = true;
	//camera4.GrabCameraEvents = true;
	int cameranum = 0;
	//int sync_signal_counter = 1;
	bool sync_parameter = false;

	for (int camera_sensor_number = 0; camera_sensor_number < tlFactory.EnumerateDevices(devices); camera_sensor_number++)
	{
		//using namespace Pylon;

		//String_t serialNumber = devices[i].GetSerialNumber();

		if (camera_sensor_number == 0)
		{
			//1�����
			{
				// Before using any pylon methods, the pylon runtime must be initialized. 
				PylonInitialize();
				//����豸
				CTlFactory& tlFactory = CTlFactory::GetInstance();

				////���ÿ�߼�ͼ��ƫ����
				//camera1.Width.SetValue(150);
				//camera1.Height.SetValue(200);
				//camera1.OffsetX.SetValue(580);
				//camera1.OffsetY.SetValue(300);

				//Disable acquisition start trigger if available
				{
					GenApi::IEnumEntry* acquisitionStart = camera1.TriggerSelector.GetEntry(TriggerSelector_AcquisitionStart);
					//if (acquisitionStart && GenApi::IsAvailable(acquisitionStart))
					{
						camera1.TriggerMode.SetValue(TriggerMode_On);
						camera1.LineSelector.SetValue(LineSelector_Line1);
						camera1.LineMode.SetValue(LineMode_Input);
						camera1.TriggerSource.SetValue(TriggerSource_Line1);
						camera1.TriggerActivation.SetValue(TriggerActivation_FallingEdge);
						camera1.TriggerSelector.SetValue(TriggerSelector_FrameStart);
					}
				}

				// Set pixel format to Mono8 if available. if ( GenApi::IsAvailable( camera.PixelFormat.GetEntry(PixelFormat_Mono8)))
				{
					camera1.PixelFormat.SetValue(PixelFormat_Mono8);
				}
				//�����ع�ʱ��
				camera1.ExposureAuto.SetValue(ExposureAuto_Off);
				camera1.GainAuto.SetValue(GainAuto_Off);
				//camera1.ExposureTime.SetValue(120);
				//camera1.Gain.SetValue(5);
				//500 by default
				// The parameter MaxNumBuffer can be used to control the count of buffers
				// allocated for grabbing. The default value of this parameter is 10.
				camera1.MaxNumBuffer = 5000;
				// Start the grabbing of c_countOfImagesToGrab images.
				// The camera device is parameterized with a default configuration which
				// sets up free-running continuous acquisition.
				////����֡��
				//camera1.AcquisitionFrameRate.SetValue(150);

			}
			//����Ϊ2DͿ�����Ĳ���
			camera1.Width.SetValue(theApp.shorttime);//2021.4.13�޸ģ����ڸĽ������ֽ׶����ã��������ĳЩ�����޷����õ�����
			camera1.Height.SetValue(theApp.longtime);
			camera1.AcquisitionFrameRate.SetValue(theApp.wait);
		}
		if (camera_sensor_number == 1)
		{
			//2�����
			{
				// Before using any pylon methods, the pylon runtime must be initialized. 
				PylonInitialize();
				//����豸
				CTlFactory& tlFactory = CTlFactory::GetInstance();
				//���ÿ�߼�ͼ��ƫ����
				//camera2.Width.SetValue(SetDataExchange::editImagelongth);
				//camera2.Height.SetValue(SetDataExchange::editImagewidth);
				//camera2.Width.SetValue(150);
				//camera2.Height.SetValue(200);

				//camera2.OffsetX.SetValue(606);
				//camera2.OffsetY.SetValue(296);

				{
					GenApi::IEnumEntry* acquisitionStart = camera2.TriggerSelector.GetEntry(TriggerSelector_AcquisitionStart);
					{
						camera2.TriggerMode.SetValue(TriggerMode_On);
						camera2.LineSelector.SetValue(LineSelector_Line1);
						camera2.LineMode.SetValue(LineMode_Input);
						camera2.TriggerSource.SetValue(TriggerSource_Line1);
						camera2.TriggerActivation.SetValue(TriggerActivation_FallingEdge);
						camera2.TriggerSelector.SetValue(TriggerSelector_FrameStart);
						camera2.BslImmediateTriggerMode.SetValue(BslImmediateTriggerMode_On);
					}
				}

				{
					camera2.PixelFormat.SetValue(PixelFormat_Mono8);
				}
				//�����ع�ʱ��
				camera2.ExposureAuto.SetValue(ExposureAuto_Off);
				camera2.GainAuto.SetValue(GainAuto_Off);
				//camera2.ExposureTime.SetValue(100);
				//camera2.Gain.SetValue(5);

				camera2.MaxNumBuffer = 5000;

				////����֡��
				//camera2.AcquisitionFrameRate.SetValue(150);
			}
			camera2.Width.SetValue(theApp.shorttime);
			camera2.Height.SetValue(theApp.longtime);
			camera2.AcquisitionFrameRate.SetValue(theApp.wait);
		}
		if (camera_sensor_number == 2)
		{
			//3�����
			{
				// Before using any pylon methods, the pylon runtime must be initialized. 
				PylonInitialize();
				//����豸
				CTlFactory& tlFactory = CTlFactory::GetInstance();
				////���ÿ�߼�ͼ��ƫ����
				//camera3.Width.SetValue(150);
				//camera3.Height.SetValue(200);

				//camera3.OffsetX.SetValue(560);
				//camera3.OffsetY.SetValue(270);
				//camera3.Width.SetValue(SetDataExchange::editImagelongth);
				//camera3.Height.SetValue(SetDataExchange::editImagewidth);

				{
					GenApi::IEnumEntry* acquisitionStart = camera3.TriggerSelector.GetEntry(TriggerSelector_AcquisitionStart);
					{
						camera3.TriggerMode.SetValue(TriggerMode_On);
						camera3.LineSelector.SetValue(LineSelector_Line1);
						camera3.LineMode.SetValue(LineMode_Input);
						camera3.TriggerSource.SetValue(TriggerSource_Line1);
						camera3.TriggerActivation.SetValue(TriggerActivation_FallingEdge);
						camera3.TriggerSelector.SetValue(TriggerSelector_FrameStart);
					}
				}

				{
					camera3.PixelFormat.SetValue(PixelFormat_Mono8);
				}
				//�����ع�ʱ��
				camera3.ExposureAuto.SetValue(ExposureAuto_Off);
				camera3.GainAuto.SetValue(GainAuto_Off);
				//camera3.ExposureTime.SetValue(200);
				//camera3.Gain.SetValue(10);

				camera3.MaxNumBuffer = 5000;

				////����֡��
				//camera3.AcquisitionFrameRate.SetValue(150);
			}
			camera3.Width.SetValue(theApp.shorttime);
			camera3.Height.SetValue(theApp.longtime);
			camera3.AcquisitionFrameRate.SetValue(theApp.wait);
		}
		if (camera_sensor_number == 3)
		{
			//4�����
			{
				// Before using any pylon methods, the pylon runtime must be initialized. 
				PylonInitialize();
				//����豸
				CTlFactory& tlFactory = CTlFactory::GetInstance();
				////���ÿ�߼�ͼ��ƫ����
				//camera4.Width.SetValue(150);
				//camera4.Height.SetValue(200);

				//camera4.OffsetX.SetValue(510);
				//camera4.OffsetY.SetValue(292);
				//camera4.Width.SetValue(SetDataExchange::editImagelongth);
				//camera4.Height.SetValue(SetDataExchange::editImagewidth);
				//if (SetDataExchange::editSensor == 4)
				//{
				//	camera4.OffsetX.SetValue(SetDataExchange::editShift_x);
				//	camera4.OffsetY.SetValue(SetDataExchange::editShift_y);
				//}

				{
					GenApi::IEnumEntry* acquisitionStart = camera4.TriggerSelector.GetEntry(TriggerSelector_AcquisitionStart);
					{
						camera4.TriggerMode.SetValue(TriggerMode_On);
						camera4.LineSelector.SetValue(LineSelector_Line1);
						camera4.LineMode.SetValue(LineMode_Input);
						camera4.TriggerSource.SetValue(TriggerSource_Line1);
						camera4.TriggerActivation.SetValue(TriggerActivation_FallingEdge);
						camera4.TriggerSelector.SetValue(TriggerSelector_FrameStart);
					}
				}

				{
					camera4.PixelFormat.SetValue(PixelFormat_Mono8);
				}
				//�����ع�ʱ��
				camera4.ExposureAuto.SetValue(ExposureAuto_Off);
				camera4.GainAuto.SetValue(GainAuto_Off);
				//camera4.ExposureTime.SetValue(100);
				//camera4.Gain.SetValue(5);

				camera4.MaxNumBuffer = 5000;

				////����֡��
				//camera4.AcquisitionFrameRate.SetValue(150);
			}
			camera4.Width.SetValue(theApp.shorttime);
			camera4.Height.SetValue(theApp.longtime);
			camera4.AcquisitionFrameRate.SetValue(theApp.wait);
		}
	}

//while (1)
//{

switch (tlFactory.EnumerateDevices(devices))
{
case 1:
	camera1.StartGrabbing();//2021.4.13�޸� ��ͼ��startgrabbing���ڻ�ȡ�ź�֮��
	break;
case 2:
	camera1.StartGrabbing();
	camera2.StartGrabbing();
	break;
case 3:
	camera1.StartGrabbing();
	camera2.StartGrabbing();
	camera3.StartGrabbing();
	break;
case 4:
	camera1.StartGrabbing();
	camera2.StartGrabbing();
	camera3.StartGrabbing();
	camera4.StartGrabbing();
	break;
}

	CPylonImageWindow win;
	if (theApp.mode == 1)
		//if(1 == 1)
	{
		win.Create(1, 0, 0, 520, 520);
	}
	else
	{
		win.Create(1, 0, 0, 170, 320);
	}
	//ofstream count_figure_number("��ͼģʽ��¼��.txt",ofstream::app);//��¼��ͼ�Ķκź�ͼƬ��
	//count_figure_number << "����ʼͼƬ������ ";

	//while (1)
	{
		//����ʼ������������ÿһ����ʼλ�õ�ͼƬ��
		//theApp.counter_in_segment = theApp.setdata_figurenum[robot[1].currenttrajid][sync_signal_counter];
		
		//theApp.current_image_index = theApp.sync_figurenum[robot[1].currenttrajid][sync_signal_counter];
		//theApp.current_imagecolor_index = theApp.sync_figurenum[robot[1].currenttrajid][sync_signal_counter];
		//cout << theApp.counter_in_segment << endl;
		
		//while (Comm.GetTrajStart(1) == 0)
		//{
		//	Sleep(10);
		//}

		//if (sync_signal_counter == theApp.sync_parameter[robot[1].currenttrajid][sync_signal_counter]||sync_parameter == true)
		//if (theApp.sync_parameter[robot[1].currenttrajid][sync_signal_counter] == -1 && sync_parameter == true)
		//if (theApp.setdata_parameter[robot[1].currenttrajid][sync_signal_counter] == -1 && sync_parameter == false)
		//{
		//	if (theApp.mode == 1)
		//	{
		//		//cout << sync_signal_counter << endl;
		//		//����Ϊ2DͿ�����Ĳ���
		//		if(theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 1)
		//		{
		//			//cout << "camera1���ò���" << sync_signal_counter << endl;
		//			//camera1.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][1]);
		//			//camera1.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][1]);
		//			camera1.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][1]);//390
		//			camera1.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][1]);//178
		//			camera1.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][1]);
		//			camera1.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][1]);
		//			//camera1.ExposureTime.SetValue(39240);
		//			camera1.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][1]);
		//			//cout << theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][1] << endl;
		//			//camera1.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][1]);
		//			string number = camera1.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device1: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}

		//		if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 2)
		//		{
		//			//cout << "camera2���ò���" << sync_signal_counter << endl;
		//			//cout << "camera2���ò���" << theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][2] << endl;
		//			//cout << "camera2���ò���" << theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][2] << endl;
		//			//camera2.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][2]);
		//			//camera2.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][2]);
		//			camera2.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][2]);//390
		//			camera2.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][2]);//178
		//			camera2.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][2]);
		//			camera2.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][2]);
		//			//camera1.ExposureTime.SetValue(39240);
		//			camera2.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][2]);
		//			//camera2.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][2]);
		//			//camera2.OverlapMode.SetValue(OverlapMode_On);
		//			//camera2.BslImmediateTriggerMode.SetValue(BslImmediateTriggerMode_On);
		//			//camera2.LineDebouncerTime.SetValue(10);
		//			string number = camera2.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device2: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}

		//		if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 3)
		//		{
		//			//cout << "camera3���ò���" << sync_signal_counter << endl;
		//			//camera3.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][3]);
		//			//camera3.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][3]);
		//			camera3.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][3]);//390
		//			camera3.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][3]);//178
		//			camera3.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][3]);
		//			camera3.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][3]);
		//			//camera1.ExposureTime.SetValue(39240);
		//			camera3.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][3]);
		//			//camera3.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][3]);
		//			string number = camera3.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device3: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}
		//		if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 4)
		//		{
		//			//cout << "camera4���ò���" << sync_signal_counter << endl;
		//			//cout << "camera4���ò���" << theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][4] << endl;
		//			//camera4.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][4]);
		//			//camera4.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][4]);
		//			camera4.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][4]);//390
		//			camera4.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][4]);//178
		//			camera4.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][4]);
		//			camera4.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][4]);
		//			//camera1.ExposureTime.SetValue(39240);
		//			//camera4.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][4]);
		//			camera4.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][4]);

		//			//camera4.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][4]);
		//			string number = camera4.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device4: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}
		//	}
		//	if (theApp.mode == 2)
		//	{
		//		//����Ϊ3DͿ�����Ĳ���
		//		if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 1)
		//		{
		//			camera1.Width.SetValue(150);
		//			camera1.Height.SetValue(200);
		//			camera1.OffsetX.SetValue(560);
		//			camera1.OffsetY.SetValue(224);
		//			camera1.ExposureTime.SetValue(150);
		//			camera1.Gain.SetValue(0);
		//			camera1.Gamma.SetValue(1.0);
		//			camera1.AcquisitionFrameRate.SetValue(150);
		//			string number = camera1.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device1: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}
		//		if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 2)
		//		{
		//			camera2.Width.SetValue(150);
		//			camera2.Height.SetValue(200);
		//			camera2.OffsetX.SetValue(582);
		//			camera2.OffsetY.SetValue(250);
		//			camera2.ExposureTime.SetValue(150);
		//			camera2.Gain.SetValue(0);
		//			camera2.Gamma.SetValue(1.0);
		//			camera2.AcquisitionFrameRate.SetValue(150);
		//			string number = camera2.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device2: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}
		//		if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 3)
		//		{
		//			camera3.Width.SetValue(150);
		//			camera3.Height.SetValue(200);
		//			camera3.OffsetX.SetValue(560);
		//			camera3.OffsetY.SetValue(262);
		//			camera3.ExposureTime.SetValue(150);
		//			camera3.Gain.SetValue(0);
		//			camera3.Gamma.SetValue(1.0);
		//			camera3.AcquisitionFrameRate.SetValue(150);
		//			string number = camera3.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device3: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}
		//		if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 4)
		//		{
		//			camera4.Width.SetValue(150);
		//			camera4.Height.SetValue(200);
		//			camera4.OffsetX.SetValue(524);
		//			camera4.OffsetY.SetValue(246);
		//			camera4.ExposureTime.SetValue(150);
		//			camera4.Gain.SetValue(0);
		//			camera4.Gamma.SetValue(1.0);
		//			camera4.AcquisitionFrameRate.SetValue(150);
		//			string number = camera4.GetDeviceInfo().GetSerialNumber();
		//			str.Format(_T("Using device4: %s"), number);
		//			AppendMessage(str, pEditCtrl);
		//		}
		//	}
		//	sync_parameter = false;
		//	CString str;
		//	str.Format(_T("�κ�: %d������ʼͼƬΪ�ڣ�%d ��"), sync_signal_counter, theApp.counter_in_segment);
		//	AppendMessage(str, pEditCtrl);
		//	count_figure_number <<theApp.counter_in_segment;
		//	count_figure_number <<"   ";
		//}

		

		//bool b1,b2,b3,b4;
		//b1 = camera1.LineStatus.GetValue();
		//b2 = camera2.LineStatus.GetValue();
		//b3 = camera3.LineStatus.GetValue();
		//b4 = camera4.LineStatus.GetValue();

		int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
		int segment_num = measurepoint[pointindex].segmentnum;
		int sensorid = measurepoint[pointindex].sensorid;

		int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
		int last_s_id = measurepoint[last_pointindex].sensorid;
		int last_segment_num = measurepoint[last_pointindex].segmentnum;


// 		if (measurepoint[pointindex].segmentnum != last_segment_num)///////�������˻��������
// 		{
// 			theApp.counter_in_segment = 0;
// 		}

		//�����ⲿ����������лص�
		std::thread c1(camera1caitu);
		c1.detach();
		std::thread c2(camera2caitu);
		c2.detach();		
		std::thread c3(camera3caitu);
		c3.detach();		
		std::thread c4(camera4caitu);
		c4.detach();

		//camera1.RetrieveResult(INFINITE, ptrGrabResult1, TimeoutHandling_ThrowException);
		//camera2.RetrieveResult(INFINITE, ptrGrabResult2, TimeoutHandling_ThrowException);
		//camera3.RetrieveResult(INFINITE, ptrGrabResult3, TimeoutHandling_ThrowException);
		//camera4.RetrieveResult(INFINITE, ptrGrabResult4, TimeoutHandling_ThrowException);
	}
	ptrGrabResult1.Release();
	ptrGrabResult2.Release();
	ptrGrabResult3.Release();
	ptrGrabResult4.Release();

	}
	catch (Exception e)
	{
		MessageBox(NULL, _T(e.what()), _T("grap fail"),MB_OK);
	}
}
class CSampleImageEventHandler1 : public CBaslerUniversalImageEventHandler
{
public:
	virtual void OnImageGrabbed(CBaslerUniversalInstantCamera& camera1, CBaslerUniversalGrabResultPtr& ptrGrabResult1)
	{
		//bool isGrabbingFlag;
		//cout << "CSampleImageEventHandler1::OnImageGrabbed called." << std::endl;

		// �ȴ��ⲿ����
		camera1.TriggerMode.SetValue(TriggerMode_Off);
		sync_parameter = true;	
	};
};
class CSampleImageEventHandler2 : public CBaslerUniversalImageEventHandler
{
public:
	virtual void OnImageGrabbed(CBaslerUniversalInstantCamera& camera2, CBaslerUniversalGrabResultPtr& ptrGrabResult2)
	{
		//bool isGrabbingFlag;
		//cout << "CSampleImageEventHandler2::OnImageGrabbed called." << std::endl;

		// �ȴ��ⲿ����
		camera2.TriggerMode.SetValue(TriggerMode_Off);
		sync_parameter = true;

	};
};
class CSampleImageEventHandler3 : public CBaslerUniversalImageEventHandler
{
public:
	virtual void OnImageGrabbed(CBaslerUniversalInstantCamera& camera3, CBaslerUniversalGrabResultPtr& ptrGrabResult3)
	{
		//bool isGrabbingFlag;
		//cout << "CSampleImageEventHandler3::OnImageGrabbed called." << std::endl;

		// �ȴ��ⲿ����
		camera3.TriggerMode.SetValue(TriggerMode_Off);
		sync_parameter = true;
	};
};
class CSampleImageEventHandler4 : public CBaslerUniversalImageEventHandler
{
public:
	virtual void OnImageGrabbed(CBaslerUniversalInstantCamera& camera4, CBaslerUniversalGrabResultPtr& ptrGrabResult4)
	{
		//bool isGrabbingFlag;
		//cout << "CSampleImageEventHandler4::OnImageGrabbed called." << std::endl;

		// �ȴ��ⲿ����
		camera4.TriggerMode.SetValue(TriggerMode_Off);
		sync_parameter = true;
	};
};
void camera1caitu()
{
	while (1) 
	{
		if (newcameranumbers < 1)
			break;
		camera1.RetrieveResult(INFINITE, ptrGrabResult1, TimeoutHandling_ThrowException);
		mut.lock();
		//cout << "�߳�1������" << endl;

		while (camera1.IsGrabbing())
		{
			//cout << "1�Ų�ͼ����" << endl;
			//cout << theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][1] << endl;
			//double framecs1 = camera1.AcquisitionFrameRate.GetValue();
			//double framecs2 = camera1.ResultingFrameRate.GetValue();
			//cout << "���֡��" << framecs1 << "    " << framecs2 << endl;
			//int offsetx1 = camera1.OffsetX.GetValue();
			//int offsety1 = camera1.OffsetY.GetValue();
			//cout << "���1ƫ��" << offsetx1 << "    " << offsety1 << endl;
			//cout << "sync_signal_counter:" << sync_signal_counter << endl;
			//int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
			//int segment_num = measurepoint[pointindex].segmentnum;
			//int sensorid = measurepoint[pointindex].sensorid;
			//int sensorid = 1;

			int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
			int last_s_id = measurepoint[last_pointindex].sensorid;
			int last_segment_num = measurepoint[last_pointindex].segmentnum;

			//cout << "last_pointindex:" << last_pointindex << " last_s_id:" << last_s_id << " last_segment_num:" << last_segment_num << endl;
			{
				bool b = camera1.LineStatus.GetValue();
				if (b == true)
				{
					//AppendMessage(_T("��ȡ��IO�ڵ͵�ƽ"), pEditCtrl);
					////theApp.counter_in_segment = 0;
					camera1.TriggerMode.SetValue(TriggerMode_On);
					//camera1.StopGrabbing();
					//camera1.StartGrabbing();
					sync_parameter = false;
					//cout << theApp.counter_in_segment << endl;
					//cout << theApp.camera_cap_total_counter[1] << endl;
					/*Change_parameters();*/
					sync_signal_counter++;

					//cout << theApp.counter_in_segment << endl;
					//cout << theApp.camera_cap_total_counter[1] << endl;

					mut.unlock();
					//cout << "�߳�1������" << endl;
					break;
				}
			}
			//Sleep(10);
			//camera1.RetrieveResult(INFINITE, ptrGrabResult1, TimeoutHandling_ThrowException);
			unsigned char* imagebuffer = NULL;
			CString str, strTemp;
			int pointindex;
			int sensorid = 1;
			USES_CONVERSION;
			if(DETECTION_MODE == true)
			Comm.SetRobMeaACK(1, TRUE);
			//AppendMessage("onframe 1", pEditCtrl);
			pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
			//cout << "pointindex:" << pointindex<< endl;
			if (theApp.mode == 1)//2DͿ��
			{
				//AppendMessage("2D", pEditCtrl);

				//�����߳���
				//mutex mut;
				//mut.lock();
				//cout << "�߳�1������" << endl;
				//theApp.counter_in_segment++;
				//mut.unlock();

				//str.Format(_T("2D: %d_%d"), pointindex, theApp.counter_in_segment);
				//AppendMessage(str, pEditCtrl);
				//if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml)
				if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml &&theApp.counter_in_segment_succed == measurepoint[pointindex].pointid_ml || DETECTION_MODE == false)
				{
					//cout << "pointindex = " << pointindex << endl;
					//cout << "theApp.counter_in_segment = " << theApp.counter_in_segment << endl;
					theApp.last_point_ml++;
					robot[1].pointid++;

					theApp.camera_cap_total_counter[sensorid]++;

					//cout << theApp.counter_in_segment << endl;
					//cout << theApp.camera_cap_total_counter[1] << endl;

					theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
					theApp.MeasuredPointIndex[1]++;

					//cout << "theApp.current_image_index:" <<theApp.current_image_index << endl;
					//cout << "sync_signal_counter:" << sync_signal_counter << endl;

					theApp.image_save_index[theApp.current_image_index][0] = 1;//�����˺�
					//theApp.image_save_index[theApp.current_image_index][1] = measurepoint[pointindex].trajid;//�켣��
					theApp.image_save_index[theApp.current_image_index][2] = measurepoint[pointindex].pointid_ml;// ���
					theApp.image_save_index[theApp.current_image_index][1] = theApp.body_type_ml;//�켣��
					//theApp.image_save_index[theApp.current_image_index][2] = theApp.current_image_index;// ���
					theApp.image_save_index[theApp.current_image_index][3] = theApp.counter_in_segment;//���к�
					theApp.image_save_index[theApp.current_image_index][4] = 0;//����ǰͼ��
					theApp.image_save_index[theApp.current_image_index][5] = 0;//
					theApp.image_save_index[theApp.current_image_index][6] = theApp.current_image_index;//������
					theApp.image_save_index[theApp.current_image_index][7] = 0;///δ����
					theApp.image_save_index[theApp.current_image_index][8] = sensorid;///�����
					theApp.image_save_index[theApp.current_image_index][9] = sync_signal_counter;///�κ�

					//theApp.counter_in_segment++;
					theApp.counter_in_segment_succed++;
					//�ж�֡����Ч��
					if (!ptrGrabResult1.IsValid())
					{
						//theApp.image_save_index[theApp.current_image_index][5] = 0;//�ǻ�ͼ��
						AppendMessage("camera1: frame is invalid!\r\n", pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
						//	return;
					}
					else
					{
						theApp.image_save_index[theApp.current_image_index][5] = 1;//�Ǻ�ͼ��
																				   //MbufPut2d(theApp.image_color_bayer[theApp.current_image_index - 1], 0, 0, width, height, imagebuffer);
																				   //MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);
						imagebuffer = (uint8_t*)ptrGrabResult1->GetBuffer();
						MbufPut2d(theApp.image_save[theApp.current_image_index], 0, 0, ptrGrabResult1->GetWidth(), ptrGrabResult1->GetHeight(), imagebuffer);
						//camera1.RetrieveResult(INFINITE, ptrGrabResult1, TimeoutHandling_ThrowException);
						//cout << "theApp.current_image_index:" << theApp.current_image_index << endl;
						//int GetWidth1 = ptrGrabResult->GetWidth();
						//int GetHeight1 = ptrGrabResult->GetHeight();
						//cv::Mat image111 = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult->GetBuffer());
						//cv::imwrite("111.jpg",image111);
						//cv::Mat image111 = cv::Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8U, imagebuffer);
						//cout << theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][1] << endl;
						//cv::imwrite(cv::format("%d.jpg", theApp.counter_in_segment), image111);
						//cv::imwrite("222.jpg", image222);
						//MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);

						//str.Format(_T("Camera[1]: index=%d, %d of %d,counter in segment %d: Buffer is Good!"), pointindex, theApp.current_image_index, robot[1].traj[robot[1].currenttrajid].pointsum, theApp.counter_in_segment);
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
						//AppendMessage(str, pEditCtrl);
					

					theApp.current_image_index++;

					//str.Format(_T("Camera[1]: %d, %d, %d, %d, %d. %d"), theApp.current_image_index, robot[1].pointid, theApp.camera_cap_total_counter[sensorid], theApp.camera_cap_g_counter[sensorid], theApp.camera_cap_ng_counter[sensorid], theApp.counter_in_segment);
					//AppendMessage(str, pEditCtrl);

					//if (robot[1].pointid <= robot[1].traj[robot[1].currenttrajid].pointsum)
					//{
					//	//PrepareForCapture(robot[1].currenttrajid, robot[1].pointid + 1);
					//}
					*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����(��ͼʱ��ע��)
					//cout << "��Ч��Ϊ��" << theApp.last_point_ml << endl;

					pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
					last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																									 //   if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																									 //{
																									 //	theApp.counter_in_segment = 0;
																									 //	//camera1.StopGrabbing();
																									 //	//break;
																									 //}
				}
					}
				//if (ptrGrabResult1.IsValid()) 
					camera1.RetrieveResult(INFINITE, ptrGrabResult1, TimeoutHandling_ThrowException);
				
				theApp.counter_in_segment++;
				//if ((robot[1].pointid >= robot[1].traj[robot[1].currenttrajid].pointsum) ||
				//	(measurepoint[pointindex].segmentnum != last_segment_num))///////�������˻��������

			}


			if (theApp.mode == 2)//3DͿ��
			{
				//AppendMessage("3D", pEditCtrl);

				theApp.current_sensorid = 1;

				// 						theApp.counter_in_segment++;
				//pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
				//segment_num = measurepoint[pointindex].segmentnum;
				//sensorid = measurepoint[pointindex].sensorid;
				if (ptrGrabResult1->GrabSucceeded())
				{
					const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult1->GetBuffer();

					if (ptrGrabResult1.IsValid())
					{
						if (theApp.current_sensorid != theApp.last_sensorid)//������ˣ���������һ�������ͼƬ
						{
							theApp.image_index = theApp.image_index - (theApp.image_index % IMAGESOFAREA);
						}
						theApp.image_index++;

						dirName = theApp.ImageDir.GetBuffer(256);

						if (SetDataExchange::radiobtngroup1 == 0 && theApp.image_index % IMAGESOFAREA == 1)
						{
							try {
								cv::Mat image1 = cv::Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult1->GetBuffer());
								int number = 0000;
								std::thread second(savethread1, number, pointindex, theApp.image_index, image1);
								second.join();
								//second.detach();
							}
							catch (...)
							{
							}

						}
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
					}
					else
					{
						AppendMessage(_T("Image can't get"), pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
					}

					//ImageProcess(ptrGrabResult1);

					if (theApp.image_index % IMAGESOFAREA == 0)//IMAGESOFAREA(20)��ͼ����Ϊһ�����򣬲ɼ�20��ͼƬ��ǰ��+1
					{
						theApp.last_point_ml++;
						robot[1].pointid++;

						theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
						theApp.MeasuredPointIndex[1]++;

						*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����
																		   //pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
																		   //last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																		   //if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																		   //{
																		   //	//theApp.counter_in_segment = 0;
																		   //	camera1.StopGrabbing();
																		   //	break;
																		   //}
					}

					theApp.camera_cap_total_counter[sensorid]++;
					theApp.last_sensorid = 1;


				}



			}
			//CPylonImageWindow win;
			//if (theApp.m_showcheck == 1)
			//{
			//	win.SetImage(ptrGrabResult1);
			//	
			//	win.Show();
			//}
			//else
			//{
			//	win.Hide();
			//}
		}
		Change_parameters();
		////ͳһ����ʼ��Ƭ�ţ���ͼģʽ��ע�ͣ����ģʽ�·ſ���
		//theApp.counter_in_segment = theApp.setdata_figurenum[robot[1].currenttrajid][sync_signal_counter];
		theApp.counter_in_segment = 1;
		//cout << "���öγ�ʼֵ��" << theApp.counter_in_segment << endl;
		//cout << "����ˢ��1����" << endl;
	}
}
void camera2caitu()
{
	while (1) {
		if (newcameranumbers < 2)
			break;
		camera2.RetrieveResult(INFINITE, ptrGrabResult2, TimeoutHandling_ThrowException);
		mut.lock();
		//cout << "�߳�2������" << endl;

		while (camera2.IsGrabbing())
		{
			//cout << "2�Ų�ͼ����" << endl;
			//cout << theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][2] << endl;
			//double framecs1 = camera2.AcquisitionFrameRate.GetValue();
			//double framecs2 = camera2.ResultingFrameRate.GetValue();
			//cout << "���֡��" << framecs1 << "    " << framecs2 << endl;
			//int offsetx2 = camera2.OffsetX.GetValue();
			//int offsety2 = camera2.OffsetY.GetValue();
			//cout << "���2ƫ��" << offsetx2 << "    " << offsety2 << endl;
			//cout << "sync_signal_counter:" << sync_signal_counter << endl;
			//int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
			//int segment_num = measurepoint[pointindex].segmentnum;
			////int sensorid = measurepoint[pointindex].sensorid;
			//int sensorid = 2;

			int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
			int last_s_id = measurepoint[last_pointindex].sensorid;
			int last_segment_num = measurepoint[last_pointindex].segmentnum;
			{
				bool b = camera2.LineStatus.GetValue();
				if (b == true)
				{
					//AppendMessage(_T("��ȡ��IO�ڵ͵�ƽ"), pEditCtrl);
					////theApp.counter_in_segment = 0;
					camera2.TriggerMode.SetValue(TriggerMode_On);
					//camera2.StopGrabbing();
					//camera2.StartGrabbing();
					sync_parameter = false;
					//cout << theApp.counter_in_segment << endl;
					//cout << theApp.camera_cap_total_counter[2] << endl;
					//Change_parameters();
					sync_signal_counter++;
					mut.unlock();
					//cout << "�߳�2������" << endl;
					break;
				}
			}
			//Sleep(10);
			//camera2.RetrieveResult(INFINITE, ptrGrabResult2, TimeoutHandling_ThrowException);
			unsigned char* imagebuffer = NULL;
			CString str, strTemp;
			int pointindex;
			int sensorid = 2;
			USES_CONVERSION;
			if (DETECTION_MODE == true)
			Comm.SetRobMeaACK(1, TRUE);
			//AppendMessage("onframe 2", pEditCtrl);
			pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
			if (theApp.mode == 1)//2DͿ��
			{
				//AppendMessage("2D", pEditCtrl);

				//�����߳���
				//mutex mut;
				//mut.lock();
				//cout << "�߳�2������" << endl;
				//theApp.counter_in_segment++;
				//cout << "chuxian" << endl;
				//mut.unlock();
				//str.Format(_T("2D: %d_%d"), pointindex, theApp.counter_in_segment);
				//AppendMessage(str, pEditCtrl);
				//if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml)
				if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml &&theApp.counter_in_segment_succed == measurepoint[pointindex].pointid_ml || DETECTION_MODE == false)
				{
					//cout << "pointindex = " << pointindex << endl;
					//cout << "theApp.counter_in_segment = " << theApp.counter_in_segment << endl;
					theApp.last_point_ml++;
					robot[1].pointid++;

					theApp.camera_cap_total_counter[sensorid]++;

					theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
					theApp.MeasuredPointIndex[1]++;

					theApp.image_save_index[theApp.current_image_index][0] = 1;//�����˺�
					//theApp.image_save_index[theApp.current_image_index][1] = measurepoint[pointindex].trajid;//�켣��
					theApp.image_save_index[theApp.current_image_index][2] = measurepoint[pointindex].pointid_ml;// ���
					theApp.image_save_index[theApp.current_image_index][1] = theApp.body_type_ml;//�켣��
					//theApp.image_save_index[theApp.current_image_index][2] = theApp.current_image_index;// ���
					theApp.image_save_index[theApp.current_image_index][3] = theApp.counter_in_segment;//���к�
					theApp.image_save_index[theApp.current_image_index][4] = 0;//����ǰͼ��
					theApp.image_save_index[theApp.current_image_index][5] = 0;//
					theApp.image_save_index[theApp.current_image_index][6] = theApp.current_image_index;//������
					theApp.image_save_index[theApp.current_image_index][7] = 0;//δ����
					theApp.image_save_index[theApp.current_image_index][8] = sensorid;///�����
					theApp.image_save_index[theApp.current_image_index][9] = sync_signal_counter;///�κ�

					//theApp.counter_in_segment++;
					theApp.counter_in_segment_succed++;
					//�ж�֡����Ч��
					if (!ptrGrabResult2.IsValid())
					{
						//theApp.image_save_index[theApp.current_image_index][5] = 0;//�ǻ�ͼ��
						AppendMessage("camera2: frame is invalid!\r\n", pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
						//	return;
					}
					else
					{
						theApp.image_save_index[theApp.current_image_index][5] = 1;//�Ǻ�ͼ��
																				   //MbufPut2d(theApp.image_color_bayer[theApp.current_image_index - 1], 0, 0, width, height, imagebuffer);
																				   //MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);
						imagebuffer = (uint8_t*)ptrGrabResult2->GetBuffer();
						//cv::Mat image222 = cv::Mat(ptrGrabResult2->GetHeight(), ptrGrabResult2->GetWidth(), CV_8U, imagebuffer);
						//cv::imwrite(cv::format("%d.jpg", theApp.counter_in_segment), image222);
						MbufPut2d(theApp.image_save[theApp.current_image_index], 0, 0, ptrGrabResult2->GetWidth(), ptrGrabResult2->GetHeight(), imagebuffer);
						//MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);

						//str.Format(_T("Camera[2]: index=%d, %d of %d,counter in segment %d: Buffer is Good!"), pointindex, theApp.current_image_index, robot[1].traj[robot[1].currenttrajid].pointsum, theApp.counter_in_segment);
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
						//AppendMessage(str, pEditCtrl);
					

					theApp.current_image_index++;

					//str.Format(_T("Camera[2]: %d, %d, %d, %d, %d. %d"), theApp.current_image_index, robot[1].pointid, theApp.camera_cap_total_counter[sensorid], theApp.camera_cap_g_counter[sensorid], theApp.camera_cap_ng_counter[sensorid], theApp.counter_in_segment);
					//AppendMessage(str, pEditCtrl);

					//if (robot[1].pointid <= robot[1].traj[robot[1].currenttrajid].pointsum)
					//{
					//	//PrepareForCapture(robot[1].currenttrajid, robot[1].pointid + 1);
					//}
					*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����
					//cout << "��Ч��Ϊ��" << theApp.last_point_ml << endl;

					pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
					last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																									 //if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																									 //{
																									 //	theApp.counter_in_segment = 0;
																									 //	//camera2.StopGrabbing();
																									 //	//break;
																									 //}
				}
				}
				//if (ptrGrabResult2.IsValid()) 
					camera2.RetrieveResult(INFINITE, ptrGrabResult2, TimeoutHandling_ThrowException);
				
				theApp.counter_in_segment++;
				//if ((robot[1].pointid >= robot[1].traj[robot[1].currenttrajid].pointsum) ||
				//	(measurepoint[pointindex].segmentnum != last_segment_num))///////�������˻��������						
			}


			if (theApp.mode == 2)//3DͿ��
			{
				//AppendMessage("3D", pEditCtrl);

				theApp.current_sensorid = 2;

				//theApp.counter_in_segment++;
				//int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
				//int segment_num = measurepoint[pointindex].segmentnum;
				//int sensorid = measurepoint[pointindex].sensorid;
				if (ptrGrabResult2->GrabSucceeded())
				{
					const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult2->GetBuffer();

					if (ptrGrabResult2.IsValid())
					{
						if (theApp.current_sensorid != theApp.last_sensorid)//������ˣ���������һ�������ͼƬ
						{
							theApp.image_index = theApp.image_index - (theApp.image_index % IMAGESOFAREA);
						}
						theApp.image_index++;

						dirName = theApp.ImageDir.GetBuffer(256);

						if (SetDataExchange::radiobtngroup1 == 0 && theApp.image_index % IMAGESOFAREA == 1)
						{
							try
							{
								cv::Mat image2 = cv::Mat(ptrGrabResult2->GetHeight(), ptrGrabResult2->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult2->GetBuffer());
								int number = 0000;
								std::thread third(savethread1, number, pointindex, theApp.image_index, image2);
								third.join();
								//third.detach();
							}
							catch (...)
							{
							}
						}
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
					}
					else
					{
						AppendMessage(_T("Image can't get"), pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
					}

					//ImageProcess(ptrGrabResult2);

					if (theApp.image_index % IMAGESOFAREA == 0)//IMAGESOFAREA(20)��ͼ����Ϊһ�����򣬲ɼ�20��ͼƬ��(�ɼ�����21��ͼʱ)��ǰ��+1
					{
						theApp.last_point_ml++;
						robot[1].pointid++;

						theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
						theApp.MeasuredPointIndex[1]++;

						*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����
																		   //pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
																		   //last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																		   //if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																		   //{
																		   //	//theApp.counter_in_segment = 0;
																		   //	camera2.StopGrabbing();
																		   //	break;
																		   //}
					}

					theApp.camera_cap_total_counter[sensorid]++;
					theApp.last_sensorid = 2;
				}

			}
			//CPylonImageWindow win;
			//if (theApp.m_showcheck == 1)
			//{
			//	win.SetImage(ptrGrabResult2);
			//	win.Show();
			//}
			//else
			//{
			//	win.Hide();
			//}
		}
		Change_parameters();
		////ͳһ����ʼ��Ƭ�ţ���ͼģʽ��ע�ͣ����ģʽ�·ſ���
		//theApp.counter_in_segment = theApp.setdata_figurenum[robot[1].currenttrajid][sync_signal_counter];
		theApp.counter_in_segment = 1;
		//cout << "���öγ�ʼֵ��" << theApp.counter_in_segment << endl;
		//cout << "����ˢ��2����" << endl;
	}
}
void camera3caitu()
{
	while (1) 
	{
		if (newcameranumbers < 3)
			break;
		camera3.RetrieveResult(INFINITE, ptrGrabResult3, TimeoutHandling_ThrowException);
		mut.lock();
		//cout << "�߳�3������" << endl;

		while (camera3.IsGrabbing())
		{
			//double framecs1 = camera3.AcquisitionFrameRate.GetValue();
			//double framecs2 = camera3.ResultingFrameRate.GetValue();
			//cout << "���֡��" << framecs1 << "    " << framecs2 << endl;
			//int offsetx3 = camera3.OffsetX.GetValue();
			//int offsety3 = camera3.OffsetY.GetValue();
			//cout << "���3ƫ��" << offsetx3 << "    " << offsety3 << endl;
			//cout << "sync_signal_counter:" << sync_signal_counter << endl;
			//int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
			//int segment_num = measurepoint[pointindex].segmentnum;
			////int sensorid = measurepoint[pointindex].sensorid;
			//int sensorid = 3;

			int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
			int last_s_id = measurepoint[last_pointindex].sensorid;
			int last_segment_num = measurepoint[last_pointindex].segmentnum;
			{
				bool b = camera3.LineStatus.GetValue();
				if (b == true)
				{
					//AppendMessage(_T("��ȡ��IO�ڵ͵�ƽ"), pEditCtrl);
					////theApp.counter_in_segment = 0;
					camera3.TriggerMode.SetValue(TriggerMode_On);
					//camera3.StopGrabbing();
					//camera3.StartGrabbing();
					sync_parameter = false;
					//Change_parameters();
					sync_signal_counter++;
					mut.unlock();
					//cout << "�߳�3������" << endl;
					break;
				}
			}
			//Sleep(10);
			//camera3.RetrieveResult(INFINITE, ptrGrabResult3, TimeoutHandling_ThrowException);
			unsigned char* imagebuffer = NULL;
			CString str, strTemp;
			int pointindex;
			int sensorid = 3;
			USES_CONVERSION;
			if (DETECTION_MODE == true)
			Comm.SetRobMeaACK(1, TRUE);
			pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];

			if (theApp.mode == 1)//2DͿ��
			{
				//AppendMessage("2D", pEditCtrl);

				//�����߳���
				//mutex mut;
				//mut.lock();
				//cout << "�߳�3������" << endl;
				//theApp.counter_in_segment++;
				//mut.unlock();
				//str.Format(_T("2D: %d_%d"), pointindex, theApp.counter_in_segment);
				//AppendMessage(str, pEditCtrl);
				//if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml)
				if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml &&theApp.counter_in_segment_succed == measurepoint[pointindex].pointid_ml || DETECTION_MODE == false)
				{
					//cout << "pointindex = " << pointindex << endl;
					//cout << "theApp.counter_in_segment = " << theApp.counter_in_segment << endl;
					theApp.last_point_ml++;
					robot[1].pointid++;

					theApp.camera_cap_total_counter[sensorid]++;

					theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
					theApp.MeasuredPointIndex[1]++;

					theApp.image_save_index[theApp.current_image_index][0] = 1;//�����˺�
					//theApp.image_save_index[theApp.current_image_index][1] = measurepoint[pointindex].trajid;//�켣��
					theApp.image_save_index[theApp.current_image_index][2] = measurepoint[pointindex].pointid_ml;// ���
					theApp.image_save_index[theApp.current_image_index][1] = theApp.body_type_ml;//�켣��
					//theApp.image_save_index[theApp.current_image_index][2] = theApp.current_image_index;// ���
					theApp.image_save_index[theApp.current_image_index][3] = theApp.counter_in_segment;//���к�
					theApp.image_save_index[theApp.current_image_index][4] = 0;//����ǰͼ��
					theApp.image_save_index[theApp.current_image_index][5] = 0;//
					theApp.image_save_index[theApp.current_image_index][6] = theApp.current_image_index;//������
					theApp.image_save_index[theApp.current_image_index][7] = 0;///δ����
					theApp.image_save_index[theApp.current_image_index][8] = sensorid;///�����
					theApp.image_save_index[theApp.current_image_index][9] = sync_signal_counter;///�κ�
					
					//theApp.counter_in_segment++;
					theApp.counter_in_segment_succed++;
					//�ж�֡����Ч��
					if (!ptrGrabResult3.IsValid())
					{
						//theApp.image_save_index[theApp.current_image_index][5] = 0;//�ǻ�ͼ��
						AppendMessage("camera3: frame is invalid!\r\n", pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
						//	return;
					}
					else
					{
						theApp.image_save_index[theApp.current_image_index][5] = 1;//�Ǻ�ͼ��
																				   //MbufPut2d(theApp.image_color_bayer[theApp.current_image_index - 1], 0, 0, width, height, imagebuffer);
																				   //MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);
						imagebuffer = (uint8_t*)ptrGrabResult3->GetBuffer();
						//cv::Mat image333 = cv::Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8U, imagebuffer);
						//cout << theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][1] << endl;
						//cv::imwrite(cv::format("%d.jpg", theApp.counter_in_segment), image333);	
						MbufPut2d(theApp.image_save[theApp.current_image_index], 0, 0, ptrGrabResult3->GetWidth(), ptrGrabResult3->GetHeight(), imagebuffer);
						//MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);

						//str.Format(_T("Camera[3]: index=%d, %d of %d,counter in segment %d: Buffer is Good!"), pointindex, theApp.current_image_index, robot[1].traj[robot[1].currenttrajid].pointsum, theApp.counter_in_segment);
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
						//AppendMessage(str, pEditCtrl);
					
					
					theApp.current_image_index++;

					//str.Format(_T("Camera[3]: %d, %d, %d, %d, %d. %d"), theApp.current_image_index, robot[1].pointid, theApp.camera_cap_total_counter[sensorid], theApp.camera_cap_g_counter[sensorid], theApp.camera_cap_ng_counter[sensorid], theApp.counter_in_segment);
					//AppendMessage(str, pEditCtrl);

					//if (robot[1].pointid <= robot[1].traj[robot[1].currenttrajid].pointsum)
					//{
					//	//PrepareForCapture(robot[1].currenttrajid, robot[1].pointid + 1);
					//}
					*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����
					//cout << "��Ч��Ϊ��" << theApp.last_point_ml << endl;

					pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
					last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																									 //if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																									 //{
																									 //	theApp.counter_in_segment = 0;
																									 //	//camera3.StopGrabbing();
																									 //	//break;
																									 //}
				}
				}
				//if (ptrGrabResult3.IsValid()) 
					camera3.RetrieveResult(INFINITE, ptrGrabResult3, TimeoutHandling_ThrowException);
				
				theApp.counter_in_segment++;
				//if ((robot[1].pointid >= robot[1].traj[robot[1].currenttrajid].pointsum) ||
				//	(measurepoint[pointindex].segmentnum != last_segment_num))///////�������˻��������

			}


			if (theApp.mode == 2)//3DͿ��
			{
				//AppendMessage("3D", pEditCtrl);

				theApp.current_sensorid = 3;

				// 						theApp.counter_in_segment++;
				// 						int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
				// 						int segment_num = measurepoint[pointindex].segmentnum;
				// 						int sensorid = measurepoint[pointindex].sensorid;
				if (ptrGrabResult3->GrabSucceeded())
				{
					const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult3->GetBuffer();

					if (ptrGrabResult3.IsValid())
					{
						if (theApp.current_sensorid != theApp.last_sensorid)//������ˣ���������һ�������ͼƬ
						{
							theApp.image_index = theApp.image_index - (theApp.image_index % IMAGESOFAREA);
						}
						theApp.image_index++;

						dirName = theApp.ImageDir.GetBuffer(256);

						if (SetDataExchange::radiobtngroup1 == 0 && theApp.image_index % IMAGESOFAREA == 1)
						{
							try
							{
								cv::Mat image3 = cv::Mat(ptrGrabResult3->GetHeight(), ptrGrabResult3->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult3->GetBuffer());
								int number = 0000;
								std::thread forth(savethread1, number, pointindex, theApp.image_index, image3);
								forth.join();
								//forth.detach();
							}
							catch (...)
							{
							}
						}
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
					}
					else
					{
						AppendMessage(_T("Image can't get"), pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
					}

					//ImageProcess(ptrGrabResult3);

					if (theApp.image_index % IMAGESOFAREA == 0)//IMAGESOFAREA(20)��ͼ����Ϊһ�����򣬲ɼ�20��ͼƬ��(�ɼ�����21��ͼʱ)��ǰ��+1
					{
						theApp.last_point_ml++;
						robot[1].pointid++;

						theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
						theApp.MeasuredPointIndex[1]++;

						*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����

																		   //pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
																		   //last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																		   //if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																		   //{
																		   //	//theApp.counter_in_segment = 0;
																		   //	camera3.StopGrabbing();
																		   //	break;
																		   //}

					}
					theApp.camera_cap_total_counter[sensorid]++;
					theApp.last_sensorid = 3;
				}

			}
			/*CPylonImageWindow win;
			if (theApp.m_showcheck == 1)
			{
			win.SetImage(ptrGrabResult3);
			win.Show();
			}
			else
			{
			win.Hide();
			}*/
		}
		Change_parameters();
		////ͳһ����ʼ��Ƭ�ţ���ͼģʽ��ע�ͣ����ģʽ�·ſ���
		//theApp.counter_in_segment = theApp.setdata_figurenum[robot[1].currenttrajid][sync_signal_counter];
		theApp.counter_in_segment = 1;
		//cout << "���öγ�ʼֵ��" << theApp.counter_in_segment << endl;
		//cout << "����ˢ��3����" << endl;
	}
}
void camera4caitu()
{
	while (1) {
		if (newcameranumbers < 4)
			break;
		camera4.RetrieveResult(INFINITE, ptrGrabResult4, TimeoutHandling_ThrowException);
		mut.lock();
		//cout << "�߳�4������" << endl;

		while (camera4.IsGrabbing())
		{
			//double framecs1 = camera4.AcquisitionFrameRate.GetValue();
			//double framecs2 = camera4.ResultingFrameRate.GetValue();
			//cout << "���֡��" << framecs1 << "    " << framecs2 << endl;
			//int offsetx4 = camera4.OffsetX.GetValue();
			//int offsety4 = camera4.OffsetY.GetValue();
			//cout << "���4ƫ��" << offsetx4 << "    " << offsety4 << endl;
			//cout << "sync_signal_counter:" << sync_signal_counter << endl;
			//int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
			//int segment_num = measurepoint[pointindex].segmentnum;
			////int sensorid = measurepoint[pointindex].sensorid;
			//int sensorid = 4;

			int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
			int last_s_id = measurepoint[last_pointindex].sensorid;
			int last_segment_num = measurepoint[last_pointindex].segmentnum;
			{
				bool b = camera4.LineStatus.GetValue();
				if (b == true)
				{
					//AppendMessage(_T("��ȡ��IO�ڵ͵�ƽ"), pEditCtrl);
					////theApp.counter_in_segment = 0;
					camera4.TriggerMode.SetValue(TriggerMode_On);
					//camera4.StopGrabbing();
					//camera4.StartGrabbing();
					sync_parameter = false;
					//Change_parameters();
					sync_signal_counter++;
					mut.unlock();
					//cout << "�߳�4������" << endl;
					break;
				}
			}
			//Sleep(10);
			//camera4.RetrieveResult(INFINITE, ptrGrabResult4, TimeoutHandling_ThrowException);
			unsigned char* imagebuffer = NULL;
			CString str, strTemp;
			int pointindex;
			int sensorid = 4;
			USES_CONVERSION;
			if (DETECTION_MODE == true)
			Comm.SetRobMeaACK(1, TRUE);
			//AppendMessage("onframe 4", pEditCtrl);
			pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
			if (theApp.mode == 1)//2DͿ��
			{
				//AppendMessage("2D", pEditCtrl);

				//�����߳���
				//mutex mut;
				//mut.lock();
				//cout << "�߳�4������" << endl;
				//theApp.counter_in_segment++;
				//mut.unlock();
				//str.Format(_T("2D: %d_%d"), pointindex, theApp.counter_in_segment);
				//AppendMessage(str, pEditCtrl);
				//pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
				//if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml)
				if (theApp.counter_in_segment == measurepoint[pointindex].m_index_in_segment && measurepoint[pointindex].trajid == theApp.body_type_ml &&theApp.counter_in_segment_succed == measurepoint[pointindex].pointid_ml || DETECTION_MODE == false)
				{
					//cout << "pointindex = " << pointindex << endl;
					//cout << "theApp.counter_in_segment = " << theApp.counter_in_segment << endl;
					theApp.last_point_ml++;
					robot[1].pointid++;

					theApp.camera_cap_total_counter[sensorid]++;

					theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
					theApp.MeasuredPointIndex[1]++;

					theApp.image_save_index[theApp.current_image_index][0] = 1;//�����˺�
					//theApp.image_save_index[theApp.current_image_index][1] = measurepoint[pointindex].trajid;//�켣��
					theApp.image_save_index[theApp.current_image_index][2] = measurepoint[pointindex].pointid_ml;// ���
					theApp.image_save_index[theApp.current_image_index][1] = theApp.body_type_ml;//�켣��
					//theApp.image_save_index[theApp.current_image_index][2] = theApp.current_image_index;// ���
					theApp.image_save_index[theApp.current_image_index][3] = theApp.counter_in_segment;//���к�
					theApp.image_save_index[theApp.current_image_index][4] = 0;//����ǰͼ��
					theApp.image_save_index[theApp.current_image_index][5] = 0;//
					theApp.image_save_index[theApp.current_image_index][6] = theApp.current_image_index;//������
					theApp.image_save_index[theApp.current_image_index][7] = 0;///δ����
					theApp.image_save_index[theApp.current_image_index][8] = sensorid;///�����
					theApp.image_save_index[theApp.current_image_index][9] = sync_signal_counter;///�κ�
					
					//theApp.counter_in_segment++;
					theApp.counter_in_segment_succed++;
					//�ж�֡����Ч��
					if (!ptrGrabResult4.IsValid())
					{
						//theApp.image_save_index[theApp.current_image_index][5] = 0;//�ǻ�ͼ��
						AppendMessage("camera4: frame is invalid!\r\n", pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
						//	return;
					}
					else
					{
						theApp.image_save_index[theApp.current_image_index][5] = 1;//�Ǻ�ͼ��
																				   //MbufPut2d(theApp.image_color_bayer[theApp.current_image_index - 1], 0, 0, width, height, imagebuffer);
																				   //MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);
						imagebuffer = (uint8_t*)ptrGrabResult4->GetBuffer();
						MbufPut2d(theApp.image_save[theApp.current_image_index], 0, 0, ptrGrabResult4->GetWidth(), ptrGrabResult4->GetHeight(), imagebuffer);
						//MbufBayer(theApp.image_color_bayer[theApp.current_image_index -1], theApp.image_save[theApp.current_image_index], M_DEFAULT, M_BAYER_RG);

						//str.Format(_T("Camera[4]: index=%d, %d of %d,counter in segment %d: Buffer is Good!"), pointindex, theApp.current_image_index, robot[1].traj[robot[1].currenttrajid].pointsum, theApp.counter_in_segment);
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
						//AppendMessage(str, pEditCtrl);
					
					
					theApp.current_image_index++;
					//Pylon::DisplayImage(1, ptrGrabResult);
					//str.Format(_T("Camera[4]: %d, %d, %d, %d, %d. %d"), theApp.current_image_index, robot[1].pointid, theApp.camera_cap_total_counter[sensorid], theApp.camera_cap_g_counter[sensorid], theApp.camera_cap_ng_counter[sensorid], theApp.counter_in_segment);
					//AppendMessage(str, pEditCtrl);

					//if (robot[1].pointid <= robot[1].traj[robot[1].currenttrajid].pointsum)
					//{
					//	//PrepareForCapture(robot[1].currenttrajid, robot[1].pointid + 1);
					//}
					*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����
					//cout << "��Ч��Ϊ��" << theApp.last_point_ml << endl;

					pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
					last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																									 //if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																									 //{
																									 //	theApp.counter_in_segment = 0;
																									 //	//camera4.StopGrabbing();
																									 //	//break;
																									 //}
				}
				}
				//if (ptrGrabResult4.IsValid()) 
					camera4.RetrieveResult(INFINITE, ptrGrabResult4, TimeoutHandling_ThrowException);
				
				theApp.counter_in_segment++;
				//if ((robot[1].pointid >= robot[1].traj[robot[1].currenttrajid].pointsum) ||
				//	(measurepoint[pointindex].segmentnum != last_segment_num))///////�������˻��������

			}


			if (theApp.mode == 2)//3DͿ��
			{
				//AppendMessage("3D", pEditCtrl);

				theApp.current_sensorid = 4;

				//theApp.counter_in_segment++;
				int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
				int segment_num = measurepoint[pointindex].segmentnum;
				int sensorid = measurepoint[pointindex].sensorid;
				if (ptrGrabResult4->GrabSucceeded())
				{
					const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult4->GetBuffer();

					if (ptrGrabResult4.IsValid())
					{
						if (theApp.current_sensorid != theApp.last_sensorid)//������ˣ���������һ�������ͼƬ
						{
							theApp.image_index = theApp.image_index - (theApp.image_index % IMAGESOFAREA);
						}
						theApp.image_index++;

						dirName = theApp.ImageDir.GetBuffer(256);

						if (SetDataExchange::radiobtngroup1 == 0 && theApp.image_index % IMAGESOFAREA == 1)
						{
							try
							{
								cv::Mat image4 = cv::Mat(ptrGrabResult4->GetHeight(), ptrGrabResult4->GetWidth(), CV_8U, (uint8_t*)ptrGrabResult4->GetBuffer());
								int number = 0000;
								std::thread fifth(savethread1, number, pointindex, theApp.image_index, image4);
								fifth.join();
								//fifth.detach();
							}
							catch (...)
							{
							}
						}
						theApp.camera_cap_g_counter[sensorid]++;
						measurepoint[pointindex].measured = true;
						measurepoint[pointindex].image_cap_ng = false;
					}
					else
					{
						AppendMessage(_T("Image can't get"), pEditCtrl);
						theApp.camera_cap_ng_counter[sensorid]++;
					}

					//ImageProcess(ptrGrabResult4);

					if (theApp.image_index % IMAGESOFAREA == 0)//IMAGESOFAREA(20)��ͼ����Ϊһ�����򣬲ɼ�20��ͼƬ��(�ɼ�����21��ͼʱ)��ǰ��+1
					{
						theApp.last_point_ml++;
						robot[1].pointid++;

						theApp.MeasuredPoint[1][theApp.MeasuredPointIndex[1]] = pointindex;
						theApp.MeasuredPointIndex[1]++;

						*(Comm.LPDataResult + 55) = theApp.last_point_ml;  //�Ѳ�����ŷ�����ѯ����
																		   //pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
																		   //last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
																		   //if (measurepoint[pointindex].segmentnum != measurepoint[last_pointindex].segmentnum)///////�������˻��������
																		   //{
																		   //	//theApp.counter_in_segment = 0;
																		   //	camera4.StopGrabbing();
																		   //	break;
																		   //}
					}

					theApp.camera_cap_total_counter[sensorid]++;
					theApp.last_sensorid = 4;

				}

			}
			/*CPylonImageWindow win;
			if (theApp.m_showcheck == 1)
			{
			win.SetImage(ptrGrabResult4);
			win.Show();
			}
			else
			{
			win.Hide();
			}*/
		}
		Change_parameters();
		////ͳһ����ʼ��Ƭ�ţ���ͼģʽ��ע�ͣ����ģʽ�·ſ���
		//theApp.counter_in_segment = theApp.setdata_figurenum[robot[1].currenttrajid][sync_signal_counter];
		theApp.counter_in_segment = 1;
		//cout << "���öγ�ʼֵ��" << theApp.counter_in_segment << endl;
		//cout << "����ˢ��4����" << endl;
	}
}

void Change_parameters()
{
	ofstream count_figure_number("��ͼģʽ��¼��.txt", ofstream::app);//��¼��ͼ�Ķκź�ͼƬ��
	CString str;
	if (theApp.setdata_parameter[robot[1].currenttrajid][sync_signal_counter] == -1 && sync_parameter == false)
	{
		if (theApp.mode == 1)
		{
			//cout << sync_signal_counter << endl;
			//����Ϊ2DͿ�����Ĳ���
			if(theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 1)
			{
				//cout << "camera1���ò���" << sync_signal_counter << endl;
				//camera1.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][1]);
				//camera1.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][1]);
				camera1.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][1]);//390
				camera1.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][1]);//178
				camera1.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][1]);
				camera1.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][1]);
				//camera1.ExposureTime.SetValue(39240);
				camera1.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][1]);
				//cout << theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][1] << endl;
				//camera1.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][1]);
				string number = camera1.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("ˢ�²��� device1: %s"), number);
				AppendMessage(str, pEditCtrl);
			}

			if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 2)
			{
				//cout << "camera2���ò���" << sync_signal_counter << endl;
				//cout << "camera2���ò���" << theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][2] << endl;
				//cout << "camera2���ò���" << theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][2] << endl;
				//camera2.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][2]);
				//camera2.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][2]);
				camera2.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][2]);//390
				camera2.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][2]);//178
				camera2.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][2]);
				camera2.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][2]);
				//camera1.ExposureTime.SetValue(39240);
				camera2.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][2]);
				//camera2.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][2]);
				//camera2.OverlapMode.SetValue(OverlapMode_On);
				//camera2.BslImmediateTriggerMode.SetValue(BslImmediateTriggerMode_On);
				//camera2.LineDebouncerTime.SetValue(10);
				string number = camera2.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("ˢ�²��� device2: %s"), number);
				AppendMessage(str, pEditCtrl);
			}

			if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 3)
			{
				//cout << "camera3���ò���" << sync_signal_counter << endl;
				//camera3.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][3]);
				//camera3.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][3]);
				camera3.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][3]);//390
				camera3.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][3]);//178
				camera3.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][3]);
				camera3.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][3]);
				//camera1.ExposureTime.SetValue(39240);
				camera3.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][3]);
				//camera3.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][3]);
				string number = camera3.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("ˢ�²��� device3: %s"), number);
				AppendMessage(str, pEditCtrl);
			}
			if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 4)
			{
				//cout << "camera4���ò���" << sync_signal_counter << endl;
				//cout << "camera4���ò���" << theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][4] << endl;
				//camera4.Width.SetValue(theApp.setdata_image_longth[robot[1].currenttrajid][sync_signal_counter][4]);
				//camera4.Height.SetValue(theApp.setdata_image_width[robot[1].currenttrajid][sync_signal_counter][4]);
				camera4.OffsetX.SetValue(theApp.setdata_shift_x[robot[1].currenttrajid][sync_signal_counter][4]);//390
				camera4.OffsetY.SetValue(theApp.setdata_shift_y[robot[1].currenttrajid][sync_signal_counter][4]);//178
				camera4.Gain.SetValue(theApp.setdata_gain[robot[1].currenttrajid][sync_signal_counter][4]);
				camera4.Gamma.SetValue(theApp.setdata_gamma[robot[1].currenttrajid][sync_signal_counter][4]);
				//camera1.ExposureTime.SetValue(39240);
				//camera4.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][4]);
				camera4.ExposureTime.SetValue(theApp.setdata_exposure_time[robot[1].currenttrajid][sync_signal_counter][4]);

				//camera4.AcquisitionFrameRate.SetValue(theApp.setdata_frame_rate[robot[1].currenttrajid][sync_signal_counter][4]);
				string number = camera4.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("ˢ�²��� device4: %s"), number);
				AppendMessage(str, pEditCtrl);
			}
		}
		if (theApp.mode == 2)
		{
			//����Ϊ3DͿ�����Ĳ���
			if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 1)
			{
				camera1.Width.SetValue(150);
				camera1.Height.SetValue(200);
				camera1.OffsetX.SetValue(560);
				camera1.OffsetY.SetValue(224);
				camera1.ExposureTime.SetValue(150);
				camera1.Gain.SetValue(0);
				camera1.Gamma.SetValue(1.0);
				camera1.AcquisitionFrameRate.SetValue(150);
				string number = camera1.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("Using device1: %s"), number);
				AppendMessage(str, pEditCtrl);
			}
			if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 2)
			{
				camera2.Width.SetValue(150);
				camera2.Height.SetValue(200);
				camera2.OffsetX.SetValue(582);
				camera2.OffsetY.SetValue(250);
				camera2.ExposureTime.SetValue(150);
				camera2.Gain.SetValue(0);
				camera2.Gamma.SetValue(1.0);
				camera2.AcquisitionFrameRate.SetValue(150);
				string number = camera2.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("Using device2: %s"), number);
				AppendMessage(str, pEditCtrl);
			}
			if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 3)
			{
				camera3.Width.SetValue(150);
				camera3.Height.SetValue(200);
				camera3.OffsetX.SetValue(560);
				camera3.OffsetY.SetValue(262);
				camera3.ExposureTime.SetValue(150);
				camera3.Gain.SetValue(0);
				camera3.Gamma.SetValue(1.0);
				camera3.AcquisitionFrameRate.SetValue(150);
				string number = camera3.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("Using device3: %s"), number);
				AppendMessage(str, pEditCtrl);
			}
			if (theApp.setdata_Sensor[robot[1].currenttrajid][sync_signal_counter] == 4)
			{
				camera4.Width.SetValue(150);
				camera4.Height.SetValue(200);
				camera4.OffsetX.SetValue(524);
				camera4.OffsetY.SetValue(246);
				camera4.ExposureTime.SetValue(150);
				camera4.Gain.SetValue(0);
				camera4.Gamma.SetValue(1.0);
				camera4.AcquisitionFrameRate.SetValue(150);
				string number = camera4.GetDeviceInfo().GetSerialNumber();
				str.Format(_T("Using device4: %s"), number);
				AppendMessage(str, pEditCtrl);
			}
		}
		sync_parameter = false;
		CString str;
		if (sync_signal_counter == 1)
		{	
			str.Format(_T("�κ�: %d������ʼͼƬΪ�ڣ�%d ��"), sync_signal_counter, 1);
		}
		else
		{
			str.Format(_T("�κ�: %d������ʼͼƬΪ�ڣ�%d ��"), sync_signal_counter, theApp.counter_in_segment);
			//str.Format(_T("�κ�: %d������ʼͼƬΪ�ڣ�%d ��"), sync_signal_counter, theApp.camera_cap_total_counter[1] + 1);
			
			AppendMessage(str, pEditCtrl);
			count_figure_number << theApp.counter_in_segment;
			count_figure_number << "   ";
		}
	}
}