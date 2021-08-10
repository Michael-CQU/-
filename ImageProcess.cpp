#include"ImageProcess"

void ImageProcess(CBaslerUniversalGrabResultPtr &ptrGrabResult)
{
	CString str;
	// 	if (robot[1].currenttrajid == 0)
	// 	{
	// 		robot[1].currenttrajid = 1;
	// 	}
	int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid + 1];
	int segment_num = measurepoint[pointindex].segmentnum;
	int sensorid = measurepoint[pointindex].sensorid;

	int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//��һ��������
	int last_s_id = measurepoint[last_pointindex].sensorid;
	int last_segment_num = measurepoint[last_pointindex].segmentnum;

	cv::Mat OnGetFrame_image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U,
		(uint8_t*)ptrGrabResult->GetBuffer());
	if (OnGetFrame_image.empty())
	{
		std::cout << "δ����ͼƬ��ͼƬΪ��" << std::endl;
	}

	//if (color_of_glue==0)//�ڽ�
	if (SetDataExchange::gluecolor == 1)//�ڽ�
	{
		Mat OnGetFrame_image0;

		///////////�ڽ�ͼ��Ԥ����
		blur(OnGetFrame_image, OnGetFrame_image0, Size(5, 5));//��ֵ�˲�
		cv::threshold(OnGetFrame_image0, OnGetFrame_image0, 150, 255, CV_THRESH_BINARY);
		//��ȡ�Զ����
		cv::Mat element0 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(18, 18));
		//cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
		morphologyEx(OnGetFrame_image0, OnGetFrame_image0, cv::MORPH_CLOSE, element0);
		//morphologyEx(OnGetFrame_image0, OnGetFrame_image0, cv::MORPH_ERODE, element1);
		//MORPH_OPEN�����㡢MORPH_CLOSE�����㡢MORPH_ERODE��ʴ��MORPH_DILATE���ͣ��ڵװ����ñ����������

		//��ͼƬ˳ʱ����ת90�ȣ�ͼ���м�������ˮƽ��
		cv::Mat OnGetFrame_image1;
		transpose(OnGetFrame_image0, OnGetFrame_image1);
		flip(OnGetFrame_image1, OnGetFrame_image1, 1);

		// 	//ͼ���м���������ֱ,������ת
		// 	cv::Mat OnGetFrame_image1 = OnGetFrame_image0;

		int row = OnGetFrame_image1.rows, col = OnGetFrame_image1.cols;//ͼ����������
		float width = 0;//����
		double shift_maxHeight;//��ߵ��ƫ����
		double shift_coordinate;//�յ��ƫ����

		std::vector<cv::Point2i> loction;//�洢����������
		std::vector<float>Distance; //�洢͹�����굽ֱ�߾��루�߶ȣ�
		std::vector<cv::Point2i> coordinate;//�洢�յ�����

		//����ͼ��
		for (int i = 0; i < row; i++)
		{
			// �趨ͼ������Դָ�뼰���ͼ������ָ��
			uchar *pSrcData = OnGetFrame_image1.ptr<uchar>(i);
			for (int j = 0; j < col; j++)
			{
				if (pSrcData[j] == 255)
				{
					//loction.push_back(cv::Point2i(j, i));
					loction.push_back(cv::Point2i(i, col - j));
					//loction.push_back(cv::Point2i(i, col - (j + (int)lineWide / 2)-1));
					break;
				}
			}
		}

		if (loction.size() != 0)
		{
			//��ȡ�����ڵ�ľ���
			for (int i = 0; i < loction.size() - 1; i++)
			{
				float d;
				d = sqrt(pow((loction[i + 1].x - loction[i].x), 2) + pow((loction[i + 1].y - loction[i].y), 2));//�����ڵ�ľ���
				Distance.push_back(d);
			}
		}

		for (int i = 0; i < Distance.size(); i++)
		{
			if (Distance[i] > theApp.setdata_height[robot[1].currenttrajid][segment_num][sensorid] + 10)
			{
				coordinate.push_back(cv::Point2i(loction[i].x, loction[i].y));
				coordinate.push_back(cv::Point2i(loction[i + 1].x, loction[i + 1].y));
				break;
			}
		}


		auto maxDistance = std::max_element(Distance.begin(), Distance.end());//���������ֵ�������
		int maxPosition = maxDistance - Distance.begin();//���������ֵ��λ��

		//��������pointindex�Ƿ����1����robot[1].pointid=0��pointindex=1ʱ��last_s_id��last_segment_numΪ0��Ҳ����뵽if�У��ͳ�ʼ��һ��
		//if ((measurepoint[pointindex].sensorid != last_s_id) || (measurepoint[pointindex].segmentnum != last_segment_num))///////��������߻�����
		if (theApp.current_sensorid != theApp.last_sensorid)///////�������
		{
			first_exist_flag = 0;
		}
		//�ж�λ���Ƿ���ƫ�ƣ���¼�����ص�x����---ͼ���м�������ˮƽ��
		if (coordinate.size() == 2 && first_exist_flag == 0)//���������յ��һ����ֵ��ʱ��
		{
			first_exist_flag = 1;//�����趨һ������ʹ��first_exist_flag������0����
			first_x = coordinate[0].x;
		}

		int area_image_index = theApp.image_index % IMAGESOFAREA;//1ÿ20��Ϊһ��������һ�������ڵı��1~20
		if (area_image_index == 0)
		{
			area_image_index = IMAGESOFAREA;
		}

		switch (coordinate.size())
		{
		case 0:
			//�Ͻ�
			if (Distance.size() != 0 && loction.size() != 0)
			{
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\�Ͻ�%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x08;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				theApp.ImageProcessResult[area_image_index - 1] = 0x08;//�Ͻ�  //�ڼ�������ĵڼ�֡��0-19��
			}
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x40;//������
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\error%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x40;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
			}
			break;

		case 2:
			std::cout << "[Width of glue is]��" << *maxDistance << "  ����" << std::endl;

			//ƫ��
			shift_coordinate = coordinate[0].x - first_x;
			if (shift_coordinate >= theApp.setdata_shift[robot[1].currenttrajid][segment_num][sensorid])
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x10;//λ��ƫ��
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\Ϳ��ƫ��%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x10;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			//̫��
			if (*maxDistance > theApp.setdata_toowide_width[robot[1].currenttrajid][segment_num][sensorid])
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x04;
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\Ϳ������%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x04;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			//̫խ
			if (*maxDistance < theApp.setdata_toonarrow_width[robot[1].currenttrajid][segment_num][sensorid])
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x02;
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\Ϳ����խ%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x02;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			//Ϳ���ϸ�
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x80;//Ϳ���������ϸ�
			}
			break;

		default:
			theApp.ImageProcessResult[area_image_index - 1] = 0x40;//������û������
			std::cout << "error" << std::endl;
			if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
			{
				//imwrite(dirName + "_ng" + cv::format("\\error%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
				int number = 0x20;
				std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
				sixth.join();
				//second.detach();
			}
			break;
		}

		//���������
		if (area_image_index == IMAGESOFAREA)//ÿһ���������һ��ͼƬ�����������
		{
			//������������ʵ����ֻ�����һ�����������
			int maxCount = 0, Mode_index = 0, nCount = 0;
			for (int i = 0; i < IMAGESOFAREA; i++)
			{
				for (int j = 0; j < IMAGESOFAREA; j++)
				{
					if (theApp.ImageProcessResult[j] == theApp.ImageProcessResult[i])
						nCount++;
				}
				if (nCount > maxCount)
				{
					maxCount = nCount;
					Mode_index = i;
				}
				nCount = 0;
			}
			measurepoint[pointindex].MeasureResult = 0x00;

			switch (theApp.ImageProcessResult[Mode_index])//������ֵ
			{
				//�Ͻ����
			case 0x08:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])//�������ֵĸ��������ڵ���nʱ����Ϊ������Ͻ�
				{
					measurepoint[pointindex].MeasureResult |= 0x08;
					str.Format(_T("�� %d ������Ͻ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);//�Ͻ�����Ƶ����ߣ���Ҳ���٣�������Ϊ�Ǻϸ��
					AppendMessage(str, pEditCtrl);						
				}
				break;

				//ƫ�����
			case 0x10:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x10;
					str.Format(_T("�� %d ������Ϳ��λ��ƫ��"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x04://̫��
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x04;
					str.Format(_T("�� %d ������Ϳ������"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x02://̫խ
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x02;
					str.Format(_T("�� %d ������Ϳ����խ"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x80://�ϸ�
				measurepoint[pointindex].MeasureResult |= 0x80;
				str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x20://Ϳ������δ��ȫ
				measurepoint[pointindex].MeasureResult |= 0x20;
				str.Format(_T("�� %d ������Ϳ������δ��ȫ"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x40://������
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("�� %d �����������1"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			default:
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("�� %d �����������2"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;
			}
		}
		*(Comm.LPDataResult + 60 + measurepoint[pointindex].pointid_ml) = measurepoint[pointindex].MeasureResult;
	}

	//cout << "���" << Comm.GetRobPointID(1) << endl;
	//cout << "�켣��" << Comm.GetRobTrajID(1) << endl;

	//if (color_of_glue == 1)
	if (SetDataExchange::gluecolor == 0)//�׽�
	{
		Mat OnGetFrame_image0;

		////�׽�ͼ��Ԥ����
		blur(OnGetFrame_image, OnGetFrame_image0, Size(5, 5));//��ֵ�˲�
		cv::threshold(OnGetFrame_image0, OnGetFrame_image0, 180, 255, CV_THRESH_BINARY);
		//��ȡ�Զ����
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		morphologyEx(OnGetFrame_image0, OnGetFrame_image0, cv::MORPH_DILATE, element);

		//��ͼƬ˳ʱ����ת90�ȣ�ͼ���м�������ˮƽ��
		cv::Mat OnGetFrame_image1;
		transpose(OnGetFrame_image0, OnGetFrame_image1);
		flip(OnGetFrame_image1, OnGetFrame_image1, 1);

		// 	//ͼ���м���������ֱ,������ת
		// 	cv::Mat OnGetFrame_image1 = OnGetFrame_image0;

		int row = OnGetFrame_image1.rows, col = OnGetFrame_image1.cols;//ͼ����������
		int n_left = 0, n_right = 0;//��ߺ��ұ߳��ֹյ�ʱ��loction�ж�Ӧ����������
		float d_sum = 0;//͹�����굽ֱ�߾���֮�ͣ��������
		float width = 0;//����
		double shift_maxHeight;//��ߵ��ƫ����
		double shift_coordinate;//�յ��ƫ����

		std::vector<cv::Point2i> loction;//�洢����������
		std::vector<float>Distance; //�洢͹�����굽ֱ�߾��루�߶ȣ�
		std::vector<cv::Point2i> coordinate;//�洢�յ�����

		//����ͼ��
		for (int i = 0; i < row; i++)
		{
			// �趨ͼ������Դָ�뼰���ͼ������ָ��
			uchar *pSrcData = OnGetFrame_image1.ptr<uchar>(i);
			for (int j = 0; j < col; j++)
			{
				if (pSrcData[j] == 255)
				{
					//loction.push_back(cv::Point2i(j, i));
					loction.push_back(cv::Point2i(i, col - j));
					//loction.push_back(cv::Point2i((j + (int)lineWide / 2), i));//ͼ���м���������ֱ��
					//loction.push_back(cv::Point2i(i, col - (j + (int)lineWide / 2)));//ͼ���м�������ˮƽ��
					break;
				}
			}
		}

		//��ȡ���е㵽ֱ�߾���
		for (int i = 0; i < loction.size(); i++)
		{
			float d;
			d = sqrt((pow(((loction[i].x - loction[0].x)* (loction[loction.size() - 1].y - loction[0].y) +
				(loction[i].y - loction[0].y)* (loction[0].x - loction[loction.size() - 1].x)), 2)) /
				(pow((loction[loction.size() - 1].y - loction[0].y), 2) + pow((loction[0].x - loction[loction.size() - 1].x), 2)));
			Distance.push_back(d);
		}	

		//���������ж��Ƿ��йյ���֣����У���¼�յ�����
		for (int i = 0; i < Distance.size(); i++)
		{
			if (Distance[i] >= theApp.setdata_height[robot[1].currenttrajid][segment_num][sensorid]*2.934
				&& Distance[i + 1] >= theApp.setdata_height[robot[1].currenttrajid][segment_num][sensorid]*2.934
				&& Distance[i + 2] >= theApp.setdata_height[robot[1].currenttrajid][segment_num][sensorid]*2.934)
			{
				coordinate.push_back(cv::Point2i(loction[i].x, loction[i].y));
				n_left = i;
				break;
			}
		}
		//���������ж��Ƿ��йյ���֣����У���¼�յ�����
		for (int n = (Distance.size() - 1); n >= 0; n--)
		{
			if (Distance[n] >= theApp.setdata_height[robot[1].currenttrajid][segment_num][sensorid] * 2.934
				&& Distance[n - 1] >= theApp.setdata_height[robot[1].currenttrajid][segment_num][sensorid] * 2.934
				&& Distance[n - 2] >= theApp.setdata_height[robot[1].currenttrajid][segment_num][sensorid] * 2.934)
			{
				coordinate.push_back(cv::Point2i(loction[n].x, loction[n].y));
				n_right = n;
				break;
			}
		}

		//��߶�֮�ͣ������
		for (int i = n_left; i < n_right; i++)
		{
			d_sum += Distance[i];
		}
		auto maxHeight = std::max_element(Distance.begin(), Distance.end());//��߶ȵ����ֵ�����߶�
		int maxPosition = maxHeight - Distance.begin();//��߶ȵ����ֵ��λ��

		//��������pointindex�Ƿ����1����robot[1].pointid=0��pointindex=1ʱ��last_s_id��last_segment_numΪ0��Ҳ����뵽if�У��ͳ�ʼ��һ��
		//if ((measurepoint[pointindex].sensorid != last_s_id) || (measurepoint[pointindex].segmentnum != last_segment_num))///////��������߻�����
		if (theApp.current_sensorid != theApp.last_sensorid)///////�������
		{
			first_exist_flag = 0;
		}
		//�ж�λ���Ƿ���ƫ�ƣ���¼�����ص�x����---ͼ���м�������ˮƽ��
		if (coordinate.size() == 2 && first_exist_flag == 0)//���������յ��һ����ֵ��ʱ��
		{
			first_x = loction[n_left].x;
			first_exist_flag = 1;//�����趨һ������ʹ��first_exist_flag������0
		}


		//cout << "̫խ��׼"<<theApp.setdata_toonarrow_width[robot[1].currenttrajid][segment_num][sensorid] << endl;
		//cout << "̫���׼" << theApp.setdata_toowide_width[robot[1].currenttrajid][segment_num][sensorid] << endl;
		//cout<<"cuntu"<< theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] << endl;
		//cout << "�켣��ֵ " << robot[1].currenttrajid << endl;
		//cout << "�ε�ֵ " << segment_num << endl;
		//cout << "�����ֵ " << sensorid << endl;

		int area_image_index = theApp.image_index % IMAGESOFAREA;//1ÿ20��Ϊһ��������һ�������ڵı��1~20
		if (area_image_index == 0)
		{
			area_image_index = IMAGESOFAREA;
		}

		//�ж��Ƿ��йյ㣨�İ�������
		switch (coordinate.size())
		{
		case 0:
			if (Distance.size() != 0 && loction.size() != 0)
			{
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\�Ͻ�%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x08;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
				theApp.ImageProcessResult[area_image_index - 1] = 0x08;//�Ͻ�  //�ڼ�������ĵڼ�֡��0-19��
			}
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x40;//������
				 //std::cout << "�ɼ�ͼ����û�м�����" << std::endl;
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\error%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x40;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
			}
			break;

		case 1:
			if (n_left >= 5 && n_right >= 5)//�޶���������ֱ�ߵ������
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x20;//Ϳ������δ��ȫ
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\Ϳ������δ��ȫ%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x20;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
			}
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x40;//������
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\error%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x40;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
			}
			break;

		case 2:
		{
			width = sqrt(pow((coordinate[1].x - coordinate[0].x), 2) + pow((coordinate[1].y - coordinate[0].y), 2));//�����յ�֮����룬�����
			// 			std::cout << "[Area of glue is]��" << d_sum << "  ����" << std::endl;
			// 			std::cout << "[Width of glue is]��" << width << "  ����" << std::endl;
			// 			std::cout << "[Height of glue is]��" << *maxHeight << "  ����" << std::endl;

			//ƫ�����
			//shift_maxHeight = loction[maxPosition].x - first_x;//���߶ȵ�ƫ������ֻ����x�����ƫ�ƣ�---������ˮƽ
			//shift_maxHeight = loction[maxPosition].y - first_y;//���߶ȵ�ƫ������ֻ����x�����ƫ�ƣ�---��������ֱ
			//shift_coordinate = loction[n_left].y - first_y;
            shift_coordinate = loction[n_left].x - first_x;
			if (shift_coordinate >= theApp.setdata_shift[robot[1].currenttrajid][segment_num][sensorid]*6.229)
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x10;//λ��ƫ��
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\Ϳ��ƫ��%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x10;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
				break;
			}

			//̫��
			if (width > theApp.setdata_toowide_width[robot[1].currenttrajid][segment_num][sensorid]*6.229
				|| d_sum > theApp.setdata_toowide_area[robot[1].currenttrajid][segment_num][sensorid]*20.977
				|| *maxHeight > theApp.setdata_toowide_height[robot[1].currenttrajid][segment_num][sensorid]*2.934)
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x04;//̫��
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\Ϳ������%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x04;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//sixth.detach();
				}
				break;
			}

			//̫խ
			if (width < theApp.setdata_toonarrow_width[robot[1].currenttrajid][segment_num][sensorid]*6.229
				|| d_sum < theApp.setdata_toonarrow_area[robot[1].currenttrajid][segment_num][sensorid]*20.977
				|| *maxHeight < theApp.setdata_toonarrow_height[robot[1].currenttrajid][segment_num][sensorid]*2.934)
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x02;//̫խ
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\Ϳ����խ%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x02;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x80;//Ϳ���������ϸ�
			}

		}
		break;

		default:
			theApp.ImageProcessResult[area_image_index - 1] = 0x40;//������û������
			if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
			{
				//imwrite(dirName + "_ng" + cv::format("\\error%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
				int number = 0x40;
				std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
				sixth.join();
				//second.detach();
			}
			break;
		}

		//���������
		if (area_image_index == IMAGESOFAREA)//ÿһ���������һ��ͼƬ�����������
		{
			if (pointindex % 2 == 1)//��������������Ѻ�ʮ��ͼƬ�����Result_Connection
			{
				for (int m = IMAGESOFAREA / 2; m < IMAGESOFAREA; m++)
				{
					theApp.Result_Connection[m - IMAGESOFAREA / 2] = theApp.ImageProcessResult[m];
				}
			}
			if (pointindex % 2 == 0)//�������ż������ǰʮ��ͼƬ�����Result_Connection
			{
				for (int m = 0; m < IMAGESOFAREA / 2; m++)
				{
					theApp.Result_Connection[m + IMAGESOFAREA] = theApp.ImageProcessResult[m];
				}
			}

			//������ImageProcessResult������ʵ����ֻ�����һ�����������
			int maxCount = 0, Mode_index = 0, nCount = 0;
			for (int i = 0; i < IMAGESOFAREA; i++) 
			{
				for (int j = 0; j < IMAGESOFAREA; j++)
				{
					if (theApp.ImageProcessResult[j] == theApp.ImageProcessResult[i])
						nCount++;
				}

				if (nCount > maxCount)
				{
					maxCount = nCount;
					Mode_index = i;
				}
				nCount = 0;
			}

			//������Result_Connection����
			int maxCount1 = 0, Mode_index1 = 0, nCount1 = 0;
			for (int i = 0; i < IMAGESOFAREA; i++) 
			{
				for (int j = 0; j < IMAGESOFAREA; j++)
				{
					if (theApp.Result_Connection[j] == theApp.Result_Connection[i])
						nCount1++;
				}

				if (nCount1 > maxCount1)
				{
					maxCount1 = nCount1;
					Mode_index1 = i;
				}
				nCount1 = 0;
			}
			measurepoint[pointindex].MeasureResult = 0x00;

			switch (theApp.ImageProcessResult[Mode_index])//������ֵ
			{
				//�Ͻ����
			case 0x08:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])//�������ֵĸ��������ڵ���nʱ����Ϊ������Ͻ�
				{
					measurepoint[pointindex].MeasureResult |= 0x08;
					str.Format(_T("�� %d ������Ͻ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);//�Ͻ�����Ƶ����ߣ���Ҳ���٣�������Ϊ�Ǻϸ��	
					AppendMessage(str, pEditCtrl);
					
				}
				break;

				//ƫ�����
			case 0x10:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x10;
					str.Format(_T("�� %d ������Ϳ��λ��ƫ��"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x04://̫��
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x04;
					str.Format(_T("�� %d ������Ϳ������"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x02://̫խ
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x02;
					str.Format(_T("�� %d ������Ϳ����խ"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x80://�ϸ�
				measurepoint[pointindex].MeasureResult |= 0x80;
				str.Format(_T("�� %d ������Ϳ���ϸ�"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x20://Ϳ������δ��ȫ
				measurepoint[pointindex].MeasureResult |= 0x20;
				str.Format(_T("�� %d ������Ϳ��δ��ȫ"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x40://������
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("�� %d �����������_�ӳ����޼�����"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			default:
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("�� %d �����������2"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;
			}

			//���Ӵ�������ֵ����ҪΪ�˼���ã�������ϸ���ҪΪ�˷�ֹ���������ų���ͼƬ����������ֿ����¸��������ⲻ������
			if (theApp.ImageProcessResult[Mode_index]==0x80)
			{
				switch (theApp.Result_Connection[Mode_index1])
				{
					//�Ͻ�
				case 0x08:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])//���Ӵ����������ֵĸ��������ڵ���1.2nʱ��Ϊ��������Ͻ�
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x08;
							measurepoint[pointindex].MeasureResult |= 0x08;
							str.Format(_T("�� %d ������Ͻ�"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("�� %d ������Ͻ�"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//ƫ��
				case 0x10:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x10;
							measurepoint[pointindex].MeasureResult |= 0x10;
							str.Format(_T("�� %d ������λ��ƫ��"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("�� %d ������λ��ƫ��"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//̫��
				case 0x04:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x04;
							measurepoint[pointindex].MeasureResult |= 0x04;
							str.Format(_T("�� %d ������Ϳ������"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("�� %d ������Ϳ������"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//̫խ
				case 0x02:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x02;
							measurepoint[pointindex].MeasureResult |= 0x02;
							str.Format(_T("�� %d ������Ϳ����խ"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("�� %d ������Ϳ����խ"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//�ϸ��ݲ����
				case 0x80:
					break;

					//Ϳ��δ��ȫ
				case 0x20:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x20;
							measurepoint[pointindex].MeasureResult |= 0x20;
							str.Format(_T("�� %d ������Ϳ��δ��ȫ"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("�� %d ������Ϳ��δ��ȫ"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//Ϳ������_�ӳ����޼�����
				case 0x40:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x40;
							measurepoint[pointindex].MeasureResult |= 0x40;
							str.Format(_T("�� %d ���������_�ӳ����޼�����"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("�� %d ���������_�ӳ����޼�����"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

				default:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x40;
							measurepoint[pointindex].MeasureResult |= 0x40;
							str.Format(_T("�� %d ���������"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("�� %d ���������"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;
				}
			}			
		}

		*(Comm.LPDataResult + 60 + measurepoint[pointindex].pointid_ml) = measurepoint[pointindex].MeasureResult;

		if (pointindex % 2 == 0)//�������ż�����Ѻ�ʮ��ͼƬ�����Result_Connection
		{
			for (int m = IMAGESOFAREA / 2; m < IMAGESOFAREA; m++)
			{
				theApp.Result_Connection[m - IMAGESOFAREA / 2] = theApp.ImageProcessResult[m];
			}
		}

		loction.shrink_to_fit();
		Distance.shrink_to_fit();
		coordinate.shrink_to_fit();
	}

	//t0 = (double)(getTickCount() - t0) / getTickFrequency();
	//printf("runtime = %3lfms\n", t0 * 1000);
	//waitKey(0);
	//FreeConsole();// �ͷſ���̨��Դ	
}
void ProcessImageThread()
{
	CString str;
	while(!sys_exit)
	{
		EnterCriticalSection(&theApp.Critical);

		if ((theApp.IsCompleteCycle))
		{
			if (theApp.current_image_index > 0)
			{
				for (int i = 0;i < theApp.current_image_index;i++)
				{
					if (theApp.image_save_index[i][4] == 1)
					{
						continue;
					}
					//////////////////////////
					if (theApp.image_save_index[i][5] != 0)  ////�ɼ����Ǻõ�ͼ��Ŵ���
					{
					
						int pointindex = theApp.PointIndex[1][robot[1].currenttrajid][i+1];
						measurepoint[pointindex].CoordinateCompute(i);
						theApp.image_save_index[i][4] = 1;
						theApp.image_processed_index++;

						//CString strTemp;
						//strTemp.Format(_T("image verified index:%d"), theApp.image_processed_index);
						//AppendMessage(strTemp, pEditCtrl);
					}
				}

				if ((theApp.image_processed_index + 1) >= robot[1].traj[robot[1].currenttrajid].pointsum)
				{
					theApp.image_all_processed = true;
				}
			}
			else
			{
				theApp.image_all_processed = true;
			}
		}
		else
		{
			theApp.image_all_processed = true;
		}

		LeaveCriticalSection(&theApp.Critical);

		Sleep(300);
	}
}