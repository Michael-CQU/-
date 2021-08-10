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

	int last_pointindex = theApp.PointIndex[1][robot[1].currenttrajid][robot[1].pointid];//上一个点的序号
	int last_s_id = measurepoint[last_pointindex].sensorid;
	int last_segment_num = measurepoint[last_pointindex].segmentnum;

	cv::Mat OnGetFrame_image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U,
		(uint8_t*)ptrGrabResult->GetBuffer());
	if (OnGetFrame_image.empty())
	{
		std::cout << "未读到图片或图片为空" << std::endl;
	}

	//if (color_of_glue==0)//黑胶
	if (SetDataExchange::gluecolor == 1)//黑胶
	{
		Mat OnGetFrame_image0;

		///////////黑胶图像预处理
		blur(OnGetFrame_image, OnGetFrame_image0, Size(5, 5));//均值滤波
		cv::threshold(OnGetFrame_image0, OnGetFrame_image0, 150, 255, CV_THRESH_BINARY);
		//获取自定义核
		cv::Mat element0 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(18, 18));
		//cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
		morphologyEx(OnGetFrame_image0, OnGetFrame_image0, cv::MORPH_CLOSE, element0);
		//morphologyEx(OnGetFrame_image0, OnGetFrame_image0, cv::MORPH_ERODE, element1);
		//MORPH_OPEN开运算、MORPH_CLOSE闭运算、MORPH_ERODE腐蚀、MORPH_DILATE膨胀，黑底白字用闭运算和膨胀

		//将图片顺时针旋转90度，图像中激光条是水平的
		cv::Mat OnGetFrame_image1;
		transpose(OnGetFrame_image0, OnGetFrame_image1);
		flip(OnGetFrame_image1, OnGetFrame_image1, 1);

		// 	//图像中激光条是竖直,不用旋转
		// 	cv::Mat OnGetFrame_image1 = OnGetFrame_image0;

		int row = OnGetFrame_image1.rows, col = OnGetFrame_image1.cols;//图像行数列数
		float width = 0;//求宽度
		double shift_maxHeight;//最高点的偏移量
		double shift_coordinate;//拐点的偏移量

		std::vector<cv::Point2i> loction;//存储激光条坐标
		std::vector<float>Distance; //存储凸起坐标到直线距离（高度）
		std::vector<cv::Point2i> coordinate;//存储拐点坐标

		//遍历图像
		for (int i = 0; i < row; i++)
		{
			// 设定图像数据源指针及输出图像数据指针
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
			//获取两相邻点的距离
			for (int i = 0; i < loction.size() - 1; i++)
			{
				float d;
				d = sqrt(pow((loction[i + 1].x - loction[i].x), 2) + pow((loction[i + 1].y - loction[i].y), 2));//两相邻点的距离
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


		auto maxDistance = std::max_element(Distance.begin(), Distance.end());//求距离的最大值，即宽度
		int maxPosition = maxDistance - Distance.begin();//求距离的最大值的位置

		//不用区分pointindex是否等于1，当robot[1].pointid=0，pointindex=1时，last_s_id和last_segment_num为0，也会进入到if中，和初始化一致
		//if ((measurepoint[pointindex].sensorid != last_s_id) || (measurepoint[pointindex].segmentnum != last_segment_num))///////换相机或者换段了
		if (theApp.current_sensorid != theApp.last_sensorid)///////换相机了
		{
			first_exist_flag = 0;
		}
		//判断位置是否发生偏移，记录首像素的x坐标---图像中激光条是水平的
		if (coordinate.size() == 2 && first_exist_flag == 0)//存在两个拐点和一个峰值的时候
		{
			first_exist_flag = 1;//随意设定一个数，使得first_exist_flag不等于0即可
			first_x = coordinate[0].x;
		}

		int area_image_index = theApp.image_index % IMAGESOFAREA;//1每20张为一个区域，在一个区域内的编号1~20
		if (area_image_index == 0)
		{
			area_image_index = IMAGESOFAREA;
		}

		switch (coordinate.size())
		{
		case 0:
			//断胶
			if (Distance.size() != 0 && loction.size() != 0)
			{
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\断胶%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x08;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				theApp.ImageProcessResult[area_image_index - 1] = 0x08;//断胶  //第几个区域的第几帧（0-19）
			}
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x40;//检测出错
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
			std::cout << "[Width of glue is]：" << *maxDistance << "  像素" << std::endl;

			//偏移
			shift_coordinate = coordinate[0].x - first_x;
			if (shift_coordinate >= theApp.setdata_shift[robot[1].currenttrajid][segment_num][sensorid])
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x10;//位置偏移
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\涂胶偏移%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x10;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			//太宽
			if (*maxDistance > theApp.setdata_toowide_width[robot[1].currenttrajid][segment_num][sensorid])
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x04;
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\涂胶过宽%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x04;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			//太窄
			if (*maxDistance < theApp.setdata_toonarrow_width[robot[1].currenttrajid][segment_num][sensorid])
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x02;
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\涂胶过窄%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x02;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			//涂胶合格
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x80;//涂胶正常，合格
			}
			break;

		default:
			theApp.ImageProcessResult[area_image_index - 1] = 0x40;//检测出错，没检测出来
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

		//处理结果输出
		if (area_image_index == IMAGESOFAREA)//每一个区域最后一张图片检测完后再输出
		{
			//求数组众数，实际上只针对于一个众数的情况
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

			switch (theApp.ImageProcessResult[Mode_index])//众数的值
			{
				//断胶输出
			case 0x08:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])//众数出现的个数，大于等于n时才认为该区域断胶
				{
					measurepoint[pointindex].MeasureResult |= 0x08;
					str.Format(_T("第 %d 块区域断胶"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);//断胶出现频率最高，但也很少，整体认为是合格的
					AppendMessage(str, pEditCtrl);						
				}
				break;

				//偏移输出
			case 0x10:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x10;
					str.Format(_T("第 %d 块区域涂胶位置偏移"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x04://太宽
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x04;
					str.Format(_T("第 %d 块区域涂胶过宽"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x02://太窄
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x02;
					str.Format(_T("第 %d 块区域涂胶过窄"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x80://合格
				measurepoint[pointindex].MeasureResult |= 0x80;
				str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x20://涂胶部分未拍全
				measurepoint[pointindex].MeasureResult |= 0x20;
				str.Format(_T("第 %d 块区域涂胶部分未拍全"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x40://检测出错
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("第 %d 块区域检测出错1"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			default:
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("第 %d 块区域检测出错2"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;
			}
		}
		*(Comm.LPDataResult + 60 + measurepoint[pointindex].pointid_ml) = measurepoint[pointindex].MeasureResult;
	}

	//cout << "点号" << Comm.GetRobPointID(1) << endl;
	//cout << "轨迹号" << Comm.GetRobTrajID(1) << endl;

	//if (color_of_glue == 1)
	if (SetDataExchange::gluecolor == 0)//白胶
	{
		Mat OnGetFrame_image0;

		////白胶图像预处理
		blur(OnGetFrame_image, OnGetFrame_image0, Size(5, 5));//均值滤波
		cv::threshold(OnGetFrame_image0, OnGetFrame_image0, 180, 255, CV_THRESH_BINARY);
		//获取自定义核
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		morphologyEx(OnGetFrame_image0, OnGetFrame_image0, cv::MORPH_DILATE, element);

		//将图片顺时针旋转90度，图像中激光条是水平的
		cv::Mat OnGetFrame_image1;
		transpose(OnGetFrame_image0, OnGetFrame_image1);
		flip(OnGetFrame_image1, OnGetFrame_image1, 1);

		// 	//图像中激光条是竖直,不用旋转
		// 	cv::Mat OnGetFrame_image1 = OnGetFrame_image0;

		int row = OnGetFrame_image1.rows, col = OnGetFrame_image1.cols;//图像行数列数
		int n_left = 0, n_right = 0;//左边和右边出现拐点时在loction中对应的坐标序数
		float d_sum = 0;//凸起坐标到直线距离之和（求面积）
		float width = 0;//求宽度
		double shift_maxHeight;//最高点的偏移量
		double shift_coordinate;//拐点的偏移量

		std::vector<cv::Point2i> loction;//存储激光条坐标
		std::vector<float>Distance; //存储凸起坐标到直线距离（高度）
		std::vector<cv::Point2i> coordinate;//存储拐点坐标

		//遍历图像
		for (int i = 0; i < row; i++)
		{
			// 设定图像数据源指针及输出图像数据指针
			uchar *pSrcData = OnGetFrame_image1.ptr<uchar>(i);
			for (int j = 0; j < col; j++)
			{
				if (pSrcData[j] == 255)
				{
					//loction.push_back(cv::Point2i(j, i));
					loction.push_back(cv::Point2i(i, col - j));
					//loction.push_back(cv::Point2i((j + (int)lineWide / 2), i));//图像中激光条是竖直的
					//loction.push_back(cv::Point2i(i, col - (j + (int)lineWide / 2)));//图像中激光条是水平的
					break;
				}
			}
		}

		//获取所有点到直线距离
		for (int i = 0; i < loction.size(); i++)
		{
			float d;
			d = sqrt((pow(((loction[i].x - loction[0].x)* (loction[loction.size() - 1].y - loction[0].y) +
				(loction[i].y - loction[0].y)* (loction[0].x - loction[loction.size() - 1].x)), 2)) /
				(pow((loction[loction.size() - 1].y - loction[0].y), 2) + pow((loction[0].x - loction[loction.size() - 1].x), 2)));
			Distance.push_back(d);
		}	

		//从左至右判断是否有拐点出现，若有，记录拐点坐标
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
		//从右至左判断是否有拐点出现，若有，记录拐点坐标
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

		//求高度之和，即面积
		for (int i = n_left; i < n_right; i++)
		{
			d_sum += Distance[i];
		}
		auto maxHeight = std::max_element(Distance.begin(), Distance.end());//求高度的最大值，即高度
		int maxPosition = maxHeight - Distance.begin();//求高度的最大值的位置

		//不用区分pointindex是否等于1，当robot[1].pointid=0，pointindex=1时，last_s_id和last_segment_num为0，也会进入到if中，和初始化一致
		//if ((measurepoint[pointindex].sensorid != last_s_id) || (measurepoint[pointindex].segmentnum != last_segment_num))///////换相机或者换段了
		if (theApp.current_sensorid != theApp.last_sensorid)///////换相机了
		{
			first_exist_flag = 0;
		}
		//判断位置是否发生偏移，记录首像素的x坐标---图像中激光条是水平的
		if (coordinate.size() == 2 && first_exist_flag == 0)//存在两个拐点和一个峰值的时候
		{
			first_x = loction[n_left].x;
			first_exist_flag = 1;//随意设定一个数，使得first_exist_flag不等于0
		}


		//cout << "太窄标准"<<theApp.setdata_toonarrow_width[robot[1].currenttrajid][segment_num][sensorid] << endl;
		//cout << "太宽标准" << theApp.setdata_toowide_width[robot[1].currenttrajid][segment_num][sensorid] << endl;
		//cout<<"cuntu"<< theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] << endl;
		//cout << "轨迹的值 " << robot[1].currenttrajid << endl;
		//cout << "段的值 " << segment_num << endl;
		//cout << "相机的值 " << sensorid << endl;

		int area_image_index = theApp.image_index % IMAGESOFAREA;//1每20张为一个区域，在一个区域内的编号1~20
		if (area_image_index == 0)
		{
			area_image_index = IMAGESOFAREA;
		}

		//判断是否有拐点（鼓包）出现
		switch (coordinate.size())
		{
		case 0:
			if (Distance.size() != 0 && loction.size() != 0)
			{
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\断胶%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x08;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
				theApp.ImageProcessResult[area_image_index - 1] = 0x08;//断胶  //第几个区域的第几帧（0-19）
			}
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x40;//检测出错
				 //std::cout << "采集图像中没有激光条" << std::endl;
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
			if (n_left >= 5 && n_right >= 5)//限定在两端是直线的情况下
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x20;//涂胶部分未拍全
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\涂胶部分未拍全%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x20;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
			}
			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x40;//检测出错
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
			width = sqrt(pow((coordinate[1].x - coordinate[0].x), 2) + pow((coordinate[1].y - coordinate[0].y), 2));//两个拐点之间距离，即宽度
			// 			std::cout << "[Area of glue is]：" << d_sum << "  像素" << std::endl;
			// 			std::cout << "[Width of glue is]：" << width << "  像素" << std::endl;
			// 			std::cout << "[Height of glue is]：" << *maxHeight << "  像素" << std::endl;

			//偏移输出
			//shift_maxHeight = loction[maxPosition].x - first_x;//最大高度的偏移量（只考虑x坐标的偏移）---激光条水平
			//shift_maxHeight = loction[maxPosition].y - first_y;//最大高度的偏移量（只考虑x坐标的偏移）---激光条竖直
			//shift_coordinate = loction[n_left].y - first_y;
            shift_coordinate = loction[n_left].x - first_x;
			if (shift_coordinate >= theApp.setdata_shift[robot[1].currenttrajid][segment_num][sensorid]*6.229)
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x10;//位置偏移
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\涂胶偏移%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x10;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					//sixth.detach();
					sixth.join();
				}
				break;
			}

			//太宽
			if (width > theApp.setdata_toowide_width[robot[1].currenttrajid][segment_num][sensorid]*6.229
				|| d_sum > theApp.setdata_toowide_area[robot[1].currenttrajid][segment_num][sensorid]*20.977
				|| *maxHeight > theApp.setdata_toowide_height[robot[1].currenttrajid][segment_num][sensorid]*2.934)
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x04;//太宽
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\涂胶过宽%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x04;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//sixth.detach();
				}
				break;
			}

			//太窄
			if (width < theApp.setdata_toonarrow_width[robot[1].currenttrajid][segment_num][sensorid]*6.229
				|| d_sum < theApp.setdata_toonarrow_area[robot[1].currenttrajid][segment_num][sensorid]*20.977
				|| *maxHeight < theApp.setdata_toonarrow_height[robot[1].currenttrajid][segment_num][sensorid]*2.934)
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x02;//太窄
				if (theApp.setdata_radiobtngroup1[robot[1].currenttrajid][segment_num][sensorid] == 1)
				{
					//imwrite(dirName + "_ng" + cv::format("\\涂胶过窄%d_%d.jpg", pointindex, area_image_index), OnGetFrame_image);
					int number = 0x02;
					std::thread sixth(savethread1, number, pointindex, theApp.image_index, OnGetFrame_image);
					sixth.join();
					//second.detach();
				}
				break;
			}

			else
			{
				theApp.ImageProcessResult[area_image_index - 1] = 0x80;//涂胶正常，合格
			}

		}
		break;

		default:
			theApp.ImageProcessResult[area_image_index - 1] = 0x40;//检测出错，没检测出来
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

		//处理结果输出
		if (area_image_index == IMAGESOFAREA)//每一个区域最后一张图片检测完后再输出
		{
			if (pointindex % 2 == 1)//区域号是奇数，把后十张图片结果给Result_Connection
			{
				for (int m = IMAGESOFAREA / 2; m < IMAGESOFAREA; m++)
				{
					theApp.Result_Connection[m - IMAGESOFAREA / 2] = theApp.ImageProcessResult[m];
				}
			}
			if (pointindex % 2 == 0)//区域号是偶数，把前十张图片结果给Result_Connection
			{
				for (int m = 0; m < IMAGESOFAREA / 2; m++)
				{
					theApp.Result_Connection[m + IMAGESOFAREA] = theApp.ImageProcessResult[m];
				}
			}

			//求数组ImageProcessResult众数，实际上只针对于一个众数的情况
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

			//求数组Result_Connection众数
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

			switch (theApp.ImageProcessResult[Mode_index])//众数的值
			{
				//断胶输出
			case 0x08:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])//众数出现的个数，大于等于n时才认为该区域断胶
				{
					measurepoint[pointindex].MeasureResult |= 0x08;
					str.Format(_T("第 %d 块区域断胶"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);//断胶出现频率最高，但也很少，整体认为是合格的	
					AppendMessage(str, pEditCtrl);
					
				}
				break;

				//偏移输出
			case 0x10:
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x10;
					str.Format(_T("第 %d 块区域涂胶位置偏移"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x04://太宽
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x04;
					str.Format(_T("第 %d 块区域涂胶过宽"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x02://太窄
				if (maxCount >= theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
				{
					measurepoint[pointindex].MeasureResult |= 0x02;
					str.Format(_T("第 %d 块区域涂胶过窄"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				else
				{
					measurepoint[pointindex].MeasureResult |= 0x80;
					str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
					AppendMessage(str, pEditCtrl);
				}
				break;

			case 0x80://合格
				measurepoint[pointindex].MeasureResult |= 0x80;
				str.Format(_T("第 %d 块区域涂胶合格"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x20://涂胶部分未拍全
				measurepoint[pointindex].MeasureResult |= 0x20;
				str.Format(_T("第 %d 块区域涂胶未拍全"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			case 0x40://检测出错
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("第 %d 块区域检测出错_视场中无激光条"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;

			default:
				measurepoint[pointindex].MeasureResult |= 0x40;
				str.Format(_T("第 %d 块区域检测出错2"), pointindex);
				AppendMessage(str, pEditCtrl);
				break;
			}

			//连接处众数的值，主要为了检错用，不输出合格，主要为了防止连续若干张出错图片被两个区域分开导致各区域均检测不出错误
			if (theApp.ImageProcessResult[Mode_index]==0x80)
			{
				switch (theApp.Result_Connection[Mode_index1])
				{
					//断胶
				case 0x08:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])//连接处的众数出现的个数，大于等于1.2n时认为两区域均断胶
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x08;
							measurepoint[pointindex].MeasureResult |= 0x08;
							str.Format(_T("第 %d 块区域断胶"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("第 %d 块区域断胶"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//偏移
				case 0x10:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x10;
							measurepoint[pointindex].MeasureResult |= 0x10;
							str.Format(_T("第 %d 块区域位置偏移"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("第 %d 块区域位置偏移"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//太宽
				case 0x04:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x04;
							measurepoint[pointindex].MeasureResult |= 0x04;
							str.Format(_T("第 %d 块区域涂胶过宽"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("第 %d 块区域涂胶过宽"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//太窄
				case 0x02:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x02;
							measurepoint[pointindex].MeasureResult |= 0x02;
							str.Format(_T("第 %d 块区域涂胶过窄"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("第 %d 块区域涂胶过窄"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//合格，暂不输出
				case 0x80:
					break;

					//涂胶未拍全
				case 0x20:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x20;
							measurepoint[pointindex].MeasureResult |= 0x20;
							str.Format(_T("第 %d 块区域涂胶未拍全"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("第 %d 块区域涂胶未拍全"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;

					//涂胶出错_视场中无激光条
				case 0x40:
					if (pointindex > 1)
					{
						if (maxCount1 >= 1.4*theApp.setdata_imagenum[robot[1].currenttrajid][segment_num][sensorid])
						{
							measurepoint[pointindex - 1].MeasureResult = 0x00;
							measurepoint[pointindex].MeasureResult = 0x00;
							measurepoint[pointindex - 1].MeasureResult |= 0x40;
							measurepoint[pointindex].MeasureResult |= 0x40;
							str.Format(_T("第 %d 块区域出错_视场中无激光条"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("第 %d 块区域出错_视场中无激光条"), pointindex);
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
							str.Format(_T("第 %d 块区域出错"), pointindex - 1);
							AppendMessage(str, pEditCtrl);
							str.Format(_T("第 %d 块区域出错"), pointindex);
							AppendMessage(str, pEditCtrl);
						}
					}
					break;
				}
			}			
		}

		*(Comm.LPDataResult + 60 + measurepoint[pointindex].pointid_ml) = measurepoint[pointindex].MeasureResult;

		if (pointindex % 2 == 0)//区域号是偶数，把后十张图片结果给Result_Connection
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
	//FreeConsole();// 释放控制台资源	
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
					if (theApp.image_save_index[i][5] != 0)  ////采集的是好的图像才处理
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