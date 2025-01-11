#include "SSLReconstruction.h"

SSLReconstruction::SSLReconstruction(int ps_num, int img_width, int img_height, int T1, int T2, int T3)
{
	this->m_ps_num = ps_num;
	this->m_width = img_width;
	this->m_height = img_height;
	this->m_frequency = new int [3] {T1, T2, T3};
	m_T13 = T1 - T3;
	m_T23 = T2 - T3;
	m_T123 = m_T13 - m_T23;
	m_k_cam = (cv::Mat_<float>(3, 3) << 
		4917.83770754812, 0, 1214.55428929112,
		0, 4923.82638685160, 1032.77712790899,
		0, 0, 1);
	m_k_dlp = (cv::Mat_<float>(3, 3) << 
		1257.86693007908, 0, 473.175940813014,
		0, 3154.36722942004, 810.775389044632,
		0, 0, 1);
	m_rt_cam = (cv::Mat_<float>(3, 4) <<
		0.0282843801595056, 0.999453513215491, 0.0171075644147791, -167.273600072281,
		-0.991514508545440, 0.0258791135103581, 0.127394076894544, 48.7112427545775,
		0.126881729113720, -0.0205656608240837, 0.991704633654588, 1030.39786715420);
	m_rt_dlp = (cv::Mat_<float>(3, 4) <<
		0.0156316443525661, 0.998161200177202, -0.0585650933205363, -96.8716349512309,
		-0.992163655512276, 0.0227461190972831, 0.122857212838845, 30.6138996645475,
		0.123963431605260, 0.0561856968170766, 0.990694824402463, 950.178896112994);
}

void SSLReconstruction::loadImg()
{
	for (int i = 0; i < this->m_ps_num * 3; i++)
	{
		string path = m_path + "/" + to_string(i + 1) + ".bmp";
		cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
		m_imgs.push_back(img);
	}
}


cv::Mat SSLReconstruction::CalWrappedPhase(int index)
{
	vector<cv::Mat> imgs;
	for (int i = index * m_ps_num;i < index * m_ps_num + m_ps_num;i++)
	{
		imgs.push_back(m_imgs[i]);
	}
	cv::Mat sin_sum = cv::Mat::zeros(imgs[0].size(), CV_32F);
	cv::Mat cos_sum = cv::Mat::zeros(imgs[0].size(), CV_32F);

	for (int i = 0; i < this->m_ps_num; i++)
	{
		cv::Mat temp = imgs[i].clone(); // 使用 psImg[0] 的数据初始化 temp
		temp.convertTo(temp, CV_32FC1);
		float pk = 2 * i * CV_PI / m_ps_num;

		sin_sum += temp * sin(pk);
		cos_sum += temp * cos(pk);
	}
	cv::Mat pha(sin_sum.size(), CV_32F);
	for (int col = 0; col < m_width; col++)
	{
		for (int row = 0; row < m_height; row++)
		{
			pha.at<float>(row, col) = atan2(sin_sum.at<float>(row, col), cos_sum.at<float>(row, col));
		}
	}
	cv::Mat A = sin_sum.mul(sin_sum) + cos_sum.mul(cos_sum);
	sqrt(A, m_B);
	m_B = m_B * 2 / m_ps_num;
	m_B_mask = m_B > 10;
	m_B_mask.convertTo(m_B_mask, CV_32FC1);
	pha = -pha;
	cv::Mat pha_low_mask = pha <= 0;
	pha_low_mask.convertTo(pha_low_mask, CV_32FC1);
	pha = pha + pha_low_mask / 255 * CV_2PI;
	return pha;
}

cv::Mat SSLReconstruction::HeterodynePhase(cv::Mat phase1, int T1, cv::Mat phase2, int T2)
{
	int T12 = T1 - T2;
	float K = (float)T1 / float(T12);
	cv::Mat phase_diff = phase1 - phase2;
	cv::Mat index = phase_diff < 0;
	index.convertTo(index, CV_32FC1);
	for (int row = 0;row < m_height;row++)
	{
		for (int col = 0;col < m_width;col++)
		{
			if (index.at<float>(row, col) > 0)
			{
				phase_diff.at<float>(row, col) += CV_2PI;
			}
		}
	}

	cv::Mat period = ((phase_diff * K - phase1) / (CV_2PI));
	for (int row = 0;row < m_height;row++)
	{
		for (int col = 0;col < m_width;col++)
		{
			period.at<float>(row, col) = round(period.at<float>(row, col));
		}

	}
	cv::Mat pha = (period * CV_2PI + phase1) / K;

	return pha;
}

void SSLReconstruction::CombinePhase()
{
	cv::Mat phase1 = m_phase123 * float(m_frequency[0]);
	cv::Mat phase2 = ((m_phase123 * m_frequency[1] - m_wrapped_phase2) / CV_2PI) * CV_2PI + m_wrapped_phase2;
	cv::Mat phase3 = ((m_phase123 * m_frequency[2] - m_wrapped_phase3) / CV_2PI) * CV_2PI + m_wrapped_phase3;
	for (int row = 0;row < m_height;row++)
	{
		for (int col = 0;col < m_width;col++)
		{
			phase2.at<float>(row, col) = round(phase2.at<float>(row, col));
			phase3.at<float>(row, col) = round(phase3.at<float>(row, col));
		}
	}
	m_phase_abs = (phase1 + phase2 + phase3) / (m_frequency[0] + m_frequency[1] + m_frequency[2]);

}

void SSLReconstruction::FilterPhase()
{
	//滤波条件：对每个像素点：1.最大值大于245；2.最小值小于10；3.最大最小值差值小于10
	for (int row = 0;row < m_height;row++)
	{
		for (int col = 0;col < m_width;col++)
		{
			int max_val = 0, min_val = 255;
			for (int i = 0;i < m_imgs.size();i++)
			{
				
				if (m_imgs[i].at<uchar>(row, col) > max_val)
				{
					max_val = m_imgs[i].at<uchar>(row, col);
				}
				if (m_imgs[i].at<uchar>(row, col) < min_val)
				{
					min_val = m_imgs[i].at<uchar>(row, col);
				}
			}
			if (max_val > 253 || abs(max_val - min_val) <= 2 || min_val < 2)
			{
				m_phase_abs.at<float>(row, col) = 0;
			}
		}
	}
	//因为无效点都设置的-100，如果直接归一化会让分母变得很大，进而导致每个元素都差不多，所以显示的时候基本都是纯白。
	//为了正常显示，要除以整幅图里最大的值

}
void SSLReconstruction::saveToCSV(const cv::Mat& mat, const std::string& filename)
{
	std::ofstream file(filename);
	if (!file.is_open())
	{
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	for (int i = 0; i < mat.rows; ++i)
	{
		for (int j = 0; j < mat.cols; ++j)
		{
			file << mat.at<float>(i, j);
			if (j < mat.cols - 1)
				file << ",";
		}
		file << "\n";
	}

	file.close();
	std::cout << "Saved to " << filename << " successfully." << std::endl;
}

cv::Mat SSLReconstruction::MonoFilter()
{
	cv::Mat mask = cv::Mat::zeros(m_phase_abs.size(), CV_32FC1);
	for (int row = 0;row < m_height;row++)
	{
		for (int col = 1;col < m_width - 1;col++)
		{
			if (m_phase_abs.at<float>(row, col) > m_phase_abs.at<float>(row, col - 1)
				&& m_phase_abs.at<float>(row, col) < m_phase_abs.at<float>(row, col + 1))
			{
				mask.at<float>(row, col) = 1;
			}
		}
	}
	return mask;

}

void SSLReconstruction::Mono()
{
	for (int row = 0;row < m_height;row++)
	{
		for (int col = 1;col < m_width-1;col++)
		{
			if (col<1 || col>m_width - 2 || row<1 || row>m_height - 2)
			{
				m_phase_abs.at<float>(row, col) = -100;
			}
			else if (m_phase_abs.at<float>(row, col) != -100)
			{
				if (m_phase_abs.at<float>(row, col-1) != -100)
				{
					if (
						 m_phase_abs.at<float>(row, col) < m_phase_abs.at<float>(row, col - 1))
					{
						m_phase_abs.at<float>(row, col) = -100;
					}
				}
				if (m_phase_abs.at<float>(row, col + 1) != -100)
				{
					if ( m_phase_abs.at<float>(row, col + 1) < m_phase_abs.at<float>(row, col))
					{
						m_phase_abs.at<float>(row, col) = -100;
					}
				}
				if (m_phase_abs.at<float>(row, col - 1) == 0 && m_phase_abs.at<float>(row, col + 1) == -100)
				{
					m_phase_abs.at<float>(row, col) = -100;
				}
				/*if (m_phase_abs.at<float>(row - 1, col) != -100)
				{
					if (m_phase_abs.at<float>(row, col) - m_phase_abs.at<float>(row - 1, col) > (CV_PI / m_width / 4))
					{
						m_phase_abs.at<float>(row, col) = -100;
					}
				}
				if (m_phase_abs.at<float>(row + 1, col) != -100)
				{
					if (m_phase_abs.at<float>(row + 1, col) - m_phase_abs.at<float>(row, col) > (CV_PI / m_width / 4))
					{
						m_phase_abs.at<float>(row + 1, col) = -100;
					}
				}*/
				if (m_phase_abs.at<float>(row - 1, col) == -100 && m_phase_abs.at<float>(row + 1, col) == -100)
				{
					m_phase_abs.at<float>(row, col) = -100;
				}
			}
		}
	}
}
cv::Mat SSLReconstruction::Decode()
{
	//loadImg();  不需要了，直接外部传个vector来
	m_wrapped_phase1 = CalWrappedPhase(0);
	m_wrapped_phase2 = CalWrappedPhase(1);
	m_wrapped_phase3 = CalWrappedPhase(2);
	m_phase13 = HeterodynePhase(m_wrapped_phase1, m_frequency[0], m_wrapped_phase3, m_frequency[2]);
	m_phase23 = HeterodynePhase(m_wrapped_phase2, m_frequency[1], m_wrapped_phase3, m_frequency[2]);
	m_phase123 = HeterodynePhase(m_phase13, m_T13, m_phase23, m_T23);
	CombinePhase();

	//FilterPhase();
	//调制度滤波
	m_B_mask.convertTo(m_B_mask, CV_32FC1);
	//单调性滤波
	cv::Mat mask = MonoFilter();
	for (int col = 0;col < m_width;col++)
	{
		for (int row = 0;row < m_height;row++)
		{
			if (m_B_mask.at<float>(row, col) == 0 || mask.at<float>(row, col) == 0)
			{
				m_phase_abs.at<float>(row, col) = 0;
			}
		}
	}
	return m_phase_abs;
}

void SSLReconstruction::CloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SSLReconstruction::Reconstruction()
{
	cv::Mat phase = Decode();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cv::Mat A(3, 3, CV_32FC1);
	cv::Mat b=cv::Mat(3, 1, CV_32FC1);
	cv::Mat xyz;
	cv::Mat m_q_cam = m_k_cam * m_rt_cam;
	cv::Mat m_q_dlp = m_k_dlp * m_rt_dlp;

	//重建
	for (int row = 1;row < m_height;row++)
	{
		for (int col = 1;col < m_width;col++)
		{
			if (phase.at<float>(row, col) > 0)
			{
				float up=phase.at<float>(row, col) * 912 / (2 * CV_PI);
				float uc = col;
				float vc = row;

				// A 的计算
				A.at<float>(0, 0) = m_q_cam.at<float>(0, 0) - m_q_cam.at<float>(2, 0) * uc;
				A.at<float>(0, 1) = m_q_cam.at<float>(0, 1) - m_q_cam.at<float>(2, 1) * uc;
				A.at<float>(0, 2) = m_q_cam.at<float>(0, 2) - m_q_cam.at<float>(2, 2) * uc;

				A.at<float>(1, 0) = m_q_cam.at<float>(1, 0) - m_q_cam.at<float>(2, 0) * vc;
				A.at<float>(1, 1) = m_q_cam.at<float>(1, 1) - m_q_cam.at<float>(2, 1) * vc;
				A.at<float>(1, 2) = m_q_cam.at<float>(1, 2) - m_q_cam.at<float>(2, 2) * vc;

				A.at<float>(2, 0) = m_q_dlp.at<float>(0, 0) - m_q_dlp.at<float>(2, 0) * up;
				A.at<float>(2, 1) = m_q_dlp.at<float>(0, 1) - m_q_dlp.at<float>(2, 1) * up;
				A.at<float>(2, 2) = m_q_dlp.at<float>(0, 2) - m_q_dlp.at<float>(2, 2) * up;

				// b 的计算
				b.at<float>(0, 0) = m_q_cam.at<float>(2, 3) * uc - m_q_cam.at<float>(0, 3);
				b.at<float>(1, 0) = m_q_cam.at<float>(2, 3) * vc - m_q_cam.at<float>(1, 3);
				b.at<float>(2, 0) = m_q_dlp.at<float>(2, 3) * up - m_q_dlp.at<float>(0, 3);

				cv::solve(A, b, xyz);
				
				pcl::PointXYZ points;
				points.x = xyz.at<float>(0, 0);
				points.y = xyz.at<float>(1, 0);
				points.z = xyz.at<float>(2, 0);
				cloud->push_back(points);
			}
		}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filterd(new pcl::PointCloud < pcl::PointXYZ >);
	CloudPointFilter(cloud, cloud_filterd);

	return cloud_filterd;
}

