#include "Camera.h"

Camera::Camera(Ui::scannerClass* ui, QObject* parent)
	: QObject(parent),
	scanner_ui(ui)
{
}


Camera::~Camera()
{}

void Camera::Open_Device()
{
	try
	{
		//使用其他接口之前，必须执行初始化
		IGXFactory::GetInstance().Init();
		//枚举设备

		GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
		IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);
		if (0 == vectorDeviceInfo.size())
		{
			m_bIsOpen = false;
			scanner_ui->labelCamStatus->setText("no device found!");
			return;
		}
		//打开第一个设备以及设备下面第一个流
		ObjDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(
			vectorDeviceInfo[0].GetSN(),
			GX_ACCESS_EXCLUSIVE);

		ObjStreamPtr = ObjDevicePtr->OpenStream(0);
		m_bIsOpen = true;

		//获取远端设备属性控制器
		ObjFeatureControlPtr = ObjDevicePtr->GetRemoteFeatureControl();
		//获取流层属性控制器
		objStreamFeatureControlPtr = ObjStreamPtr->GetFeatureControl();
		
		////提高网络相机的采集性能,设置方法参考以下代码（目前只有千兆网系列相机支持设置最优包长）。
		GX_DEVICE_CLASS_LIST objDeviceClass =
			ObjDevicePtr->GetDeviceInfo().GetDeviceClass();
		if (GX_DEVICE_CLASS_GEV == objDeviceClass)
		{
			//判断设备是否支持流通道数据包功能
			if (true == ObjFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
			{
				//获取当前网络环境的最优包长值
				int nPacketSize = ObjStreamPtr->GetOptimalPacketSize();//将最优包长值设置为当前设备的流通道包长值
				ObjFeatureControlPtr->GetIntFeature(
					"GevSCPSPacketSize")->SetValue(nPacketSize);
			}
		}

		//ObjFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("NewestOnly");
		ObjStreamPtr->SetAcqusitionBufferNumber(10);


		

	
		scanner_ui->labelCamStatus->setText("device opened!");

	}
	catch (CGalaxyException& e)
	{
		scanner_ui->labelCamStatus->setText(QString::fromStdString(e.what()));
	}
	catch (std::exception& e)
	{
		scanner_ui->labelCamStatus->setText(QString::fromStdString(e.what()));
	}
}
void Camera::Close_Device()
{
	if (!m_bIsOpen) {
		ShowError("Device is not opened!");
		return;
	}
	try
	{
		//判断是否已停止采集
		if (m_bIsSnap)
		{
			//发送停采命令
			ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

			//关闭流层采集
			ObjStreamPtr->StopGrab();

			//注销采集回调
			ObjStreamPtr->UnregisterCaptureCallback();
		}
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	try
	{
		//关闭流对象
		ObjStreamPtr->Close();

	}
	catch (CGalaxyException)
	{
		//do noting
	}
	try
	{
		//关闭设备
		ObjDevicePtr->Close();
		scanner_ui->labelCamStatus->setText(QString("close device success！"));
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	m_bIsOpen = false;
	m_bIsSnap = false;

}
void Camera::Start_Snap()
{
	if (!m_bIsOpen) {
		ShowError("Please open device first!");
		return;
	}
	try {
		objStreamFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("OldestFirstOverwrite");
		//注册回调采集
		pCaptureEventHandler = new CSampleCaptureEventHandler();
		ObjStreamPtr->RegisterCaptureCallback(pCaptureEventHandler, this);

		//发送开采命令
		ObjStreamPtr->StartGrab();
		ObjFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
		m_bIsSnap = true;
		scanner_ui->labelCamStatus->setText("start snapping!");

		
		ObjFeatureControlPtr->GetIntFeature("DeviceLinkSelector")->SetValue(0);
		ObjFeatureControlPtr->GetEnumFeature("DeviceLinkThroughputLimitMode")->SetValue("On");
		//string s = m_objFeatureControlPtr->GetEnumFeature("DeviceLinkThroughputLimitMode")->GetValue();

		ObjFeatureControlPtr->GetFloatFeature("AcquisitionFrameRate")->SetValue(79);
		double d = ObjFeatureControlPtr->GetFloatFeature("AcquisitionFrameRate")->GetValue();


	
		ObjFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
		ObjFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off");


		

		/*ObjFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
		ObjFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue("Line0");*/
	}
	catch (CGalaxyException& e)
	{
		scanner_ui->labelCamStatus->setText(QString::fromStdString(e.what()));
	}
	catch (std::exception& e)
	{
		scanner_ui->labelCamStatus->setText(QString::fromStdString(e.what()));
	}

}
void Camera::Stop_Snap()
{
	if (!m_bIsSnap)
	{
		ShowError("Is not capturing!");
		return;
	}
	//发送停采命令
	ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();
	ObjStreamPtr->StopGrab();
	scanner_ui->labelCamStatus->setText("stop snapping!");




	/*	ObjFeatureControlPtr->GetCommandFeature("TriggerSoftware")->Execute();
		QThread::msleep(30);
			emit Camera::save();
		*/

}
void Camera::setExpTime() 
{
	if (!m_bIsSnap)
	{
		ShowError("Please start capturing first!");
		return;
	}
	if (scanner_ui->lineEdit_CamExpTime->text().toDouble() < 8333)
	{
		ShowError("Minimum exposure time is 8333 us!");
		scanner_ui->lineEdit_CamExpTime->setText("8333");
		return;
	}
	if (scanner_ui->lineEdit_CamExpTime->text().toDouble() > 1000000) 
	{
		ShowError("Maximum exposure time is 1000000 us!");
		scanner_ui->lineEdit_CamExpTime->setText("1000000");
		return;
	}
	std::cout << scanner_ui->lineEdit_CamExpTime->text().toDouble() << std::endl;	
	
	
	ObjFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(scanner_ui->lineEdit_CamExpTime->text().toDouble());
	double d = ObjFeatureControlPtr->GetFloatFeature("ExposureTime")->GetValue();

	if (scanner_ui->lineEdit_CamExpTime->text().toDouble() >= 130000)
	{
		scanner_ui->lineEdit_PatSeqPatPeriod->setText(scanner_ui->lineEdit_CamExpTime->text());
	}
	else {
		scanner_ui->lineEdit_PatSeqPatPeriod->setText("200000");
	}
	
	
	scanner_ui->lineEdit_PatSeqPatExpTime->setText(scanner_ui->lineEdit_CamExpTime->text());



}
//void Camera::auto_baoguang()
//{
//
//
//	if (ObjFeatureControlPtr->IsImplemented("ExposureAuto")) {
//		QString mode = scanner_ui->comboBox->currentText();  // 获取选中的文本
//		scanner_ui->labelCamStatus->setText(mode.toStdString().c_str());
//
//		// 将 QString 转换为 std::string 或 const char*，然后传递给 SetValue
//		ObjFeatureControlPtr->GetEnumFeature("ExposureAuto")->SetValue(mode.toStdString().c_str());
//
//	}
//	else {
//		scanner_ui->labelCamStatus->setText("ExposureAuto feature not implemented.");
//	}
//
//	//ObjFeatureControlPtr->GetEnumFeature("ExposureAuto")->SetValue("on");
//	//string s = m_objFeatureControlPtr->GetEnumFeature("ExposureAuto")->GetValue();
//	//c_ui->labelBottom->setText("auto baoguang!");
//
//}
void Camera::save()
{
	// 获取当前时间
	// 获取当前时间
	auto now = std::chrono::system_clock::now();
	auto time = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::localtime(&time);

	// 获取当前时间的毫秒部分
	auto duration = now.time_since_epoch();
	auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;

	// 格式化时间为字符串（例如：20240918151800762）
	std::ostringstream filename;
	filename << std::put_time(&tm, "%Y%m%d%H%M%S") << std::setfill('0') << std::setw(3) << millis << ".BMP";

	// 生成完整的文件名
	std::string filePath = "./img/" + filename.str();

	// 保存图像
	if (cv::imwrite(filePath, captureImgMat)) {
		std::cout << "图像保存成功: " << filePath << std::endl;
	}
	else {
		std::cerr << "图像保存失败: " << filePath << std::endl;
	}
	
		// 保存图像
	//cv::imwrite("./img.bmp", captureImgMat);
	}
void Camera::ShowError(const char* str)
{
	QString title("LightCrafter Error Message");
	QString text(str);
	QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, scanner_ui->layoutWidget);
	msgBox.exec();
}

//重写回调函数
void CSampleCaptureEventHandler::DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
{
	Camera* camUiPtr = (Camera*)pUserParam;
	camUiPtr->txt_width = QString::number(objImageDataPointer->GetWidth());
	camUiPtr->txt_height = QString::number(objImageDataPointer->GetHeight());

	camUiPtr->captureImgMat.create(objImageDataPointer->GetHeight(), objImageDataPointer->GetWidth(), CV_8UC3);
	void* pRGB24Buffer = NULL;
	//假设原始数据是BayerRG8图像
	pRGB24Buffer = objImageDataPointer->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);
	memcpy(camUiPtr->captureImgMat.data, pRGB24Buffer, (objImageDataPointer->GetHeight()) *
		(objImageDataPointer->GetWidth()) * 3);


	// 上下翻转图像
	cv::flip(camUiPtr->captureImgMat, camUiPtr->captureImgMat, 0);  // 0 表示在垂直方向（上下）翻转
	//显示图片
	//获取的到mat数据转换为image(mat为CV_8UC1)
	QImage image(camUiPtr->captureImgMat.data, camUiPtr->captureImgMat.cols, camUiPtr->captureImgMat.rows,
		camUiPtr->captureImgMat.step, QImage::Format_BGR888);
	
	//获取摄像图像
	/*uchar* pBuffer = NULL;
	QImage* m_pImage;
	pBuffer = (uchar*)objImageDataPointer->ConvertToRaw8(GX_BIT_0_7);
	m_pImage = new QImage(pBuffer, 2448, 2048, QImage::Format_Indexed8);
	QImage* imShow = NULL;
	m_pImage->scaled(camUiPtr->scanner_ui->labelPic->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
	QPixmap* pixmap = NULL;
	camUiPtr->scanner_ui->labelPic->setScaledContents(true);
	camUiPtr->scanner_ui->labelPic->setPixmap(QPixmap::fromImage(*m_pImage));*/


	QImage imShow = image.scaled(camUiPtr->scanner_ui->labelPic->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	QPixmap pixmap = QPixmap::fromImage(imShow);
	camUiPtr->scanner_ui->labelPic->setPixmap(pixmap);	

	//存图！！！！
	if (camUiPtr->triggerMode)
	{
		camUiPtr->save();
		std::cout << "111" << std::endl;
	}
	
	//buffer不足导致丢帧数 
	CGXFeatureControlPointer objStreamFeatureControlPtr = camUiPtr->ObjStreamPtr->GetFeatureControl();

	int s = objStreamFeatureControlPtr->GetIntFeature(
		"StreamLostFrameCount")->GetValue();
	std::cout << s << std::endl;
		


}

//修改触发模式为外部触发
void Camera::setExTrigger()
{

	//ObjFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
	ObjFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("On");
	
	ObjFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue("Line0");
}

//修改触发模式为内部触发
void Camera::setInTrigger()
{
	ObjFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
	ObjFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off");
}


