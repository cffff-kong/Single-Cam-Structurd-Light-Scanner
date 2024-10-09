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
		//ʹ�������ӿ�֮ǰ������ִ�г�ʼ��
		IGXFactory::GetInstance().Init();
		//ö���豸

		GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
		IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);
		if (0 == vectorDeviceInfo.size())
		{
			m_bIsOpen = false;
			scanner_ui->labelCamStatus->setText("no device found!");
			return;
		}
		//�򿪵�һ���豸�Լ��豸�����һ����
		ObjDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(
			vectorDeviceInfo[0].GetSN(),
			GX_ACCESS_EXCLUSIVE);

		ObjStreamPtr = ObjDevicePtr->OpenStream(0);
		m_bIsOpen = true;

		//��ȡԶ���豸���Կ�����
		ObjFeatureControlPtr = ObjDevicePtr->GetRemoteFeatureControl();
		//��ȡ�������Կ�����
		objStreamFeatureControlPtr = ObjStreamPtr->GetFeatureControl();
		
		////�����������Ĳɼ�����,���÷����ο����´��루Ŀǰֻ��ǧ����ϵ�����֧���������Ű�������
		GX_DEVICE_CLASS_LIST objDeviceClass =
			ObjDevicePtr->GetDeviceInfo().GetDeviceClass();
		if (GX_DEVICE_CLASS_GEV == objDeviceClass)
		{
			//�ж��豸�Ƿ�֧����ͨ�����ݰ�����
			if (true == ObjFeatureControlPtr->IsImplemented("GevSCPSPacketSize"))
			{
				//��ȡ��ǰ���绷�������Ű���ֵ
				int nPacketSize = ObjStreamPtr->GetOptimalPacketSize();//�����Ű���ֵ����Ϊ��ǰ�豸����ͨ������ֵ
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
		//�ж��Ƿ���ֹͣ�ɼ�
		if (m_bIsSnap)
		{
			//����ͣ������
			ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

			//�ر�����ɼ�
			ObjStreamPtr->StopGrab();

			//ע���ɼ��ص�
			ObjStreamPtr->UnregisterCaptureCallback();
		}
	}
	catch (CGalaxyException)
	{
		//do noting
	}

	try
	{
		//�ر�������
		ObjStreamPtr->Close();

	}
	catch (CGalaxyException)
	{
		//do noting
	}
	try
	{
		//�ر��豸
		ObjDevicePtr->Close();
		scanner_ui->labelCamStatus->setText(QString("close device success��"));
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
		//ע��ص��ɼ�
		pCaptureEventHandler = new CSampleCaptureEventHandler();
		ObjStreamPtr->RegisterCaptureCallback(pCaptureEventHandler, this);

		//���Ϳ�������
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
	//����ͣ������
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
//		QString mode = scanner_ui->comboBox->currentText();  // ��ȡѡ�е��ı�
//		scanner_ui->labelCamStatus->setText(mode.toStdString().c_str());
//
//		// �� QString ת��Ϊ std::string �� const char*��Ȼ�󴫵ݸ� SetValue
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
	// ��ȡ��ǰʱ��
	// ��ȡ��ǰʱ��
	auto now = std::chrono::system_clock::now();
	auto time = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::localtime(&time);

	// ��ȡ��ǰʱ��ĺ��벿��
	auto duration = now.time_since_epoch();
	auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() % 1000;

	// ��ʽ��ʱ��Ϊ�ַ��������磺20240918151800762��
	std::ostringstream filename;
	filename << std::put_time(&tm, "%Y%m%d%H%M%S") << std::setfill('0') << std::setw(3) << millis << ".BMP";

	// �����������ļ���
	std::string filePath = "./img/" + filename.str();

	// ����ͼ��
	if (cv::imwrite(filePath, captureImgMat)) {
		std::cout << "ͼ�񱣴�ɹ�: " << filePath << std::endl;
	}
	else {
		std::cerr << "ͼ�񱣴�ʧ��: " << filePath << std::endl;
	}
	
		// ����ͼ��
	//cv::imwrite("./img.bmp", captureImgMat);
	}
void Camera::ShowError(const char* str)
{
	QString title("LightCrafter Error Message");
	QString text(str);
	QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, scanner_ui->layoutWidget);
	msgBox.exec();
}

//��д�ص�����
void CSampleCaptureEventHandler::DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam)
{
	Camera* camUiPtr = (Camera*)pUserParam;
	camUiPtr->txt_width = QString::number(objImageDataPointer->GetWidth());
	camUiPtr->txt_height = QString::number(objImageDataPointer->GetHeight());

	camUiPtr->captureImgMat.create(objImageDataPointer->GetHeight(), objImageDataPointer->GetWidth(), CV_8UC3);
	void* pRGB24Buffer = NULL;
	//����ԭʼ������BayerRG8ͼ��
	pRGB24Buffer = objImageDataPointer->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);
	memcpy(camUiPtr->captureImgMat.data, pRGB24Buffer, (objImageDataPointer->GetHeight()) *
		(objImageDataPointer->GetWidth()) * 3);


	// ���·�תͼ��
	cv::flip(camUiPtr->captureImgMat, camUiPtr->captureImgMat, 0);  // 0 ��ʾ�ڴ�ֱ�������£���ת
	//��ʾͼƬ
	//��ȡ�ĵ�mat����ת��Ϊimage(matΪCV_8UC1)
	QImage image(camUiPtr->captureImgMat.data, camUiPtr->captureImgMat.cols, camUiPtr->captureImgMat.rows,
		camUiPtr->captureImgMat.step, QImage::Format_BGR888);
	
	//��ȡ����ͼ��
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

	//��ͼ��������
	if (camUiPtr->triggerMode)
	{
		camUiPtr->save();
		std::cout << "111" << std::endl;
	}
	
	//buffer���㵼�¶�֡�� 
	CGXFeatureControlPointer objStreamFeatureControlPtr = camUiPtr->ObjStreamPtr->GetFeatureControl();

	int s = objStreamFeatureControlPtr->GetIntFeature(
		"StreamLostFrameCount")->GetValue();
	std::cout << s << std::endl;
		


}

//�޸Ĵ���ģʽΪ�ⲿ����
void Camera::setExTrigger()
{

	//ObjFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
	ObjFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("On");
	
	ObjFeatureControlPtr->GetEnumFeature("TriggerSource")->SetValue("Line0");
}

//�޸Ĵ���ģʽΪ�ڲ�����
void Camera::setInTrigger()
{
	ObjFeatureControlPtr->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
	ObjFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("Off");
}


