#include "scanner.h"
#include "dlpc350_usb.h"

scanner::scanner(QWidget* parent)
	: QWidget(parent),
	ui(new Ui::scannerClass)
{
	ui->setupUi(this);

	//Camera
	camera = new Camera(ui, this);
	connect(ui->btnOpenCam, &QPushButton::clicked, camera, &Camera::Open_Device);
	connect(ui->btnCloseCam, &QPushButton::clicked, camera, &Camera::Close_Device);
	connect(ui->btnStartSnap, &QPushButton::clicked, camera, &Camera::Start_Snap);
	connect(ui->btnStopSnap, &QPushButton::clicked, camera, &Camera::Stop_Snap);
	//connect(ui->btnAuto, &QPushButton::clicked, camera, &Camera::auto_baoguang);
	//connect(ui->comboBox, &QComboBox::currentTextChanged, camera, &Camera::auto_baoguang);
	connect(ui->btnSave, &QPushButton::clicked, camera, &Camera::save);
	connect(ui->lineEdit_CamExpTime, &QLineEdit::returnPressed, camera, &Camera::setExpTime);
	

	//DLP
	dlp = new DLP4500(ui, this, camera);
	DLPC350_USB_Init();
	ui->btnDLPConnect->setEnabled(DLPC350_USB_IsConnected());
	m_usbPollTimer = new QTimer(this);
	m_usbPollTimer->setInterval(2000);
	connect(m_usbPollTimer, &QTimer::timeout, dlp, &DLP4500::timerTimeout);
	//connect(m_usbPollTimer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
	m_usbPollTimer->start();

	connect(ui->btnDLPConnect, &QPushButton::clicked, dlp, & DLP4500::showDLP4500Connect);
	connect(ui->checkBoxSLMode, &QCheckBox::clicked, dlp, &DLP4500::setMode);
	ui->checkBoxSLMode->setChecked(true);
	ui->checkBoxSLMode->setEnabled(false);

	connect(ui->btnProject, &QPushButton::clicked, dlp, &DLP4500::startToPro);
	connect(ui->btnStopPro, &QPushButton::clicked, dlp, &DLP4500::stopToPro);

	ui->lineEdit_PatSeqPatPeriod->setReadOnly(true);
	ui->lineEdit_PatSeqPatExpTime->setReadOnly(true);
	
}

scanner::~scanner()
{}

void scanner::closeEvent(QCloseEvent* event) {
	{
		try
		{
			//判断是否停止采集
			if (camera->m_bIsSnap)
			{
				//发送停采命令
				camera->ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();

				//关闭流层通道
				camera->ObjStreamPtr->StopGrab();

				//注销采集回调
				camera->ObjStreamPtr->UnregisterCaptureCallback();

				camera->m_bIsSnap = false;
			}
		}
		catch (CGalaxyException& e)
		{

		}


		try
		{
			//判断是否关闭设备
			if (camera->m_bIsOpen)
			{
				//关闭流对象
				camera->ObjStreamPtr->Close();

				//关闭设备
				camera->ObjDevicePtr->Close();

				camera->m_bIsOpen = false;
			}
		}
		catch (CGalaxyException& e)
		{
		}


		try
		{
			//释放设备资源
			IGXFactory::GetInstance().Uninit();
		}
		catch (CGalaxyException& e)
		{
		}


		if (camera->pCaptureEventHandler != NULL)
		{
			delete camera->pCaptureEventHandler;
			camera->pCaptureEventHandler = NULL;
		}

		//event->accept();
	}

}
void scanner::ShowError(const char* str)
{
	QString title("LightCrafter Error Message");
	QString text(str);
	QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
	msgBox.exec();
}
