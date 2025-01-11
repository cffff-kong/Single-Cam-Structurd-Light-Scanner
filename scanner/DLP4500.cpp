#include "DLP4500.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hidapi.h"

#include "dlpc350_common.h"
#include "dlpc350_error.h"
#include "dlpc350_usb.h"
#include "dlpc350_api.h"
#include "dlpc350_flashDevice.h"
#include "dlpc350_BMPParser.h"
#include "dlpc350_firmware.h"
#include "dlpc350_version.h"
#include <scanner.h>
#include <QMessageBox>

#define MAX_NUM_RETRIES 5
DLP4500::DLP4500(Ui::scannerClass* ui, QObject* parent, Camera* camera)
	: QObject(parent),
	scanner_ui(ui),
	cam(camera)
{

}

DLP4500::~DLP4500()
{}
void DLP4500::ShowError(const char* str)
{
	QString title("LightCrafter Error Message");
	QString text(str);
	QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, scanner_ui->layoutWidget);
	msgBox.exec();
}
void DLP4500::timerTimeout(void)
{
	if (DLPC350_USB_IsConnected())
	{
		scanner_ui->btnDLPConnect->setEnabled(DLPC350_USB_IsConnected());
		scanner_ui->btnDLPConnect->setText("Connected");
		scanner_ui->labelDLP4500Status->setText("DLP4500 is connected");
	}
	else
	{	//因为投影仪不会第一时间在软件打开时就连接，所以会进入这个else分支
		if (DLPC350_USB_Open() == 0)
		{
			scanner_ui->btnDLPConnect->setEnabled(DLPC350_USB_IsConnected());
			//ShowError("projector has been connected");
			unsigned char firmwareTag[33];
			if (DLPC350_GetFirmwareTagInfo(&firmwareTag[0]) == 0)
			{
				QString str((char*)firmwareTag);
				scanner_ui->labelFirmware->setText(str);
			}
			//emit on_connectButton_clicked();
		}
		scanner_ui->btnDLPConnect->setText("UnConnected");
		scanner_ui->labelDLP4500Status->setText("DLP4500 is not connected");
	}
}

void DLP4500::showDLP4500Connect()
{
	//scanner_ui->labelDLP4500Status->setText("DLP4500 is connected");
	//ui.connectButton->setEnabled(DLPC350_USB_IsConnected());
	//ShowError("projector has been connected");


}

void DLP4500::setMode()
{
	int trigMode = 0;
	bool isExtPatDisplayMode = false;


	SetDLPC350InPatternMode();



	//Update all the settings under the page
	if (DLPC350_GetPatternTriggerMode(&trigMode) == 0)
	{
		if (trigMode <= 2)
		{
			if (DLPC350_GetPatternDisplayMode(&isExtPatDisplayMode) == 0)
			{


				if (!isExtPatDisplayMode)
				{
					//ui.radioButton_PatSeqSrcFrmFlash->setChecked(true);
					//emit on_radioButton_PatSeqSrcFrmFlash_clicked();
				}
			}
		}
	}

}
void DLP4500::SetDLPC350InPatternMode()
{
	int i = 0;
	bool mode;
	unsigned int patMode;

	//Check if it is in Pattern Mode
	DLPC350_GetMode(&mode);
	if (mode == false)
	{
		//Switch to Pattern Mode
		DLPC350_SetMode(true);
		SleeperThread::msleep(100);
		while (1)
		{
			DLPC350_GetMode(&mode);
			if (mode)
				break;
			SleeperThread::msleep(100);
			if (i++ > MAX_NUM_RETRIES)
				break;
		}
	}
	else
	{
		//First stop pattern sequence
		DLPC350_GetPatternDisplay(&patMode);
		//if it is in PAUSE or RUN mode
		if (patMode != 0)
		{
			//emit on_pushButton_PatSeqCtrlStop_clicked();
		}
	}

	return;
}

void DLP4500::phaseShifting_12()
{
	emit DLP4500::stopToPro();
	cam->setExTrigger();
	DLPC350_ClearPatLut();
	cam->triggerMode = true;
	for (int i = 0; i < 4; i++) {
		if (DLPC350_AddToPatLut(0, 0, 8, 7, 0, 1, 1, 0) < 0)
		{
			ShowError("error Updating LUT");
		}
		if (DLPC350_AddToPatLut(0, 1, 8, 7, 0, 1, 0, 0) < 0)
		{
			ShowError("error Updating LUT");
		}
		if (DLPC350_AddToPatLut(0, 2, 8, 7, 0, 1, 0, 0) < 0)
		{
			ShowError("error Updating LUT");
		}
	}
	//检查是否勾选sequence
	if (DLPC350_SetPatternDisplayMode(0) < 0)
	{
		ShowError("error set from flash");
	}
	//写死吧，不给repeat和once的选项了
	if (true)
	{
		numPatterns = 12;
	}
	else
	{
		numPatterns = 1;
	}
	//int DLPC350_SetPatternConfig(unsigned int numLutEntries, bool repeat, unsigned int numPatsForTrigOut2, unsigned int numImages)
	if (DLPC350_SetPatternConfig(numPatterns, false, numPatterns, 12) < 0)
	{
		ShowError("error Sending Pattern Config");

	}
	if (DLPC350_SetExposure_FramePeriod(scanner_ui->lineEdit_PatSeqPatExpTime->text().toInt(),
		scanner_ui->lineEdit_PatSeqPatPeriod->text().toInt()) < 0)
	{
		ShowError("error Sending Exposure period");

	}
	//Configure Trigger Mode - 0(External) or 1(internal)
	if (DLPC350_SetPatternTriggerMode(1) < 0)
	{
		ShowError("error Sending trigger Mode");

	}
	//Send Pattern LUT
	if (DLPC350_SendPatLut() < 0)
	{

		ShowError("error Sending Pattern LUT");

	}
	unsigned char splashLut[64];
	for (int i = 0; i < 4; i++)		//36/3=12
	{
		splashLut[i] = i;
	}
	if (DLPC350_SendImageLut(&splashLut[0], 12) < 0)
	{

		ShowError("error Sending Image LUT");

	}
	/************************开始投影**********************************/
	if (DLPC350_StartPatLutValidate() < 0)
	{
		ShowError("error check LUT data");

	}
	int i = 0;
	unsigned int status;
	bool ready;
	QEventLoop loop;
	do
	{
		if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
		{
			ShowError("error validating LUT data");
		}

		if (ready)
		{
			break;
		}
		else
		{
			QTimer::singleShot(1000, &loop, SLOT(quit()));
			loop.exec();
		}

		if (i++ > MAX_NUM_RETRIES)
			break;
	} while (1);

	if (status != 0)
	{
		ShowError("Sequence validation FAILED!");
	}

	if ((status & BIT0) == BIT0)
	{
		ShowError("Exposure or frame period OUT OF RANGE");
	}
	if ((status & BIT1) == BIT1)
	{
		ShowError("Pattern number in lookup table INVALID");
	}
	if ((status & BIT2) == BIT2)
	{
		ShowError("Continued output trigger OVERLAPS black vector");
	}
	if ((status & BIT3) == BIT3)
	{
		ShowError("Black vector MISSING when exposure less than frame period");
	}
	if ((status & BIT4) == BIT4)
	{
		ShowError("Difference between exposure and frame period less than 230us");
	}

	if (DLPC350_PatternDisplay(2) < 0)
	{
		ShowError("error play");
	}
	
	// 设置延迟，比如延迟2000毫秒（2秒）
	QTimer::singleShot(scanner_ui->lineEdit_PatSeqPatPeriod->text().toDouble()*15/1000, this, [this]() {
		cam->setInTrigger();
		cam->triggerMode = false;
		cout << cam->ssl->m_imgs.size() << endl;
		if (cam->ssl->m_imgs.size()== 12){
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
			cloud = cam->ssl->Reconstruction();
			pcl::io::savePCDFileASCII("test.pcd", *cloud);
			cout << "点云生成完成" << endl;
		}
		else
		{
			ShowError("Scan error  please scan again");
		}
		//释放m_imgs的内存
		cam->ssl->m_imgs.clear();

		});
}

void DLP4500::calibrate()
{
	emit DLP4500::stopToPro();
	DLPC350_ClearPatLut();
	for (int i = 0; i < 24; i++) {
		if (DLPC350_AddToPatLut(0, 0, 8, 7, 0, 1, 1, 0) < 0)
		{
			ShowError("error Updating LUT");
		}
		if (DLPC350_AddToPatLut(0, 1, 8, 7, 0, 1, 0, 0) < 0)
		{
			ShowError("error Updating LUT");
		}
		if (DLPC350_AddToPatLut(0, 2, 8, 7, 0, 1, 0, 0) < 0)
		{
			ShowError("error Updating LUT");
		}
	}

	//检查是否勾选sequence
	if (DLPC350_SetPatternDisplayMode(0) < 0)
	{
		ShowError("error set from flash");
	}


	//写死吧，不给repeat和once的选项了
	if (true)
	{
		numPatterns = 72;
	}
	else
	{
		numPatterns = 1;
	}

	//if (DLPC350_SetPatternConfig(numPatterns, ui.radioButton_PatSeqDispRunContinuous->isChecked(), numPatterns, 18) < 0)
	if (DLPC350_SetPatternConfig(numPatterns, false, numPatterns, numPatterns) < 0)
	{
		ShowError("error Sending Pattern Config");

	}


	if (DLPC350_SetExposure_FramePeriod(scanner_ui->lineEdit_PatSeqPatExpTime->text().toInt(),
		scanner_ui->lineEdit_PatSeqPatPeriod->text().toInt()) < 0)
	{
		ShowError("error Sending Exposure period");

	}



	//Configure Trigger Mode - 0(External) or 1(internal)
	if (DLPC350_SetPatternTriggerMode(1) < 0)
	{
		ShowError("error Sending trigger Mode");

	}


	//Send Pattern LUT
	if (DLPC350_SendPatLut() < 0)
	{

		ShowError("error Sending Pattern LUT");

	}

	unsigned char splashLut[24];
	for (int i = 0; i < 24; i++)		// 72/3=24
	{
		splashLut[i] = i;
	}


	if (DLPC350_SendImageLut(&splashLut[0], 24) < 0)
	{

		ShowError("error Sending Image LUT");

	}

	/************************开始投影**********************************/
	if (DLPC350_StartPatLutValidate() < 0)
	{
		ShowError("error check LUT data");

	}



	int i = 0;
	unsigned int status;
	bool ready;

	QEventLoop loop;

	do
	{
		if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
		{
			ShowError("error validating LUT data");
		}

		if (ready)
		{
			break;
		}
		else
		{
			QTimer::singleShot(1000, &loop, SLOT(quit()));
			loop.exec();
		}

		if (i++ > MAX_NUM_RETRIES)
			break;
	} while (1);

	if (status != 0)
	{
		ShowError("Sequence validation FAILED!");
	}

	if ((status & BIT0) == BIT0)
	{
		ShowError("Exposure or frame period OUT OF RANGE");
	}
	if ((status & BIT1) == BIT1)
	{
		ShowError("Pattern number in lookup table INVALID");
	}
	if ((status & BIT2) == BIT2)
	{
		ShowError("Continued output trigger OVERLAPS black vector");
	}
	if ((status & BIT3) == BIT3)
	{
		ShowError("Black vector MISSING when exposure less than frame period");
	}
	if ((status & BIT4) == BIT4)
	{
		ShowError("Difference between exposure and frame period less than 230us");
	}
	cam->setExTrigger();
	cam->triggerMode = true;
	QThread::msleep(1000);

	if (DLPC350_PatternDisplay(2) < 0)
	{
		ShowError("error play");
	}
	
	QTimer::singleShot(scanner_ui->lineEdit_PatSeqPatPeriod->text().toDouble() * 73 / 1000, this, [this]() {
		// 第一个延迟函数
		emit showWhite();

		// 在第一个函数执行完之后，再开始第二个延迟
		QTimer::singleShot(scanner_ui->lineEdit_PatSeqPatPeriod->text().toDouble() * 2 / 1000, this, [this]() {
			cam->setInTrigger();
			cam->triggerMode = false;
			stopToPro();
			});
		});

}

void DLP4500::showWhite() {
	DLPC350_ClearPatLut();
	if (DLPC350_AddToPatLut(0, 24, 1, 7, 1, 0, 1, 0) < 0)
	{
		ShowError("error Updating LUT");
	}

	//检查是否勾选sequence
	if (DLPC350_SetPatternDisplayMode(0) < 0)
	{
		ShowError("error set from flash");
	}


	//写死吧，不给repeat和once的选项了
	if (true)
	{
		numPatterns = 1;
	}
	else
	{
		numPatterns = 1;
	}

	//if (DLPC350_SetPatternConfig(numPatterns, ui.radioButton_PatSeqDispRunContinuous->isChecked(), numPatterns, 18) < 0)
	if (DLPC350_SetPatternConfig(numPatterns, true, numPatterns, numPatterns) < 0)
	{
		ShowError("error Sending Pattern Config");

	}


	if (DLPC350_SetExposure_FramePeriod(130000,
		130000) < 0)
	{
		ShowError("error Sending Exposure period");

	}



	//Configure Trigger Mode - 0(External) or 1(internal)
	if (DLPC350_SetPatternTriggerMode(1) < 0)
	{
		ShowError("error Sending trigger Mode");

	}


	//Send Pattern LUT
	if (DLPC350_SendPatLut() < 0)
	{

		ShowError("error Sending Pattern LUT");

	}

	unsigned char splashLut[1];
	splashLut[0] = 0;


	if (DLPC350_SendImageLut(&splashLut[0], 1) < 0)
	{

		ShowError("error Sending Image LUT");

	}

	/************************开始投影**********************************/
	if (DLPC350_StartPatLutValidate() < 0)
	{
		ShowError("error check LUT data");

	}
	int i = 0;
	unsigned int status;
	bool ready;

	QEventLoop loop;

	do
	{
		if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
		{
			ShowError("error validating LUT data");
		}

		if (ready)
		{
			break;
		}
		else
		{
			QTimer::singleShot(1000, &loop, SLOT(quit()));
			loop.exec();
		}

		if (i++ > MAX_NUM_RETRIES)
			break;
	} while (1);

	if (status != 0)
	{
		ShowError("Sequence validation FAILED!");
	}

	if ((status & BIT0) == BIT0)
	{
		ShowError("Exposure or frame period OUT OF RANGE");
	}
	if ((status & BIT1) == BIT1)
	{
		ShowError("Pattern number in lookup table INVALID");
	}
	if ((status & BIT2) == BIT2)
	{
		ShowError("Continued output trigger OVERLAPS black vector");
	}
	if ((status & BIT3) == BIT3)
	{
		ShowError("Black vector MISSING when exposure less than frame period");
	}
	if ((status & BIT4) == BIT4)
	{
		ShowError("Difference between exposure and frame period less than 230us");
	}

	if (DLPC350_PatternDisplay(2) < 0)
	{
		ShowError("error play");
	}
}

void DLP4500::showPattern() {
	DLPC350_ClearPatLut();
	if (DLPC350_AddToPatLut(0, 0, 8, 7, 0, 0, 1, 0) < 0)
	{
		ShowError("error Updating LUT");
	}

	//检查是否勾选sequence
	if (DLPC350_SetPatternDisplayMode(0) < 0)
	{
		ShowError("error set from flash");
	}


	//写死吧，不给repeat和once的选项了
	if (true)
	{
		numPatterns = 1;
	}
	else
	{
		numPatterns = 1;
	}

	//if (DLPC350_SetPatternConfig(numPatterns, ui.radioButton_PatSeqDispRunContinuous->isChecked(), numPatterns, 18) < 0)
	if (DLPC350_SetPatternConfig(numPatterns, true, numPatterns, numPatterns) < 0)
	{
		ShowError("error Sending Pattern Config");

	}


	if (DLPC350_SetExposure_FramePeriod(130000,
		130000) < 0)
	{
		ShowError("error Sending Exposure period");

	}



	//Configure Trigger Mode - 0(External) or 1(internal)
	if (DLPC350_SetPatternTriggerMode(1) < 0)
	{
		ShowError("error Sending trigger Mode");

	}


	//Send Pattern LUT
	if (DLPC350_SendPatLut() < 0)
	{

		ShowError("error Sending Pattern LUT");

	}

	unsigned char splashLut[1];
	splashLut[0] = 0;


	if (DLPC350_SendImageLut(&splashLut[0], 1) < 0)
	{

		ShowError("error Sending Image LUT");

	}

	/************************开始投影**********************************/
	if (DLPC350_StartPatLutValidate() < 0)
	{
		ShowError("error check LUT data");

	}
	int i = 0;
	unsigned int status;
	bool ready;

	QEventLoop loop;

	do
	{
		if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
		{
			ShowError("error validating LUT data");
		}

		if (ready)
		{
			break;
		}
		else
		{
			QTimer::singleShot(1000, &loop, SLOT(quit()));
			loop.exec();
		}

		if (i++ > MAX_NUM_RETRIES)
			break;
	} while (1);

	if (status != 0)
	{
		ShowError("Sequence validation FAILED!");
	}

	if ((status & BIT0) == BIT0)
	{
		ShowError("Exposure or frame period OUT OF RANGE");
	}
	if ((status & BIT1) == BIT1)
	{
		ShowError("Pattern number in lookup table INVALID");
	}
	if ((status & BIT2) == BIT2)
	{
		ShowError("Continued output trigger OVERLAPS black vector");
	}
	if ((status & BIT3) == BIT3)
	{
		ShowError("Black vector MISSING when exposure less than frame period");
	}
	if ((status & BIT4) == BIT4)
	{
		ShowError("Difference between exposure and frame period less than 230us");
	}

	if (DLPC350_PatternDisplay(2) < 0)
	{
		ShowError("error play");
	}
}
void DLP4500::startToPro()
{
	switch (scanner_ui->comboBoxPatternType->currentIndex()) {
	case 0:
		emit phaseShifting_12();
		break;
	case 1:
		emit showWhite();
		break;
	case 2:
		emit showPattern();
	}
}

void DLP4500::stopToPro()
{
	int i = 0;
	unsigned int patMode;

	DLPC350_PatternDisplay(0);
	SleeperThread::msleep(100);
	while (1)
	{
		DLPC350_GetPatternDisplay(&patMode);
		if (patMode == 0)
			break;
		else
			DLPC350_PatternDisplay(0);
		SleeperThread::msleep(100);
		if (i++ > MAX_NUM_RETRIES)
			break;
	}
}
