#pragma once

#include <QObject>
#include <QThread>
#include <QMessageBox>
#include <GalaxyIncludes.h>
#include <QString>
#include "dlpc350_common.h"
#include "dlpc350_firmware.h"
#include "ui_scanner.h"
#include "Camera.h"
#include "SSLReconstruction.h"
#include "CloudPoint.h"
class SleeperThread : public QThread
{
public:
	static void msleep(unsigned long msecs)
	{
		QThread::msleep(msecs);
	}
};

class DLP4500 : public QObject
{
	Q_OBJECT

public:
	DLP4500(Ui::scannerClass* ui, QObject* parent, Camera* camera,CloudPoint* cloud);
	~DLP4500();
	void timerTimeout(void);
	void showDLP4500Connect();
	void setMode();
	void SetDLPC350InPatternMode();
	void stopToPro();
	void startToPro();
private:
	void ShowError(const char* str);
	int numPatterns;
	void phaseShifting_12();
	void calibrate();
	void showPattern();
	void showWhite();

public:
	Ui::scannerClass* scanner_ui;
	Camera* cam;
	CloudPoint* dlp_cloud;
private:
	CloudPoint* cloudPoint;
	
};
