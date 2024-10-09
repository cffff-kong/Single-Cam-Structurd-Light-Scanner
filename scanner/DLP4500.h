#pragma once

#include <QObject>
#include "dlpc350_common.h"
#include "dlpc350_firmware.h"
#include <QThread>
#include "ui_scanner.h"
#include <QMessageBox>
#include <Camera.h>
#include <GalaxyIncludes.h>
#include <QThread>
#include <QString>

class SleeperThread : public QThread
{
public:
	static void msleep(unsigned long msecs)
	{
		QThread::msleep(msecs);
	}
};

class DLP4500  : public QObject
{
	Q_OBJECT

public:
	DLP4500(Ui::scannerClass* ui,QObject *parent,Camera *camera);
	~DLP4500();

	Ui::scannerClass* scanner_ui;
	Camera* cam;
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

	
	

};
