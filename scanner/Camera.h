#pragma once

#include <QObject>
#include <GalaxyIncludes.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ui_scanner.h"
#include <QThread>
#include <QMessageBox>
#include "SSLReconstruction.h"
class Camera  : public QObject
{
	Q_OBJECT


public:
	Camera(Ui::scannerClass *ui,QObject *parent);
	~Camera();
	IDeviceOfflineEventHandler* pDeviceOfflineEventHandler = NULL;//掉线事件回调对象
	GX_DEVICE_OFFLINE_CALLBACK_HANDLE hDeviceOffline = NULL;//注册设备掉线
	IFeatureEventHandler* pFeatureEventHandler = NULL;//<远端设备事件回调对象
	ICaptureEventHandler* pCaptureEventHandler = NULL;//<采集回调对象
	CGXDevicePointer ObjDevicePtr;//设备指针
	CGXStreamPointer ObjStreamPtr;//流指针
	CGXFeatureControlPointer objStreamFeatureControlPtr;//流属性控制器指针
	CGXFeatureControlPointer ObjFeatureControlPtr;//（远端）设备属性控制器指针
	
	cv::Mat captureImgMat;//相机拍摄的图片

	QString choose_triggerMode;//选择触发模式
	QString txt_triggerSource;//触发源
	int txt_exposeTime;//曝光时间
	double txt_gain;//增益
	QString txt_width;//图像宽
	QString txt_height;//图像高
	QString txt_factoryName;//厂商
	QString txt_basicMode;
	bool		m_bIsOpen=false;				//打开设备标识
	bool		m_bIsSnap=false;				//采集图像标识
	Ui::scannerClass * scanner_ui;
	bool triggerMode=false;
	SSLReconstruction* ssl;
private:
	

private:
	void ShowError(const char* str);
public:
	void Open_Device();//open the device
	void Close_Device();//close the device
	void Start_Snap();//start picking the picture
	void Stop_Snap();
	void setExpTime();
	//stop picking the picture
	//void auto_baoguang();
	/**
	 * @brief 保存图片的函数
	 */
	void save();
	void setExTrigger();
	void setInTrigger();

};


//用户继承掉线事件处理类
class CSampleDeviceOfflineEventHandler : public IDeviceOfflineEventHandler
{
public:
	void DoOnDeviceOfflineEvent(void* pUserParam)
	{
		std::cout << "收到设备掉线事件!" <<std::endl;
	}
};

//用户继承远端设备事件处理类
class CSampleCaptureEventHandler : public ICaptureEventHandler
{
public:
	void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam);
	//void Measurement(CImageDataPointer& objImageDataPointer, void* pUserParam);
};