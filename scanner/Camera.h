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
	IDeviceOfflineEventHandler* pDeviceOfflineEventHandler = NULL;//�����¼��ص�����
	GX_DEVICE_OFFLINE_CALLBACK_HANDLE hDeviceOffline = NULL;//ע���豸����
	IFeatureEventHandler* pFeatureEventHandler = NULL;//<Զ���豸�¼��ص�����
	ICaptureEventHandler* pCaptureEventHandler = NULL;//<�ɼ��ص�����
	CGXDevicePointer ObjDevicePtr;//�豸ָ��
	CGXStreamPointer ObjStreamPtr;//��ָ��
	CGXFeatureControlPointer objStreamFeatureControlPtr;//�����Կ�����ָ��
	CGXFeatureControlPointer ObjFeatureControlPtr;//��Զ�ˣ��豸���Կ�����ָ��
	
	cv::Mat captureImgMat;//��������ͼƬ

	QString choose_triggerMode;//ѡ�񴥷�ģʽ
	QString txt_triggerSource;//����Դ
	int txt_exposeTime;//�ع�ʱ��
	double txt_gain;//����
	QString txt_width;//ͼ���
	QString txt_height;//ͼ���
	QString txt_factoryName;//����
	QString txt_basicMode;
	bool		m_bIsOpen=false;				//���豸��ʶ
	bool		m_bIsSnap=false;				//�ɼ�ͼ���ʶ
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
	 * @brief ����ͼƬ�ĺ���
	 */
	void save();
	void setExTrigger();
	void setInTrigger();

};


//�û��̳е����¼�������
class CSampleDeviceOfflineEventHandler : public IDeviceOfflineEventHandler
{
public:
	void DoOnDeviceOfflineEvent(void* pUserParam)
	{
		std::cout << "�յ��豸�����¼�!" <<std::endl;
	}
};

//�û��̳�Զ���豸�¼�������
class CSampleCaptureEventHandler : public ICaptureEventHandler
{
public:
	void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam);
	//void Measurement(CImageDataPointer& objImageDataPointer, void* pUserParam);
};