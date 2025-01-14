#include "CloudPoint.h"

CloudPoint::CloudPoint(Ui::scannerClass* ui,QObject *parent)
	: QObject(parent),
	cloud_ui(ui)
{
}

CloudPoint::~CloudPoint()
{}

void CloudPoint::ShowCloudPoint()
{
	//cloud_cloudpoint.reset();  // ʹ������ָ��ʱ������ reset() �ͷ���Դ
	cloud_viewer->updatePointCloud<pcl::PointXYZ>(cloud_cloudpoint, "cloud");
	cloud_viewer->resetCamera();
	cloud_ui->qvtkWidget->update();
	cloud_ui->label_6->setText("Cloud Point");
}

void CloudPoint::initialVtkWidget()
{
	//���Ƽ���
	cloud_cloudpoint.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	cloud_viewer->addPointCloud<pcl::PointXYZ>(cloud_cloudpoint, "cloud");
	cloud_viewer->setBackgroundColor(0, 0, 0);
	cloud_ui->qvtkWidget->SetRenderWindow(cloud_viewer->getRenderWindow());
	cloud_viewer->setupInteractor(cloud_ui->qvtkWidget->GetInteractor(), cloud_ui->qvtkWidget->GetRenderWindow());
	cloud_ui->qvtkWidget->update();
}

