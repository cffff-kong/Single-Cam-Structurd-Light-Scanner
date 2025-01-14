#pragma once

#include <QObject>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "ui_scanner.h"
class CloudPoint : public QObject
{
	Q_OBJECT

public:
	CloudPoint(Ui::scannerClass* ui, QObject* parent);
	~CloudPoint();
	Ui::scannerClass* cloud_ui;

public slots:
	void initialVtkWidget();
	void ShowCloudPoint();

public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cloudpoint;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;
};
