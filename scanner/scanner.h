#pragma once
#include <QtWidgets/QWidget>
#include <GalaxyIncludes.h>
#include <QTimer>
#include <QMessageBox>
#include <vtkRenderWindow.h>

#include "Camera.h"
#include "DLP4500.h"
#include "ui_scanner.h"
#include "CloudPoint.h"
class scanner : public QWidget
{
    Q_OBJECT

public:
    scanner(QWidget* parent = nullptr);
    ~scanner();
    void ShowError(const char* str);

private:
    Ui::scannerClass* ui;
    Camera* camera;
    CloudPoint* cloud;
   
    //关闭窗口事件重写
    void closeEvent(QCloseEvent* event);
private:
    DLP4500* dlp;
    QTimer* m_usbPollTimer;

};
