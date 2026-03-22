#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <QObject>
#include <QThread>
#include "WorkerSample.h"
#include "workertrigger.h"
#include "WorkerDatabase.h"
#include "WorkerMQTT.h"
#include "workStationDataStructure.h"
#include "WorkerCamera.h"
#include "WorkerImageProcess.h"

class Workstation : public QObject
{
  Q_OBJECT

public:
    explicit Workstation(QObject *parent = nullptr);
    ~Workstation();

    void StartTrigger();
    void SettingQThread();
    void Init(QString iniSessionName);
    int ReadSetting();  // 读取配置文件（从setting.ini加载参数）

    QString m_iniSessionName; //配置文件session名
    QString m_location;       //工位位置
    int m_serialNumber;       //工位序号
    WorkStation_DATA m_workstation_param; //工位参数

    QThread m_thread_sample;
    WorkerSample *p_worker_sample;

    QThread m_thread_trigger;
    WorkerTrigger *p_worker_trigger;

    QThread m_thread_database;
    WorkerDatabase *p_worker_database;

    QThread m_thread_mqtt;
    WorkerMQTT *p_worker_mqtt;

    QThread m_thread_camera;
    WorkerCamera *p_worker_cam;  //相机指针

    QThread m_thread_imageProcess;
    WorkerImageProcess *p_worker_image_process; //图像处理

signals:
    //界面刷新开始
    void start_loop_trigger();
};
#endif // CONTROLLER_H
