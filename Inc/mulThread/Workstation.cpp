// Controller.cpp
#include "Workstation.h"
#include <QThread>
#include <QDebug>

#include "MyApplication.h"
#include "frmmain.h"
#include "ini/settings.h"

Workstation::Workstation(QObject *parent) : QObject(parent) {
    // frmMain *pfrmMain=(frmMain *)parent;
}

void Workstation::SettingQThread() {
    //Sample线程
    p_worker_sample = new WorkerSample();
    p_worker_sample->moveToThread(&m_thread_sample);
    connect(&m_thread_sample, &QThread::finished, p_worker_sample, &QObject::deleteLater);

    //trigger界面刷新线程，用触发器老自动停止
    p_worker_trigger = new WorkerTrigger();
    p_worker_trigger->init(m_workstation_param);
    p_worker_trigger->moveToThread(&m_thread_trigger);
    connect(&m_thread_trigger, &QThread::finished, p_worker_trigger, &QObject::deleteLater);

    //database线程
    p_worker_database = new WorkerDatabase();
    p_worker_database->init(m_workstation_param);
    p_worker_database->moveToThread(&m_thread_database);
    //绑定信号：sig_connect_db() ⬅➡ p_worker_database.connectDB()
    connect(&m_thread_database, &QThread::started, p_worker_database, &WorkerDatabase::connectDB);
    connect(&m_thread_database, &QThread::finished, p_worker_database, &QObject::deleteLater);

    //mqtt线程
    // p_worker_mqtt = new WorkerMQTT();
    // p_worker_mqtt->init(m_workstation_param);
    // p_worker_mqtt->moveToThread(&m_thread_mqtt);
    // connect(&m_thread_mqtt, &QThread::started, p_worker_mqtt, &WorkerMQTT::startConnectMqtt);
    // connect(&m_thread_mqtt, &QThread::finished, p_worker_mqtt, &QObject::deleteLater);

    //相机线程
    p_worker_cam = new WorkerCamera(m_workstation_param.camIp);
    p_worker_cam->initParam(m_workstation_param);
    p_worker_cam->moveToThread(&m_thread_camera);
    connect(&m_thread_camera, &QThread::started, p_worker_cam, &WorkerCamera::GrabFrame);
    // connect(&m_thread_camera, &QThread::started, p_worker_cam, &WorkerCamera::GrabFrameDebug); //Debug

    //模型管理类
    auto &modelManager = ModelManager::getInstance();
    modelManager.initDetModel(m_workstation_param.det_model_path, m_workstation_param.det_param_path,
                              ImgProcessTool::FindDevice("gpu"));
    modelManager.initOCRModel(m_workstation_param.ocr_det_1_path, m_workstation_param.ocr_det_2_path,
                              m_workstation_param.ocr_recog_2_path,
                              ImgProcessTool::FindDevice("cpu"));

    //图像处理线程
    p_worker_image_process = new WorkerImageProcess(m_workstation_param.m_location, p_worker_cam);
    p_worker_image_process->init(m_workstation_param);
    p_worker_image_process->moveToThread(&m_thread_imageProcess);

    //工位类的触发信号绑定trigger的工作函数
    connect(this, &Workstation::start_loop_trigger, p_worker_trigger, &WorkerTrigger::on_doSomething);
    //trigger的mqtt触发信号绑定mqtt的工作函数
    // connect(p_worker_trigger, &WorkerTrigger::sig_mqtt_trigger, p_worker_mqtt, &WorkerMQTT::mqtt_public_mydata);
    //todo:将处理mqtt纠错数据的线程里的槽绑定到mqtt上
    //相机类触发信号绑定图像处理类工作函数
    connect(p_worker_cam, &WorkerCamera::sig_imgProcess, p_worker_image_process, &WorkerImageProcess::imgProcess,
            Qt::QueuedConnection);
    //相机类触发信号绑定图像处理类配置窗口句柄函数
    connect(p_worker_cam, &WorkerCamera::sig_setHandle, p_worker_image_process, &WorkerImageProcess::SetWinHandle,
            Qt::QueuedConnection);

    //启动线程
    m_thread_sample.start();
    m_thread_trigger.start();
    m_thread_database.start();
    m_thread_mqtt.start();
    m_thread_imageProcess.start();
    m_thread_camera.start();
}

Workstation::~Workstation() {
    MyApplication *pApp = (MyApplication *) qApp;

    //触发进程停止
    pApp->isStopTrigger = true;
    m_thread_trigger.quit();
    m_thread_trigger.wait();

    m_thread_sample.quit();
    m_thread_sample.wait();

    m_thread_database.quit();
    m_thread_database.wait();

    Sleep(100);
}

void Workstation::StartTrigger(void) {
    emit start_loop_trigger();
}

void Workstation::Init(QString iniSessionName) {
    m_iniSessionName = iniSessionName;
    ReadSetting();
    SettingQThread();
}

// 读取配置文件（从setting.ini加载参数到setting结构体）
int Workstation::ReadSetting(void) {
    // 创建INI配置文件处理对象（指定配置文件为setting.ini）
    IniSettings setting_ini("setting.ini");

    // 0.工位信息
    m_workstation_param.m_serialNumber = setting_ini.getValue(m_iniSessionName, "serialNumber").toInt();
    m_serialNumber = m_workstation_param.m_serialNumber;
    m_workstation_param.m_location = setting_ini.getValue(m_iniSessionName, "location");
    m_location = m_workstation_param.m_location;
    m_workstation_param.m_loop_refresh_ms = setting_ini.getValue(m_iniSessionName, "loop_refresh_ms").toInt();

    // 1.读取相机配置
    m_workstation_param.camIp = setting_ini.getValue(m_iniSessionName, "front_ip_addr"); // 相机ip
    m_workstation_param.camExp = setting_ini.getValue(m_iniSessionName, "front_expTime").toInt(); // 曝光时间
    m_workstation_param.camGain = setting_ini.getValue(m_iniSessionName, "front_gain").toInt(); // 增益
    m_workstation_param.camFPS = setting_ini.getValue(m_iniSessionName, "front_frameRate").toInt(); // 帧率
    m_workstation_param.camImgWidth = setting_ini.getValue(m_iniSessionName, "front_camera_res_width").toInt(); // 图像宽
    m_workstation_param.camImgHeight = setting_ini.getValue(m_iniSessionName, "front_camera_res_height").toInt(); // 图像高
    m_workstation_param.test_img_path = setting_ini.getValue(m_iniSessionName, "test_img_filename"); // 图像高

    // 2.图像处理配置
    // m_workstation_param.device = setting_ini.getValue(m_iniSessionName, "device");
    m_workstation_param.run_mode = setting_ini.getValue(m_iniSessionName, "mode").toInt();
    m_workstation_param.ocrBufferPath = setting_ini.getValue(m_iniSessionName, "ocrBufferPath");
    m_workstation_param.detBufferPath = setting_ini.getValue(m_iniSessionName, "detBufferPath");
    m_workstation_param.remotePathPrefix = setting_ini.getValue(m_iniSessionName, "remotePath");

    m_workstation_param.roi_region = setting_ini.getValue(m_iniSessionName, "roi_region");
    m_workstation_param.det_model_path = setting_ini.getValue(m_iniSessionName, "det_model");
    m_workstation_param.det_param_path = setting_ini.getValue(m_iniSessionName, "det_param");
    m_workstation_param.ocr_det_1_path = setting_ini.getValue(m_iniSessionName, "ocr_det_1");
    m_workstation_param.ocr_det_2_path = setting_ini.getValue(m_iniSessionName, "ocr_det_2");
    m_workstation_param.ocr_recog_2_path = setting_ini.getValue(m_iniSessionName, "ocr_recog_2");
    m_workstation_param.affine_x_pre = setting_ini.getValue(m_iniSessionName, "affine_x_pre");
    m_workstation_param.affine_y_pre = setting_ini.getValue(m_iniSessionName, "affine_y_pre");
    m_workstation_param.affine_x_post = setting_ini.getValue(m_iniSessionName, "affine_x_post");
    m_workstation_param.affine_y_post = setting_ini.getValue(m_iniSessionName, "affine_y_post");
    m_workstation_param.max_age = setting_ini.getValue(m_iniSessionName, "max_age");
    m_workstation_param.min_hits = setting_ini.getValue(m_iniSessionName, "min_hits");
    m_workstation_param.iouThreshold = setting_ini.getValue(m_iniSessionName, "iouThreshold");

    // 3.读取数据库配置
    m_workstation_param.dbType = setting_ini.getValue(m_iniSessionName, "DbType"); // 数据库类型
    m_workstation_param.dbConnName = setting_ini.getValue(m_iniSessionName, "dbConnName"); // 连接名称
    m_workstation_param.dbName = setting_ini.getValue(m_iniSessionName, "DbName"); // 数据库名
    m_workstation_param.dbHostName = setting_ini.getValue(m_iniSessionName, "HostName"); // 主机地址
    m_workstation_param.dbHostPort = setting_ini.getValue(m_iniSessionName, "HostPort").toInt(); // 通信端口
    m_workstation_param.dbUserName = setting_ini.getValue(m_iniSessionName, "UserName"); // 用户名称
    m_workstation_param.dbUserPwd = setting_ini.getValue(m_iniSessionName, "UserPwd"); // 用户密码
    m_workstation_param.dbTableName = setting_ini.getValue(m_iniSessionName, "TableName"); // 数据库表名
    m_workstation_param.dbKeyName = setting_ini.getValue(m_iniSessionName, "KeyName"); // 数据库表的字段名

    // 4.读取MQTT配置
    m_workstation_param.mqttUse = setting_ini.getValue(m_iniSessionName, "mqtt_use").toInt(); // 是否启用MQTT
    m_workstation_param.mqttTriggerTime = setting_ini.getValue(m_iniSessionName, "mqtt_trigger_time").toInt(); // 触发时间
    m_workstation_param.mqttHostname = setting_ini.getValue(m_iniSessionName, "mqtt_hostname"); // 服务器主机名
    m_workstation_param.mqttPort = setting_ini.getValue(m_iniSessionName, "mqtt_port").toInt(); // 服务器端口
    m_workstation_param.mqttSaveImagePath = setting_ini.getValue(m_iniSessionName, "mqtt_save_image_path"); // 图像保存路径
    m_workstation_param.mqttImageCycleNum = setting_ini.getValue(m_iniSessionName, "mqtt_image_cycle_num").toInt();
    // 图像循环数量
    m_workstation_param.mqttPublicMsg = setting_ini.getValue(m_iniSessionName, "mqtt_public_msg"); // 收识别错误结果的主题

    return 0; // 读取成功返回0
}
