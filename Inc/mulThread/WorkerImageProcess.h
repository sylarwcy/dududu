//
// Created by sylar on 25-10-24.
//

#ifndef IMAGEPROCESSMANAGER_H
#define IMAGEPROCESSMANAGER_H

#include <QObject>
#include <QMutex>
#include "ImgProGlobal.h"
#include "WorkerCamera.h"
#include "workStationDataStructure.h"

class WorkerImageProcess : public QObject {
    Q_OBJECT

public:
    explicit WorkerImageProcess(const QString& processorType, WorkerCamera* workerCamera, QObject *parent = nullptr);
    // explicit WorkerImageProcess(const QString& processorType, QObject *parent = nullptr);
    ~WorkerImageProcess();
    void init(const WorkStation_DATA &paramData);

//************************************************************************
public slots:
    void imgProcess(const HalconCpp::HObject &image);
    void SetWinHandle(const HalconCpp::HTuple &pro);
    void SaveImg(const HObject &image, const DetRes &detRes);
    void SaveImg(const std::vector<ocrInfo>& ocrRes);
    // void SaveImgSrc(const HObject &image);  //存det对应的原图
    void SaveSrcImg(const HObject& image); //存全帧率的原图
//************************************************************************

signals:
    // 处理完成信号
    // void imageProcessed();
    // 处理错误信号
    // void imageProcessError();

private:
    HTuple winHandle_pro;
    ImageProcessor* m_processor;  // 图像处理器
    WorkerCamera* m_workerCamera;
    // QMutex m_processorMutex;
    int m_counter = 0;

    // 创建处理器的工厂方法
    ImageProcessor* createProcessor(const QString& type);
};


#endif //IMAGEPROCESSMANAGER_H
