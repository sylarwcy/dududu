//
// Created by sylar on 25-10-22.
//

#ifndef WORKERCAMERA_H
#define WORKERCAMERA_H
#include <QsLog.h>
#include <QDebug>
#include "myCameraGigE.h"
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include "workStationDataStructure.h"

// #include <QMetaType>
// Q_DECLARE_METATYPE(HObject)
// Q_DECLARE_METATYPE(HTuple)

class WorkerCamera : public QObject {
    Q_OBJECT

public:
    explicit WorkerCamera(const QString &ip, QObject *parent = nullptr);

    ~WorkerCamera() {
        // if (m_pConvertedBuffer != NULL) {
        //     free(m_pConvertedBuffer);
        //     m_pConvertedBuffer = NULL;
        // }
    };

    void reconnect();

    void initParam(const WorkStation_DATA &param); //初始化相机
    void SetHandle(HTuple &ori, HTuple &pro);

    int camIndex;
    QString m_cameraIP;
    QString m_location;
    HTuple m_winHandle_ori;
    HTuple m_winHandle_pro;
    int m_camImgWidth;             // 图像宽
    int m_camImgHeight;            // 图像高
    QString m_test_img_path;       // 测试图路径
    QThread m_thread_openCamera;
    MyCameraGigE *cam; //相机类

    static bool HtupleIsEmpty(HTuple &value);
    void ImgToHObject(MV_FRAME_OUT &stImageInfo, HObject &ho_image, int &nWidth, int &nHeight);
    bool ConvertPixelFormat(MV_FRAME_OUT &stImageInfo, unsigned char* &pConvertedBuffer, int &nConvertedSize);
    static bool DirExistEx(QString fullPath);
    void SingleFrameSaveToDisk(HObject &ho_image);
    static void SingleFrameSaveToDisk(HObject &ho_image, const QString &fileName);

    int m_runMode;  //存图-0，识别-1
    QString ocrBufferPath;
    QString detBufferPath;
    uint frameNumTotal;

signals:
    void sig_imgProcess(const HalconCpp::HObject &image);
    void sig_setHandle(const HalconCpp::HTuple &pro);

public slots:
    void GrabFrame();
    void GrabFrameDebug();
    void onImageProcessFinished(); // 图像处理完成槽函数
private:
    bool m_imageProcessingReady; // 图像处理线程是否就绪
    QMutex m_processingMutex;    // 处理状态互斥锁
    bool m_bIsErrorLogged;
    // unsigned char* m_pConvertedBuffer;  // 转换后的缓冲区
    // int m_nConvertedBufferSize;         // 转换缓冲区大小
    // bool m_bConversionReady;            // 转换是否就绪
};


#endif //WORKERCAMERA_H
