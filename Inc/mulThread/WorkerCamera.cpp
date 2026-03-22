//
// Created by sylar on 25-10-22.
//

#include "WorkerCamera.h"
#include <mmsystem.h>
#include <QElapsedTimer>
#include "MyApplication.h"

WorkerCamera::WorkerCamera(const QString &ip, QObject *parent)
    : QObject{parent} {
    m_cameraIP = ip;
    cam = new MyCameraGigE(m_cameraIP);
    camIndex = -1;
    m_imageProcessingReady = true; // 初始状态为就绪
}

void WorkerCamera::initParam(const WorkStation_DATA &paramData) {
    m_runMode = paramData.run_mode;
    ocrBufferPath = paramData.ocrBufferPath;
    detBufferPath = paramData.detBufferPath;
    m_location = paramData.m_location;

    if (DirExistEx(ocrBufferPath+"/dataSet"))
        QLOG_INFO() << QString("%1路径创建<成功>").arg(ocrBufferPath);
    else
        QLOG_ERROR() << QString("%1路径创建<失败>").arg(ocrBufferPath);
    if (DirExistEx(ocrBufferPath+"/localBuffer"))
        QLOG_INFO() << QString("%1路径创建<成功>").arg(detBufferPath);
    else
        QLOG_ERROR() << QString("%1路径创建<失败>").arg(detBufferPath);
    if (DirExistEx(detBufferPath+"/src"))
        QLOG_INFO() << QString("%1路径创建<成功>").arg(detBufferPath);
    else
        QLOG_ERROR() << QString("%1路径创建<失败>").arg(detBufferPath);
    if (DirExistEx(detBufferPath+"/render"))
        QLOG_INFO() << QString("%1路径创建<成功>").arg(detBufferPath);
    else
        QLOG_ERROR() << QString("%1路径创建<失败>").arg(detBufferPath);
    if (DirExistEx(detBufferPath+"/fullFPS"))
        QLOG_INFO() << QString("%1路径创建<成功>").arg(detBufferPath);
    else
        QLOG_ERROR() << QString("%1路径创建<失败>").arg(detBufferPath);

    m_camImgWidth = paramData.camImgWidth;
    m_camImgHeight = paramData.camImgHeight;
    m_test_img_path = paramData.test_img_path;
}

void WorkerCamera::SetHandle(HTuple &ori, HTuple &pro) {
    m_winHandle_ori = ori;
    m_winHandle_pro = pro;
    cam->SetHandle(m_winHandle_ori, m_winHandle_pro);
    emit sig_setHandle(m_winHandle_pro);
}

void WorkerCamera::reconnect() {
    if (cam->m_bOpenDevice)
        cam->m_pcMyCamera->Close();
    while (true) {
        cam->FindCameraGigE();
        bool openRes = cam->OpenCamera();
        if (openRes) {
            cam->m_pcMyCamera->RegisterExceptionCallBack(MyCameraGigE::ReconnectDevice, cam);
            cam->m_pcMyCamera->StartGrabbing();
            break;
        } else {
            cam->m_bOpenDevice = false;
            QThread::msleep(100);
        }
    }
}

void WorkerCamera::GrabFrameDebug() {
    MyApplication *pApp = (MyApplication *) qApp;

    QString str;
    QElapsedTimer cycle_time;

    MV_FRAME_OUT stImageInfo = {0};
    int nRet = MV_OK;

    HObject ho_image;
    HObject ho_imageCrop;
    HObject ho_imageScale;
    HObject ho_ImageScaleMax;
    HTuple hv_height, hv_width;
    int nHeight, nWidth;


    QString imgPathPrefix = "test/";
    QString imgPathPostfix = ".jpg";
    int i = 1;
    while (true) {
        cycle_time.start();
        // if (m_location == "stagingSouth")
        //     ReadImage(&ho_image, "test_south.jpg");
        // else
        //     ReadImage(&ho_image, "test_north.jpg");
        if (i==106) i=1;
        ReadImage(&ho_image, (imgPathPrefix+QString::number(i++)+imgPathPostfix).toStdString().c_str());

        //重新获取图像尺寸
        GetImageSize(ho_image, &hv_width, &hv_height);
        nWidth = hv_width[0];
        nHeight = hv_height[0];

        ++frameNumTotal;

        {
            QMutexLocker locker(&m_processingMutex);
            if (m_imageProcessingReady) {
                // 设置为处理中状态
                m_imageProcessingReady = false;
                // 发送图像进行处理
                emit sig_imgProcess(ho_image); //发送信号由图像处理线程进行识别
            } else {
                // 图像处理线程繁忙，丢弃当前帧
                qDebug() << "图像处理线程繁忙，丢弃帧号:" << frameNumTotal;
            }
        }

        //显示原始图像
        if (!HtupleIsEmpty(m_winHandle_ori)) {
            if (HDevWindowStack::IsOpen()) {
                SetPart(m_winHandle_ori, 0, 0, nHeight, nWidth);
                DispObj(ho_image, m_winHandle_ori);
            }
        }

        //等待时间
        timeBeginPeriod(1);
        Sleep(pApp->pNodeData->setting.sample_period);
        timeEndPeriod(1);

        //总循环时间计算
        cam->thread_cycle_ms = cycle_time.elapsed();
    }
}

void WorkerCamera::GrabFrame() {
    MyApplication *pApp = (MyApplication *) qApp;

    QString str;
    QElapsedTimer cycle_time;

    MV_FRAME_OUT stImageInfo = {0};
    int nRet = MV_OK;

    HObject ho_image;
    HObject ho_imageCrop;
    HObject ho_imageScale;
    HObject ho_ImageScaleMax;
    HTuple hv_height, hv_width;
    int nHeight, nWidth;

    if (!cam->CheckStatus())
        reconnect();

    while (cam->m_bThreadState) {
        cycle_time.start();

        //非测试模式处理
        nRet = cam->m_pcMyCamera->GetImageBuffer(&stImageInfo, 1000);

        if (nRet == MV_OK) {
            //用于保存图片
            EnterCriticalSection(&cam->m_hSaveImageMux);
            if (NULL == cam->m_pSaveImageBuf || stImageInfo.stFrameInfo.nFrameLen > cam->m_nSaveImageBufSize) {
                if (cam->m_pSaveImageBuf) {
                    free(cam->m_pSaveImageBuf);
                    cam->m_pSaveImageBuf = NULL;
                }

                cam->m_pSaveImageBuf = (unsigned char *) malloc(
                    sizeof(unsigned char) * stImageInfo.stFrameInfo.nFrameLen);
                if (cam->m_pSaveImageBuf == NULL) {
                    LeaveCriticalSection(&cam->m_hSaveImageMux);
                    return;
                }
                cam->m_nSaveImageBufSize = stImageInfo.stFrameInfo.nFrameLen;
            }
            memcpy(cam->m_pSaveImageBuf, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nFrameLen);
            memcpy(&cam->m_stImageInfo, &(stImageInfo.stFrameInfo), sizeof(MV_FRAME_OUT_INFO_EX));
            LeaveCriticalSection(&cam->m_hSaveImageMux);

            //转换为halcon格式,返回ho_image,nWidth,nHeight
            ImgToHObject(stImageInfo, ho_image, nWidth, nHeight);

            //重新获取图像尺寸
            GetImageSize(ho_image, &hv_width, &hv_height);
            nWidth = hv_width[0];
            nHeight = hv_height[0];

            ++frameNumTotal;

            {
                QMutexLocker locker(&m_processingMutex);
                if (m_imageProcessingReady) {
                    // 设置为处理中状态
                    m_imageProcessingReady = false;
                    // 发送图像进行处理
                    HObject ho_image_copy;
                    CopyImage(ho_image,&ho_image_copy);
                    emit sig_imgProcess(ho_image_copy); //发送信号由图像处理线程进行识别
                }
                // else {
                //     // 图像处理线程繁忙，丢弃当前帧
                //     qDebug() << "图像处理线程繁忙，丢弃帧号:" << frameNumTotal;
                // }
            }

            //显示原始图像s
            if (!HtupleIsEmpty(m_winHandle_ori)) {
                if (HDevWindowStack::IsOpen()) {
                    SetPart(m_winHandle_ori, 0, 0, nHeight, nWidth);
                    DispObj(ho_image, m_winHandle_ori);
                }
            }

            cam->m_pcMyCamera->FreeImageBuffer(&stImageInfo); //释放本次缓存
        } else {
            if (MV_TRIGGER_MODE_ON == cam->m_nTriggerMode)
                Sleep(5);

            QLOG_ERROR() << QString("m_pcMyCamera->GetImageBuffer函数发生错误,等待中...");
        }

        //等待时间
        timeBeginPeriod(1);
        Sleep(pApp->pNodeData->setting.sample_period);
        timeEndPeriod(1);

        //总循环时间计算
        cam->thread_cycle_ms = cycle_time.elapsed();
    }
}

// 图像处理完成槽函数
void WorkerCamera::onImageProcessFinished() {
    QMutexLocker locker(&m_processingMutex);
    m_imageProcessingReady = true;
}

// 像素格式转换函数
/*bool WorkerCamera::ConvertPixelFormat(MV_FRAME_OUT &stImageInfo, unsigned char* &pConvertedBuffer, int &nConvertedSize) {
    // 检查输入参数
    if (stImageInfo.pBufAddr == NULL) {
        QLOG_ERROR() << "转换失败：输入图像数据为空";
        return false;
    }

    // 准备转换参数
    MV_CC_PIXEL_CONVERT_PARAM stConvertPixelInfo = {0};

    // 输入图像信息
    stConvertPixelInfo.nWidth = stImageInfo.stFrameInfo.nWidth;
    stConvertPixelInfo.nHeight = stImageInfo.stFrameInfo.nHeight;
    stConvertPixelInfo.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
    stConvertPixelInfo.pSrcData = (unsigned char*)stImageInfo.pBufAddr;
    stConvertPixelInfo.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;

    // 输出图像信息
    // 计算需要的缓冲区大小（RGB8格式，每个像素3字节）
    int nOutputBufferSize = stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight * 3;

    // 检查是否需要重新分配缓冲区
    if (m_pConvertedBuffer == NULL || m_nConvertedBufferSize < nOutputBufferSize) {
        if (m_pConvertedBuffer != NULL) {
            free(m_pConvertedBuffer);
            m_pConvertedBuffer = NULL;
        }

        m_pConvertedBuffer = (unsigned char*)malloc(nOutputBufferSize);
        if (m_pConvertedBuffer == NULL) {
            QLOG_ERROR() << "转换失败：无法分配输出缓冲区";
            m_nConvertedBufferSize = 0;
            return false;
        }

        m_nConvertedBufferSize = nOutputBufferSize;
    }

    stConvertPixelInfo.enDstPixelType = PixelType_Gvsp_RGB8_Packed; // 假设有这个枚举值
    stConvertPixelInfo.nDstBufferSize = nOutputBufferSize;
    stConvertPixelInfo.pDstBuffer = m_pConvertedBuffer;

    // 执行像素格式转换
    int nRet = MV_CC_ConvertPixelType(nullptr, &stConvertPixelInfo);

    if (nRet != MV_OK) {
        QLOG_ERROR() << "像素格式转换失败，错误码:" << nRet;
        return false;
    }

    pConvertedBuffer = m_pConvertedBuffer;
    nConvertedSize = nOutputBufferSize;

    // QLOG_INFO() << QString("像素格式转换成功: %1x%2, 格式: %3 -> RGB8")
    //                    .arg(stImageInfo.stFrameInfo.nWidth)
    //                    .arg(stImageInfo.stFrameInfo.nHeight)
    //                    .arg(stImageInfo.stFrameInfo.enPixelType);

    return true;
}

// 修改后的ImgToHObject函数
void WorkerCamera::ImgToHObject(MV_FRAME_OUT &stImageInfo, HObject &ho_image, int &nWidth, int &nHeight) {


    unsigned char* pConvertedBuffer = NULL;
    int nConvertedSize = 0;

    // 进行像素格式转换
    if (ConvertPixelFormat(stImageInfo, pConvertedBuffer, nConvertedSize)) {
        // 转换成功，使用转换后的RGB数据
        nHeight = stImageInfo.stFrameInfo.nHeight;
        nWidth = stImageInfo.stFrameInfo.nWidth;

        // 分离RGB通道
        int pixelCount = nWidth * nHeight;
        HBYTE *pRed = pConvertedBuffer;
        HBYTE *pGreen = pConvertedBuffer + pixelCount;
        HBYTE *pBlue = pConvertedBuffer + pixelCount * 2;

        // 生成Halcon图像
        GenImage3(&ho_image, "byte", nWidth, nHeight,
                  (Hlong)pRed, (Hlong)pGreen, (Hlong)pBlue);

        // QLOG_INFO() << "使用SDK转换后的RGB图像创建Halcon图像";
    } else {
        // 转换失败，使用原始数据创建单通道图像
        QLOG_INFO() << "像素格式转换失败，使用原始数据";
        HBYTE *pData = (HBYTE *)stImageInfo.pBufAddr;
        nHeight = stImageInfo.stFrameInfo.nHeight;
        nWidth = stImageInfo.stFrameInfo.nWidth;

        // 创建三个相同通道的单通道图像
        GenImage3(&ho_image, "byte", nWidth, nHeight,
                  (Hlong)pData, (Hlong)pData, (Hlong)pData);
    }


    // 如果获取尺寸与设定文件不同，改变
    if (nWidth != m_camImgWidth || nHeight != m_camImgHeight) {
        QLOG_INFO() << "获取的相机图像尺寸与设定不符";
        ZoomImageSize(ho_image, &ho_image, m_camImgWidth, m_camImgHeight, "constant");
    }

    // 检查转换后图像的通道数
    HTuple hv_channels;
    try {
        CountChannels(ho_image, &hv_channels);
        QLOG_INFO() << "转换后图像通道数: " << hv_channels.I();
    }
    catch (HalconCpp::HException &e) {
        QLOG_ERROR() << "无法获取图像通道数: " << e.ErrorText();
    }
}*/

void WorkerCamera::ImgToHObject(MV_FRAME_OUT &stImageInfo, HObject &ho_image, int &nWidth, int &nHeight) {
    HBYTE *pData = (HBYTE *) stImageInfo.pBufAddr;

    //非测试模式
    nHeight = stImageInfo.stFrameInfo.nHeight;
    nWidth = stImageInfo.stFrameInfo.nWidth;

    //转换为halcon格式
    // GenImage1(&ho_image, "byte", nWidth, nHeight, (Hlong) pData);
    HObject ho_bayer_image;
    GenImage1(&ho_bayer_image, "byte", nWidth, nHeight, (Hlong)pData);
    CfaToRgb(ho_bayer_image, &ho_image, "bayer_gr", "bilinear_enhanced");

    //如果获取尺寸与设定文件不同，改变
    if (nWidth != m_camImgWidth || nHeight != m_camImgHeight) {
        QLOG_INFO() << "获取的相机图像尺寸与设定不符";
        ZoomImageSize(ho_image, &ho_image, m_camImgWidth, m_camImgHeight, "constant");
    }
}



void WorkerCamera::SingleFrameSaveToDisk(HObject &ho_image) {
    static QString processSavePath;
    bool isOK;

    //获取时间
    QDateTime time = QDateTime::currentDateTime();
    int day = time.date().day();
    int hour = time.time().hour();
    int min = time.time().minute();
    int sec = time.time().second();
    int msec = time.time().msec();

    QString saveIDPath = QString("%1/%2/%3/").arg(ocrBufferPath).arg(day).arg(hour);
    QString fileName = QString("%1_%2_%3_%4.jpg")
            .arg(hour)
            .arg(min)
            .arg(sec)
            .arg(msec);
    QString pathName = saveIDPath + fileName;

    //不存在就创建
    isOK = DirExistEx(saveIDPath);
    if (isOK)
        WriteImage(ho_image, "jpeg", 0, pathName.toStdString().c_str());
}

//给图像处理类用的
void WorkerCamera::SingleFrameSaveToDisk(HObject &ho_image, const QString &fileName) {
    WriteImage(ho_image, "jpeg", 0, fileName.toStdString().c_str());
}

bool WorkerCamera::DirExistEx(QString fullPath) {
    QDir dir(fullPath);
    if (dir.exists())
        return true;

    //不存在当前目录，创建，可创建多级目录
    bool ok = dir.mkpath(fullPath);
    return ok;
}

bool WorkerCamera::HtupleIsEmpty(HTuple &value) {
    HTuple length;
    TupleLength(value, &length);

    return length.I() == 0;
}
