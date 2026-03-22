//
// Created by sylar on 25-10-24.
//

#include "WorkerImageProcess.h"

#include <QDateTime>

#include "myCameraGigE.h"
#include "QsLog.h"
#include "WorkerCamera.h"

WorkerImageProcess::WorkerImageProcess(const QString &processorType, WorkerCamera *workerCamera, QObject *parent)
    : QObject(parent), m_processor(nullptr), m_workerCamera(workerCamera) {
    m_processor = createProcessor(processorType);
    if (m_processor)
        QLOG_INFO() << QString("WorkerImageProcess created with processor: %1").arg(processorType);
    else
        QLOG_ERROR() << QString("Failed to create processor for type: %1").arg(processorType);
}

WorkerImageProcess::~WorkerImageProcess() {
    // QMutexLocker locker(&m_processorMutex);
    if (m_processor) {
        delete m_processor;
        m_processor = nullptr;
    }
}

void WorkerImageProcess::init(const WorkStation_DATA &paramData) {
    if (!m_processor) {
        QLOG_ERROR() << "Processor is null, cannot initialize";
        return;
    }
    // 将字符串参数转换为HTuple
    HTuple hv_x_pre, hv_y_pre, hv_x_post, hv_y_post;

    // 解析逗号分隔的字符串为HTuple
    QStringList x_pre_list = paramData.affine_x_pre.split(',');
    for (const QString &val: x_pre_list)
        hv_x_pre.Append(val.toDouble());

    QStringList y_pre_list = paramData.affine_y_pre.split(',');
    for (const QString &val: y_pre_list)
        hv_y_pre.Append(val.toDouble());

    QStringList x_post_list = paramData.affine_x_post.split(',');
    for (const QString &val: x_post_list)
        hv_x_post.Append(val.toDouble());

    QStringList y_post_list = paramData.affine_y_post.split(',');
    for (const QString &val: y_post_list)
        hv_y_post.Append(val.toDouble());

    std::vector<int> region;

    QStringList gundao_region_list = paramData.roi_region.split(',');
    for (const QString &val: gundao_region_list)
        region.emplace_back(val.toInt());

    QLOG_INFO() << "start init m_processor：" << m_processor->className;

    // m_processor->m_DeviceType = paramData.device;
    m_processor->winHandle_pro = winHandle_pro;
    m_processor->ocrBufferPath = paramData.ocrBufferPath;
    m_processor->detBufferPath = paramData.detBufferPath;
    m_processor->remotePath = paramData.remotePathPrefix;
    m_processor->m_serialNumber = paramData.m_serialNumber;
    m_processor->init(
        hv_x_pre, hv_y_pre, hv_x_post, hv_y_post,
        region,
        paramData.max_age.toInt(), paramData.min_hits.toInt(), paramData.iouThreshold.toDouble()
    );

    QLOG_INFO() << "finish init m_processor：" << m_processor->className;
}

ImageProcessor *WorkerImageProcess::createProcessor(const QString &type) {
    if (type.toLower() == "hotcharging" || type.contains("hot", Qt::CaseInsensitive))
        return new HotChargingImgPro(this);
    if (type.toLower() == "stackertable" || type.contains("stacker", Qt::CaseInsensitive))
        return new StackerTableImgPro(this);
    // if (type.toLower() == "stagingnorth" || type.contains("north", Qt::CaseInsensitive))
    //     return new StagingNorthImgPro(this);
    // if (type.toLower() == "stagingsouth" || type.contains("south", Qt::CaseInsensitive))
    //     return new StagingSouthImgPro(this);
    // 可以继续添加其他类型匹配
    // else if (type.toLower() == "othertype") {
    //     return new OtherImgPro(this);
    // }

    QLOG_WARN() << QString("Unknown processor type: %1, using default display").arg(type);
    return nullptr;
}

void WorkerImageProcess::imgProcess(const HalconCpp::HObject &image) {
    QElapsedTimer cycle_time;
    HObject affineImage;
    // if (m_processor->className =="HotCharging") return;
    // if (m_processor->className =="StackerTable") return;

    // std::vector<ocrInfo> ocrValues;
    // ocrInfo ocrValue{"AB12345Q355",cv::Point{1050,630},160,20,0};
    // ocrValues.emplace_back(ocrValue);
    // HObject img;
    // ReadImage(&img,"test_bug.jpg");

    if (m_processor) {
        cycle_time.start();
        m_processor->AffineImg(image, affineImage);
        // QLOG_INFO() << m_processor->className << "affine-耗时：" << cycle_time.elapsed() << " ms";
        auto detRes = m_processor->detInfer(affineImage);
        // QLOG_INFO() <<"1";
        // DetRes detRes;
        // QLOG_INFO() << m_processor->className << "det-耗时：" << cycle_time.elapsed() << " ms";
        auto ocrRes = m_processor->ocrInfer(affineImage, m_processor->className);
        // QLOG_INFO() <<"2";
        // std::vector<ocrInfo> ocrRes;
        // QLOG_INFO() << m_processor->className << "ocr-耗时：" << cycle_time.elapsed() << " ms";
        // ImgProcessTool::PreProcessOCRRes(ocrValues);
        ImgProcessTool::PreProcessOCRRes(ocrRes);
        // 使用指定的处理器进行处理
        // QLOG_INFO() << m_processor->className << "imgprocess-耗时：" << cycle_time.elapsed() << " ms";
        // QLOG_INFO() <<"3";
        if (m_counter++ % 4 == 0) {
            SaveImg(affineImage, detRes);
            // SaveImg(ocrRes);
            // SaveSrcImg(affineImage);
            m_counter = 1;
        }

        // 界面显示图像
        HTuple nWidth, nHeight;
        HObject ROI_0, ImageReduced, ImagePart;
        GenRectangle1(&ROI_0, 0, 0, 900, 2000);
        ReduceDomain(affineImage, ROI_0, &ImageReduced);
        CropDomain(ImageReduced, &ImagePart);
        GetImageSize(ImagePart, &nWidth, &nHeight);

        m_processor->processImage(ImagePart, detRes, ocrRes);
        // m_processor->processImage(ImagePart, detRes, ocrValues);

        if (!WorkerCamera::HtupleIsEmpty(winHandle_pro)) {
            HDevWindowStack::SetActive(winHandle_pro);
            if (HDevWindowStack::IsOpen()) {
                //解决刷新问题
                SetSystem("flush_graphic", "false");
                // 设置背景色为黑色
                SetWindowParam(winHandle_pro, "background_color", "black");
                // 清除窗口
                ClearWindow(winHandle_pro);
                SetPart(winHandle_pro, 0, 0, nHeight-1, nWidth-1);
                DispObj(ImagePart, winHandle_pro);

                // 画框
                // 设置绘制参数
                SetDraw(winHandle_pro, "margin"); // 仅显示轮廓
                SetLineWidth(winHandle_pro, 2); // 线宽2像素
                SetColor(winHandle_pro, "green"); // 框颜色为绿色

                // 获取检测结果
                const std::vector<DetectionResult> &detections = detRes.getOriginalResults();

                // 遍历所有检测结果并绘制矩形框
                for (const auto &detection: detections) {
                    if (m_processor->className == "HotCharging")
                        if (detection.label != "hot")
                            continue;

                    if (m_processor->className == "StackerTable")
                        if (detection.label != "table")
                            continue;

                    // 从cv::Rect转换为Halcon矩形参数
                    HTuple hv_Row1 = detection.boundingBox.y;
                    HTuple hv_Column1 = detection.boundingBox.x;
                    HTuple hv_Row2 = detection.boundingBox.y + detection.boundingBox.height;
                    HTuple hv_Column2 = detection.boundingBox.x + detection.boundingBox.width;

                    // 生成矩形区域
                    HObject ho_Rectangle;
                    GenRectangle1(&ho_Rectangle, hv_Row1, hv_Column1, hv_Row2, hv_Column2);

                    // 显示矩形框
                    DispObj(ho_Rectangle, winHandle_pro);

                    // 可选：在框旁边显示置信度
                    // HTuple hv_Confidence = detection.confidence;
                    HTuple hv_TextRow = hv_Row1; // 在框上方显示
                    HTuple hv_TextCol = hv_Column1;
                    // HTuple hv_Text = "Conf: " + (hv_Confidence * 100).TupleString(".1f") + "%";
                    HTuple hv_Text = detection.label.toStdString().c_str();

                    // 设置文本颜色和大小
                    // SetColor(winHandle_pro, "yellow");
                    SetFont(winHandle_pro, "default-Bold-12");

                    // 显示文本
                    DispText(winHandle_pro, hv_Text, "image",
                             hv_TextRow, hv_TextCol, "red", "box", "false");
                }

                SetSystem("flush_graphic", "true");
            }
        }
    }

    m_workerCamera->onImageProcessFinished();
    // qDebug() << "-----------------------------------------------------------------------------------------------";
}

void WorkerImageProcess::SetWinHandle(const HalconCpp::HTuple &pro) {
    winHandle_pro = pro;
}

void WorkerImageProcess::SaveSrcImg(const HObject &image) {
    //获取时间
    QDateTime time = QDateTime::currentDateTime();
    int day = time.date().day();
    int hour = time.time().hour();
    int min = time.time().minute();
    int sec = time.time().second();
    int msec = time.time().msec();

    QString srcPathPrefix = QString("%1/fullFPS/").arg(m_processor->detBufferPath);
    // QString fileName = QString("%1_%2_%3_%4_%5.jpg")
    //         .arg(day)
    //         .arg(hour)
    //         .arg(min)
    //         .arg(sec)
    //         .arg(msec);
    QString fileName = QString::asprintf("%02d_%02d_%02d_%02d_%03d.jpg",
                                         day, hour, min, sec, msec);

    QString srcPathName = srcPathPrefix + fileName;

    WriteImage(image, "jpeg", 0, srcPathName.toStdString().c_str());
    // WriteImage(image, "bmp", 0, srcPathName.toStdString().c_str());
}


void WorkerImageProcess::SaveImg(const HObject &image, const DetRes &detRes) {
    if (detRes.getOriginalRects().empty()) return;

    //获取时间
    QDateTime time = QDateTime::currentDateTime();
    int day = time.date().day();
    int hour = time.time().hour();
    int min = time.time().minute();
    int sec = time.time().second();
    int msec = time.time().msec();

    QString srcPathPrefix = QString("%1/src/").arg(m_processor->detBufferPath);
    QString renderPathPrefix = QString("%1/render/").arg(m_processor->detBufferPath);
    // QString fileName = QString("%1_%2_%3_%4_%5.jpg")
    //         .arg(day)
    //         .arg(hour)
    //         .arg(min)
    //         .arg(sec)
    //         .arg(msec);
    QString fileName = QString::asprintf("%02d_%02d_%02d_%02d_%03d.jpg",
                                         day, hour, min, sec, msec);
    QString srcPathName = srcPathPrefix + fileName;
    QString renderPathName = renderPathPrefix + fileName;

    WriteImage(image, "jpeg", 0, srcPathName.toStdString().c_str());

    HObject imgCopy;
    CopyImage(image, &imgCopy);
    std::vector<cv::Rect_<int> > tmp = detRes.getOriginalRects();
    for (auto res: tmp) {
        HObject region;
        // res.boundingBox.y = res.boundingBox.y < 0 ? 0 : res.boundingBox.y;
        // res.boundingBox.x = res.boundingBox.x < 0 ? 0 : res.boundingBox.x;
        // res.boundingBox.height = (res.boundingBox.y + res.boundingBox.height) > 900
        //                              ? 900 - res.boundingBox.y
        //                              : res.boundingBox.height;
        // res.boundingBox.width = (res.boundingBox.x + res.boundingBox.width) > 2000
        //                             ? 2000 - res.boundingBox.x
        //                             : res.boundingBox.width;
        GenRectangle1(&region, res.y, res.x, res.y + res.height,
                      res.x + res.width);
        PaintRegion(region, imgCopy, &imgCopy, ((HTuple(255).Append(0)).Append(0)),
                    "margin");
    }
    WriteImage(imgCopy, "jpeg", 0, renderPathName.toStdString().c_str());
}

void WorkerImageProcess::SaveImg(const std::vector<ocrInfo> &ocrRes) {
    if (ocrRes.empty()) return;

    //获取时间
    QDateTime time = QDateTime::currentDateTime();
    int day = time.date().day();
    int hour = time.time().hour();
    int min = time.time().minute();
    int sec = time.time().second();
    int msec = time.time().msec();
    QString fileName_prefix = QString::asprintf("%02d_%02d_%02d_%02d_%03d",
                                                day, hour, min, sec, msec);
    QString srcPathPrefix = QString("%1/dataSet/").arg(m_processor->ocrBufferPath);

    for (auto res: ocrRes) {
        if (!res.isValid) {
            QString fileName = QString("%1_%2.jpg").arg(fileName_prefix).arg(res.word);
            QString srcPathName = srcPathPrefix + fileName;
            WriteImage(res.img, "jpeg", 0, srcPathName.toStdString().c_str());
        }
    }
}
