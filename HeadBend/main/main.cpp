#include "frmmain.h"
#include "appinit.h"
#include "appconfig.h"
#include "quihelper.h"

#include "MyApplication.h"

#include <QSystemSemaphore>
#include <QSharedMemory>

#include "imgAI.h"
#include "myCameraGigE.h"

#define display true

void testFunction();

int main(int argc, char* argv[])
{
    //****************************************************************************************************
    // testFunction();
    //****************************************************************************************************
    // HObject img;
    // ReadImage(&img,"test.jpg");
    // WriteImage(img,"jpeg",0,"\\\\192.168.0.195/Temp/staging_test.jpg");
    // _sleep(2000);
    // return 0;

    QString exeDir = QCoreApplication::applicationDirPath();
    QByteArray exeDirBytes = QDir::toNativeSeparators(exeDir).toLocal8Bit();
    bool success = qputenv("HALCONROOT", exeDirBytes);

#if defined(_WIN32)
    SetSystem("use_window_thread", "true");
#endif

    // file was stored with local-8-bit encoding
    //   -> set the interface encoding accordingly
    SetHcppInterfaceStringEncodingIsUtf8(false);

    // Default settings used in HDevelop (can be omitted)
    SetSystem("width", 2000);
    SetSystem("height", 900);
    //
    // try {
    //     HObject imgPart;
    //     ReadImage(&imgPart,"test_bug.jpg");
    //     QString imgNameLocalCrop = QString("%1/test_%2.jpg").arg("\\\\192.168.0.196/Temp/").arg("test");
    //     WriteImage(imgPart, "jpeg", 0, imgNameLocalCrop.toStdString().c_str());
    // }catch (HException &exception){
    //     QLOG_INFO("图片传不出去了");
    // }


    // 注册 Halcon 类型
    qRegisterMetaType<HalconCpp::HObject>("HalconCpp::HObject");
    qRegisterMetaType<HalconCpp::HTuple>("HalconCpp::HTuple");
    //设置不应用操作系统设置比如字体
    QApplication::setDesktopSettingsAware(false);
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::Floor);

    //应用实例
    MyApplication a(argc, argv);

    //运行一次
    QSystemSemaphore sema("QT_AppFrame", 1, QSystemSemaphore::Open);
    //在临界区操作共享内存SharedMemory
    sema.acquire();
    //全局对象名
    QSharedMemory mem("QT_AppFrameObject");
    //如果全局对象以存在则退出
    if (!mem.create(1))
    {
        qDebug() << mem.errorString();
        sema.release();

        return 0;
    }
    sema.release();

    //应用设定
    AppInit::Instance()->start();
    AppInit::Instance()->initStyle(":/qss/silvery.css");

    //读取相机标定设置
    AppConfig::readConfig();

    frmMain w;
    w.show();


    return a.exec();
}

void testFunction()
{
    // 0. 随机生成颜色，仅用于显示
    RNG rng(0xFFFFFFFF);
    Scalar_<int> randColor[20];
    for (int i = 0; i < 20; i++)
        rng.fill(randColor[i], RNG::UNIFORM, 0, 256);


    HObject img, affine_img;
    //0. 获取可用的inference设备
    HTuple DeviceHandle = HalconPreprocessing::GetSuitableInferDevice()[0]; //按控制台输出，指定设备

    //1. 初始化钢板检测模块
    HalconDet ins_Det;
    ins_Det.Init("PlateDetectionModel_0805.hdl", "DetectionPreprocessParam_0805.hdict", DeviceHandle);
    //2. 初始化钢板号识别模块
    HalconOCR ins_OCR;
    ins_OCR.Init("detection.hdl", "detection_0808.hdl", "recognition_0808.hdl", DeviceHandle);
    //3. 初始化图像预处理模块
    HalconPreprocessing ins_PreProcessing;
    // ins_PreProcessing.AffineInit();
    //4. 初始化跟踪器
    SORTTracker tracker(5, 3, 0.3); // max_age=1, min_hits=3, iouThreshold=0.3
    //5. 计时器初始化
    LARGE_INTEGER frequency, start, end;
    QueryPerformanceFrequency(&frequency); // 获取计时器频率

    std::cout << "**** 初始化完成 ****" << std::endl;

    //处理视频流
    for (int i = 45; i < 339; ++i)
    {
        // std::string imgPath = "test_src/"+std::to_string(i) + ".jpg";
        // ReadImage(&img,imgPath.c_str());
        if (i == 338) i = 45;
        std::cout << "frameNum: " << i << std::endl;
        std::string imgPath = "test/" + std::to_string(i) + ".jpg";
        ReadImage(&affine_img, imgPath.c_str());

        QueryPerformanceCounter(&start); // 开始计时

        //halcon透视变换矫正图像
        // ins_PreProcessing.Affine(img, affine_img);
        //halcon目标检测推理
        auto detRes = ins_Det.Infer(affine_img);
        // detRes.filterByConfidence();
        auto detRects = detRes.getOriginalRects();
        if (!detRects.empty())
        {
            std::cout << "confidence: " << detRes.getOriginalConfs().at(0) << std::endl;
            std::cout << "BBox: [" << detRects.at(0).x << ", "
                << detRects.at(0).y << ", "
                << detRects.at(0).width << ", "
                << detRects.at(0).height << "]\n";
        }

        // 更新跟踪器并获取结果
        vector<TrackingBox> frameTrackingResult = tracker.update(detRects);

        QueryPerformanceCounter(&end); // 结束计时
        double duration = (end.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart; // 计算时间（单位：毫秒）
        std::cout << "time: " << duration << " 毫秒" << std::endl;

        // 处理结果
        for (const auto& plate : frameTrackingResult)
        {
            std::cout << "Plate ID: " << plate.id
                // << " | State: " << PlateStateToString(plate.state)
                // << " | Flow: " << FlowDirectionToString(plate.flow)
                // << " | Tracked: " << (plate.isTracked ? "Yes" : "No")
                << " | BBox: [" << plate.box.x << ", "
                << plate.box.y << ", "
                << plate.box.width << ", "
                << plate.box.height << "]\n";
            std::cout << "---------------------------------------------------------------------------------" <<
                std::endl;
        }
        if (display)
        {
            // 读取图像，绘制结果并显示
            Mat img = imread(imgPath);
            if (img.empty())
                continue;

            // cv::resize(img, img, cv::Size(512, 320));
            // for (auto tb : frameTrackingResult) {
            //     cv::rectangle(img, tb.box, randColor[tb.id % 20], 2, 8, 0);
            //     rectangle(img, tb.box, Scalar(0, 255, 0), 2);
            //     putText(img, to_string(tb.id),
            //             Point(tb.box.x, tb.box.y - 5),
            //             FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
            // }
            // 绘制所有轨迹
            // tracker.drawAllTracks(img);

            imshow("display", img);
            cv::waitKey(30);
        }
    }
}
