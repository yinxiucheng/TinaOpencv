#include <jni.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <android/native_window_jni.h>

using namespace cv;
ANativeWindow *window = 0;


class CascadeDetectorAdapter : public DetectionBasedTracker::IDetector {
public:
    CascadeDetectorAdapter(cv::Ptr<cv::CascadeClassifier> detector) :
            IDetector(),
            Detector(detector) {
        CV_Assert(detector);
    }

    void detect(const cv::Mat &Image, std::vector<cv::Rect> &objects) {
        Detector->detectMultiScale(Image, objects, scaleFactor, minNeighbours, 0, minObjSize,
                                   maxObjSize);
    }

    virtual ~CascadeDetectorAdapter() {

    }

private:
    CascadeDetectorAdapter();

    cv::Ptr<cv::CascadeClassifier> Detector;
};

DetectionBasedTracker* tracker = 0;

extern "C"
JNIEXPORT void JNICALL
Java_com_tina_tinaopencv_MainActivity_init(JNIEnv *env, jobject instance, jstring model_) {
    const char *model = env->GetStringUTFChars(model_, 0);
    if (tracker) {
        tracker->stop();
        delete tracker;
        tracker = 0;
    }
    //智能指针
    Ptr<CascadeClassifier> classifier = makePtr<CascadeClassifier>
            ("/Users/xiuchengyin/Downloads/FFmpeg_Third_Jar/OpenCV-android-sdk/sdk/etc/lbpcascades/lbpcascade_frontalface.xml");
    //创建一个跟踪适配器
    Ptr<CascadeDetectorAdapter> mainDetector = makePtr<CascadeDetectorAdapter>(classifier);

    Ptr<CascadeClassifier> classifier1 = makePtr<CascadeClassifier>
            ("/Users/xiuchengyin/Downloads/FFmpeg_Third_Jar/OpenCV-android-sdk/sdk/etc/lbpcascades/lbpcascade_frontalface.xml");
    Ptr<CascadeDetectorAdapter> trackingDetector = makePtr<CascadeDetectorAdapter>(classifier1);

    //拿去用的跟踪器
    DetectionBasedTracker::Parameters DetectorParams;

    tracker = makePtr<DetectionBasedTracker>(mainDetector, trackingDetector, DetectorParams);

    //开启跟踪器
    tracker->run();
    env->ReleaseStringUTFChars(model_, model);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_tina_tinaopencv_MainActivity_setSurface(JNIEnv *env, jobject instance, jobject surface) {

    if (window) {
        ANativeWindow_release(window);
        window = 0;
    }
    window = ANativeWindow_fromSurface(env, surface);
}


extern "C"
JNIEXPORT void JNICALL
Java_com_tina_tinaopencv_MainActivity_postData(JNIEnv *env, jobject instance, jbyteArray data_,
                                               jint w, jint h, jint cameraId) {
    jbyte *data = env->GetByteArrayElements(data_, NULL);


    env->ReleaseByteArrayElements(data_, data, 0);
}


extern "C"
JNIEXPORT void JNICALL
Java_com_tina_tinaopencv_MainActivity_release(JNIEnv *env, jobject instance) {
    if (tracker) {
        tracker->stop();
        delete tracker;
        tracker = 0;
    }
}