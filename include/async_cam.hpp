#ifndef ASYNCCAM
#define ASYNCCAM

#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <thread>
#include <string>

using cv::Point, cv::Mat, cv::VideoCapture, cv::waitKey;
using std::cout, std::atomic_flag, std::string, std::thread, std::cerr, std::snprintf;

namespace EMIRO
{
#ifdef _WIN32
#define OS_NAME "Windows"
#elif __linux__
#define OS_NAME "Linux"
#else
#define OS_NAME "Other"
#endif

    static void refresh_frame(VideoCapture *cap, Mat *frame, atomic_flag *lock_flag, bool *running)
    {
        while (&running)
        {
            if (lock_flag->test_and_set(std::memory_order_acquire))
            {
                cap->read(*frame);
                lock_flag->clear();
            }
            waitKey(10);
        }
    }
    class AsyncCam
    {
    private:
        VideoCapture cap;
        Mat frame;
        bool running = false;

    public:
        AsyncCam(string path = "/dev/video0", int width = 640, int height = 480);
        AsyncCam(int device_id = 0, int width = 640, int height = 480);
        void start();
        void getobject(Point &point);
        void stop();
        ~AsyncCam();
    };

    AsyncCam::AsyncCam(string path, int width, int height)
    {
        cap = VideoCapture(path);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }

    AsyncCam::AsyncCam(int device_id, int width, int height)
    {
#ifdef _WIN32
        cap = VideoCapture(0);
#elif __linux__
        cap = VideoCapture(device_id);
        if (!cap.isOpened())
        {
            char buffer_int[5];
            const char temp1[26] = "v4l2src device=/dev/video";
            const char temp2[23] = " ! video/x-raw, width=";
            const char temp3[10] = ", height=";
            const char temp4[51] = " ! videoconvert ! video/x-raw,format=BGR ! appsink";
            char temp_char[256] = "";
            strcat(temp_char, temp1);
            std::snprintf(buffer_int, sizeof(buffer_int), "%d", device_id);
            strcat(temp_char, buffer_int);
            strcat(temp_char, temp2);
            snprintf(buffer_int, sizeof(buffer_int), "%d", width);
            strcat(temp_char, buffer_int);
            strcat(temp_char, temp3);
            std::snprintf(buffer_int, sizeof(buffer_int), "%d", height);
            strcat(temp_char, buffer_int);
            strcat(temp_char, temp4);
            cap = VideoCapture(temp_char);
            if (!cap.isOpened())
            {
                cerr << "Failed openning camera. Available camera : \n";
                system("ls /dev/video*");
                cout << '\n';
                exit(EXIT_FAILURE);
            }
        }
#else
        cout << "Unsupported OS" << endl;);
        exit(EXIT_FAILURE);
#endif
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        if (!cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }
    }

    void AsyncCam::start()
    {
    }
    void AsyncCam::getobject(Point &point)
    {
    }
    void AsyncCam::stop()
    {
    }

    AsyncCam::~AsyncCam()
    {
        if (cap.isOpened())
        {
            cap.release();
            cout << "Camera closed\n";
        }
    }
}

#endif