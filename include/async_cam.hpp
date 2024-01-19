#ifndef ASYNCCAM
#define ASYNCCAM

#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <thread>
#include <string>
#include <vector>
#include <unistd.h>

using cv::Point, cv::Mat, cv::VideoCapture, cv::waitKey, cv::Vec3f, cv::Scalar;
using std::cout, std::atomic_flag, std::string, std::thread, std::cerr, std::snprintf, std::vector;

typedef std::chrono::_V2::system_clock::time_point tpoint;

namespace EMIRO
{
#ifdef _WIN32
#define OS_NAME "Windows"
#elif __linux__
#define OS_NAME "Linux"
#else
#define OS_NAME "Other"
#endif

    static void refresh_frame(VideoCapture &cap, Scalar &low, Scalar &high, vector<Vec3f> &out_circles,
                              atomic_flag &lock_flag, bool &running)
    {
        Mat frame;
        int frame_cnt = 0;
        cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        tpoint start_time = std::chrono::high_resolution_clock::now();
        tpoint current_time;
        cout << std::fixed << std::setprecision(3);
        while (running)
        {
            // std::cout << "thread run\n";
            cap >> frame;
            cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
            cv::inRange(frame, low, high, frame);
            cv::dilate(frame, frame, dilate_element);
            cv::GaussianBlur(frame, frame, cv::Size(31, 31), 0, 0);
            while (lock_flag.test_and_set(std::memory_order_acquire))
                ;
            out_circles.clear();
            cv::HoughCircles(frame, out_circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);
            lock_flag.clear();
            frame_cnt++;
            current_time = std::chrono::high_resolution_clock::now();
            int64_t elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
            if (elapsed >= 1000000)
            {
                start_time = current_time;
                cout << "FPS: " << static_cast<double>(frame_cnt) / (elapsed / 1000000.0) << "   \r";
                cout.flush();
                frame_cnt = 0;
            }
            cv::imshow("Frame", frame);
            waitKey(1);
        }
    }
    class AsyncCam
    {
    private:
        VideoCapture cap;
        Mat frame;
        bool running = false;
        Scalar high = Scalar(255, 255, 255);
        Scalar low = Scalar(0, 0, 0);
        vector<Vec3f> circles;
        thread th;
        atomic_flag frames_lock;
        bool thread_en = true;

    public:
        AsyncCam(string path = "/dev/video0", int width = 640, int height = 480);
        AsyncCam(int device_id = 0, int width = 640, int height = 480);
        void start();
        void getobject(vector<Vec3f> &out_circles);
        void stop();
        ~AsyncCam();
    };

    AsyncCam::AsyncCam(string path, int width, int height) : frames_lock(ATOMIC_FLAG_INIT)
    {
        cap = VideoCapture(path);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        // set fps
        cap.set(cv::CAP_PROP_FPS, 30);
    }

    AsyncCam::AsyncCam(int device_id, int width, int height) : frames_lock(ATOMIC_FLAG_INIT)
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

        th = thread(refresh_frame, std::ref(cap), std::ref(low), std::ref(high), std::ref(circles),
                    std::ref(frames_lock), std::ref(thread_en));
    }

    void AsyncCam::start()
    {
        if (running)
        {
            std::cout << "Camera is already running\n";
            return;
        }
        th.detach();
    }
    void AsyncCam::getobject(vector<Vec3f> &out_circles)
    {
        while (frames_lock.test_and_set(std::memory_order_acquire))
            ;
        out_circles = circles;
        frames_lock.clear();
    }
    void AsyncCam::stop()
    {
        thread_en = false;
        sleep(1);
    }

    AsyncCam::~AsyncCam()
    {
        while (thread_en)
            stop();
        if (cap.isOpened())
        {
            cap.release();
            cout << "Camera closed\n";
        }
    }
}

#endif