#ifndef ASYNCCAM
#define ASYNCCAM

#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <thread>
#include <string>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#elif __linux__
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#endif

#pragma region Namespace
using cv::Point, cv::Mat, cv::VideoCapture, cv::waitKey, cv::Vec3f, cv::Scalar_, cv::Scalar;
using std::cout, std::atomic_flag, std::string, std::thread, std::cerr, std::snprintf, std::vector;
#pragma endregion

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

    static void refresh_frame(VideoCapture &cap, Scalar_<int> &low, Scalar_<int> &high, vector<Vec3f> &out_circles,
                              atomic_flag &lock_flag, bool &running)
    {
        Mat origin, frame;
        int frame_cnt = 0;
        cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        tpoint start_time = std::chrono::high_resolution_clock::now();
        tpoint current_time;
        cout << std::fixed << std::setprecision(3);
        cout << "Color Range: [" << high.val[0] << '-' << low.val[0] << ", " << low.val[1] << '-' << high.val[1] << ", " << low.val[2] << '-' << high.val[2] << "]\n";
        Scalar local_low = Scalar(low.val[0], low.val[1], low.val[2]);
        Scalar local_high = Scalar(high.val[0], high.val[1], high.val[2]);
        while (running)
        {
            cap >> origin;
            cv::cvtColor(origin, frame, cv::COLOR_BGR2HSV);
            cv::inRange(frame, local_low, local_high, frame);
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
            cv::imshow("Frame", origin);
            waitKey(1);
        }
    }

#ifndef KEYBOARD_INTERRUPT
#define KEYBOARD_INTERRUPT
    struct Keyboard
    {
    private:
#ifdef __linux__
        int ch;
        int oldf;
        struct termios oldt, newt;
#endif

    public:
        Keyboard()
        {
#ifdef __linux__
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
#endif
        }

        char get_key()
        {
#ifdef __linux__
            return getchar();
#elif _WIN32
            if (_kbhit())
            {
                return _getch()
            }
#endif
        }

        ~Keyboard()
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            fcntl(STDIN_FILENO, F_SETFL, oldf);
        }
    };

#endif

    class AsyncCam
    {
    private:
        VideoCapture cap;
        Mat frame;
        Scalar_<int> high = Scalar_<int>(255, 255, 255);
        Scalar_<int> low = Scalar_<int>(0, 0, 0);
        vector<Vec3f> circles;
        thread th;
        atomic_flag frames_lock;
        string camera_str;
        int camera_idx = -1, width = 0, height = 0;
        bool running = false;
        bool thread_en = true;

        void highH_handler(int value, void *userdata)
        {
            high.val[0] = value;
        }

        void lowH_handler(int value, void *userdata)
        {
            low.val[0] = value;
        }

        void highS_handler(int value, void *userdata)
        {
            high.val[1] = value;
        }

        void lowS_handler(int value, void *userdata)
        {
            low.val[1] = value;
        }

        void highV_handler(int value, void *userdata)
        {
            high.val[2] = value;
        }

        void lowV_handler(int value, void *userdata)
        {
            low.val[2] = value;
        }

        void keyboard_init()
        {
        }

    public:
        AsyncCam(string path = "/dev/video0", int width = 640, int height = 480);
        AsyncCam(int device_id = 0, int width = 640, int height = 480);
        void calibrate();
        void start();
        void getobject(vector<Vec3f> &out_circles);
        void stop();
        ~AsyncCam();
    };

    AsyncCam::AsyncCam(string path, int width, int height) : frames_lock(ATOMIC_FLAG_INIT), width(width), height(height)
    {
        cap = VideoCapture(path);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        // set fps
        cap.set(cv::CAP_PROP_FPS, 30);
        if (!cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }

        camera_str = path;
        camera_idx = -1;
    }

    AsyncCam::AsyncCam(int device_id, int width, int height) : frames_lock(ATOMIC_FLAG_INIT), width(width), height(height)
    {
#ifdef _WIN32
        cap = VideoCapture(device_id);
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

        camera_idx = device_id;
        camera_str = "";
    }

    inline void AsyncCam::calibrate()
    {
        // Check if camera is running
        if (running)
        {
            std::cout << "Camera is already running. Please stop it first\n";
            return;
        }

        // select camera
        if (camera_idx > -1)
            cap.open(camera_idx);
        else
            cap.open(camera_str);

        // Fail handling
        if (!cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }

        cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
        cv::namedWindow("Result", cv::WINDOW_NORMAL);
        cv::createTrackbar("High H", "Calibration", &high[0], 255, nullptr);
        cv::createTrackbar("High S", "Calibration", &high[1], 255, nullptr);
        cv::createTrackbar("High V", "Calibration", &high[2], 255, nullptr);
        cv::createTrackbar("Low H", "Calibration", &low[0], 100, nullptr);
        cv::createTrackbar("Low S", "Calibration", &low[1], 100, nullptr);
        cv::createTrackbar("Low V", "Calibration", &low[2], 100, nullptr);

        Mat local_frame;
        cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        while (true)
        {
            cap >> local_frame;
            cout << '[' << high.val[0] << "," << high.val[1] << "," << high.val[2] << "] [" << low.val[0] << "," << low.val[1] << "," << low.val[2] << "] => Detected : ";

            // Process output
            cv::cvtColor(local_frame, frame, cv::COLOR_BGR2HSV);
            cv::inRange(frame,
                        cv::Scalar(low.val[0], low.val[1], low.val[2]),
                        cv::Scalar(high.val[0], high.val[1], high.val[2]), frame);
            cv::dilate(frame, frame, dilate_element);
            cv::GaussianBlur(frame, frame, cv::Size(31, 31), 0, 0);

            // Get circles
            circles.clear();
            cv::HoughCircles(frame, circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

            std::cout << circles.size() << "       \r";
            cout.flush();

            for (auto &c : circles)
            {
                cv::Point center(cvRound(c[0]), cvRound(c[1]));
                int radius = cvRound(c[2]);
                // circle center
                cv::circle(local_frame, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                // circle outline
                cv::circle(local_frame, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
            }

            // Show output
            cv::imshow("Calibration", local_frame);
            cv::imshow("Result", frame);
            if (cv::waitKey(1) == 'q')
                break;
        }

        cap.release();
        cv::destroyAllWindows();
        sleep(2);
        cout << "Calibration done                     \n";
    }

    inline void AsyncCam::start()
    {
        if (running)
        {
            std::cout << "Camera is already running\n";
            return;
        }
        if (camera_idx > -1)
            cap.open(camera_idx);
        else
            cap.open(camera_str);
        // Set camera resolution
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        if (!cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }
        running = true;

        cout << "Detection started\n";
        th = thread(refresh_frame, std::ref(cap), std::ref(low), std::ref(high), std::ref(circles),
                    std::ref(frames_lock), std::ref(thread_en));
        th.detach();
    }
    inline void AsyncCam::getobject(vector<Vec3f> &out_circles)
    {
        while (frames_lock.test_and_set(std::memory_order_acquire))
            ;
        out_circles = circles;
        frames_lock.clear();
    }
    inline void AsyncCam::stop()
    {
        thread_en = false;
        sleep(1);
        cap.release();
        cv::destroyAllWindows();
        cout << "Thread stopped. Camera closed\n";
    }

    AsyncCam::~AsyncCam()
    {
        while (thread_en)
            stop();
        if (cap.isOpened())
        {
            cap.release();
            cv::destroyAllWindows();
            cout << "Camera closed\n";
        }
    }
}

#endif