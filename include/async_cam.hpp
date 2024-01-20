#ifndef ASYNCCAM
#define ASYNCCAM

// OpenCV includes
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

// Standard C++
#include <iostream>
#include <atomic>
#include <thread>
#include <string>
#include <vector>
#include <iomanip>

// Platform specific
#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#elif __linux__
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#endif

#pragma region Namespace
using cv::Point, cv::Mat, cv::VideoCapture, cv::waitKey, cv::Vec3f, cv::Scalar_, cv::Scalar, cv::cvtColor, cv::inRange, cv::dilate, cv::GaussianBlur, cv::getStructuringElement, cv::destroyAllWindows, cv::namedWindow, cv::createTrackbar;
using std::cout, std::atomic_flag, std::string, std::thread, std::cerr, std::snprintf, std::vector, std::setprecision, std::fixed;
#pragma endregion

typedef std::chrono::_V2::system_clock::time_point tpoint;
typedef std::chrono::high_resolution_clock time_clock;

namespace EMIRO
{

    struct FrameSet
    {
    public:
        VideoCapture cap;
        Mat original_frame, frame;
        int width = 0, height = 0;
        Scalar_<int> high = Scalar_<int>(255, 255, 255);
        Scalar_<int> low = Scalar_<int>(0, 0, 0);
        vector<Vec3f> circles;
        atomic_flag lock_flag;
        float fps = 0;
        bool running = false;
        FrameSet() : lock_flag(ATOMIC_FLAG_INIT) {}
    };

    /**
     * @brief Thread for asynchronusly refreshing frames
     *
     * @param cap Video Capture
     * @param low Scalar for low range
     * @param high Scalar for high range
     * @param out_circles Vector for detected circles
     * @param lock_flag Atomic flag for thread
     * @param running Boolean to control thread
     */
    static void refresh_frame(FrameSet &frameset)
    {
        int frame_cnt = 0;
        Mat original;
        Mat dilate_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        tpoint start_time = time_clock::now();
        tpoint current_time;
        cout << fixed << setprecision(3);

        Scalar local_low = Scalar(frameset.low.val[0], frameset.low.val[1], frameset.low.val[2]);
        Scalar local_high = Scalar(frameset.high.val[0], frameset.high.val[1], frameset.high.val[2]);
        float local_fps = 0.0f;

        while (frameset.running && frameset.cap.isOpened())
        {
            frameset.cap >> original;
            cvtColor(original, frameset.frame, cv::COLOR_BGR2HSV);
            inRange(frameset.frame, local_low, local_high, frameset.frame);
            dilate(frameset.frame, frameset.frame, dilate_element);
            GaussianBlur(frameset.frame, frameset.frame, cv::Size(31, 31), 0, 0);
            while (frameset.lock_flag.test_and_set(std::memory_order_acquire))
                ;
            frameset.circles.clear();
            cv::HoughCircles(frameset.frame, frameset.circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);
            original.copyTo(frameset.original_frame);
            frameset.fps = local_fps;
            frameset.lock_flag.clear();
            frame_cnt++;
            current_time = time_clock::now();
            int64_t elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
            if (elapsed >= 1000000)
            {
                start_time = current_time;
                local_fps = static_cast<float>(frame_cnt) / (elapsed / 1000000.0);
                std::cout << "FPS : " << local_fps << " [" << frameset.original_frame.size().width << "x" << frameset.original_frame.size().height << "]   \r";
                std::cout.flush();
                frame_cnt = 0;
            }
            waitKey(1);
        }
    }

#pragma region Keyboard
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
                return _getch()
#endif
        }

        ~Keyboard()
        {
#ifdef __linux__
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            fcntl(STDIN_FILENO, F_SETFL, oldf);
#endif
        }
    };
#endif
#pragma endregion

    class AsyncCam
    {
    private:
        FrameSet frameset;
        thread th;
        string camera_str;
        int camera_idx = -1;

    public:
        int width = 0, height = 0;

    public:
        /**
         * @brief Construct a new Async Camera using string device path
         *
         * @param path Path of camera device
         * @param width Width of camera frame
         * @param height Height of camera frame
         */
        AsyncCam(string path = "/dev/video0", int width = 640, int height = 480);

        /**
         * @brief Construct a new Async Camera using integer device id
         *
         * @param device_id Camera device id. 0 for default
         * @param width Width of camera frame
         * @param height Height of camera frame
         */
        AsyncCam(int device_id = 0, int width = 640, int height = 480);

        /**
         * @brief Do calibration of low and high threshold values
         *
         */
        void calibrate();

        /**
         * @brief Start thread for object detection
         *
         */
        void start();

        /**
         * @brief Get detected circles from thread
         *
         * @param out_circles Output vector of detected circles
         */
        void getobject(vector<Vec3f> &out_circles, Mat &out_frame);

        /**
         * @brief Stopping detection thread
         *
         */
        void stop();

        ~AsyncCam();
    };

    AsyncCam::AsyncCam(string path, int width, int height)
    {
        frameset.width = width;
        frameset.height = height;
        frameset.cap = VideoCapture(path);
        frameset.cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        frameset.cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        // set fps
        frameset.cap.set(cv::CAP_PROP_FPS, 30);
        if (!frameset.cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }

        camera_str = path;
        camera_idx = -1;
    }

    AsyncCam::AsyncCam(int device_id, int width, int height) : width(width), height(height)
    {
        frameset.width = width;
        frameset.height = height;
#ifdef _WIN32
        frameset.cap = VideoCapture(device_id);
#elif __linux__
        frameset.cap = VideoCapture(device_id);
        if (!frameset.cap.isOpened())
        {
            char buffer_int[5];
            const char temp1[26] = "v4l2src device=/dev/video";
            const char temp2[23] = " ! video/x-raw, width=";
            const char temp3[10] = ", height=";
            const char temp4[51] = " ! videoconvert ! video/x-raw,format=BGR ! appsink";
            char temp_char[256] = "";
            strcat(temp_char, temp1);
            snprintf(buffer_int, sizeof(buffer_int), "%d", device_id);
            strcat(temp_char, buffer_int);
            strcat(temp_char, temp2);
            snprintf(buffer_int, sizeof(buffer_int), "%d", width);
            strcat(temp_char, buffer_int);
            strcat(temp_char, temp3);
            snprintf(buffer_int, sizeof(buffer_int), "%d", height);
            strcat(temp_char, buffer_int);
            strcat(temp_char, temp4);
            frameset.cap = VideoCapture(temp_char);
            if (!frameset.cap.isOpened())
            {
                cerr << "Failed openning camera.";
#ifdef __linux__
                cerr << " Available camera : \n";
                system("ls /dev/video*");
#endif
                cout << '\n';
                exit(EXIT_FAILURE);
            }
        }
#else
        cout << "Unsupported OS" << endl;);
        exit(EXIT_FAILURE);
#endif
        frameset.cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        frameset.cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        if (!frameset.cap.isOpened())
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
        if (frameset.running)
        {
            std::cout << "Camera is already running. Please stop it first\n";
            return;
        }

        // select camera
        if (camera_idx > -1)
            frameset.cap.open(camera_idx);
        else
            frameset.cap.open(camera_str);

        // Fail handling
        if (!frameset.cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }

        // Prepare window
        namedWindow("Calibration", cv::WINDOW_NORMAL);
        namedWindow("Result", cv::WINDOW_NORMAL);
        createTrackbar("High H", "Calibration", &frameset.high[0], 255, nullptr);
        createTrackbar("High S", "Calibration", &frameset.high[1], 255, nullptr);
        createTrackbar("High V", "Calibration", &frameset.high[2], 255, nullptr);
        createTrackbar("Low H", "Calibration", &frameset.low[0], 100, nullptr);
        createTrackbar("Low S", "Calibration", &frameset.low[1], 100, nullptr);
        createTrackbar("Low V", "Calibration", &frameset.low[2], 100, nullptr);

        Mat local_frame, frame;
        vector<Vec3f> circles;
        Mat dilate_element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
        while (true)
        {
            frameset.cap >> local_frame;
            cout << '[' << frameset.high.val[0] << "," << frameset.high.val[1] << "," << frameset.high.val[2] << "] [" << frameset.low.val[0] << "," << frameset.low.val[1] << "," << frameset.low.val[2] << "] => Detected : ";

            // Process output
            cvtColor(local_frame, frame, cv::COLOR_BGR2HSV);
            inRange(frame,
                    cv::Scalar(frameset.low.val[0], frameset.low.val[1], frameset.low.val[2]),
                    cv::Scalar(frameset.high.val[0], frameset.high.val[1], frameset.high.val[2]), frame);
            dilate(frame, frame, dilate_element);
            GaussianBlur(frame, frame, cv::Size(31, 31), 0, 0);

            // Get circles
            circles.clear();
            HoughCircles(frame, circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

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
            if (waitKey(1) == 'q')
                break;
        }

        frameset.cap.release();
        destroyAllWindows();
        cout << "Calibration done                     \n";
    }

    inline void AsyncCam::start()
    {
#ifdef _WIN32
        Sleep(2000);
#elif __linux__
        sleep(2);
#endif
        if (frameset.running)
        {
            std::cout << "Camera is already running\n";
            return;
        }
        if (camera_idx > -1)
            frameset.cap.open(camera_idx);
        else
            frameset.cap.open(camera_str);
        // Set camera resolution
        frameset.cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        frameset.cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        if (!frameset.cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }
        frameset.running = true;

        cout << "Detection started\n";
        th = thread(refresh_frame, std::ref(frameset));
        th.detach();
    }

    inline void AsyncCam::getobject(vector<Vec3f> &out_circles, Mat &out_frame)
    {
        while (frameset.lock_flag.test_and_set(std::memory_order_acquire))
            ;
        out_circles = frameset.circles;
        out_frame = frameset.original_frame;
        frameset.lock_flag.clear();
    }

    inline void AsyncCam::stop()
    {
        frameset.running = false;
        frameset.cap.release();
        destroyAllWindows();
        cout << "Thread stopped. Camera closed\n";
    }

    AsyncCam::~AsyncCam()
    {
        while (frameset.running)
            stop();
        if (frameset.cap.isOpened())
        {
            frameset.cap.release();
            destroyAllWindows();
            cout << "Camera closed\n";
        }
    }
}

#endif