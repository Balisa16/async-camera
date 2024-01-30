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
#include <mutex>

// Platform specific
#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#elif __linux__
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#endif

#ifndef TERMINAL_COLOR_STYLE
#define C_RESET "\033[0m"
#define C_BLACK "\033[30m"
#define C_RED "\033[31m"
#define C_GREEN "\033[32m"
#define C_YELLOW "\033[33m"
#define C_BLUE "\033[34m"
#define C_MAGENTA "\033[35m"
#define C_CYAN "\033[36m"
#define C_WHITE "\033[37m"
#define S_BOLD "\033[1m"
#define S_ITALIC "\033[3m"
#define S_UNDERLINE "\033[4m"
#endif

using namespace cv;
using namespace std;

typedef chrono::_V2::system_clock::time_point tpoint;
typedef chrono::high_resolution_clock time_clock;

namespace EMIRO
{
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
    public:
        enum class ThreadStatus
        {
            NONE,
            START,
            RUNNING,
            STOP
        };
        struct FrameSet
        {
        public:
            VideoCapture cap;
            Mat original_frame, frame;
            int width = 0, height = 0;
            Scalar_<int> high = Scalar_<int>(255, 255, 255);
            Scalar_<int> low = Scalar_<int>(0, 0, 0);
            vector<Vec3f> circles;
            std::mutex _mu;
            float fps = 0;
            ThreadStatus status = ThreadStatus::NONE;
            FrameSet() {}
        };

    private:
        FrameSet frameset;
        thread th;
        string camera_str;
        int camera_idx = -1;
        tpoint last_time = time_clock::now();

    private:
        /**
         * @brief Refresh frame asynchronously
         *
         * @param frameset Frame set
         * @see AsyncCam::FrameSet
         */
        static void refresh_frame(FrameSet &frameset);

    public:
        int width = 0, height = 0;

    public:
        /**
         * @brief Make control object movement is smooth
         *
         * @param point current point
         * @param buffer_size Size of buffer. Higher is more smooth but lag
         */
        static void point_buffer(Point &point, const int &buffer_size);

        /**
         * @brief Rotate a point
         *
         * @param p A point which want to rotate.
         * @param angle angle of rotation
         */
        static void rotate_point(Point &p, const double &angle);

        /**
         * @brief Check if current point is out of center range.
         * When it is out of range, this will calcuate new point which have same direction
         *
         * @param p Current point and output calculation of new point if it is out of range
         * @param px_radius Range of radius point tolerance from center of image which px_radius < image_height/2 (if image is landscape).
         */
        static void adjust_point(Point &p, const int &px_radius);
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
         * @brief Set the color range for object detection
         *
         * @param high High Limit of color (in HSV format)
         * @param low Low Limit of color (in HSV format)
         */
        void set_range(const Scalar_<int> &high, const Scalar_<int> &low);

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

        /**
         * @brief Syncronize target fps
         * @note This must called in frame loop
         * @param fps Target FPS (float) where 1.0f <= fps <= 60.0f
         */
        void sync_fps(float fps = 30);

        ~AsyncCam();
    };

}

#endif