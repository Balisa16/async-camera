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

typedef std::chrono::_V2::system_clock::time_point tpoint;
typedef std::chrono::high_resolution_clock time_clock;

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
            atomic_flag lock_flag;
            float fps = 0;
            ThreadStatus status = ThreadStatus::NONE;
            FrameSet() : lock_flag(ATOMIC_FLAG_INIT) {}
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

    void AsyncCam::refresh_frame(FrameSet &frameset)
    {
        frameset.status = ThreadStatus::START;
        int frame_cnt = 0;
        Mat original;
        Mat dilate_element = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
        tpoint start_time = time_clock::now();
        tpoint current_time;
        cout << fixed << setprecision(3);

        Scalar local_low = Scalar(frameset.low.val[0], frameset.low.val[1], frameset.low.val[2]);
        Scalar local_high = Scalar(frameset.high.val[0], frameset.high.val[1], frameset.high.val[2]);
        float local_fps = 0.0f;

        frameset.status = ThreadStatus::RUNNING;
        while (frameset.status == ThreadStatus::RUNNING && frameset.cap.isOpened())
        {
            frameset.cap >> original;
            cvtColor(original, frameset.frame, COLOR_BGR2HSV);
            inRange(frameset.frame, local_low, local_high, frameset.frame);
            dilate(frameset.frame, frameset.frame, dilate_element);
            GaussianBlur(frameset.frame, frameset.frame, Size(31, 31), 0, 0);
            while (frameset.lock_flag.test_and_set(std::memory_order_acquire))
                ;
            frameset.circles.clear();
            HoughCircles(frameset.frame, frameset.circles, HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);
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
                // std::cout << "FPS : " << local_fps << " [" << frameset.original_frame.size().width << "x" << frameset.original_frame.size().height << "]   \r";
                // std::cout.flush();
                frame_cnt = 0;
            }
            waitKey(1);
        }
        frameset.status = ThreadStatus::STOP;
    }

    void AsyncCam::point_buffer(Point &point, const int &buffer_size)
    {
        static vector<Point> buffer;
        buffer.push_back(point);

        if (buffer.size() > buffer_size)
            buffer.erase(buffer.begin());

        const float nat = 5.0f / (float)buffer.size();

        float Sn = 0.0f, _sig = 0.0f;
        point = {0, 0};
        for (int i = 0; i < buffer.size(); i++)
        {
            // Calculate sigmoid expression: k / (1 + e^(a + bx))
            _sig = 0.8f / (1 + std::exp(10 - 3 * nat * i)) + 0.2f;
            point.x += (buffer[i].x * _sig);
            point.y += (buffer[i].y * _sig);
            Sn += _sig;
        }
        point.x /= Sn;
        point.y /= Sn;
    }

    void AsyncCam::rotate_point(Point &p, const double &angle)
    {
        double cos_theta = std::cos(angle);
        double sin_theta = std::sin(angle);

        // Rotation matrix
        double newX = p.x * cos_theta - p.y * sin_theta;
        double newY = p.x * sin_theta + p.y * cos_theta;

        p.x = newX;
        p.y = newY;
    }

    void AsyncCam::adjust_point(Point &p, const int &px_radius)
    {
        float dist = std::sqrt(p.x * p.x + p.y * p.y);
        if (dist <= px_radius)
            return;

        float ratio = px_radius / dist;
        p.x *= ratio;
        p.y *= ratio;
    }

    AsyncCam::AsyncCam(string path, int width, int height)
    {
        frameset.width = width;
        frameset.height = height;
        frameset.cap = VideoCapture(path);
        frameset.cap.set(CAP_PROP_FRAME_WIDTH, width);
        frameset.cap.set(CAP_PROP_FRAME_HEIGHT, height);
        // set fps
        frameset.cap.set(CAP_PROP_FPS, 30);
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
        frameset.cap.set(CAP_PROP_FRAME_WIDTH, width);
        frameset.cap.set(CAP_PROP_FRAME_HEIGHT, height);

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
        // Check thread status
        while (frameset.lock_flag.test_and_set(std::memory_order_acquire))
            ;
        if (frameset.status == ThreadStatus::RUNNING)
        {

            std::cout << C_YELLOW << S_BOLD << "Camera is already running. Please stop it first." << C_RESET << '\n';
            return;
        }
        frameset.lock_flag.clear();

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

        frameset.cap.set(CAP_PROP_FRAME_WIDTH, width);
        frameset.cap.set(CAP_PROP_FRAME_HEIGHT, height);

        // Prepare window
        namedWindow("Calibration", WINDOW_AUTOSIZE);
        namedWindow("Result", WINDOW_NORMAL);
        resizeWindow("Result", width, height);
        createTrackbar("High H", "Calibration", &frameset.high[0], 255, nullptr);
        createTrackbar("High S", "Calibration", &frameset.high[1], 255, nullptr);
        createTrackbar("High V", "Calibration", &frameset.high[2], 255, nullptr);
        createTrackbar("Low H", "Calibration", &frameset.low[0], 100, nullptr);
        createTrackbar("Low S", "Calibration", &frameset.low[1], 100, nullptr);
        createTrackbar("Low V", "Calibration", &frameset.low[2], 100, nullptr);

        Mat local_frame, frame;
        vector<Vec3f> circles;
        Mat dilate_element = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
        while (true)
        {
            frameset.cap >> local_frame;

            // Process output
            cvtColor(local_frame, frame, COLOR_BGR2HSV);
            inRange(frame,
                    Scalar(frameset.low.val[0], frameset.low.val[1], frameset.low.val[2]),
                    Scalar(frameset.high.val[0], frameset.high.val[1], frameset.high.val[2]), frame);
            dilate(frame, frame, dilate_element);
            GaussianBlur(frame, frame, Size(31, 31), 0, 0);

            // Get circles
            circles.clear();
            HoughCircles(frame, circles, HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

            cout.flush();

            for (auto &c : circles)
            {
                Point center(cvRound(c[0]), cvRound(c[1]));
                int radius = cvRound(c[2]);
                // circle center
                circle(local_frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
                // circle outline
                circle(local_frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);
            }

            if (!circles.empty())
                cout << C_GREEN << S_BOLD << "[OK] => " << circles.size() << " circles." << C_RESET << "   \r";
            else
                cout << C_RED << S_BOLD << "[NONE]" << C_RESET << "                \r";

            // Show output
            imshow("Calibration", local_frame);
            imshow("Result", frame);
            if (waitKey(1) == 'q')
                break;
        }

        frameset.cap.release();
        destroyAllWindows();
    }

    inline void AsyncCam::start()
    {
#ifdef _WIN32
        Sleep(2000);
#elif __linux__
        sleep(2);
#endif
        while (frameset.lock_flag.test_and_set(std::memory_order_acquire))
            ;
        if (frameset.status == ThreadStatus::RUNNING)
        {
            std::cout << C_YELLOW << S_BOLD << "Camera is already running." << C_RESET << '\n';
            return;
        }
        frameset.lock_flag.clear();

        if (camera_idx > -1)
            frameset.cap.open(camera_idx);
        else
            frameset.cap.open(camera_str);

        // Set camera resolution
        frameset.cap.set(CAP_PROP_FRAME_WIDTH, width);
        frameset.cap.set(CAP_PROP_FRAME_HEIGHT, height);

        if (!frameset.cap.isOpened())
        {
            cerr << "Failed openning camera\n";
            exit(EXIT_FAILURE);
        }

        cout << C_GREEN << S_BOLD << "Detection started." << C_RESET << '\n';
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
        while (frameset.lock_flag.test_and_set(std::memory_order_acquire))
            ;
        sleep(1);

        // Trigger to close loop
        frameset.status = ThreadStatus::NONE;

        frameset.lock_flag.clear();
        frameset.cap.release();
        destroyAllWindows();
    }

    void AsyncCam::sync_fps(float fps)
    {
        if (fps < 1.0f)
            fps = 15.0f;
        else if (fps > 60.0f)
            fps = 30.0f;
        while (std::chrono::duration_cast<std::chrono::microseconds>(time_clock::now() - last_time).count() < 1000000.0f / fps)
            ;
        last_time = time_clock::now();
    }

    AsyncCam::~AsyncCam()
    {
        // Wait for thread to finish
        while (frameset.status == ThreadStatus::RUNNING)
        {
            stop();
            sleep(1);
        }

        // Just give waiting time for thread to finish
        sleep(2);

        // Make sure camera is closed
        if (frameset.cap.isOpened())
        {
            frameset.cap.release();
            destroyAllWindows();
        }

        std::cout << C_GREEN << S_BOLD << "Detection Finished." << C_RESET << '\n';
    }
}

#endif