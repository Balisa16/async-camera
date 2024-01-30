#include <async_cam.hpp>

namespace EMIRO
{
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
            {
                std::lock_guard<std::mutex> lock(frameset._mu);
                frameset.circles.clear();
                HoughCircles(frameset.frame, frameset.circles, HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);
                original.copyTo(frameset.original_frame);
                frameset.fps = local_fps;
            }
            frame_cnt++;
            current_time = time_clock::now();
            int64_t elapsed = chrono::duration_cast<chrono::microseconds>(current_time - start_time).count();
            if (elapsed >= 1000000)
            {
                start_time = current_time;
                local_fps = static_cast<float>(frame_cnt) / (elapsed / 1000000.0);
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
            _sig = 0.8f / (1 + exp(10 - 3 * nat * i)) + 0.2f;
            point.x += (buffer[i].x * _sig);
            point.y += (buffer[i].y * _sig);
            Sn += _sig;
        }
        point.x /= Sn;
        point.y /= Sn;
    }

    void AsyncCam::set_range(const Scalar_<int> &high, const Scalar_<int> &low)
    {
        if (frameset.status == ThreadStatus::RUNNING)
        {
            cout << S_BOLD << C_YELLOW << "Camera is already running. " << C_RESET << "Please stop it first.\n";
            return;
        }
        frameset.high = high;
        frameset.low = low;
    }

    void AsyncCam::rotate_point(Point &p, const double &angle)
    {
        double cos_theta = cos(angle);
        double sin_theta = sin(angle);

        // Rotation matrix
        double newX = p.x * cos_theta - p.y * sin_theta;
        double newY = p.x * sin_theta + p.y * cos_theta;

        p.x = newX;
        p.y = newY;
    }

    void AsyncCam::adjust_point(Point &p, const int &px_radius)
    {
        float dist = sqrt(p.x * p.x + p.y * p.y);
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

    void AsyncCam::calibrate()
    {
        // Check thread status
        {
            std::lock_guard<std::mutex> lock(frameset._mu);
            if (frameset.status == ThreadStatus::RUNNING)
            {
                cout << C_YELLOW << S_BOLD << "Camera is already running. Please stop it first." << C_RESET << '\n';
                return;
            }
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

    void AsyncCam::start()
    {
        {
            std::lock_guard<std::mutex> lock(frameset._mu);
            if (frameset.status == ThreadStatus::RUNNING)
            {
                cout << C_YELLOW << S_BOLD << "Camera is already running." << C_RESET << '\n';
                return;
            }
        }
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
        th = thread(refresh_frame, ref(frameset));
        th.detach();
    }

    void AsyncCam::getobject(vector<Vec3f> &out_circles, Mat &out_frame)
    {
        std::lock_guard<std::mutex> lock(frameset._mu);
        out_circles = frameset.circles;
        out_frame = frameset.original_frame;
    }

    inline void AsyncCam::stop()
    {
        {
            std::lock_guard<std::mutex> lock(frameset._mu);
            frameset.status = ThreadStatus::NONE;
        }

        if (th.joinable())
            th.join();

        while (frameset.status != ThreadStatus::STOP)
#ifdef _WIN32
            Sleep(10);
#elif __linux__
            usleep(10000);
#endif

        frameset.cap.release();
        destroyAllWindows();
    }

    void AsyncCam::sync_fps(float fps)
    {
        if (fps < 1.0f)
            fps = 15.0f;
        else if (fps > 60.0f)
            fps = 30.0f;
        while (chrono::duration_cast<chrono::microseconds>(time_clock::now() - last_time).count() < 1000000.0f / fps)
            ;
        last_time = time_clock::now();
    }

    AsyncCam::~AsyncCam()
    {
        // Wait for thread to finish
        if (frameset.status == ThreadStatus::RUNNING)
            stop();

        // Make sure camera is closed
        if (frameset.cap.isOpened())
        {
            frameset.cap.release();
            destroyAllWindows();
        }

        cout << C_GREEN << S_BOLD << "Detection Finished." << C_RESET << '\n';
    }
}