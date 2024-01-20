#include <async_cam.hpp>

using EMIRO::AsyncCam;

int main()
{
    AsyncCam cam(0, 640, 480);
    cam.calibrate();
    cam.start();
    vector<Vec3f> circles;
    Mat frame;
    Vec3f selected_object;
    EMIRO::Keyboard kb;
    while (true)
    {
        cam.getobject(circles, frame);
        if (!circles.empty())
        {
            selected_object = circles[0];

            // Select largest object
            if (circles.size() > 1)
            {
                int largest_radius = 0;
                for (auto &c : circles)
                {
                    Point p(c[0], c[1]);
                    int radius = cvRound(c[2]);

                    if (radius > largest_radius)
                    {
                        largest_radius = radius;
                        selected_object = c;
                    }
                }
            }

            cv::Point center(cvRound(selected_object[0]), cvRound(selected_object[1]));
            int radius = cvRound(selected_object[2]);

            // circle center and outline
            cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
            cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

            // draw center line
        }
        cv::line(frame, cv::Point(cam.width / 2, 0), cv::Point(cam.width / 2, cam.height), cv::Scalar(255, 255, 0, 255), 2);
        cv::line(frame, cv::Point(0, cam.height / 2), cv::Point(cam.width, cam.height / 2), cv::Scalar(255, 255, 0, 255), 2);

        cv::imshow("Result", frame);
        cv::waitKey(1);
        if (kb.get_key() == 27)
        {
            std::cout << "Stopped by interrupt\n";
            break;
        }
        // usleep(100000);
    }
    cam.stop();
    return 0;
}