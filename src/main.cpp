#include <async_cam.hpp>

using EMIRO::AsyncCam;

int main()
{
    AsyncCam cam(0, 640, 480);
    cam.calibrate();
    cam.start();
    int cnt = 500;
    vector<Vec3f> circles;
    EMIRO::Keyboard kb;
    while (cnt)
    {
        cnt--;
        cam.getobject(circles);
        if (kb.get_key() == 27)
        {
            std::cout << "Stopped by interrupt\n";
            break;
        }
        usleep(100000);
    }
    cam.stop();
    return 0;
}