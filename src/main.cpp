#include <async_cam.hpp>

using EMIRO::AsyncCam;

int main()
{
    AsyncCam cam(0, 640, 480);
    cam.start();
    int cnt = 500;
    vector<Vec3f> circles;
    while (cnt)
    {
        cnt--;
        cam.getobject(circles);
        usleep(100000);
    }
    cam.stop();
    return 0;
}