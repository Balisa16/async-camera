#include <async_cam.hpp>

using EMIRO::AsyncCam;

int main()
{
    AsyncCam cam(0, 640, 480);
    cam.start();
    return 0;
}