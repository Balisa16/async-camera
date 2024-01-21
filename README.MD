# Asynchronous Camera

This project is created to avoid lag or buffering when the main thread is occupied with a multitude of computational tasks, causing significant delays. Actually this project just a part of my UAV Robot project. In my UAV Robot project i'm using Jetson Nano for computational platform and used WebCam as camera. In this case, Jetson Nano is bad to get image from camera. By asyncronusly frame updates it can solve my problem

## Dependencies
1. OpenCV