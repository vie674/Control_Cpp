// pub_sub_video.cpp

#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <chrono>

cv::Mat shared_frame;
std::mutex mtx;
std::condition_variable cc;
bool frame_available = false;
bool video_end = false;

void publisher() {
    cv::VideoCapture cap("output_video.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open video file\n";
        video_end = true;
        cc.notify_all();
        return;
    }

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                video_end = true;
            }
            cc.notify_all();
            break;
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            frame.copyTo(shared_frame);
            frame_available = true;
        }
        cc.notify_all(); // Thông báo cho tất cả subscriber

        std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 30 FPS
    }
}

void subscriber(int id) {
    while (true) {
        cv::Mat local_frame;

        {
            std::unique_lock<std::mutex> lock(mtx);
            cc.wait(lock, [] { return frame_available || video_end; });

            if (video_end && !frame_available) {
                break;
            }

            shared_frame.copyTo(local_frame);
        }

        if (!local_frame.empty()) {
            cv::imshow("Subscriber " + std::to_string(id), local_frame);
            cv::waitKey(1);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "[Subscriber " << id << "] Exiting...\n";
}

int main() {
    std::thread pub(publisher);
    std::thread sub1(subscriber, 1);
    std::thread sub2(subscriber, 2);

    pub.join();
    sub1.join();
    sub2.join();

    cv::destroyAllWindows();
    return 0;
}
