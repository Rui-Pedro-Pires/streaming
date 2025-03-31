#include <opencv2/opencv.hpp>
#include <zenoh.hxx>
#include <iostream>
#include <vector>
#include <csignal>
#include <thread>

bool running = true;
std::mutex frame_mutex;
std::condition_variable frame_cv;
std::queue<cv::Mat> frame_queue;
bool new_frame = false;

void signalHandler(int signum) {
    std::cout << "Interrupt received, shutting down..." << std::endl;
    running = false;
    frame_cv.notify_all(); // Wake up display thread if it's waiting
}

// This function will be executed in a worker thread, NOT the main thread
void subscriber_thread_function(zenoh::Session session) {
    auto subscriber = session.declare_subscriber(
        "video/stream",
        [](const zenoh::Sample& sample)
        {
            std::cout << "Received video frame" << std::endl;
            std::vector<uchar> buffer(sample.get_payload().as_vector());

            if (buffer.empty()) {
                std::cerr << "Received empty buffer" << std::endl;
                return;
            }

            // Try to decode using various methods
            cv::Mat frame;
            
            // First try standard JPEG/PNG decoding
            frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
            
            if (!frame.empty()) {
                // Queue the frame for display on main thread
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    // Limit queue size to avoid memory issues
                    if (frame_queue.size() > 5) {
                        frame_queue.pop(); // Remove oldest frame if queue is too long
                    }
                    frame_queue.push(frame.clone()); // Clone to ensure data ownership
                    new_frame = true;
                }
                frame_cv.notify_one(); // Notify main thread
                
                std::cout << "Frame queued for display: " << frame.cols << "x" << frame.rows << std::endl;
            }
        }, zenoh::closures::none);
    
    std::cout << "Subscription active, waiting for frames..." << std::endl;
    
    // Keep the subscriber thread running until program terminates
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Subscriber thread shutting down" << std::endl;
}

int main() {
    // Register signal handler
    signal(SIGINT, signalHandler);

    // Initialize Zenoh session
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));

    // Start subscriber in a background thread, leaving main thread for UI
    std::thread zenoh_thread(subscriber_thread_function, std::move(session));

    std::cout << "Subscribing to video/stream..." << std::endl;

    // Create window in the main thread
    cv::namedWindow("Video Stream", cv::WINDOW_AUTOSIZE);
    
    // Main thread loop for displaying frames
    while (running) {
        cv::Mat frame_to_show;
        bool got_frame = false;
        
        {
            std::unique_lock<std::mutex> lock(frame_mutex);
            if (new_frame) {
                if (!frame_queue.empty()) {
                    frame_to_show = frame_queue.front();
                    frame_queue.pop();
                    got_frame = true;
                    
                    // Reset the flag if the queue is now empty
                    new_frame = !frame_queue.empty();
                }
            } else {
                // Wait for a short time or until a new frame arrives
                frame_cv.wait_for(lock, std::chrono::milliseconds(100), 
                                 [&]{ return new_frame || !running; });
            }
        }
        
        // Display the frame outside the lock (in the main thread)
        if (got_frame && !frame_to_show.empty()) {
            cv::imshow("Video Stream", frame_to_show);
        }
        
        // Process UI events with a short timeout - MUST be called in the main thread
        int key = cv::waitKey(10);
        if (key == 27) { // ESC key
            running = false;
        }
    }
    
    // Clean up
    cv::destroyAllWindows();
    
    // Wait for the Zenoh thread to finish
    if (zenoh_thread.joinable()) {
        zenoh_thread.join();
    }
    
    std::cout << "Program terminated gracefully" << std::endl;
    return 0;
}