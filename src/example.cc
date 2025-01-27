#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>


struct Data {
    int value;
};

class Example {
    public:
        void initialize_thread() {
            thread = std::thread(&Example::thread_function, this);
        };
        void stop_thread() {
            running = false;
            thread.join();
        };
        Data data {0};
        std::mutex mutex;

    private:
        void thread_function() {
            while (running) {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    std::cout << "Data value: " << data.value << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        };
        std::atomic<bool> running{true};
        std::thread thread;
};


int main(int argc, char *argv[]) {
    // Create an instance of the Example class and start the thread
    Example example;
    example.initialize_thread();

    // Update the data value
    for (int i = 0; i < 10; i++) {
        {
            std::lock_guard<std::mutex> lock(example.mutex);
            example.data.value = i;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Stop the thread
    example.stop_thread();
    std::cout << "Thread Ending " << std::endl;

    return 0;
}
