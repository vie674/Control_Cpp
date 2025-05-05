#include <iostream>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

void bind_to_core(std::thread& t, int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    int result = pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        std::cerr << "Failed to set thread affinity\n";
    }
}

void task() {
    std::cout << "Thread running on core " << sched_getcpu() << "\n";
}

int main() {
    std::thread t(task);
    bind_to_core(t, 2); // bind thread to core 2
    t.join();
    return 0;
}
