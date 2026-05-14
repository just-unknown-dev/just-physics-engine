#pragma once

#include <condition_variable>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

// Box2D 3.x simplified task callback — just a context pointer, no range indices.
typedef void b2TaskCallback(void* taskContext);

// Returned by enqueueTask; consumed by finishTask to wait for completion.
struct TaskGroup {
    std::future<void> future;
};

class ThreadPool {
public:
    explicit ThreadPool(int numWorkers);
    ~ThreadPool();

    // Box2D 3.x callback bridges.
    // Returns a heap-allocated TaskGroup*; caller must pass it to finishTask().
    TaskGroup* enqueueTask(b2TaskCallback* fn, void* taskContext);

    // Blocks until the task completes, then deletes the group.
    void finishTask(TaskGroup* group);

    int workerCount() const;

private:
    struct WorkItem {
        b2TaskCallback*    fn;
        void*              context;
        std::promise<void> promise;
    };

    void workerLoop();

    std::vector<std::thread>  _workers;
    std::queue<WorkItem>      _queue;
    std::mutex                _mutex;
    std::condition_variable   _cv;
    bool                      _stop = false;
};
