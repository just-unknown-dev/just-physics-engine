#include "thread_pool.h"

ThreadPool::ThreadPool(int numWorkers) {
    for (int i = 0; i < numWorkers; ++i) {
        _workers.emplace_back([this] { workerLoop(); });
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lk(_mutex);
        _stop = true;
    }
    _cv.notify_all();
    for (auto& t : _workers) {
        if (t.joinable()) t.join();
    }
}

int ThreadPool::workerCount() const {
    return static_cast<int>(_workers.size());
}

TaskGroup* ThreadPool::enqueueTask(b2TaskCallback* fn, void* taskContext) {
    auto* group = new TaskGroup();
    std::promise<void> promise;
    group->future = promise.get_future();

    {
        std::unique_lock<std::mutex> lk(_mutex);
        _queue.push({fn, taskContext, std::move(promise)});
    }
    _cv.notify_one();
    return group;
}

void ThreadPool::finishTask(TaskGroup* group) {
    group->future.get(); // blocks; rethrows any exception from the worker
    delete group;
}

void ThreadPool::workerLoop() {
    while (true) {
        WorkItem item;
        {
            std::unique_lock<std::mutex> lk(_mutex);
            _cv.wait(lk, [this] { return _stop || !_queue.empty(); });
            if (_stop && _queue.empty()) return;
            item = std::move(_queue.front());
            _queue.pop();
        }
        try {
            item.fn(item.context);
            item.promise.set_value();
        } catch (...) {
            item.promise.set_exception(std::current_exception());
        }
    }
}
