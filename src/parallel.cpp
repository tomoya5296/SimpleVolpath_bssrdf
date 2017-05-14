#include "parallel.h"

#include <cstring>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

class WorkerTask;

static thread_local int threadID;
static std::mutex workerListMutex;
static std::condition_variable condval;
static WorkerTask* workerTask;

class WorkerTask {
public:
    WorkerTask(const std::function<void(int)>& f, int csize, int tasks)
        : func{ f }
        , chunkSize{ csize }
        , nTasks { tasks } {
    }

    bool finished() const {
        return currentIndex >= nTasks && activeWorkers == 0;
    }

    const std::function<void(int)>& func;
    const int chunkSize;
    const int nTasks;
    bool isWorking = false;
    int currentIndex = 0;
    int activeWorkers = 0;
};

class WorkerTaskManager {
public:
    WorkerTaskManager(const std::function<void(int)>& f,
                      int chunkSize, int nTasks) {
        if (workerTask == nullptr) {
            workerTask = new WorkerTask(f, chunkSize, nTasks);
        }
    }

    ~WorkerTaskManager() {
        delete workerTask;
        workerTask = nullptr;
    }
};

static void workerThreadFunc(int threadIndex) {
    threadID = threadIndex;
    std::unique_lock<std::mutex> lock(workerListMutex);
    while (!workerTask->finished()) {
        if (!workerTask->isWorking) {
            condval.wait(lock);
        } else {
            int indexStart = workerTask->currentIndex;
            int indexEnd   = std::min(workerTask->nTasks, indexStart + workerTask->chunkSize);
            workerTask->currentIndex = indexEnd;
            if (workerTask->currentIndex == workerTask->nTasks) {
                workerTask->isWorking = false;
            }
            workerTask->activeWorkers++;
            lock.unlock();

            for (int i = indexStart; i < indexEnd; i++) {
                workerTask->func(i);
            }
            lock.lock();

            workerTask->activeWorkers--;
            if (workerTask->finished()) {
                printf("OK!!");
                condval.notify_all();
            }
        }
    }
}

void parallel_for(int start, int end, const std::function<void(int)>& func,
                  ParallelSchedule schedule) {
    const int nTasks = (end - start);
    const int nThreads = numSystemThreads();
    const int chunkSize = schedule == ParallelSchedule::Dynamic ? 1 : (nTasks + nThreads - 1) / nThreads;
    WorkerTaskManager manager(func, chunkSize, nTasks);

    std::vector<std::thread> threads;
    threadID = 0;
    for (int i = 0; i < nThreads - 1; i++) {
        threads.emplace_back(workerThreadFunc, i + 1);
    }

    workerListMutex.lock();
    workerTask->isWorking = true;
    workerListMutex.unlock();

    {
        std::unique_lock<std::mutex> lock(workerListMutex);
        condval.notify_all();

        while (!workerTask->finished()) {
            int indexStart = workerTask->currentIndex;
            int indexEnd   = std::min(workerTask->nTasks, indexStart + workerTask->chunkSize);
            workerTask->currentIndex = indexEnd;
            if (workerTask->currentIndex == workerTask->nTasks) {
                workerTask->isWorking = false;
            }
            workerTask->activeWorkers++;
            lock.unlock();

            for (int i = indexStart; i < indexEnd; i++) {
                workerTask->func(i);
            }
            lock.lock();
            workerTask->activeWorkers--;
        }
        condval.notify_all();
    }

    for (auto& t : threads) {
        t.join();
    }
}

int numSystemThreads() {
    return std::max(1u, std::thread::hardware_concurrency());
}

int getThreadID() {
    return threadID;
}
