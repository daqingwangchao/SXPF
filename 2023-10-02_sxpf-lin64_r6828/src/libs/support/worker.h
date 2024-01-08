#ifndef WORKER_H_
#define WORKER_H_

#include "interlockedqueue.h"
#include <thread>
#include <functional>


template <typename WORK>
using WorkHandler = std::function<void(const std::shared_ptr<WORK> &item)>;


/// Worker base class, which processes work directly when it is pushed to it
template <typename WORK>
class Worker
{
public:
    static void doNothing(const std::shared_ptr<WORK> &item) { }

    /// Default constructor that uses a no-op processing functor.
    /// The processing function can be changed by overriding the process()
    /// funtion in a derived class or by set calling setWorkHandler().
    /// Otherwise any WORK items pushed to this worker are simply discarded by
    /// doNothing().
    Worker() : process_(doNothing)
    { }

    /// Customizes the worker by supplying a processing function object.
    Worker(const WorkHandler<WORK> &process) : process_(process)
    { }
    virtual ~Worker() { }

    void setWorkHandler(WorkHandler<WORK> &&handler)
    {
        process_ = std::move(handler);
    }

    /// Pass a WORK item to this handler for processing.
    /// This function can be overridden to customize processing at the moment
    /// when a WORK item is first passed to the worker.
    virtual void push(const std::shared_ptr<WORK> &item) { process(item); }

    /// Short-hand for push(). Also allows a Worker to by used as a WorkHandler.
    virtual void operator()(const std::shared_ptr<WORK> &item) { push(item); }

protected:
    /// Actual proccessing of the WORK item happens here.
    /// This may be at a later moment than when push() was called.
    virtual void process(const std::shared_ptr<WORK> &item) { process_(item); }

    WorkHandler<WORK>   process_;
};


/// Derived worker class which delegates work pushed to it to a separate thread
template <typename WORK>
class WorkerThread : public Worker<WORK>
{
public:
    WorkerThread() : Worker<WORK>(), wq_(), hThread_([this]() { do_work(); })
    { }

    WorkerThread(WorkHandler<WORK> process) :
        Worker<WORK>(process), wq_(), hThread_([&]() { do_work(); })
    { }

    virtual ~WorkerThread()
    {
        auto reqExit = std::shared_ptr<WORK>(nullptr);
        push(reqExit);  // push NULL work item to signal thread to stop
        hThread_.join();// wait for thread to end and release thread resources
    }

    /// Defer work by enqueueing it in a work queue for the the worker thread.
    virtual void push(const std::shared_ptr<WORK> &item) { wq_.enqueue(item); }

protected:
    void do_work()
    {
        for (;;)
        {
            auto    item = wq_.dequeue();

            if (!item.get())    // check if thread is requested to end
                return;

            this->process(item);
        }
    }

private:
    InterlockedQueue<std::shared_ptr<WORK>> wq_;

    std::thread                             hThread_;
};

#endif /* WORKER_H_ */
