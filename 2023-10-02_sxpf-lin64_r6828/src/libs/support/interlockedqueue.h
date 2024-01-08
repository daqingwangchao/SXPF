/** @file interlockedqueue.h
 *
 * Thread-safe queue.
 */
#ifndef INTERLOCKEDQUEUE_H_
#define INTERLOCKEDQUEUE_H_

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <utility>


/** Generic queue that allows consumer and producer to reside in seperate
 *  threads
 *
 * @param T     The element type
 */
template <typename T>
class InterlockedQueue
{
public:
    /// Construct an empty queue.
    InterlockedQueue(void)
    {
        std::lock_guard<decltype(m)> lock(m);
        q.reset(new std::queue<T>());
    }

    /// Queue destructor.
    ~InterlockedQueue(void)
    {
        clear();
    }

    /// Discard all elements from the queue. 
    void clear(void)
    {
        std::lock_guard<decltype(m)> lock(m);
        while (!q->empty())
            q->pop();   // destroy queued elements while protected by m
    }

    /// @return \c true if the queue is empty, \c false otherwise
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(m);
        return q->empty();
    }

    /// Enqueue (emplace) a new element at the end of the queue.
    /// @param args     A parameter pack that is used to construct a new element
    ///                 to put into the queue.
    template <class... Args>
    void enqueue(Args&&... args)
    {
        std::lock_guard<decltype(m)> lock(m);
        q->emplace(std::forward<Args>(args)...);
        c.notify_one();
    }

    /// Unconditionally wait for an element in the queue to become available
    /// and dequeue it. This may potentially block forever.
    /// @return The dequeued element from the queue.
    T dequeue(void)
    {
        std::unique_lock<decltype(m)> lock(m);
        while (q->empty())
        {
            c.wait(lock);
        }
        T val = std::move(q->front());
        q->pop();
        return val;
    }

    /// Wait at most for timeout_us microseconds to dequeue an element into res.
    /// @param res          Output parameter.
    /// @param timeout_us   Max.\ time to wait for element to become available
    /// @return \c true if an element was dequeued, \c false if timeout expired
    ///         before an element became available.
    bool waitDequeue(T &res, int timeout_us)
    {
        std::unique_lock<decltype(m)> lock(m);

        bool not_empty = c.wait_for(lock, std::chrono::microseconds(timeout_us),
                                    [this]{ return !q->empty(); });
        if (not_empty)
        {
            res = std::move(q->front());
            q->pop();
        }
        return not_empty;
    }

    /// Wait until the queue is not empty, at most for timeout_us microseconds
    /// @param timeout_us   Max.\ time to wait for element to become available
    /// @return \c true if an element can be dequeued, \c false if timeout
    ///         expired before an element became available.
    bool waitNotEmpty(int timeout_us)
    {
        std::unique_lock<decltype(m)> lock(m);

        return c.wait_for(lock, std::chrono::microseconds(timeout_us),
                          [this]{ return !q->empty(); });
    }

    /// @return The number of elements in the queue.
    size_t size() const
    {
        std::lock_guard<decltype(m)> lock(m);
        return q->size();
    }

private:
    mutable std::mutex              m;
    std::unique_ptr<std::queue<T>>  q;
    std::condition_variable         c;
};

#endif /* INTERLOCKEDQUEUE_H_ */
