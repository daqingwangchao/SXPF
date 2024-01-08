/** @file waitfreequeue.h
 *
 * Wait-free queues without dynamic allocation, based on ringbuffer.
 */
#ifndef WAITFREEQUEUE_H_
#define WAITFREEQUEUE_H_

#include <atomic>

#if (defined(_MSC_VER) && (_MSC_VER < 1900))
// in MSVC before 2015 constexpr is not supported
#define CONSTEXPR   static inline
#else
#define CONSTEXPR   constexpr
#endif


/** Round up val to next higher integer multiple of fact */
CONSTEXPR unsigned round_up(const unsigned val, const unsigned fact)
{
    return (val + fact - 1) - (val + fact - 1) % fact;
}


/** Generic queue that allows a single consumer and a single producer to reside
 *  in separate threads or processes.
 *
 * @note Attention: Upon destruction of the queue any elements still enqueued
 *       in the buffer are leaked! This is to facilitate usage in shared data
 *       segments, where only the \em last process is allowed to destroy the
 *       remaining queue elements by calling erase().
 *
 * For proper initialization the head_ and tail_ members must be cleared to 0
 * before first use. This may be done using memset or by putting the instance
 * into the .bss segment. The derived WaitFreeQueue class template below does
 * this in its constructor.
 *
 * @param T         The element type
 * @param CAPACITY  Max.\ number of elements in queue. For optimal performance
 *                  this should be a power of 2 (to speed up modulo operations).
 * @param ALIGNMENT Alignment of individual queue elements. The default is to
 *                  use the size of the element type T. Users may wish to use
 *                  the cache alignment (e.g., 128), instead, to avoid false
 *                  sharing of cache lines between threads running on different
 *                  CPU cores.
 */
template <typename T, unsigned CAPACITY, unsigned ALIGNMENT = sizeof(T)>
class SharedMemWaitFreeQueue
{
    static const unsigned ELEM_SIZE = round_up(sizeof(T), ALIGNMENT);

public:
    /// @return \c true if the queue is empty
    bool empty() const
    {
        return head_.load(std::memory_order_acquire)
            == tail_.load(std::memory_order_acquire);
    }

    /// @return \c true if the queue is full
    bool full() const
    {
        return head_.load(std::memory_order_acquire) + CAPACITY // read at head
            == tail_.load(std::memory_order_acquire);           // write at tail
    }

    /// Remove all elemets still enqueued in the ring buffer. This function must
    /// only be called by the last user going out of scope.
    void erase()
    {
        unsigned head = head_.load(std::memory_order_acquire);
        unsigned tail = tail_.load(std::memory_order_acquire);

        while (head != tail)
        {
            el_ptr(head)->~T();
            head++;
        }
    }

    /// Enqueue a new element at the end of the queue. Only one thread is
    /// allowed to call this function.
    /// @param args     A parameter pack that is used to construct a new element
    ///                 to put into the queue.
    /// @return true if successful, false if rinig buffer was already full
    template <class... Args>
    bool enqueue(Args&&... args)
    {
        unsigned head = head_.load(std::memory_order_acquire);  // other thread
        unsigned tail = tail_.load(std::memory_order_relaxed);  // our thread

        if (head + CAPACITY == tail)
            return false;   // ring buffer is full

        // construct/forward element at/to tail using placement-new
        new (el_ptr(tail)) T(std::forward<Args>(args)...);

        // advance tail index to signal new content to consumer
        tail_.store(tail + 1, std::memory_order_release);

        return true;        // success
    }

    /// Dequeue an element from the head of the queue. Only one thread is
    /// allowed to call this function.
    /// @param res  Reference to result value.
    ///
    /// @return true if successful, false if the queue is empty
    bool dequeue(T& res)
    {
        unsigned head = head_.load(std::memory_order_relaxed);  // our thread
        unsigned tail = tail_.load(std::memory_order_acquire);  // other thread

        if (head == tail)
            return false;   // ring buffer is empty

        // return element to caller
        res = std::move(*el_ptr(head));

        // clean up (now default-constructed) slot before it is overwritten by
        // another enqueue() call
        el_ptr(head)->~T();

        // advance head index to signal free space to producer
        head_.store(head + 1, std::memory_order_release);

        return true;
    }

protected:
    T* el_ptr(unsigned idx)
    {
        return reinterpret_cast<T*>(buf_ + ELEM_SIZE * (idx % CAPACITY));
    }

    union
    {
        long double d_; // unused, for alignment
        uint64_t    u_; // unused, for alignment
        void       *p_; // unused, for alignment
        char        buf_[ELEM_SIZE * CAPACITY]; ///< ring buffer memory
    };
    std::atomic<unsigned>   head_;                      ///< read index
    std::atomic<unsigned>   tail_;                      ///< write index
};


/** Generic queue that allows a single consumer and a single producer to reside
 *  in separate threads of a single process.
 *
 * This class template uses SharedMemWaitFreeQueue as a base and just adds a
 * default constructor and destructor.
 *
 * @param T         The element type
 * @param CAPACITY  Max.\ number of elements in queue
 * @param ALIGNMENT Alignment of individual queue elements. The default is to
 *                  use the size of the element type T. Users may wish to use
 *                  the cache alignment (e.g., 128), instead, to avoid false
 *                  sharing of cache lines between threads running on different
 *                  CPU cores.
 */
template <typename T, unsigned CAPACITY, unsigned ALIGNMENT = sizeof(T)>
class WaitFreeQueue :
    public  SharedMemWaitFreeQueue<T, CAPACITY, ALIGNMENT>
{
    using SMWFQ = SharedMemWaitFreeQueue<T, CAPACITY, ALIGNMENT>;

    WaitFreeQueue(const WaitFreeQueue& other) = delete;
    WaitFreeQueue(WaitFreeQueue&& other) = delete;
    WaitFreeQueue& operator =(const WaitFreeQueue& other) = delete;
    WaitFreeQueue& operator =(WaitFreeQueue&& other) = delete;

public:
    WaitFreeQueue() { SMWFQ::head_ = 0; SMWFQ::tail_ = 0; }
    ~WaitFreeQueue() { erase(); }

    using SMWFQ::empty;
    using SMWFQ::full;
    using SMWFQ::erase;
    using SMWFQ::enqueue;
    using SMWFQ::dequeue;
};

#endif /* WAITFREEQUEUE_H_ */
