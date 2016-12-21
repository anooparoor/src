#ifndef CQUEUE_H
#define CQUEUE_H

#include <queue>
#include <boost/thread.hpp>

#include "defines.h"


template <class T>
class cqueue
{
    private:
        std::queue<T> q_;
        boost::mutex m_;
        boost::condition_variable cv_;
    public:

        void front_and_pop(T& data)
        {
            boost::mutex::scoped_lock lock(m_);
            data = q_.front();
            q_.pop();
            return;
        }

        void push(const T& data)
        {
            boost::mutex::scoped_lock lock(m_);
            q_.push(data);
        }

        bool empty()
        {
            boost::mutex::scoped_lock lock(m_);
            return q_.empty();
        }

        T& front()
        {
            boost::mutex::scoped_lock lock(m_);
            return q_.front();
        }

        void pop()
        {
            boost::mutex::scoped_lock lock(m_);
            q_.pop();
        }
};
#endif
