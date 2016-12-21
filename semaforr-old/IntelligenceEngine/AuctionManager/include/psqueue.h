#ifndef PSQUEUE_H
#define PSQUEUE_H

#include <vector>
#include <boost/thread.hpp>

#include <defines.h>


template <class T>
class psqueue
{
    private:
        typedef std::vector<T>::iterator iter;
        std::vector<T> qu;
        std::vector<iter> it;
        boost::mutex mut;
        boost::condition_variable cv;

    public:

        void push(const T& data)
        {
            bool wasempty = this->empty();
            boost::mutex::scoped_lock lock(mut);
            qu.push_back(data);
            for(iter i = it.begin(); i != it.end(); i++)
            {
                if(*i == qu.end())
                    *i = qu.rbegin();
            }
        }

        bool empty(iter i)
        {
            boost::mutex::scoped_lock lock(mut);
            if(i == qu.end())
                return true;
            return false;
        }

        T& front(iter i)
        {
            return *(i);
        }

        void pop(iter i)
        {
            i++;
            for(iter i = qu.begin(); i != qu.end(); i++)
            {
                bool found = false;
                for(std::vector<iter>::iterator j it.begin();
                    j != it.end(); j++)
                {
                    if(*j == *i)
                    {
                        found = true;
                        break;
                    }
                }
                if(!found)
                    qu.pop_front();
                else
                    break;
            }
        }

};
#endif

