#pragma once

#include <boost/thread.hpp>


namespace rpos { namespace system { namespace util {

    template<typename ResultT>
    class Waiter
    {
    public:
        Waiter()
            : flag_(false)
        {}
        ~Waiter()
        {}
        void wait() {
            boost::unique_lock<boost::shared_mutex> locker_(mutex_);
            while (!flag_)
                condition_.wait(locker_);
            flag_ = false;
        }
        template <class Rep_, class Per_>
        void wait_for(const boost::chrono::duration<Rep_, Per_>& timeout_) {
            boost::unique_lock<boost::shared_mutex> locker_(mutex_);
            while (!flag_)
            {
                auto wait_result = condition_.wait_for(locker_, timeout_);
                if (boost::cv_status::timeout == wait_result)
                    break;
            }
            flag_ = false;
        }
        ResultT waitForResult()
        {
            this->wait();
            return result_;
        }
        void notify(ResultT result)
        {
            boost::unique_lock<boost::shared_mutex> locker_(mutex_);
            this->result_ = result;
            flag_ = true;
            condition_.notify_all();
        }
        void notify() {
            flag_ = true;
            condition_.notify_all();
        }
    private:
        boost::shared_mutex mutex_;
        boost::condition_variable_any condition_;
        bool flag_;
        volatile ResultT result_;
    };

}}}