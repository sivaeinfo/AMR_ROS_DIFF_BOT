#pragma once
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <queue>
#include <vector>

namespace rpos { namespace system {

    /// \brief Execute a for-loop process for an array in parallel
    /// \param n The number of iterations. I.e., { 0, 1, ..., n - 1 } will be visited.
    /// \param function The function that will be called in the for-loop. This can be specified as a lambda expression. The type should be equivalent to boost::function<void(int)>.
    /// \param target_concurrency The number of threads that will be generated. When this is set to zero (which is the default), the hardware concurrency will be automatically used.
    template<typename Callable>
    void parallel_for(int n, Callable function, int target_concurrency = 0)
    {
        const int hint = (target_concurrency == 0) ? boost::thread::hardware_concurrency() : target_concurrency;
        const int n_threads = std::min(n, (hint == 0) ? 4 : hint);

        auto inner_loop = [&](const int thread_index)
        {
            for (int pos = 0; pos < n; pos++) {
                if ((pos % n_threads) != thread_index) { continue; }
                function(pos);
            }
        };
        std::vector<boost::thread> threads;
        for (int pos = 0; pos < n_threads; ++pos) { threads.push_back(boost::thread(inner_loop, pos)); }
        for (auto iter = threads.begin(); iter != threads.end(); iter ++) { iter->join(); }
    }

} }
