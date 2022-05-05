/**
 * Copyright (c) 2021-2022 Floyd M. Chitalu.
 * All rights reserved.
 * 
 * NOTE: This file is licensed under GPL-3.0-or-later (default). 
 * A commercial license can be purchased from Floyd M. Chitalu. 
 *  
 * License details:
 * 
 * (A)  GNU General Public License ("GPL"); a copy of which you should have 
 *      recieved with this file.
 * 	    - see also: <http://www.gnu.org/licenses/>
 * (B)  Commercial license.
 *      - email: floyd.m.chitalu@gmail.com
 * 
 * The commercial license options is for users that wish to use MCUT in 
 * their products for comercial purposes but do not wish to release their 
 * software products under the GPL license. 
 * 
 * Author(s)     : Floyd M. Chitalu
 */

#ifndef MCUT_SCHEDULER_H_
#define MCUT_SCHEDULER_H_

#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <future>
#include <memory>
#include <vector>

namespace mcut
{

    class function_wrapper
    {
    private:
        struct impl_base
        {
            virtual void call() = 0;
            virtual ~impl_base() {}
        };

        std::unique_ptr<impl_base> impl;

        template <typename F>
        struct impl_type : impl_base
        {
            F f;
            impl_type(F &&f_) : f(std::move(f_)) {}
            void call() { return f(); }
        };

    public:
        template <typename F>
        function_wrapper(F &&f) : impl(new impl_type<F>(std::move(f)))
        {
        }

        void operator()() { impl->call(); }

        function_wrapper() = default;

        function_wrapper(function_wrapper &&other) : impl(std::move(other.impl))
        {
        }

        function_wrapper &operator=(function_wrapper &&other)
        {
            impl = std::move(other.impl);
            return *this;
        }

        function_wrapper(const function_wrapper &) = delete;
        function_wrapper(function_wrapper &) = delete;
        function_wrapper &operator=(const function_wrapper &) = delete;
    };

    template <typename T>
    class thread_safe_queue
    {
    private:
        struct node
        {
            std::shared_ptr<T> data;
            std::unique_ptr<node> next;
        };

        std::mutex head_mutex;
        std::unique_ptr<node> head;
        std::mutex tail_mutex;
        node *tail;
        std::condition_variable data_cond;
        std::atomic_bool can_wait_for_data;

        std::unique_ptr<node> try_pop_head(T &value)
        {
            std::lock_guard<std::mutex> head_lock(head_mutex);
            if (head.get() == get_tail())
            {
                return std::unique_ptr<node>(nullptr);
            }
            value = std::move(*head->data);
            return pop_head();
        }

        node *get_tail()
        {
            std::lock_guard<std::mutex> tail_lock(tail_mutex);
            return tail;
        }

        // dont call directly
        std::unique_ptr<node> pop_head()
        {
            std::unique_ptr<node> old_head = std::move(head);
            head = std::move(old_head->next);
            return (old_head);
        }

        std::unique_lock<std::mutex> wait_for_data()
        {
            std::unique_lock<std::mutex> head_lock(head_mutex);
            auto until = [&]()
            { return can_wait_for_data.load() == false || head.get() != get_tail(); };
            data_cond.wait(head_lock, until);
            return head_lock;
        }

        std::unique_ptr<node> wait_pop_head(T &value)
        {
            std::unique_lock<std::mutex> head_lock(wait_for_data());
            if (can_wait_for_data.load())
            {
                value = std::move(*head->data);
                return pop_head();
            }
            else
            {
                return std::unique_ptr<node>(nullptr);
            }
        }

    public:
        thread_safe_queue() : head(new node), tail(head.get()), can_wait_for_data(true) {}
        thread_safe_queue(const thread_safe_queue &other) = delete;
        thread_safe_queue &operator=(const thread_safe_queue &other) = delete;

        void disrupt_wait_for_data()
        {
            can_wait_for_data.store(false);
            data_cond.notify_one();
        }

        void push(T new_value)
        {
            std::shared_ptr<T> new_data(std::make_shared<T>(std::move(new_value)));
            std::unique_ptr<node> p(new node);
            {
                std::lock_guard<std::mutex> tail_lock(tail_mutex);
                tail->data = new_data;
                node *const new_tail = p.get();
                tail->next = std::move(p);
                tail = new_tail;
            }
            data_cond.notify_one();
        }

        void wait_and_pop(T &value)
        {
            std::unique_ptr<node> const old_head = wait_pop_head(value);
        }

        bool try_pop(T &value)
        {
            std::unique_ptr<node> const old_head = try_pop_head(value);
            return !(!(old_head)); // https://stackoverflow.com/questions/30521849/error-on-implicit-cast-from-stdunique-ptr-to-bool
        }

        void empty()
        {
            std::lock_guard<std::mutex> head_lock(head_mutex);
            return (head.get() == get_tail());
        }
    };

    class join_threads
    {
        std::vector<std::thread> &threads;

    public:
        explicit join_threads(std::vector<std::thread> &threads_) : threads(threads_)
        {
        }
        ~join_threads()
        {
            for (unsigned long i = 0; i < threads.size(); ++i)
            {
                if (threads[i].joinable())
                    threads[i].join();
            }
        }
    };

    class thread_pool
    {
        std::atomic_bool terminate;

        std::vector<thread_safe_queue<function_wrapper>> work_queues;

        std::vector<std::thread> threads; // NOTE: must be declared after "terminate" and "work_queues"
        join_threads joiner;
        unsigned long long round_robin_scheduling_counter;

        bool try_pop_from_other_thread_queue(function_wrapper &task, const int worker_thread_id)
        {
            const unsigned num_work_queues = (unsigned)work_queues.size();
            for (unsigned i = 0; i < num_work_queues; ++i)
            {
                unsigned const other_worker_thread_id = (worker_thread_id + i + 1) % num_work_queues;
                if (work_queues[other_worker_thread_id].try_pop(task))
                {
                    return true;
                }
            }
            return false;
        }



        void worker_thread(int worker_thread_id)
        {

            do
            {
                function_wrapper task;
#if 0
                work_queues[worker_thread_id].wait_and_pop(task);
                if(terminate) {
                   break; // finished (i.e. MCUT context was destroyed)
                }
                task();
#else

                // if I can't pop any task from my queue, and I can't steal a task from
                // another thread's queue, then I'll just wait until is added to my queue.
                if (!(work_queues[worker_thread_id].try_pop(task) || try_pop_from_other_thread_queue(task, worker_thread_id)))
                {
                    work_queues[worker_thread_id].wait_and_pop(task);
                }

                if (terminate)
                {
                    break; // finished (i.e. MCUT context was destroyed)
                }

                task(); // run the task
#endif
            } while (true);
        }

    public:

        thread_pool() : terminate(false),

                        joiner(threads), round_robin_scheduling_counter(0)
        {
            unsigned int const thread_count = std::thread::hardware_concurrency();

            try
            {

                work_queues = std::vector<thread_safe_queue<function_wrapper>>(
                    thread_count);

                for (unsigned i = 0; i < thread_count; ++i)
                {

                    threads.push_back(std::thread(&thread_pool::worker_thread, this, i));
                }
            }
            catch (...)
            {
                terminate = true;
                wakeup_and_shutdown();
                throw;
            }
        }

        ~thread_pool()
        {
            terminate = true;
            wakeup_and_shutdown();
        }

        // submit empty task so that worker threads can wake up
        // with a valid (but redundant) task to then exit
        void wakeup_and_shutdown()
        {
            for (unsigned i = 0; i < get_num_threads(); ++i)
            {
                work_queues[i].disrupt_wait_for_data();
            }
        }

    public:
        /*
        The thread pool takes care of the exception safety too. Any exception thrown by the
        task gets propagated through the std::future returned from submit() , and if the function
        exits with an exception, the thread pool destructor abandons any not-yet-completed
        tasks and waits for the pool threads to finish.
    */
        template <typename FunctionType>
        std::future<typename std::result_of<FunctionType()>::type> submit(FunctionType f)
        {
            typedef typename std::result_of<FunctionType()>::type result_type;

            std::packaged_task<result_type()> task(std::move(f));
            std::future<result_type> res(task.get_future());

            unsigned long long worker_thread_id = (round_robin_scheduling_counter++) % (unsigned long long)get_num_threads();

            //printf("[MCUT]: submit to thread %d\n", (int)worker_thread_id);

            work_queues[worker_thread_id].push(std::move(task));

            return res;
        }

        size_t get_num_threads() const
        {
            return threads.size();
        }
    };

    template <typename InputStorageIteratorType, typename OutputStorageType, typename FunctionType>
    void parallel_fork_and_join(
        thread_pool &pool,
        // start of data elements to be processed in parallel
        const InputStorageIteratorType &first,
        // end of of data elements to be processed in parallel (e.g. std::map::end())
        const InputStorageIteratorType &last,
        // the ideal size of the block assigned to each thread
        typename InputStorageIteratorType::difference_type const block_size_default,
        // the function that is executed on a sub-block of element within the range [first, last)
        FunctionType &task_func,
        // the part of the result/output that is computed by the master thread (i.e. the one that is scheduling)
        // NOTE: this result must be merged which the output computed for the other
        // sub-block in the input ranges. This other data is accessed from the std::futures
        OutputStorageType &master_thread_output,
        // Future promises of data (to be merged) that is computed by worker threads
        std::vector<std::future<OutputStorageType>> &futures)
    {

        typename InputStorageIteratorType::difference_type const length = std::distance(first, last);
        typename InputStorageIteratorType::difference_type const block_size = std::min(block_size_default, length);
        typename InputStorageIteratorType::difference_type const num_blocks = (length + block_size - 1) / block_size;

        //std::cout << "length=" << length << " block_size=" << block_size << " num_blocks=" << num_blocks << std::endl;

        futures.resize(num_blocks - 1);
        InputStorageIteratorType block_start = first;

        for (typename InputStorageIteratorType::difference_type i = 0; i < (num_blocks - 1); ++i)
        {
            InputStorageIteratorType block_end = block_start;
            std::advance(block_end, block_size);

            futures[i] = pool.submit(
                [&, block_start, block_end]() -> OutputStorageType
                {
                    return task_func(block_start, block_end);
                });

            block_start = block_end;
        }

        master_thread_output = task_func(block_start, last);
    }

} // namespace mcut{

#endif // MCUT_SCHEDULER_H_