/**
 * Copyright (c) 2020-2021 CutDigital Ltd.
 * All rights reserved.
 * 
 * NOTE: This file is licensed under GPL-3.0-or-later (default). 
 * A commercial license can be purchased from CutDigital Ltd. 
 *  
 * License details:
 * 
 * (A)  GNU General Public License ("GPL"); a copy of which you should have 
 *      recieved with this file.
 * 	    - see also: <http://www.gnu.org/licenses/>
 * (B)  Commercial license.
 *      - email: contact@cut-digital.com
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

namespace mcut{

class function_wrapper
{
private:

    struct impl_base
    {
        virtual void call()=0;
        virtual ~impl_base(){}
    };

    std::unique_ptr<impl_base> impl;

    template<typename F>
    struct impl_type : impl_base
    {
        F f;
        impl_type(F&& f_) : f(std::move(f_)){}
        void call(){ return f();}
    };
    
public:

    template<typename F>
    function_wrapper(F&& f) : impl(new impl_type<F>(std::move(f)))
    {}

    void operator()() { impl->call(); }

    function_wrapper() = default;

    function_wrapper(function_wrapper&& other) : 
        impl (std::move(other.impl))
    {}

    function_wrapper& operator =(function_wrapper&& other)
    {
        impl = std::move(other.impl);
        return *this;
    }

    function_wrapper(const function_wrapper&) = delete;
    function_wrapper(function_wrapper&)=delete;
    function_wrapper& operator=(const function_wrapper&)=delete;
};

template<typename T>
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
    node* tail;
    std::condition_variable data_cond;

    std::unique_ptr<node> try_pop_head()
    {
        std::lock_guard<std::mutex> head_lock(head_mutex);
        if(head.get() == get_tail())
        {
            return std::unique_ptr<node>(nullptr);
        }
        return pop_head();
    }

    std::unique_ptr<node> try_pop_head(T& value)
    {
        std::lock_guard<std::mutex> head_lock(head_mutex);
        if(head.get() == get_tail())
        {
            return std::unique_ptr<node>(nullptr);
        }
        value = std::move(*head->data);
        return pop_head();
    }

    node* get_tail()
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
        data_cond.wait(head_lock,[&]{return head.get()!=get_tail();});
        return std::move(head_lock);          
    }
    std::unique_ptr<node> wait_pop_head()
    {
        std::unique_lock<std::mutex> head_lock(wait_for_data());    
        return pop_head();
    }
    std::unique_ptr<node> wait_pop_head(T& value)
    {
        std::unique_lock<std::mutex> head_lock(wait_for_data());    
        value=std::move(*head->data);
        return pop_head();
    }

    public:

    thread_safe_queue() : head(new node), tail(head.get()){}
    thread_safe_queue(const thread_safe_queue& other)=delete;
    thread_safe_queue& operator=(const thread_safe_queue& other)= delete;

    void push(T new_value)
    {
        std::shared_ptr<T> new_data(std::make_shared<T>(std::move(new_value)));
        std::unique_ptr<node> p(new node);
        {
            std::lock_guard<std::mutex> tail_lock(tail_mutex);
            tail->data = new_data;
            node* const new_tail = p.get();
            tail->next = std::move(p);
            tail=new_tail;
        }
        data_cond.notify_one();
    }

    std::shared_ptr<T> wait_and_pop()
    {
        std::unique_ptr<node> const old_head=wait_pop_head();
        return old_head->data;
    }
    void wait_and_pop(T& value)
    {
        std::unique_ptr<node> const old_head=wait_pop_head(value);
    }

    std::shared_ptr<T> try_pop()
    {
        std::unique_ptr<node> old_head = try_pop_head();
        return old_head ? old_head->data: std::shared_ptr<T>();
    }

    bool try_pop(T& value)
    {
        std::unique_ptr<node> const old_head = try_pop_head(value);
        return !(!(old_head)); // https://stackoverflow.com/questions/30521849/error-on-implicit-cast-from-stdunique-ptr-to-bool
    }

    void empty()
    {
        std::lock_guard<std::mutex> head_lock(head_mutex);
        return (head.get()==get_tail());
    }
};

class join_threads
{
    std::vector<std::thread>& threads;
public:
    explicit join_threads(std::vector<std::thread>& threads_):
        threads(threads_)
    {}
    ~join_threads()
    {
        for(unsigned long i=0;i<threads.size();++i)
        {
            if(threads[i].joinable())
                threads[i].join();
        }
    }
};

class thread_pool
{
    std::atomic_bool done;
    thread_safe_queue<function_wrapper> work_queue;
    std::vector<std::thread> threads; // NOTE: must be declared after "done" and "work_queue"
    join_threads joiner;
    
    void worker_thread()
    {
        while (!done)
        {
            function_wrapper task;
            if(work_queue.try_pop(task))
            {
                task();
            }
            else
            {
                std::this_thread::yield();
            }
        }
    }

    public: 

    thread_pool() : done(false), joiner(threads) {
        unsigned const thread_count = std::thread::hardware_concurrency();

        printf("[MCUT]: thread count = %d\n", (int)thread_count);

        try
        {
            for(unsigned i =0; i < thread_count; ++i)
            {
                threads.push_back(std::thread(&thread_pool::worker_thread, this));
            }
        }
        catch(...)
        {
            done=true;
            throw;
        }
        
    }

    ~thread_pool()
    {
        done = true;
    }

public:

    template<typename FunctionType>
    std::future<typename std::result_of<FunctionType()>::type> submit(FunctionType f)
    {
        typedef typename std::result_of<FunctionType()>::type result_type;

        std::packaged_task<result_type()> task(std::move(f));
        std::future<result_type> res(task.get_future());
        work_queue.push(std::move(task));
        return res;
    }

    size_t get_num_threads() const
    {
        return threads.size();
    }
};

template<typename InputStorageIteratorType, typename OutputStorageType, typename FunctionType >
void parallel_fork_and_join(
    thread_pool& pool, 
    // start of data structure to be processed in parallel
    const InputStorageIteratorType& first,
    // endof of data structure to be processed in parallel
    const InputStorageIteratorType& last,
    // the ideal size of the block assigned to each thread
    unsigned long const block_size_default,
    // the function that is executed on a sub-block of element within the range [first, last)
    FunctionType& task_func,
    // the part of the result/output that is computed by the master thread (i.e. the one that is scheduling)
    // NOTE: this result must be merged which the output computed for the other 
    // sub-block in the input ranges. This other data is accessed from the std::futures
    OutputStorageType& master_thread_output,
    // Future promises of data (to be merged) that is computed by worker threads
    std::vector<std::future<OutputStorageType> >& futures)
{

    unsigned long const length = std::distance(first, last);
    unsigned long const block_size = std::min(block_size_default, length);
    unsigned long const num_blocks=(length+block_size-1)/block_size;
    
    futures.resize(num_blocks-1);
    InputStorageIteratorType block_start=first;

    for(unsigned long i=0;i<(num_blocks-1);++i)
    {
        InputStorageIteratorType block_end = block_start;
        std::advance(block_end,block_size);

        futures[i]=pool.submit(
            [&, block_start, block_end]() -> OutputStorageType
            { 
                return task_func(block_start, block_end); 
            });

        block_start=block_end;
    }            
    
    master_thread_output = task_func(block_start, last); 
}

} // namespace mcut{

#endif // MCUT_SCHEDULER_H_