/* Original copyright info */
/*-
 * Copyright (c) 2013 Cosku Acay, http://www.coskuacay.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/* Modified by Canlin Zhang
 * Changes:
 * 1. Added incomplete struct/class support (forward declaration)
 * 2. Added unique_ptr its necessary helper functions
 */

#ifndef POOL_ALLOCATOR_H
#define POOL_ALLOCATOR_H

#include <climits>
#include <cstddef>
#include <memory>
#include <algorithm>
#include <vector>
#include <deque>
#include <cassert>

template <typename T, size_t BlockSize = 4096>
class PoolAllocator
{
public:
    /* Member types */
    using value_type = T;
    using pointer = T *;
    using const_pointer = const T *;
    using void_pointer = void *;
    using const_void_pointer = const void *;
    using reference = T &;
    using const_reference = const T &;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using propagate_on_container_copy_assignment = std::false_type;
    using propagate_on_container_move_assignment = std::true_type;
    using propagate_on_container_swap = std::true_type;
    using is_always_equal = std::true_type;

    /* Legacy Rebind struct */
    template <typename U>
    struct rebind
    {
        typedef PoolAllocator<U, BlockSize> other;
    };

    /* Member functions */
    // Default constructor
    PoolAllocator() noexcept;
    // Copy constructor
    PoolAllocator(const PoolAllocator &other) noexcept;
    // Move constructor
    PoolAllocator(PoolAllocator &&other) noexcept;
    // Templated copy
    template <class U>
    PoolAllocator(const PoolAllocator<U, BlockSize> &other) noexcept;
    // Destructor
    ~PoolAllocator() noexcept;

    // Assignment operator
    // We do not allow copy assignment for allocators
    PoolAllocator &operator=(const PoolAllocator &other) = delete;
    // Move assignment operator
    PoolAllocator &operator=(PoolAllocator &&other) noexcept;

    // Address functions
    pointer addressof(reference x) const noexcept;
    const_pointer addressof(const_reference x) const noexcept;

    // Allocation and deallocation
    pointer allocate(size_type n = 1);
    void deallocate(pointer p, size_type n = 1);

    // Construct and destory functions
    template <class U, class... Args>
    void construct(U *p, Args &&...args);
    template <class U>
    void destroy(U *p);

    // Maximum size of the pool
    size_type max_size() const noexcept;

    // Unique pointer support
    // Deleter
    struct Deleter
    {
        // Pool address so we know which pool to use for deletion
        PoolAllocator *allocator = nullptr;
        template <typename U>
        void operator()(U *ptr) const noexcept;
    };

    // make unique
    template <class... Args>
    std::unique_ptr<T, Deleter> make_unique(Args &&...args);

    // Create new object with empty constructor
    pointer new_object();

    // Create new object with arguments
    template <class... Args>
    pointer new_object(Args &&...args);

    // Delete an object
    void delete_object(pointer p);

private:
    // Allocate a memory block
    void allocateBlock();

    // Linked list to track free slots
    // Struct to store linked list nodes
    struct FreeListNode
    {
        // Previous node
        FreeListNode *prev = nullptr;
        FreeListNode *next = nullptr;
        char *ptr = nullptr;
    };
    // Free list using linked list
    FreeListNode *freeListHead_ = nullptr;
    FreeListNode *freeListEnd_ = nullptr;

    // Define a list of pointers to blocks of memory
    // Each block is a pair of pointers to the beginning and end of the block
    std::vector<std::pair<char *, char *>> blocks_;
    // Define a list of free <T> that got caught by deallocation
    std::deque<char *> free_slots_;

    // Track the ending and beginning of the current slot
    char *currentSlotBegin_ = nullptr;
    char *currentSlotAt_ = nullptr;
    char *currentSlotEnd_ = nullptr;
};

// Operators
// Operator != and ==
template <typename T1, size_t B1, typename T2, size_t B2>
bool operator==(const PoolAllocator<T1, B1> &,
                const PoolAllocator<T2, B2> &) noexcept
{
    return B1 == B2;
}

template <typename T1, size_t B1, typename T2, size_t B2>
bool operator!=(const PoolAllocator<T1, B1> &a,
                const PoolAllocator<T2, B2> &b) noexcept
{
    return !(a == b);
}

// include the implementation file
#include "pool_allocator.tcc"

#endif // POOL_ALLOCATOR_H