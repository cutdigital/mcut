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

#include "pool_allocator.h"

#ifndef POOL_ALLOCATOR_TCC
#define POOL_ALLOCATOR_TCC

// Default constructor
template <typename T, size_t BlockSize>
PoolAllocator<T, BlockSize>::PoolAllocator() noexcept
{
    // Initialize all vectors to empty
    blocks_.clear();
    free_slots_.clear();
    currentSlotBegin_ = nullptr;
    currentSlotAt_ = nullptr;
    currentSlotEnd_ = nullptr;
    freeListHead_ = nullptr;
    freeListEnd_ = nullptr;
}

// Copy constructor - do nothing
template <typename T, size_t BlockSize>
PoolAllocator<T, BlockSize>::PoolAllocator(const PoolAllocator &other) noexcept
{
}

// Move constructor
template <typename T, size_t BlockSize>
PoolAllocator<T, BlockSize>::PoolAllocator(PoolAllocator &&other) noexcept
{
    blocks_ = std::move(other.blocks_);
    free_slots_ = std::move(other.free_slots_);
    currentSlotBegin_ = other.currentSlotBegin_;
    currentSlotAt_ = other.currentSlotAt_;
    currentSlotEnd_ = other.currentSlotEnd_;
    freeListHead_ = other.freeListHead_;
    freeListEnd_ = other.freeListEnd_;

    other.blocks_.clear();
    other.free_slots_.clear();
    other.currentSlotBegin_ = nullptr;
    other.currentSlotAt_ = nullptr;
    other.currentSlotEnd_ = nullptr;
    other.freeListHead_ = nullptr;
    other.freeListEnd_ = nullptr;
}

// Templated copy - do nothing
template <typename T, size_t BlockSize>
template <class U>
PoolAllocator<T, BlockSize>::PoolAllocator(const PoolAllocator<U, BlockSize> &other) noexcept
{
}

// Destructor
template <typename T, size_t BlockSize>
PoolAllocator<T, BlockSize>::~PoolAllocator() noexcept
{
    // Free all blocks of memory
    for (auto &block : blocks_)
    {
        ::operator delete[](block.first);
    }

    // Free linked list from head to end
    FreeListNode *current = freeListHead_;
    while (current != nullptr)
    {
        FreeListNode *next = current->next;
        delete current;
        current = next;
    }
    freeListHead_ = nullptr;
    freeListEnd_ = nullptr;
}

// Move assignment operator
template <typename T, size_t BlockSize>
PoolAllocator<T, BlockSize> &
PoolAllocator<T, BlockSize>::operator=(PoolAllocator &&other) noexcept
{
    if (this != &other)
    {
        // Move the blocks and free slots
        blocks_ = std::move(other.blocks_);
        free_slots_ = std::move(other.free_slots_);
        currentSlotBegin_ = other.currentSlotBegin_;
        currentSlotAt_ = other.currentSlotAt_;
        currentSlotEnd_ = other.currentSlotEnd_;
        freeListHead_ = other.freeListHead_;
        freeListEnd_ = other.freeListEnd_;

        // Clear the other allocator
        other.blocks_.clear();
        other.free_slots_.clear();
        other.currentSlotBegin_ = nullptr;
        other.currentSlotAt_ = nullptr;
        other.currentSlotEnd_ = nullptr;
        other.freeListHead_ = nullptr;
        other.freeListEnd_ = nullptr;
    }

    return *this;
}

// Address functions
template <typename T, size_t BlockSize>
typename PoolAllocator<T, BlockSize>::pointer
PoolAllocator<T, BlockSize>::addressof(reference x) const noexcept
{
    return std::addressof(x);
}

template <typename T, size_t BlockSize>
typename PoolAllocator<T, BlockSize>::const_pointer
PoolAllocator<T, BlockSize>::addressof(const_reference x) const noexcept
{
    return std::addressof(x);
}

// Allocation functions
// Allocate a memory block
template <typename T, size_t BlockSize>
void PoolAllocator<T, BlockSize>::allocateBlock()
{
    // Allocate raw pointer for the block
    char *block_begin = reinterpret_cast<char *>(::operator new[](BlockSize));
    char *block_end = block_begin + BlockSize;
    // Store the block in the vector
    blocks_.emplace_back(block_begin, block_end);

    // Perform alignment
    size_t usable_size = BlockSize;
    void *unaligned_begin = static_cast<void *>(block_begin);
    // Get aligned pointer
    void *aligned_begin = std::align(alignof(T), sizeof(T), unaligned_begin, usable_size);
    if (aligned_begin == nullptr)
    {
        throw std::bad_alloc(); // Handle allocation failure
    }

    // Check number of usable slots
    size_t num_slots = usable_size / sizeof(T);
    if (num_slots == 0)
    {
        throw std::bad_alloc(); // Handle allocation failure
    }
    // Set the current slot pointers
    currentSlotBegin_ = reinterpret_cast<char *>(aligned_begin);
    currentSlotAt_ = currentSlotBegin_;
    currentSlotEnd_ = currentSlotBegin_ + num_slots * sizeof(T);
}

// Allocate a single object
template <typename T, size_t BlockSize>
typename PoolAllocator<T, BlockSize>::pointer
PoolAllocator<T, BlockSize>::allocate(size_type n)
{
    if (n == 0)
    {
        throw std::bad_alloc(); // Handle zero allocation request
    }
    else if (n > 1)
    {
        // For multiple objects, use std::allocator to allocate
        // Current implementation always assume n is a std::allocator<T> pointer
        // and not a PoolAllocator pointer.
        return std::allocator<T>().allocate(n);
    }
    // If we have free slots, use them
    else if (freeListEnd_ != nullptr)
    {
        // Sanity check
        assert(freeListHead_ != nullptr);
        assert(freeListEnd_->next == nullptr);

        // Get the first free slot
        char *p = reinterpret_cast<char *>(freeListEnd_->ptr);
        // Remove the node from free list
        if (freeListEnd_ == freeListHead_)
        {
            // If this is the only node, reset the head and end
            delete freeListEnd_;
            freeListHead_ = nullptr;
            freeListEnd_ = nullptr;
        }
        // Remove the end node
        else
        {
            FreeListNode *prevNode = freeListEnd_->prev;
            delete freeListEnd_;
            prevNode->next = nullptr;
            freeListEnd_ = prevNode;
        }

        return reinterpret_cast<pointer>(p);
    }
    // Increment from block
    else
    {
        // If new block is needed, allocate it
        if (currentSlotAt_ >= currentSlotEnd_ || currentSlotBegin_ == nullptr)
        {
            // Allocate a new block
            allocateBlock();
        }
        // Get the current slot pointer
        pointer p = reinterpret_cast<pointer>(currentSlotAt_);
        // Increment the current slot pointer
        currentSlotAt_ += sizeof(T);
        return p;
    }
}

// Deallocate a single object
template <typename T, size_t BlockSize>
void PoolAllocator<T, BlockSize>::deallocate(pointer p, size_type n)
{
    // If n is zero, do nothing
    if (n == 0)
    {
        return;
    }
    // If null pointer, do nothing
    else if (p == nullptr)
    {
        return;
    }
    // If n is greater than 1, use std::allocator to deallocate
    else if (n > 1)
    {
        std::allocator<T>().deallocate(p, n);
        return;
    }
    // If n is 1, deallocate single object
    else
    {
        // If free list is empty, create a new node
        if (freeListHead_ == nullptr)
        {
            assert(freeListEnd_ == nullptr);
            freeListHead_ = new FreeListNode();
            freeListHead_->ptr = reinterpret_cast<char *>(p);
            freeListHead_->prev = nullptr;
            freeListHead_->next = nullptr;
            freeListEnd_ = freeListHead_;
        }
        // If free list is not empty, add to the end of the list
        else
        {
            assert(freeListEnd_ != nullptr);
            FreeListNode *newNode = new FreeListNode();
            newNode->ptr = reinterpret_cast<char *>(p);
            newNode->prev = freeListEnd_;
            newNode->next = nullptr;
            freeListEnd_->next = newNode;
            freeListEnd_ = newNode;
        }
    }
}

// Construct an object in the allocated memory
template <typename T, size_t BlockSize>
template <class U, class... Args>
void PoolAllocator<T, BlockSize>::construct(U *p, Args &&...args)
{
    // Use placement new to construct the object in the allocated memory
    new (p) U(std::forward<Args>(args)...);
}
// Destroy an object in the allocated memory
template <typename T, size_t BlockSize>
template <class U>
void PoolAllocator<T, BlockSize>::destroy(U *p)
{
    // Call the destructor of the object
    p->~U();
}

// Maximum size of the pool
template <typename T, size_t BlockSize>
typename PoolAllocator<T, BlockSize>::size_type
PoolAllocator<T, BlockSize>::max_size() const noexcept
{
    // Calculate the maximum number of objects that can be allocated in a block
    return size_type(-1) / sizeof(T);
}

// Unique pointer support
// Deleter for unique_ptr
template <typename T, size_t BlockSize>
template <typename U>
void PoolAllocator<T, BlockSize>::Deleter::operator()(U *ptr) const noexcept
{
    static_assert(sizeof(U) > 0, "Deleter cannot be used with incomplete types");
    if (allocator && ptr)
    {
        allocator->destroy(ptr);
        allocator->deallocate(ptr, 1); // Deallocate a single object
    }
}

// Create a unique pointer with a custom deleter
template <typename T, size_t BlockSize>
template <class... Args>
inline std::unique_ptr<T, typename PoolAllocator<T, BlockSize>::Deleter>
PoolAllocator<T, BlockSize>::make_unique(Args &&...args)
{
    pointer raw = allocate(1);
    construct(raw, std::forward<Args>(args)...);
    return std::unique_ptr<T, Deleter>(raw, Deleter{this});
}

// Create a new object in the pool
// Default constructor
template <typename T, size_t BlockSize>
typename PoolAllocator<T, BlockSize>::pointer
PoolAllocator<T, BlockSize>::new_object()
{
    // Allocate a single object
    pointer p = allocate(1);
    // Construct the object in the allocated memory
    construct(p);
    return p;
}

// Create a new object in the pool with arguments
template <typename T, size_t BlockSize>
template <class... Args>
typename PoolAllocator<T, BlockSize>::pointer
PoolAllocator<T, BlockSize>::new_object(Args &&...args)
{
    // Allocate a single object
    pointer p = allocate(1);
    // Construct the object in the allocated memory with arguments
    construct(p, std::forward<Args>(args)...);
    return p;
}

// Delete an object in the pool
template <typename T, size_t BlockSize>
void PoolAllocator<T, BlockSize>::delete_object(pointer p)
{
    if (p == nullptr)
    {
        return; // No need to delete null pointers
    }
    // Call the destructor of the object
    destroy(p);
    // Deallocate the memory
    deallocate(p, 1); // Deallocate a single object
}

#endif // POOL_ALLOCATOR_TCC