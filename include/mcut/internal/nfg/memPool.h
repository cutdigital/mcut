/****************************************************************************
* NFG - Numbers for Geometry                     					        *
*                                                                           *
* Consiglio Nazionale delle Ricerche                                        *
* Istituto di Matematica Applicata e Tecnologie Informatiche                *
* Sezione di Genova                                                         *
* IMATI-GE / CNR                                                            *
*                                                                           *
* Authors: Marco Attene                                                     *
* Copyright(C) 2019: IMATI-GE / CNR                                         *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation; either version 3 of the License, or (at  *
* your option) any later version.                                           *
*                                                                           *
* This program is distributed in the hope that it will be useful, but       *
* WITHOUT ANY WARRANTY; without even the implied warranty of                *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser  *
* General Public License for more details.                                  *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see http://www.gnu.org/licenses/.       *
*                                                                           *
****************************************************************************/

#ifndef MEMPOOL_H
#define MEMPOOL_H

#include <vector>
#include <bit>
#include <bitset>
#include <cstdint>

// An N_block is a contiguous portion of memory elements.
// An N_memory_pool stores a set of N_blocks along with
// a 'stack' of pointers to these blocks.
// Upon allocation, the first free block is picked from the stack.
// If the stack is empty, the size of the pool (and the stack) is grown.
// When a block is released, its pointer is added on top of the stack.
//
// alloc is almost always O(1) [O(num_blocks) only if growth is required]
// release is always O(1)

class N_memory_pool {
	std::vector<uint8_t*> data; // Data storage. Each data[i] is an array of N_blocks
	uint8_t** stack;	// Stack of pointers to free blocks
	size_t last;		// Element past to the last in the stack
	size_t size;		// Size of the pool (total number of blocks)
	const size_t block_size; // Size of one block (number N of elements)

	void addDataBlockArray(size_t tot_size, uint8_t** fs) {
		uint8_t* data_1 = (uint8_t*)malloc(sizeof(uint8_t)*tot_size);
		data.push_back(data_1);
		uint8_t* lst = data_1 + tot_size;
		while (lst != data_1) {
			lst -= block_size;
			*fs++ = lst;
		}
	}

	void doubleDataArray() {
		uint8_t** fs = (uint8_t**)malloc(sizeof(uint8_t *) * size * 2);
		size_t i = 0, j = size;
		while (i < size) fs[j++] = stack[i++];
		free(stack);
		stack = fs;

		addDataBlockArray(block_size * size, fs);

		last += size;
		size *= 2;
	}

public:
	N_memory_pool(size_t _size, size_t _block_size) : last(_size), size(_size), block_size(_block_size) {
		stack = (uint8_t**)malloc(sizeof(uint8_t*) * size);
		uint8_t** fs = stack;
		addDataBlockArray(size * block_size, fs);
	}

	N_memory_pool(N_memory_pool&& p) noexcept;

	~N_memory_pool() {
		free(stack);
		for (uint8_t* p : data) free(p);
		data.clear();
	}

	void* alloc() {
		if (last == 0) doubleDataArray();
		return stack[--last];
	}

	void release(void* s) {	stack[last++] = (uint8_t *)s; }
};

inline N_memory_pool::N_memory_pool(N_memory_pool&& p) noexcept
	: data(p.data), stack(p.stack), last(p.last), size(p.size), block_size(p.block_size) {
	p.stack = nullptr;
	p.data.clear();
}

// An N_block is a contiguous portion of 32-bit memory elements.
// An extended_N_block is the concatentaion of [N] and an N_block.
//
//		i.e. uint32_t block_N[N+1] = { N, v1, v2, ..., vN };
// 
// An Indexed N pool (IN_pool) stores a set of extended_N_blocks along with
// a 'stack' of pointers to these extended blocks. Each pointer in the stack
// points to the second element of an extended block because the first must 
// always store the block size and is therefore reserved.
// Upon allocation, the first free block is picked from the stack.
// If the stack is empty, the size of the pool (and the stack) is grown.
// When a block is released, its pointer is added on top of the stack.
//
// alloc is almost always O(1) [O(num_blocks) only if growth is required]
// release is always O(1)

class IN_pool {
	std::vector<uint32_t*> data; // Data storage. Each data[i] is an array of extended_N_blocks
	uint32_t** stack;	// Stack of pointers to free blocks
	uint32_t last;		// Element past to the last in the stack
	uint32_t size;		// Size of the pool (total number of blocks)
	const uint32_t block_size; // Size of one block (number N of elements)

	void addDataBlockArray(uint32_t tot_size, uint32_t extended_block_size, uint32_t** fs) {
		uint32_t* data_1 = (uint32_t*)malloc(sizeof(uint32_t) * tot_size);
		data.push_back(data_1);
		uint32_t* lst = data_1 + tot_size;
		while (lst != data_1) {
			lst -= extended_block_size;
			*lst = block_size;
			*fs++ = lst + 1;
		}
	}

	void doubleDataArray() {
		const uint32_t extended_block_size = block_size + 1;

		uint32_t** fs = (uint32_t**)malloc(sizeof(uint32_t*) * size * 2);
		for (uint32_t i = 0; i < size; i++) fs[i + size] = stack[i];
		free(stack);
		stack = fs;

		addDataBlockArray(extended_block_size * size, extended_block_size, fs);

		last += size;
		size *= 2;
	}

public:
	IN_pool(uint32_t _size, uint32_t _block_size) : last(_size), size(_size), block_size(_block_size) {
		const uint32_t extended_block_size = block_size + 1;
		stack = (uint32_t**)malloc(sizeof(uint32_t*) * size);
		uint32_t** fs = stack;
		addDataBlockArray(size * extended_block_size, extended_block_size, fs);
	}

	IN_pool(IN_pool&& p) noexcept;

	~IN_pool() {
		free(stack);
		for (uint32_t* p : data) free(p);
		data.clear();
	}

	uint32_t* alloc() {
		if (last == 0) doubleDataArray();
		return stack[--last];
	}

	void release(uint32_t* s) { stack[last++] = s; }

	uint32_t blockSize() const { return block_size; }
};

inline IN_pool::IN_pool(IN_pool&& p) noexcept
	: data(p.data), stack(p.stack), last(p.last), size(p.size), block_size(p.block_size) {
	p.stack = nullptr;
	p.data.clear();
}

// A MultiPool is a collection of IN_pools having size N = 2, 4, 8, 16, ..., 2^m
// 
// Upon allocation for X elements two cases may occur:
// 1) X <= 2^m
//     Calculate the minimum value of N such that N >= X
//     Allocate a block in the corresponding IN_pool
// 2) X > 2^m 
//	   standard malloc() is used for X+1 elements
// 	   first element is set to 0 [meaning that no IN_pool was used]
// 	   a pointer to the second element is returned for use
//
// Upon release of a pointer A
//    Let V be the value of the element preceeding the one pointed by A (i.e. V = A[-1])
//    If V==0 use standard free()
//    else if V==N release a block is the corresponding IN_pool

class MultiPool {
	std::vector<IN_pool> IN_pools;
	uint32_t max_block_size;

	IN_pool& pickPoolFromSize(uint32_t bs) {
		auto cz = std::countl_zero(bs - 1);
		cz = (cz == 32) ? (31) : (cz);
		return IN_pools[31 - cz];

		//// FOR COMPILERS THAT DO NOT SUPPORT C++20 REPLACE THE ABOVE WITH THE FOLLOWING
		//std::vector<IN_pool>::iterator i = IN_pools.begin();
		//while ((*i).blockSize() < bs) i++;
		//return *i;
	}

public:
	MultiPool(uint32_t _max_block_size = 32, uint32_t init_capacity = 16) : max_block_size(_max_block_size) {
		uint32_t bs = 1;
		while (bs < max_block_size) {
			bs <<= 1;
			IN_pools.push_back(IN_pool(init_capacity, bs));
		}
	}

	void* alloc(uint32_t num_bytes) {
		const uint32_t num_els = ((num_bytes + 3) >> 2);
		if (num_els > max_block_size) {
			uint32_t* ptr;
			if ((ptr = (uint32_t*)malloc(sizeof(uint32_t) * (num_els + 1))) == NULL) return NULL;
			ptr[0] = 0;
			return ptr + 1;
		}

		return pickPoolFromSize(num_els).alloc();
	}

	void release(void* s) {
		if (s) {
			uint32_t* ps = (((uint32_t*)s) - 1);
			const uint32_t v = *ps;
			if (v == 0) {
				free(ps);
				return;
			}

			pickPoolFromSize(v).release(((uint32_t*)s));
		}
	}
};

#endif //MEMPOOL_H
