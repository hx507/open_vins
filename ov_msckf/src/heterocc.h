#pragma once
#include "hpvm.h"

#ifndef __cplusplus
#define noexcept
#endif

#ifdef __cplusplus
extern "C" {
#endif

    extern void* __hpvm_launch(void*, ...) noexcept;
    extern void __hpvm_wait(void*) noexcept;
    extern void* __hpvm_parallel_section_begin() noexcept;
    extern void __hpvm_parallel_section_end(void*) noexcept;
    extern void* __hpvm_task_begin(unsigned,...) noexcept;
    extern void __hpvm_task_end(void*) noexcept;
    extern void __hpvm_parallel_loop(unsigned, ...) noexcept;
    extern void* __hpvm_launch_begin(unsigned, ...) noexcept;
    extern void __hpvm_launch_end(void*) noexcept;
    extern void __hpvm_priv(unsigned, ...) noexcept;
    extern void __hpvm__isNonZeroLoop(long, ...) noexcept;


    extern void __hetero_priv(unsigned, ...);
    extern void* __hetero_launch(void*, ...) noexcept;
    extern void __hetero_wait(void*) noexcept;
    extern void* __hetero_section_begin() noexcept;
    extern void __hetero_section_end(void*) noexcept;
    extern void* __hetero_task_begin(unsigned,...) noexcept;
    extern void __hetero_task_end(void*) noexcept;
    extern void __hetero_parallel_loop(unsigned, ...) noexcept; 
    extern void* __hetero_launch_begin(unsigned, ...) noexcept;
    extern void __hetero_launch_end(void*) noexcept;
    extern void __hetero_copy_mem(void*, void*, size_t) noexcept;
    extern void __hetero_request_mem(void*, size_t) noexcept;
    extern void* __hetero_malloc(size_t) noexcept;
    extern void __hetero_free(void*) noexcept;
    extern void __hetero_hint(int i) noexcept;
    extern void __hetero_isNonZeroLoop(long, ...) noexcept;

#ifdef __cplusplus
}
#endif

