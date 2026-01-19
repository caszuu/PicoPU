#pragma once
#include <pico/multicore.h>

// block current core until all cores arrive at this barrier
// all cores must arrive at *all* barriers in the *same* order.
static inline void mc_barrier() {
    multicore_fifo_push_blocking(0);
    uint32_t val = multicore_fifo_pop_blocking();
}

// blocks the current core until all cores arrive at this barrier,
// then a single uint32_t of data is swapped between the cores
// all cores must arrive at *all* barriers in the *same* order.
static inline uint32_t mc_shuffle(uint32_t data) {
    multicore_fifo_push_blocking(data);
    return multicore_fifo_pop_blocking();
}

// dispatch next task to the aux core, core must not read from
// the fifo until it finishes its current dispatch
static inline void mc_dispatch(void (*func)(void *), void *user) {
    multicore_fifo_push_blocking((uint32_t)func);
    multicore_fifo_push_blocking((uint32_t)user);
}

static void mc_aux_loop() {
    while (true) {
        void *fp = (void *)multicore_fifo_pop_blocking();
        void (*func)(void *) = fp;

        void *up = (void *)multicore_fifo_pop_blocking();
        func(up);
    }
}

static void mc_init() {
    multicore_launch_core1(mc_aux_loop);
}
