#pragma once

// a small performance-counter like thing using the arm systick
// for precise tight scope timings and microbenchmarking

#ifdef PICOPU_INSTRUMENTATION
#include <hardware/structs/systick.h> 
#include <stdint.h>

static uint32_t instru_submit_count = 0;
static uint32_t instru_sample_buf[1024];

#define INSTRU_RESET_SCOPE systick_hw->cvr = 0x00ffffff;
#define INSTRU_SUBMIT_SCOPE instru_sample_buf[instru_submit_count++] = systick_hw->cvr;
#define INSTRU_SUBMIT_SCOPE_ID(id) instru_sample_buf[id] = systick_hw->cvr;

// setup chip for instrumentation
static void instru_init() {
    systick_hw->csr = 0x5;
    systick_hw->rvr = 0x00ffffff;
}

#else

#define INSTRU_RESET_SCOPE
#define INSTRU_SUBMIT_SCOPE
#define INSTRU_SUBMIT_SCOPE_ID(id)

static void instru_init() { }

#endif // PICOPU_INSTRUMENTATION