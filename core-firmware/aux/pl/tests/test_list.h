#pragma once

// test forward defines //

int test_tx_minimal();

// test list //

struct pl_test {
    const char *name;
    int (*entry)();
};

const static struct pl_test test_list[1] = {
    {"tx only - minimal", test_tx_minimal},
};

const static int test_list_len = sizeof(test_list) / sizeof(test_list[0]);
