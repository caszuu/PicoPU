#include "tests/test_list.h"
#include <pl/pl.h>

#include <pico/stdio.h>
#include <stdbool.h>
#include <stdio.h>

int main() {
    // init

    stdio_init_all();
    printf("pico-link tests - starting...\n\n");

    // loop over selected tests

    uint32_t ok_count = 0, err_count = 0;

    for (uint32_t i = 0; i < test_list_len; i++) {
        const struct pl_test *t = &test_list[i];

        printf("[%d/%d] running test %s...\n", i + 1, test_list_len, t->name);
        int err = t->entry();

        printf("[%d/%d] ran test %s - ", i + 1, test_list_len, t->name);
        if (!err) {
            printf("ok\n");
            ok_count++;
        } else {
            printf("failed with %d\n", err);
            err_count++;
        }
    }

    printf("\nfinished: %d passed, %d failed\n", ok_count, err_count);
    stdio_flush();

    while (1) {
    }
}
