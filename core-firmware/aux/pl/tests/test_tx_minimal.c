#include "mock_data.h"
#include "test_list.h"

#include <pl/pl.h>
#include <stdio.h>

DEFINE_LINK_BUFFER(tx_link_buf, 512)
DEFINE_LINK_BUFFER(rx_link_buf, 256)

int test_tx_minimal() {
    struct pl_link_config cfg = {
        .chan_count = 1,
        .is_initial = true,
        .pin_base = 0,

        // don't care about rx or tx pacing
        .rx_cb = NULL,
        .tx_cb = NULL,

        .tx_bufs = {tx_link_buf},
        .tx_buf_size = 512,

        .rx_buf = rx_link_buf,
        .rx_buf_size = 256,
    };

    pl_init(0);
    uint32_t lk = pl_init_link(0, 0, &cfg);

    for (uint32_t i = 0; i < mock_data_len / 32; i++) {
        pl_tx(lk, 0, &mock_data[i * 32], 32);
    }

    return 0;
}
