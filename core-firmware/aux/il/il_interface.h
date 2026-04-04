#pragma once
#include <stdint.h>

// misc interface //

/*
 * fetch the compute layout of the current device. the returned values must never change.
 */

void il_get_topology(uint32_t *group_count, uint32_t *group_size);

/*
 * fetch the display engine layout and capabilities. the returned values must never change.
 */

void il_get_present_features(uint32_t *present_engine_count);

/*
 * trigger an async fetch of the next command from a command buffer. [ready_cb] is called
 * when the command is ready and in-place.
 */

struct cp_cmd_queue;
void il_fetch_next_cmd(struct cp_cmd_queue *queue, void (*ready_cb)(struct cp_cmd_queue *));
