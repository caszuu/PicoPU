#pragma once

#include "cmd_proto.h"
#include "config.h"

#include <util/u_fifo.h>

#include <stdatomic.h>
#include <stdint.h>

// command queue //

/*
 * a cmd queue is an (optionally synchronous) stream of commands submited by the host.
 *
 * all inter-command synchronization is performed by the command processor is done on this layer.
 * by selectively blocking the feeding of new commands to the consumer and using the queue local
 * semaphores for order independent waiting on commands in-execution
 *
 * note: access to [sems] and [stalled_bits] is not thread-safe, only interrupt-safe.
 */

struct cp_cmd_queue {
    union cp_cmd_data next;

    // TODO: temp. replace with vma feeder
    void *cmdbuf;

    uint16_t sems[CP_MAX_QUEUE_LOCAL_SEMAPHORES];

    // if any bits in [stalled_bits] are set, the queue is stalled. in a stalled state, the queue will
    // not become ready (will not set the ready) bit even when the next command is ready for reading until
    // all semaphores from [sems] will become signaled.
    uint32_t stalled_bits;

    uint32_t queue_idx;
};

// cmd slot states //

enum cp_slot_type {
    CP_SLOT_NULL = 0,
};

struct cp_slot_base {
    enum cp_slot_type type;

    uint32_t source_queue;
    uint32_t signal_bits;
};

union cp_slot_buffer {
    struct cp_slot_base base;

    // struct cp_slot_dummy dummy;
};

// command processor //

/*
 * The command processor is the central device-wide scheduler for all host submited
 * commands.
 *
 * The command processor has three components: the command queues, command slots, and
 * all the available execution units (shader groups, presentation engines, etc.). The
 * goal of the command processor is to accept command queue submitions from the host
 * and dispatch workload over the entire device as efficiently as possible.
 *
 * When a queue is attached to the processor (see the hwq interface), it starts to
 * asynchronously fetching commands from vram. after a cmd fetch is finished, the queue
 * checks if any wait operations have to be done before the cmd can be executed. if yes,
 * the queue will transition into the stalled state until all wait operations are
 * finished. otherwise, the queue will transition into the ready state. all inter-cmd
 * blocking operations are always done in the queue layer.
 *
 * After a cmd is dispatched from a ready queue, it is moved into an in-flight command
 * slot. Once a cmd is in a slot, it musn't ever block. Any free execution units are
 * then free to pull task batches from any slot that is non-empty.
 *
 * Once a slot submits all tasks for a given command, it is considered finished. Any
 * wait operations awaiting on this cmd should be finished and the slot is refilled
 * with a new cmd from any ready queues.
 */

struct cp_state {
    // command queues //

    struct cp_cmd_queue queues[CP_MAX_QUEUES];
    atomic_uint queues_ready;

    // in-flight scheduler //

    union cp_slot_buffer slots[CP_MAX_COMMANDS_IN_FLIGHT];
    atomic_uint slots_active;

    // shader group states //

    atomic_uint groups_idle;

    uint32_t group_count;
    uint32_t group_size;
};

/*
 * initialize the command processor state.
 */

void cp_init();

/*
 * attach a command buffer to a queue. after attaching, the queue will fetch its initial cmd
 * and becomes available to the command processor.
 */

void cp_attach_queue(uint32_t queue_idx, void *cmdbuf);

/*
 * submit tasks to a specific group id. it will not schedule any work if no commands are pending.
 */

void cp_dispatch_tasks_for_group(uint32_t group_idx);

/*
 * submit tasks to all idle shader groups. it will not schedule any work if no groups
 * are idle or no commands are pending.
 */

void cp_dispatch_tasks_for_idle();
