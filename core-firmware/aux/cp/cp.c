#include "cp.h"

#include <hardware/sync.h>
#include <il/il_interface.h>

#include <assert.h>
#include <stdatomic.h>

// global command processor state //

static struct cp_state cp;

static void cp_set_queue_ready(uint32_t queue_idx) {
    atomic_fetch_or(&cp.queues_ready, 1u << queue_idx);
}

static bool cp_is_queue_attached(uint32_t queue_idx) {
    return cp.queues[queue_idx].cmdbuf;
}

static bool cp_is_group_idle(uint32_t group_idx) {
    return atomic_load(&cp.groups_idle) & (1u << group_idx);
}

static bool cp_is_device_idle() {
    return !(~atomic_load(&cp.groups_idle) & ((1u << cp.group_count) - 1));
}

static void cp_fill_slot_from_queues(uint32_t slot_idx);
static void cp_queue_next_ready(struct cp_cmd_queue *q);

// cmd queue //

/*
 * sets up a clean queue state and attaches a new command buffer for feeding.
 * initial command fetch is triggered automatically, the first command will be
 * available when the queue ready bit is set.
 */

static void cp_queue_init(struct cp_cmd_queue *q, uint32_t idx, void *cmdbuf) {
    // reset local semaphores
    memset(q->sems, 0, sizeof(q->sems));

    q->cmdbuf = cmdbuf;
    q->queue_idx = idx;

    // trigger the initial cmd fetch
    il_fetch_next_cmd(q, cp_queue_next_ready);
}

/*
 * put the queue into a stalled state if not all requested queue-local semaphores are signaled.
 * this will mask the queue ready bit until all requested semaphores are signaled by cp_queue_finished events.
 */

#define u_bits_foreach(idx, src) for (uint32_t bits = src, idx = __builtin_ctz(bits); idx; bits ^= 1u << idx, idx = __builtin_ctz(bits))

static bool cp_queue_await(struct cp_cmd_queue *q, uint32_t sem_bits) {
    uint32_t irq = save_and_disable_interrupts();

    u_bits_foreach(i, sem_bits) {
        if (q->sems[i] != 0) {
            q->stalled_bits = sem_bits;

            restore_interrupts_from_disabled(irq);
            return true;
        }
    }

    restore_interrupts_from_disabled(irq);
    return false;
}

/*
 * signal a given bitset of queue-local semophores and attempt to unstall the queue. returns if queue is ready after signal.
 */

static void cp_queue_finish(struct cp_cmd_queue *q, uint32_t sem_bits) {
    uint32_t irq = save_and_disable_interrupts();

    // decrement / signal requested semaphores

    u_bits_foreach(i, sem_bits) {
        q->sems[i]--;
    }

    // attempt to trigger the queue if stalled

    if (!(q->stalled_bits & sem_bits)) {
        restore_interrupts_from_disabled(irq);
        return; // early out if we didn't signal any stalled on sems
    }

    bool ongoing = false;
    u_bits_foreach(i, q->stalled_bits) {
        ongoing |= (bool)q->sems[i];
    }

    if (!ongoing) {
        // unstall queue and set the ready bit for the cp
        q->stalled_bits = 0;

        restore_interrupts_from_disabled(irq);
        return;
    }

    cp_set_queue_ready(q->queue_idx);

    restore_interrupts_from_disabled(irq);
    return;
}

/*
 * signal that _il_ has finished fetching the next command and [next] is now valid.
 *
 * the queue will either become ready (the ready bit will be set) or will go into the
 * stalled state based on the command wait bits.
 */

static void cp_queue_next_ready(struct cp_cmd_queue *q) {
    if (cp_queue_await(q, q->next.base.wait_bits))
        return; // queue stalled by on going work

    cp_set_queue_ready(q->queue_idx);
}

/*
 * triggers the fetch operation of the next command into [next]. After calling, [next] must
 * be considered invalid until the queue ready bit is set.
 */

static void cp_queue_fetch_next(struct cp_cmd_queue *q) {
    uint32_t sem_bits = q->next.base.signal_bits;

    uint32_t irq = save_and_disable_interrupts();
    u_bits_foreach(i, sem_bits) {
        q->sems[i]++;
    }
    restore_interrupts_from_disabled(irq);

    il_fetch_next_cmd(q, cp_queue_next_ready);
}

// graphics work submition //

static void cp_submit_gfx(uint32_t slot_idx) {
}

// cmd processing //

/*
 * signal that a slot has finished all pending tasks and the slot can be filled with
 * a new command.
 */

static void cp_finish_slot(uint32_t slot_idx) {
    union cp_slot_buffer *slot = &cp.slots[slot_idx];

    // mark the slot as free
    atomic_fetch_xor(&cp.slots_active, 1u << slot_idx);

    // trigger all ops dependent on this cmd
    if (slot->base.signal_bits)
        cp_queue_finish(&cp.queues[slot->base.source_queue], slot->base.signal_bits);

    // try to pull-in new cmd (FIXME: should probably be deferred)
    // cp.slots_awaiting |= 1u << slot_idx;
    cp_fill_slot_from_queues(slot_idx);
}

/*
 * take a cmd queue command and setup an idle slot with it. this method assumes
 * the command is ready to be dispatched.
 */

static void cp_fill_slot(uint32_t slot_idx, uint32_t queue_idx) {
    union cp_cmd_data *cmd = &cp.queues[queue_idx].next;
    union cp_slot_buffer *slot = &cp.slots[slot_idx];

    atomic_fetch_or(&cp.slots_active, 1u << slot_idx);

    switch (cmd->base.type) {
    case CP_CMD_DUMMY:
        slot->base = (struct cp_slot_base){CP_SLOT_NULL, queue_idx, cmd->base.signal_bits};

        // printf("dummy filled: q %d s %d\n", queue_idx, slot_idx);
        cp_finish_slot(slot_idx);
        break;

    default:
        break;
    }
}

/*
 * dispatch pending tasks to all available shading groups from an active slot.
 */

static void cp_schedule_slot(uint32_t slot_idx, uint32_t group_idx) {
    union cp_slot_buffer *slot = &cp.slots[slot_idx];

    switch (slot->base.type) {
        // case CP_CMD_DRAW:
        //     cp_submit_gfx(slot_idx);
        //     return;

        // case CP_CMD_PRESENT:
        //     il_present_flip(slot->present.pid, slot->present.fb);

        //     cp_finish_slot(slot_idx);
        //     return;

    default:
        assert(false);
    }
}

/*
 * pull any available commands from ready queues into an idle slot. will be a no-op
 * if no queues are ready.
 */

static void cp_fill_slot_from_queues(uint32_t slot_idx) {
    assert(~atomic_load(&cp.slots_active) & (1u << slot_idx));
    uint32_t ready_queues = atomic_load(&cp.queues_ready);

    if (!ready_queues)
        return; // no queues ready, bail

    uint32_t queue_idx = __builtin_ctz(ready_queues); // TODO: implicit priority, this is a bug

    cp_fill_slot(slot_idx, queue_idx);
    cp_queue_fetch_next(&cp.queues[queue_idx]);
}

/*
 * pull any available commands from ready queues into all inactive slots. will be a no-op
 * if no queues are ready or no slots are inactive.
 */

static void cp_fill_idle_slots_from_queues() {
    uint32_t empty_slots = ~atomic_load(&cp.slots_active) & ((1u << CP_MAX_COMMANDS_IN_FLIGHT) - 1);

    u_bits_foreach(slot_idx, empty_slots) {
        cp_fill_slot_from_queues(slot_idx);
    }
}

// cp api //

void cp_init() {
    cp = (struct cp_state){};
    il_get_topology(&cp.group_count, &cp.group_size);

    cp.groups_idle = (1u << cp.group_count) - 1;
}

void cp_attach_queue(uint32_t queue_idx, void *cmdbuf) {
    struct cp_cmd_queue *q = &cp.queues[queue_idx];
    assert(!cp_is_queue_attached(queue_idx));

    cp_queue_init(q, queue_idx, cmdbuf);
}

void cp_dispatch_tasks_for_group(uint32_t group_idx) {
    assert(cp_is_group_idle(group_idx));

    // if all slots are empty, pull new cmds from queues
    if (!cp.slots_active)
        cp_fill_idle_slots_from_queues(); // FIXME: should this be here?

    // pick a slot to pull tasks from
    uint32_t src_slot_idx = __builtin_ctz(atomic_load(&cp.slots_active));
    if (src_slot_idx == 32)
        return; // no pending cmds, bail

    cp_schedule_slot(src_slot_idx, group_idx);
}
