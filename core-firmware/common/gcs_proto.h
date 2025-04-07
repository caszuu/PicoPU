#pragma once

#include <shader_core/gcs/common.h>
#define MAX_GCS_DBG_SIZE 512

/* gcs cmd structs */

enum gcs_types {
    gcs_type_assign = 0,

    gcs_type_feedback = 16,
    gcs_type_finished,

    gcs_type_dbg = 32,
};
typedef uint16_t gcs_type_t;

// assigns a draw batch to a shader unit
// broker -> shader
struct gcs_assign_batch {
    gcs_type_t type;

    uint8_t batch_index;
    uint8_t primitive_count;

    uint32_t vertex_base;
};

// notifies the broker that the shading range for current batch is known (aka vertex stage finished)
// this allows conditionally breaking the rasterization order with other batches if allowed
// shader -> broker
struct gcs_batch_feedback {
    gcs_type_t type;

    int32_t c;
    int32_t shading_range[4];
};

// notifies the broker that the current batch was finished and shader is awaiting next scs or draw batch
// shader -> broker
struct gcs_batch_finished {
    gcs_type_t type;

    int32_t instru_systick;
};

// a message packet for debug info
// shader -> broker
struct gcs_dbg {
    gcs_type_t type;
    char dbg_message[MAX_GCS_DBG_SIZE];
};
