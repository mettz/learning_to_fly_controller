#include <string.h>

#include "rl_tools_adapter.h"

#include "network/network.h"
#include "network/network_data.h"

#define DEBUG_MODULE "PX4RL"
#include "debug.h"

// #define RL_TOOLS_DISABLE_TEST

#define CONTROL_FREQUENCY_MULTIPLE 5
#define ACTION_HISTORY_LENGTH 32

static unsigned long controller_tick = 0;

/* Global handle to reference the instantiated C-model */
STAI_NETWORK_CONTEXT_DECLARE(m_network, STAI_NETWORK_CONTEXT_SIZE)

/* Array to store the data of the activation buffers */
STAI_ALIGNED(STAI_NETWORK_ACTIVATION_1_ALIGNMENT)
static uint8_t activations_1[STAI_NETWORK_ACTIVATION_1_SIZE_BYTES];

/* Array to store the data of the input tensors */
/* -> data_in_1 is allocated in activations buffer */

/* Array to store the data of the output tensors */
/* -> data_out_1 is allocated in activations buffer */

static stai_ptr m_inputs[STAI_NETWORK_IN_NUM];
static stai_ptr m_outputs[STAI_NETWORK_OUT_NUM];
static stai_ptr m_acts[STAI_NETWORK_ACTIVATIONS_NUM];
static float action_history[ACTION_HISTORY_LENGTH][4];

// Main functions (possibly with side effects)
void rl_tools_init() {
  stai_size _dummy;

  stai_network_init(m_network);

  m_acts[0] = (stai_ptr)activations_1;
  stai_network_set_activations(m_network, m_acts, STAI_NETWORK_ACTIVATIONS_NUM);
  stai_network_get_activations(m_network, m_acts, &_dummy);
  stai_network_get_inputs(m_network, m_inputs, &_dummy);
  stai_network_get_outputs(m_network, m_outputs, &_dummy);

  for (unsigned long step_i = 0; step_i < ACTION_HISTORY_LENGTH; step_i++) {
    for (unsigned long action_i = 0; action_i < 4; action_i++) {
      action_history[step_i][action_i] = 0;
    }
  }
  controller_tick = 0;
}

char *rl_tools_get_checkpoint_name() { return "rsl_rl_policy"; }

float rl_tools_test(float *output_mem) { return 0; }

void rl_tools_control(float *state, float *actions) {
  float *input = (float *)m_inputs[0];
  float *output = (float *)m_outputs[0];

  // 1. Extract quaternion (w, x, y, z)
  float qw = state[3];
  float qx = state[4];
  float qy = state[5];
  float qz = state[6];

  // 2. Position
  input[0] = state[0]; // x
  input[1] = state[1]; // y
  input[2] = state[2]; // z

  // 3. Rotation matrix (from quaternion, row-major)
  input[3] = 1 - 2 * qy * qy - 2 * qz * qz;
  input[4] = 2 * qx * qy - 2 * qw * qz;
  input[5] = 2 * qx * qz + 2 * qw * qy;

  input[6] = 2 * qx * qy + 2 * qw * qz;
  input[7] = 1 - 2 * qx * qx - 2 * qz * qz;
  input[8] = 2 * qy * qz - 2 * qw * qx;

  input[9] = 2 * qx * qz - 2 * qw * qy;
  input[10] = 2 * qy * qz + 2 * qw * qx;
  input[11] = 1 - 2 * qx * qx - 2 * qy * qy;

  // 4. Linear velocity (state[7..9])
  input[12] = state[7];
  input[13] = state[8];
  input[14] = state[9];

  // 5. Angular velocity (state[10..12])
  input[15] = state[10];
  input[16] = state[11];
  input[17] = state[12];

  if (controller_tick % 100 == 0) {
    DEBUG_PRINT(
        "state: "
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f\n",
        input[0], input[1], input[2],
        input[3], input[4], input[5],
        input[6], input[7], input[8],
        input[9], input[10], input[11],
        input[12], input[13], input[14],
        input[15], input[16], input[17]);
  }

  // 6. Add action history (starting from index 18)
  int offset = 18;
  for (int step = 0; step < ACTION_HISTORY_LENGTH; ++step)
    for (int j = 0; j < 4; ++j)
      input[offset++] = action_history[step][j];

  uint64_t start = usecTimestamp();
  stai_return_code ret = stai_network_run(m_network, STAI_MODE_SYNC);
  uint64_t end = usecTimestamp();

  if (ret != STAI_SUCCESS) {
    DEBUG_PRINT("STAI network run failed with error code: %d\n", ret);
  }

  if (controller_tick % 1000 == 0) {
    DEBUG_PRINT("STAI network run took %llu us\n", end - start);
  }

  memcpy(actions, output, sizeof(float) * 4);

  int substep = controller_tick % CONTROL_FREQUENCY_MULTIPLE;
  if (substep == 0) {
    for (unsigned long step_i = 0; step_i < ACTION_HISTORY_LENGTH - 1;
         step_i++) {
      for (unsigned long action_i = 0; action_i < 4; action_i++) {
        action_history[step_i][action_i] = action_history[step_i + 1][action_i];
      }
    }
  }
  for (unsigned long action_i = 0; action_i < 4; action_i++) {
    float value = action_history[ACTION_HISTORY_LENGTH - 1][action_i];
    value *= substep;
    value += m_outputs[0][action_i];
    value /= substep + 1;
    action_history[ACTION_HISTORY_LENGTH - 1][action_i] = value;
  }
  controller_tick++;
}
