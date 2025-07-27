#include <string.h>

#include "rl_tools_adapter.h"

#include "network/network.h"
#include "network/network_data.h"

#define DEBUG_MODULE "PX4RL"
#include "debug.h"

// #define RL_TOOLS_DISABLE_TEST
#define RL_TOOLS_DISABLE_ACTION_HISTORY

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

#ifndef RL_TOOLS_DISABLE_ACTION_HISTORY
  for (unsigned long step_i = 0; step_i < ACTION_HISTORY_LENGTH; step_i++) {
    for (unsigned long action_i = 0; action_i < 4; action_i++) {
      action_history[step_i][action_i] = 0;
    }
  }
#endif

  controller_tick = 0;
}

char *rl_tools_get_checkpoint_name() { return "isaaclie-onnx-runtime-rl_games"; }

float rl_tools_test(float *output_mem) { return 0; }

static float clipf(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

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

#ifndef RL_TOOLS_DISABLE_ACTION_HISTORY
  // 6. Add action history (starting from index 18)
  int offset = 18;
  for (int step = 0; step < ACTION_HISTORY_LENGTH; ++step)
    for (int j = 0; j < 4; ++j)
      input[offset++] = action_history[step][j];
#endif

  stai_return_code ret = stai_network_run(m_network, STAI_MODE_SYNC);
  
  memcpy(actions, output, sizeof(float) * 4);
  for (unsigned long i = 0; i < 4; i++) {
    actions[i] = clipf(actions[i], -1.0f, 1.0f);
  }

#ifndef RL_TOOLS_DISABLE_ACTION_HISTORY
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
    value += actions[action_i];
    value /= substep + 1;
    action_history[ACTION_HISTORY_LENGTH - 1][action_i] = value;
  }
#endif
  controller_tick++;
}
