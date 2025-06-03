#include "circular_buffer.h"

void cb_init(CircularBuffer* cb) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        cb->ax[i] = 0.0f;
        cb->ay[i] = 0.0f;
        cb->az[i] = 0.0f;
    }
    cb->head = -1;
    cb->count = 0;
}

void cb_push(CircularBuffer* cb, float ax, float ay, float az) {
    cb->head = (cb->head + 1) % BUFFER_SIZE;

    cb->ax[cb->head] = ax;
    cb->ay[cb->head] = ay;
    cb->az[cb->head] = az;

    if (cb->count < BUFFER_SIZE) {
        cb->count++;
    }
}

float cb_get(const CircularBuffer* cb, int index, char axis) {
    if (index >= cb->count) return 0.0f;  // Out of bounds

    int actual_index = (cb->head - index + BUFFER_SIZE) % BUFFER_SIZE;

    switch (axis) {
        case 'x': return cb->ax[actual_index];
        case 'y': return cb->ay[actual_index];
        case 'z': return cb->az[actual_index];
        default:  return 0.0f;  // Invalid axis
    }
}

void cb_average(const CircularBuffer* cb, float* ax_avg, float* ay_avg, float* az_avg) {
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    int count = cb->count > BUFFER_SIZE ? BUFFER_SIZE : cb->count;

    for (int i = 0; i < count; ++i) {
        int idx = (cb->head - i - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        sum_ax += cb->ax[idx];
        sum_ay += cb->ay[idx];
        sum_az += cb->az[idx];
    }

    *ax_avg = sum_ax / count;
    *ay_avg = sum_ay / count;
    *az_avg = sum_az / count;
}
