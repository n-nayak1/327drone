/**
 * Ciruclar Buffer
 * Written by Milan on 6/2/2025
 */
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#define BUFFER_SIZE 20

typedef struct {
    float ax[BUFFER_SIZE];
    float ay[BUFFER_SIZE];
    float az[BUFFER_SIZE];
    int head;
    int count;
} CircularBuffer;

void cb_init(CircularBuffer* cb);
void cb_push(CircularBuffer* cb, float ax, float ay, float az);
float cb_get(const CircularBuffer* cb, int index, char axis);  // 'x', 'y', or 'z'
void cb_average(const CircularBuffer* cb, float* ax_avg, float* ay_avg, float* az_avg);


#endif
