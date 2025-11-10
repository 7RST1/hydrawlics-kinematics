#ifndef GCODE_COMMAND_QUEUE_H
#define GCODE_COMMAND_QUEUE_H

#include <Arduino.h>
#include "ArmController.h"

// Circular queue for GCode commands
class GCodeCommandQueue {
public:
    static constexpr uint8_t MAX_QUEUE_SIZE = 10;
    static constexpr uint8_t LOW_THRESHOLD = 5;

    GCodeCommandQueue();

    // Queue operations
    bool enqueue(const GCodeCommand& cmd);
    bool dequeue(GCodeCommand& outCmd);
    bool isEmpty() const;
    bool isFull() const;
    uint8_t count() const;
    void clear();

    // Threshold check
    bool shouldSendReady() const;

private:
    GCodeCommand queue[MAX_QUEUE_SIZE];
    uint8_t head;      // Points to next position to write
    uint8_t tail;      // Points to next position to read
    uint8_t size;      // Current number of items
};

#endif // GCODE_COMMAND_QUEUE_H