#include "GCodeCommandQueue.h"

GCodeCommandQueue::GCodeCommandQueue()
    : head(0), tail(0), size(0) {
}

bool GCodeCommandQueue::enqueue(const GCodeCommand& cmd) {
    if (isFull()) {
        return false;
    }

    queue[head] = cmd;
    head = (head + 1) % MAX_QUEUE_SIZE;
    size++;
    return true;
}

bool GCodeCommandQueue::dequeue(GCodeCommand& outCmd) {
    if (isEmpty()) {
        return false;
    }

    outCmd = queue[tail];
    tail = (tail + 1) % MAX_QUEUE_SIZE;
    size--;
    return true;
}

bool GCodeCommandQueue::isEmpty() const {
    return size == 0;
}

bool GCodeCommandQueue::isFull() const {
    return size >= MAX_QUEUE_SIZE;
}

uint8_t GCodeCommandQueue::count() const {
    return size;
}

void GCodeCommandQueue::clear() {
    head = 0;
    tail = 0;
    size = 0;
}

bool GCodeCommandQueue::shouldSendReady() const {
    return size < LOW_THRESHOLD;
}