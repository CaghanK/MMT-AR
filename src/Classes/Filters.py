#!/usr/bin/env python
import numpy as np

class AverageFilter:
    def __init__(self, size=10):
        self.size = size
        # FIFO queue to hold the data
        self.queue = np.zeros(size)
        # start intex of the queue, end_index = start_index - 1
        self.head_index = size - 1
        self.average = 0

    def update(self, new_data):
        self.average = self.average - (self.queue[self.head_index] / self.size) + (new_data / self.size)
        self.queue[self.head_index] = new_data
        self.start_index= (self.start_index + 1) % self.size