#! /usr/bin/env python

"""
Open List module
Author: Roberto Zegers R.
Date: Feb 22, 2021
"""

import heapq

class OpenList(object):
    def __init__(self):
        self.heap = []
        self.heap_dict = dict()

    def put(self, item, priority):
        heapq.heappush(self.heap, (priority, item))
        self.heap_dict[item] = priority

    def get(self):
        _, item = heapq.heappop(self.heap)
        self.heap_dict.pop(item)
        return item

    def remove(self, item):
        self.decrease_key(item, (-float('inf'), -float('inf')))
        self.get()

    def peek(self):
        return self.heap[0]

    def empty(self):
        return len(self.heap) == 0

    def size(self):
        return len(self.heap)

    def contains(self, item):
        return item in self.heap_dict

    def get_priority(self, item):
        return self.heap_dict[item]

    def decrease_key(self, item, new_priority):
        idx = None
        for i, val in enumerate(self.heap):
            if val[1] == item:
                idx = i
                break

        old_priority = self.heap[idx][0]
        self.heap[idx] = (new_priority, item)
        self.heap_dict[item] = new_priority
        heapq._siftdown(self.heap, 0, idx)