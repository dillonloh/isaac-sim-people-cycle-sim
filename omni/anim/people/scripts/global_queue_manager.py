# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import annotations

class GlobalQueueManager:
    """Global class which facilitates all queue interactions between characters"""

    __instance: GlobalQueueManager = None
    
    def __init__(self):
        if self.__instance is not None:
            raise RuntimeError("Only one instance of GlobalQueueManager is allowed")
        self._queues = {}
        GlobalQueueManager.__instance = self

    def create_queue(self, queue_name):
        if not queue_name in self._queues:
            self._queues[queue_name] = Queue()
        return self._queues[queue_name]

    def get_queue(self, queue_name):
        return self._queues[queue_name]

    def destroy(self):
        GlobalQueueManager.__instance = None

    @classmethod
    def get_instance(cls) -> GlobalQueueManager:
        if cls.__instance is None:
            GlobalQueueManager()
        return cls.__instance

class Queue:
    """
    Represents a logical structure for a Queue.
    """

    def __init__(self):
        self.num_spots = 0
        self.spots = []

    def create_spot(self, index, pos, rot):
        if index < self.num_spots:
            return

        if index > self.num_spots:
            raise ValueError("Invalid Queue Creation")

        self.num_spots += 1
        self.spots.append(QueueSpot(index, pos, rot))

    def get_num_spots(self):
        return self.num_spots

    def get_first_empty_spot(self):
        for spot in self.spots:
            if not spot.is_occupied(): 
                return spot

    def get_spot(self, index):
        return self.spots[index]

class QueueSpot:
    """
    Represents a logical structure for a queue spot.
    """
    def __init__(self, index, pos, rot):
        self.pos = pos
        self.rot = rot
        self.index = index
        self.occupied = False

    def get_index(self):
        return self.index
        
    def set_occupied(self, status):
        self.occupied = status
    
    def is_occupied(self):
        return self.occupied
    
    def get_transform(self):
        return (self.pos, self.rot)
    
    def get_translation(self):
        return self.pos

    def get_rotation(self):
        return self.rot
    
    def set_transform(self, pos, rot):
        self.pos = pos
        self.rot = rot


