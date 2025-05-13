
#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-07
# modified: 2025-05-07
#
# Mimics asyncio.Queue

import uasyncio as asyncio

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Queue:
    def __init__(self, maxsize=0):
        self._maxsize = maxsize
        self._queue = []
        self._get_event = asyncio.Event()
        self._put_event = asyncio.Event()
        self._put_event.set()        # ready to accept items initially
        self._unfinished_tasks = 0
        self._all_tasks_done = asyncio.Event()
        self._all_tasks_done.set()   # no outstanding tasks

    def qsize(self):
        return len(self._queue)

    def empty(self):
        return self.qsize() == 0

    def full(self):
        return 0 < self._maxsize <= self.qsize()

    @property
    def maxsize(self):
        return self._maxsize

    async def put(self, item):
        while self.full():
            self._put_event.clear()
            await self._put_event.wait()
        self._queue.append(item)
        self._unfinished_tasks += 1
        self._get_event.set()
        self._all_tasks_done.clear()

    def put_nowait(self, item):
        if self.full():
            raise QueueFull("Queue is full")
        self._queue.append(item)
        self._unfinished_tasks += 1
        self._get_event.set()
        self._all_tasks_done.clear()

    async def get(self):
        while self.empty():
            self._get_event.clear()
            await self._get_event.wait()
        item = self._queue.pop(0)
        if not self.full():
            self._put_event.set()
        return item

    def get_nowait(self):
        if self.empty():
            raise QueueEmpty("Queue is empty")
        item = self._queue.pop(0)
        if not self.full():
            self._put_event.set()
        return item

    def task_done(self):
        if self._unfinished_tasks <= 0:
            raise ValueError("task_done() called too many times")
        self._unfinished_tasks -= 1
        if self._unfinished_tasks == 0:
            self._all_tasks_done.set()

    async def join(self):
        await self._all_tasks_done.wait()

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class QueueFull(Exception):
    '''
    Raised when put_nowait() is called on a full queue.
    '''
    pass

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class QueueEmpty(Exception):
    '''
    Raised when get_nowait() is called on an empty queue.
    '''
    pass

#EOF
