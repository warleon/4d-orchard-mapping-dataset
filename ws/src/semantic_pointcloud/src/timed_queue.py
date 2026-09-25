from typing import Tuple, Protocol

from sortedcontainers.sortedlist import SortedKeyList
from genpy import Duration
import numpy as np

from genpy.rostime import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from utils.point_cloud_message import point_cloud_message_to_numpy


Item = Tuple[Time, np.ndarray]


class UnknownMsg(Protocol):
    header: Header


class TimedPointCloudQueue:
    def __init__(self, half_window: float, half_padding: float) -> None:
        self.queue: SortedKeyList[Item, Time] = SortedKeyList(key=self.getKey)
        self.half_window = Duration.from_sec(half_window)
        self.window = Duration.from_sec(2 * half_window)
        self.half_padding = Duration.from_sec(half_padding)
        self.padding = Duration.from_sec(2 * half_padding)

    def transformInput(self, msg: PointCloud2) -> Item:
        return (msg.header.stamp, point_cloud_message_to_numpy(msg))

    def getKey(self, item: Item):
        return item[0]

    def add(self, msg: PointCloud2):
        self.queue.add(self.transformInput(msg))
        first = self.getKey(self.queue[0])
        last = self.getKey(self.queue[-1])
        diff = last - first
        fullRange = Duration.from_sec(self.window.to_sec() + self.padding.to_sec())
        limit = Time.from_sec(first.to_sec() + self.half_padding.to_sec())
        if diff > fullRange:
            self.prune(limit)

    def prune(self, time: Time):
        i = self.queue.bisect_key_right(time)
        del self.queue[:i]

    def around(self, msg: UnknownMsg):
        current = msg.header.stamp
        it = self.queue.irange_key(
            current - self.half_window, current + self.half_window, (True, True)
        )

        messages = [item[1] for item in it]

        return messages

    def isFull(self):
        if not len(self.queue):
            return False

        first = self.getKey(self.queue[0])
        last = self.getKey(self.queue[-1])
        diff = last - first
        return diff > self.window
