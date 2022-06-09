# Software License Agreement (BSD License)
#
#  Copyright (c) 2018-2019, Locus Robotics
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QPen

import rospy

from rqt_bag import TimelineRenderer

from .evaluation_cache import get_cache
from .util import FIELDS


class TimelinePlotter(TimelineRenderer):
    """This class displays the velocity of the best trajectories in the timeline view."""

    def __init__(self, timeline, height=240):
        TimelineRenderer.__init__(self, timeline, msg_combine_px=height)
        self.height = height
        self.cache = get_cache()

    def get_segment_height(self, topic):
        return self.height

    def scaleValue(self, value):
        return 1.0 - (value - self.cache.v_min) / (self.cache.v_max - self.cache.v_min)

    def draw_timeline_segment(self, painter, topic, stamp_start, stamp_end, x, y, width, height):
        painter.setBrush(QBrush(Qt.white))
        painter.drawRect(x, y, width, height)

        messages = self.cache.getMessages(self.timeline.scene(), topic, rospy.Time(stamp_start), rospy.Time(stamp_end))
        if self.cache.v_min is None:
            return
        painter.setBrush(QBrush(Qt.black))
        z_val = y + height * self.scaleValue(0.0)
        painter.drawLine(x, z_val, x + width, z_val)
        RADIUS = 2

        for field, color in FIELDS.items():
            if field == 'y':
                continue
            color = getattr(Qt, color)

            last = None
            for t, msg in sorted(messages):
                best = msg.twists[msg.best_index]
                best_vel = best.traj.velocity
                p_x = self.timeline.map_stamp_to_x(t)
                if best.total >= 0.0:
                    painter.setBrush(QBrush(color))
                    painter.setPen(QPen(color, 1))
                    value = getattr(best_vel, field)
                else:
                    painter.setBrush(QBrush(Qt.red))
                    painter.setPen(QPen(Qt.red, 1))
                    value = 0.0
                p_y = y + height * self.scaleValue(value)
                if last is not None:
                    painter.drawLine(last[0], last[1], p_x, p_y)
                painter.drawEllipse(p_x - RADIUS, p_y - RADIUS, 2 * RADIUS, 2 * RADIUS)
                last = p_x, p_y
