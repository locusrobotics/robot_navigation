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

from math import cos, degrees, sin

from geometry_msgs.msg import Pose2D

from python_qt_binding.QtGui import QColor

FIELDS = {'x': 'black', 'y': 'green', 'theta': 'blue'}


def scale(v, minval, maxval, dest_size=1.0, offset=0.0):
    if minval == maxval:
        return offset
    return (v - minval) / (maxval - minval) * dest_size + offset


def subtractPose(pose_a, pose_b):
    dx = pose_a.x - pose_b.x
    dy = pose_a.y - pose_b.y
    s = sin(-pose_b.theta)
    c = cos(-pose_b.theta)
    x = dx * c - dy * s
    y = dx * s + dy * c
    return Pose2D(x, y, pose_a.theta - pose_b.theta)


def debug(pose):
    return '%.2f %.2f %.2f' % (pose.x, pose.y, degrees(pose.theta))


PALETTE = [QColor(204,  65,  37), QColor(246, 178, 107), QColor(255, 217, 102), QColor(147, 196, 125),  # noqa(E241)
           QColor(109, 158, 235), QColor(142, 124, 195), QColor(224, 102, 102), QColor(118, 165, 175),
           QColor(111, 168, 220), QColor(194, 123, 160)]
