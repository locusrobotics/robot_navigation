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

from math import cos, sin

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QPainter, QPen
from python_qt_binding.QtWidgets import QWidget

from .bounds import Bounds
from .util import subtractPose


def expand_bounds(bounds, extra=0.10):
    """Expand the negative bounds of the x axis by 10 percent so the area behind the robot is visible."""
    bounds.x_min -= (bounds.x_max - bounds.x_min) * extra


class TrajectoryWidget(QWidget):
    """This widget displays the actual trajectories of the robot's position through time in robot coordinate space."""

    def __init__(self, parent, bounds=None):
        QWidget.__init__(self, parent)
        self.evaluation = None
        self.plan = None
        self.selected = []

        if not bounds:
            self.dynamic_bounds = True
            self.bounds = Bounds()
        else:
            self.dynamic_bounds = False
            self.bounds = bounds
            expand_bounds(self.bounds)

    def setEvaluation(self, evaluation):
        self.evaluation = evaluation
        if self.dynamic_bounds:
            self.bounds = Bounds(evaluation)
            expand_bounds(self.bounds)
        self.selected = []
        self.update()

    def setSelected(self, selected):
        self.selected = selected
        self.update()

    def setPlan(self, plan):
        self.plan = plan

    def start(self, event):
        desired_width = self.bounds.x_max - self.bounds.x_min
        desired_height = self.bounds.y_max - self.bounds.y_min
        rect = event.rect()
        if desired_width == 0 or desired_height == 0:
            self.ratio = 1.0
        else:
            wr = rect.width() / desired_width
            hr = rect.height() / desired_height
            self.ratio = min(wr, hr)

        self.qp = QPainter()
        self.qp.begin(self)

    def end(self):
        self.qp.end()

    def t_x(self, x):
        return self.ratio * (x - self.bounds.x_min)

    def t_y(self, y):
        return self.ratio * (self.bounds.y_max - y)

    def drawLine(self, x0, y0, x1, y1):
        x0 = self.t_x(x0)
        y0 = self.t_y(y0)

        x1 = self.t_x(x1)
        y1 = self.t_y(y1)
        self.qp.drawLine(x0, y0, x1, y1)

    def drawCircle(self, x, y, radius):
        x = self.t_x(x)
        y = self.t_y(y)
        self.qp.drawEllipse(x - radius, y - radius, 2 * radius, 2 * radius)

    def fillRect(self, x, y, width, height):
        self.qp.fillRect(self.t_x(x), self.t_y(y), self.ratio * width, self.ratio * height, QBrush(Qt.white))

    def drawPoses(self, poses, base_pose, pose_radius=4, final_pose_radius=5, draw_final_orientation=True):
        last = None
        for pose in poses:
            if base_pose:
                pose = subtractPose(pose, base_pose)
            if last:
                self.drawLine(last.x, last.y, pose.x, pose.y)
            if pose_radius > 0:
                self.drawCircle(pose.x, pose.y, pose_radius)
            last = pose

        self.drawCircle(last.x, last.y, final_pose_radius)

        if draw_final_orientation:
            scale_radius = final_pose_radius / self.ratio
            self.drawLine(last.x, last.y,
                          last.x + scale_radius * cos(last.theta),
                          last.y + scale_radius * sin(last.theta))

    def paintEvent(self, event):
        self.start(event)

        self.qp.setPen(QPen(Qt.black, 0))

        self.fillRect(self.bounds.x_min, self.bounds.y_max,
                      self.bounds.x_max - self.bounds.x_min,
                      self.bounds.y_max - self.bounds.y_min)

        self.drawLine(self.bounds.x_min, 0, self.bounds.x_max, 0)
        self.drawLine(0, self.bounds.y_min, 0, self.bounds.y_max)

        if self.evaluation is not None:
            # Cache the brush
            brush = self.qp.brush()

            if len(self.selected) != 0:
                self.qp.setPen(QPen(Qt.gray, 0))

            for twist in self.evaluation.twists:
                self.drawPoses(twist.traj.poses, twist.traj.poses[0], 0, draw_final_orientation=False)

            for index in self.selected:
                self.qp.setPen(QPen(Qt.black, 0))
                self.qp.setBrush(self.selected[index])
                poses = self.evaluation.twists[index].traj.poses
                self.drawPoses(poses, poses[0])

            # Uncache the brush
            self.qp.setBrush(brush)

        if self.plan is not None:
            self.qp.setPen(QPen(Qt.green, 0))
            self.drawPoses(self.plan.poses, self.evaluation.twists[0].traj.poses[0], 0, 20.0)
        self.qp.end()
