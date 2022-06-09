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
from python_qt_binding.QtGui import QBrush, QPainter, QPen
from python_qt_binding.QtWidgets import QWidget

from .util import scale


class VelocitySpaceWidget(QWidget):
    """This widget displays the velocities for each trajectory in the evaluation."""

    def __init__(self, parent, selection_callback):
        QWidget.__init__(self, parent)
        self.evaluation = None
        self.vel_space = {}
        self.scores = {}
        self.n_x = 0
        self.n_theta = 0
        self.selected = []
        self.selection_callback = selection_callback
        self.vel = None

    def setVelocity(self, vel):
        self.vel = vel

    def setEvaluation(self, evaluation):
        self.evaluation = evaluation
        self.selected = []
        self.vel_space = {}
        self.scores = {}
        xs = set()
        thetas = set()
        self.x_min = self.x_max = None
        self.t_min = self.t_max = None
        for i, twist in enumerate(self.evaluation.twists):
            v = twist.traj.velocity.x, twist.traj.velocity.theta
            xs.add(v[0])
            thetas.add(v[1])
            self.vel_space[v] = i
            self.scores[v] = twist.total

            if self.x_min is None:
                self.x_min = self.x_max = v[0]
                self.t_min = self.t_max = v[1]
            else:
                self.x_min = min(self.x_min, v[0])
                self.x_max = max(self.x_max, v[0])
                self.t_min = min(self.t_min, v[1])
                self.t_max = max(self.t_max, v[1])
        self.n_x = len(xs)
        self.n_theta = len(thetas)
        self.update()

    def setSelected(self, selected):
        self.selected = selected
        self.update()

    def paintEvent(self, event):
        if self.evaluation is None or self.n_x == 0:
            return
        rect = event.rect()
        w, h = rect.width(), rect.height()
        self.left = w * 0.2
        self.width = w * 0.7
        self.top = h * 0.1
        self.height = h * 0.85

        horizontal_size = self.width / self.n_x
        vertical_size = self.height / self.n_theta
        diameter = max(5, min(horizontal_size, vertical_size))
        radius = diameter / 2

        self.qp = QPainter()
        self.qp.begin(self)
        self.qp.setBrush(QBrush(Qt.white))
        self.qp.drawRect(self.left, self.top, self.width, self.height)

        zx, zy = self.toScreen(0, 0)
        if zy >= self.top and zy <= self.top + self.height:
            self.qp.drawLine(self.left, zy, self.left + self.width, zy)
        if zx >= self.left and zx <= self.left + self.width:
            self.qp.drawLine(zx, self.top, zx, self.top + self.height)

        self.qp.drawText(self.left, 0, self.width, self.top, Qt.AlignCenter, 'x')
        self.qp.drawText(0, 0, self.left * 2, self.top, Qt.AlignCenter, '%.2f' % self.x_min)
        self.qp.drawText(w - self.left, 0, self.left, self.top, Qt.AlignCenter, '%.2f' % self.x_max)

        right_center = Qt.AlignVCenter | Qt.AlignRight
        self.qp.drawText(0, self.top, self.left - radius, self.height, right_center, 'theta')
        self.qp.drawText(0, 0, self.left - radius, self.top * 2, right_center, '%.2f' % self.t_max)
        self.qp.drawText(0, h - self.top, self.left - radius, self.top, right_center, '%.2f' % self.t_min)

        for (x, theta), index in self.vel_space.items():
            dx, dy = self.toScreen(x, theta)
            if index in self.selected:
                self.qp.setBrush(QBrush(self.selected[index]))
            else:
                self.qp.setBrush(QBrush())

            if self.scores[x, theta] < 0.0:
                self.qp.setPen(QPen(Qt.red))
            else:
                self.qp.setPen(QPen(Qt.black))
            self.qp.drawEllipse(dx - radius, dy - radius, diameter, diameter)

        if self.vel:
            self.qp.setBrush(QBrush())
            dx, dy = self.toScreen(self.vel.x, self.vel.theta)
            self.qp.drawEllipse(dx - diameter, dy - diameter, diameter * 2, diameter * 2)

        self.qp.end()

    def toScreen(self, x, theta):
        return scale(x, self.x_min, self.x_max, self.width, self.left), \
            scale(theta, self.t_max, self.t_min, self.height, self.top)

    def toVelocity(self, x, y):
        xv = (x - self.left) / self.width * (self.x_max - self.x_min) + self.x_min
        yv = self.t_max - (y - self.top) / self.height * (self.t_max - self.t_min)
        return xv, yv

    def getNearestVelocity(self, xv, tv):
        best_d = None
        best = None
        for (x, theta), index in self.vel_space.items():
            d = abs(x - xv) + abs(theta - tv)
            if best_d is None or d < best_d:
                best_d = d
                best = index
        return best

    def mousePressEvent(self, event):
        pos = event.pos()
        xv, tv = self.toVelocity(pos.x(), pos.y())

        new_selected = list(self.selected)
        selection = self.getNearestVelocity(xv, tv)
        if selection in self.selected:
            new_selected.remove(selection)
            self.selection_callback(new_selected)
        else:
            new_selected.append(selection)
            self.selection_callback(new_selected)
