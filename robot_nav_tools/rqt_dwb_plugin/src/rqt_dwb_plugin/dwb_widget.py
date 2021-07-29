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

import collections

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QSizePolicy, QSplitter, QVBoxLayout, QWidget

from .score_widget import ScoreWidget
from .sorted_twists_widget import SortedTwistsWidget
from .trajectory_widget import TrajectoryWidget
from .util import PALETTE
from .velocity_space_widget import VelocitySpaceWidget


class DWBWidget(QWidget):
    """Interface for viewing LocalPlanEvaluation messages one at a time.

    Within the individual messages, there is also the capability for highlighting/selecting a
    subset of the trajectories.

    The panel is composed of four sub-widgets. When a message is selected through the message_viewed method,
    this class sets the data for all the sub-widgets and highlights the best trajectory.

    Users can select additional trajectories through two of the widgets and update the trajectory subset
    with the setSelected method. Each of those trajectories is mapped to a different color. This class
    manages that mapping too.
    """

    def __init__(self, parent, bounds=None):
        super(DWBWidget, self).__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.widgets = []

        full_layout = QVBoxLayout(self)
        vsplitter = QSplitter(Qt.Vertical, self)
        full_layout.addWidget(vsplitter)

        widget0 = self.getTopWidget(bounds)
        vsplitter.addWidget(widget0)
        self.widgets.append(widget0)

        hsplitter = QSplitter(Qt.Horizontal, vsplitter)
        widget1 = self.getLeftWidget(bounds)
        hsplitter.addWidget(widget1)
        self.widgets.append(widget1)

        widget2 = self.getRightWidget(bounds)
        hsplitter.addWidget(widget2)
        self.widgets.append(widget2)

        hsplitter.setSizes([1e10] * 2)  # Makes them equally sized
        vsplitter.addWidget(hsplitter)

        widget3 = self.getBottomWidget(bounds)
        vsplitter.addWidget(widget3)
        self.widgets.append(widget3)

        vsplitter.setSizes([1e10] * 3)

        self.color_map = {}

    def getTopWidget(self, bounds):
        return TrajectoryWidget(self, bounds)

    def getLeftWidget(self, bounds):
        return VelocitySpaceWidget(self, self.setSelected)

    def getRightWidget(self, bounds):
        return SortedTwistsWidget(self, self.setSelected)

    def getBottomWidget(self, bounds):
        return ScoreWidget(self)

    def setEvaluation(self, msg):
        for widget in self.widgets:
            widget.setEvaluation(msg)
        self.setSelected([msg.best_index])

    def setPlan(self, plan):
        for widget in self.widgets:
            if hasattr(widget, 'setPlan'):
                widget.setPlan(plan)

    def setVelocity(self, vel):
        for widget in self.widgets:
            if hasattr(widget, 'setVelocity'):
                widget.setVelocity(vel)

    def setSelected(self, selected=[]):
        selected = self.updateColorMap(selected)
        for widget in self.widgets:
            widget.setSelected(selected)

    def updateColorMap(self, selected):
        new_color_map = collections.OrderedDict()
        used_colors = set()

        # Keep existing colors the same
        for index in selected:
            if index in self.color_map:
                new_color_map[index] = self.color_map[index]
                used_colors.add(new_color_map[index])

        # Get New Colors
        color_index = 0
        for index in selected:
            if index in self.color_map:
                continue
            while PALETTE[color_index] in used_colors:
                color_index += 1
            new_color_map[index] = PALETTE[color_index]
            color_index += 1

        self.color_map = new_color_map
        return new_color_map
