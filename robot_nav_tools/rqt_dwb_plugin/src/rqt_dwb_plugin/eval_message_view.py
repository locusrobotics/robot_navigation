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

from nav_2d_utils.conversions import pathToPath2D

from .dwb_widget import DWBWidget
from .evaluation_cache import get_cache
from .multi_topic_view import ExtraTopic, MultiTopicView


class EvalView(MultiTopicView):
    """Independent panel in rqt_bag interface for viewing LocalPlanEvaluation messages one at a time.

    Uses the DWBWidget for most of the functionality.
    """

    name = 'EvalView'

    def __init__(self, timeline, parent, topic):
        super(EvalView, self).__init__(timeline, parent, topic)

        # When this View is initialized, pre-load all the messages
        cache = get_cache()
        cache.getMessages(timeline, topic, timeline._get_start_stamp(), timeline._get_end_stamp())
        self.dwb_widget = DWBWidget(parent, cache)
        parent.layout().addWidget(self.dwb_widget)

    def get_extra_topics(self):
        base = self.main_topic.rpartition('/')[0]
        return [
            ExtraTopic(base + '/transformed_global_plan', 'nav_msgs/Path', self.plan_viewed),
            ExtraTopic(base + '/velocity', 'nav_2d_msgs/Twist2D', self.velocity_viewed)
        ]

    def message_viewed(self, bag, msg_details):
        super(EvalView, self).message_viewed(bag, msg_details)
        _, msg, _ = msg_details
        self.dwb_widget.setEvaluation(msg)

    def plan_viewed(self, bag, msg_details):
        _, msg, _ = msg_details
        self.dwb_widget.setPlan(pathToPath2D(msg))

    def velocity_viewed(self, bag, msg_details):
        _, msg, _ = msg_details
        self.dwb_widget.setVelocity(msg)
