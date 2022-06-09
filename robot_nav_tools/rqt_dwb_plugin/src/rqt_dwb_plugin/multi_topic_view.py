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

from collections import namedtuple

import rospy

from rqt_bag import TopicMessageView

"""
   The topic is a string
   The datatype can either be a string message type (i.e. 'nav_msgs/Path') or list of string message types
   The callback will take the same arguments as message_viewed
"""
ExtraTopic = namedtuple('ExtraTopic', 'topic datatype callback')


class MultiTopicView(TopicMessageView):
    """Generic tool for visualizing multiple topics in one TopicMessageView.

    Visualization is triggered from one main_topic (whose datatype is determined
    by Plugin.get_message_types) and then the other topics are determined by calling
    get_extra_topics. The other messages visualized are those appearing in a time window
    surrounding the message from the main topic.
    """

    def __init__(self, timeline, parent, main_topic, window=0.5):
        """Constructor.

        main_topic provides the topic of the messages to focus on.
        window provides the number of seconds for the surrounding time window.
        """
        super(MultiTopicView, self).__init__(timeline, parent, main_topic)
        self.main_topic = self.topic  # Equivalent to main_topic
        self.window = rospy.Duration(window)

        # confirm extra topics are in the bag
        self.extra_topic_callbacks = {}
        found_topics = timeline._get_topics()
        missing_topics = []
        for extra_topic, datatype, callback in self.get_extra_topics():
            if extra_topic not in found_topics:
                missing_topics.append(extra_topic)

            found_datatype = timeline.get_datatype(extra_topic)
            if type(datatype) == list:
                if found_datatype not in datatype:
                    rospy.logwarn('The type of extra topic {} ({}) does not match the declared types: {}'.format(
                                  extra_topic, found_datatype, ', '.join(map(str, datatype))))
                    continue
            elif datatype != found_datatype:
                rospy.logwarn('The type of extra topic {} ({}) does not match the declared type {}'.format(
                              extra_topic, found_datatype, datatype))
                continue

            self.extra_topic_callbacks[extra_topic] = callback

        if missing_topics:
            rospy.logwarn('The following extra_topics were not found in the bag: ' + ', '.join(missing_topics))

    def get_extra_topics(self):
        """Return a list of ExtraTopic tuples with extra topics to visualize. Should be overridden."""
        return []

    def get_window_messages(self, topic, ts):
        """Get all the messages within the time window."""
        return self.timeline.get_entries([topic], ts - self.window, ts + self.window)

    def get_closest_message(self, topic, ts):
        """Get the entry of the message closest to the given timestamp."""
        best = None
        best_t = None
        for entry in self.get_window_messages(topic, ts):
            if best is None or abs(entry.time - ts) < best_t:
                best = entry
                best_t = abs(entry.time - ts)
        return best

    def message_viewed(self, bag, msg_details):
        """Call the callbacks associated with the other topics.

        Automatically called whenever a new single message is focused on.
        Classes extending this class should call this super-version of the method first.
        """
        super(MultiTopicView, self).message_viewed(bag, msg_details)
        main_topic, msg, ts = msg_details
        for extra_topic, callback in self.extra_topic_callbacks.items():
            best_entry = self.get_closest_message(extra_topic, ts)
            if best_entry:
                extra_msg_details = self.timeline.read_message(bag, best_entry.position)
                callback(bag, extra_msg_details)
