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

from .bounds import Bounds
from .util import FIELDS

cache = None


def get_cache():
    global cache
    if cache is None:
        cache = EvaluationCache()
    return cache


class EvaluationCache(Bounds):
    """This class loads all the Evaluation messages from the bag files and maintains stats about the range of values."""

    def __init__(self):
        Bounds.__init__(self)
        self.cache = collections.defaultdict(dict)
        self.v_max = None
        self.v_min = None

    def getMessages(self, scene, topic, start, end):
        msgs = []
        for bag, entry in scene.get_entries_with_bags([topic], start, end):
            if entry.position in self.cache[bag]:
                msgs.append(self.cache[bag][entry.position])
                continue
            topic, msg, t = scene.read_message(bag, entry.position)
            self.cache[bag][entry.position] = t.to_sec(), msg
            msgs.append(self.cache[bag][entry.position])

            # Check trajectory bounds
            self.process_evaluation(msg)

            # Check Stats
            best = msg.twists[msg.best_index].traj.velocity
            for field in FIELDS:
                value = getattr(best, field)
                if self.v_max is None:
                    self.v_max = value
                    self.v_min = value
                else:
                    self.v_max = max(self.v_max, value)
                    self.v_min = min(self.v_min, value)

        return msgs
