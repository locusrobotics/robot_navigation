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

import rospy

from .table_widget import TableWidget


class ScoreWidget(TableWidget):
    """This widget displays the critic scores for an arbitrary number of trajectories in tabular format.

    The main data is the dwb_msgs/CriticScore[] scores part for each selected trajectory in a LocalPlanEvaluation.
    Each critic is a different row, and each trajectory is a column.

    Each critic has two columns for row-headers that display the critic's name and scale.

    Each trajectory also has column-headers that display the trajectory's velocity as well as
    column-footer(s) that show the trajectory's total score. This is left extensible for others to modify.
    """

    def __init__(self, parent):
        TableWidget.__init__(self, self.getRowHeaders())
        self.evaluation = None
        self.selected = []
        self.row_mapping = {}  # Save the index of each critic

        # Allow for potential overwrites of headers/footers
        self.row_headers = self.getRowHeaders()
        self.col_headers = self.getColHeaders()
        self.col_footers = self.getColFooters()

    def getRowHeaders(self):
        # Name has to be one of the headers
        return ['Name', 'Scale']

    def getColHeaders(self):
        return ['x velocity', 'theta velocity']

    def getColFooters(self):
        return ['Total']

    def getRowAttribute(self, score, attribute_name):
        if attribute_name == 'Scale':
            return '%.5f' % score.scale

    def getColAttribute(self, twist, attribute_name):
        if attribute_name == 'x velocity':
            return '%.2f' % twist.traj.velocity.x
        elif attribute_name == 'theta velocity':
            return '%.2f' % twist.traj.velocity.theta
        elif attribute_name == 'Total':
            return '%.2f' % twist.total

    def setEvaluation(self, evaluation):
        self.selected = []
        if evaluation is None:
            self.table.setRowCount(0)
            return
        self.evaluation = evaluation
        best = evaluation.twists[evaluation.best_index]

        # One row for each critic, column header and column footer
        self.table.setRowCount(len(best.scores) + len(self.col_headers) + len(self.col_footers))
        self.table.resize(self.size())

        # Populate the Row Headers
        for index, row_header in enumerate(self.row_headers):
            if row_header == 'Name':
                for i, name in enumerate(self.col_headers):
                    self.setValue(i, index, name)
                row_offset = len(self.col_headers)
                for i, score in enumerate(best.scores):
                    self.setValue(i + row_offset, index, score.name)
                    self.row_mapping[score.name] = i + row_offset

                row_offset = len(self.col_headers) + len(best.scores)
                for i, name in enumerate(self.col_footers):
                    self.setValue(i + row_offset, index, name)
            else:
                # Populate row headers for each critic
                row_offset = len(self.col_headers)
                for i, score in enumerate(best.scores):
                    self.setValue(i + row_offset, index, self.getRowAttribute(score, row_header))

                self.table.resizeColumnToContents(index)

        self.update()

    def updateColumn(self, twist, column, name, color):
        self.setColumnHeader(column, name)

        # Headers
        for i, name in enumerate(self.col_headers):
            self.setValue(i, column, self.getColAttribute(twist, name))

        # Main Data
        row_offset = len(self.col_headers)
        for score in twist.scores:
            if score.name in self.row_mapping:
                self.setValue(self.row_mapping[score.name], column, '%.2f' % score.raw_score)
            else:
                rospy.logwarn('Unknown row name: {}'.format(score.name))

        # Footers
        row_offset = len(self.col_headers) + len(self.row_mapping)
        for i, name in enumerate(self.col_footers):
            self.setValue(i + row_offset, column, self.getColAttribute(twist, name))

        self.setColumnColor(column, color)
        self.table.resizeColumnToContents(column)

    def getColumnName(self, index):
        if index == self.evaluation.best_index:
            return 'Best'

    def setSelected(self, selected):
        n_row_headers = len(self.row_headers)
        # One column for each row header and selected trajectory
        self.table.setColumnCount(n_row_headers + len(selected))
        for col_offset, index in enumerate(selected):
            col_name = self.getColumnName(index) or ''
            self.updateColumn(self.evaluation.twists[index], n_row_headers + col_offset, col_name, selected[index])
