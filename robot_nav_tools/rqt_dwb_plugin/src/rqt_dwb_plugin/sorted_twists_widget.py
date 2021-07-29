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

from python_qt_binding.QtWidgets import QAbstractItemView, QTableWidgetSelectionRange

from .table_widget import TableWidget

HEADERS = ['Score', 'X', 'Theta']


class SortedTwistsWidget(TableWidget):
    """This widget displays a sorted list of all the trajectories in tabular format."""

    def __init__(self, parent, selection_callback):
        TableWidget.__init__(self, HEADERS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.itemSelectionChanged.connect(self.itemSelectionChanged)
        self.evaluation = None
        self.selected = []
        self.selection_callback = selection_callback
        self.sorted_twists = []
        self.row_mapping = []
        self.update_selection = True

    def setEvaluation(self, evaluation):
        self.evaluation = evaluation
        self.selected = []
        self.table.resize(self.size())
        self.table.setRowCount(len(evaluation.twists))

        # Tuples of original index and twist, sorted by increasing total cost
        self.sorted_twists = sorted(enumerate(evaluation.twists), key=lambda x: (x[1].total < 0, x[1].total))

        self.row_mapping = [-1] * len(evaluation.twists)
        for row_index, (original_index, twist) in enumerate(self.sorted_twists):
            self.setValue(row_index, 0, '%5.2f' % twist.total)
            self.setValue(row_index, 1, '%.2f' % twist.traj.velocity.x)
            self.setValue(row_index, 2, '%.2f' % twist.traj.velocity.theta)
            self.row_mapping[original_index] = row_index
        self.update()

    def setSelected(self, selected):
        for index in self.selected:
            if index in selected:
                continue
            row = self.row_mapping[index]
            self.setRowColor(row, None)

        self.selected = selected
        self.update_selection = False

        # Deselect Everything
        self.table.setRangeSelected(QTableWidgetSelectionRange(0, 0,
                                                               self.table.rowCount() - 1, self.table.columnCount() - 1
                                                               ),
                                    False)

        for index in selected:
            row = self.row_mapping[index]
            self.setRowColor(row, selected[index])
        self.update_selection = True

    def getSelectedRows(self):
        rows = set()
        for index in self.table.selectedIndexes():
            rows.add(index.row())
        return list(rows)

    def itemSelectionChanged(self):
        if not self.update_selection:
            return
        rows = self.getSelectedRows()

        new_selected = list(self.selected)
        for row_num in rows:
            selection = self.sorted_twists[row_num][0]
            if selection in self.selected:
                new_selected.remove(selection)
            else:
                new_selected.append(selection)
        self.selection_callback(new_selected)
