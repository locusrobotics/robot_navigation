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

from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QAbstractItemView, QHeaderView, QTableWidget, QTableWidgetItem, QWidget

WHITE = QColor(255, 255, 255)


class TableWidget(QWidget):
    """Workaround for apparent bug in embedding QTableWidget.

    For unknown reasons, embedding a QTableWidget in a Widget directly results in the Table being
    displayed in the wrong spot. To avoid that, this Widget embeds it within a widget within the widget.
    It's widgets all the way down. This class also provides a few helper functions for quickly setting
    table properties.
    """

    def __init__(self, headers, stretch_column=0):
        QWidget.__init__(self)
        self.table = QTableWidget(self)
        self.table.setColumnCount(len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(stretch_column, QHeaderView.Stretch)
        self.table.resize(self.size())
        self.table.resizeColumnsToContents()
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)

    def resizeEvent(self, event):
        self.table.resize(event.size())

    def setValue(self, x, y, value):
        self.table.setItem(x, y, QTableWidgetItem(str(value)))

    def setColumnHeader(self, column, value):
        self.table.setHorizontalHeaderItem(column, QTableWidgetItem(str(value)))

    def setColor(self, x, y, color):
        item = self.table.item(x, y)
        if not item:
            return
        if color is None:
            color = WHITE
        item.setBackground(color)

    def setRowColor(self, row, color):
        for column in range(self.table.columnCount()):
            self.setColor(row, column, color)

    def setColumnColor(self, column, color):
        for row in range(self.table.rowCount()):
            self.setColor(row, column, color)
