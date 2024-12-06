# adapted from https://www.pythonguis.com/faq/pyqt-drag-drop-widgets/from PyQt5 import QtCore
from PyQt5 import QtCore
from PyQt5.QtCore import QMimeData, Qt, pyqtSignal
from PyQt5.QtGui import QDrag, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QGridLayout,
    QWidget,
    QComboBox,
    QLineEdit
)

class DragTargetIndicator(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        #self.setContentsMargins(25, 5, 25, 5)
        self.setStyleSheet("QLabel { background-color: #ccc; border: 1px solid black; }")

    def set_size(self, size):
        self.setFixedSize(size)


class DragItem(QWidget):
    def __init__(self, w):
        super().__init__()
        self.widget = w
        self.widget.destroyed.connect(self.child_destroyed)
        self.setObjectName('main')
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.setStyleSheet('QWidget#main {background-color: lightcyan; border: 1px solid black;}')
        
        self.layout.addWidget(w)

    def set_data(self, data):
        self.data = data

    def child_destroyed(self):
        self.deleteLater()
        
    def mouseMoveEvent(self, e):
        if e.buttons() == Qt.LeftButton:
            drag = QDrag(self)
            mime = QMimeData()
            drag.setMimeData(mime)
            
            pixmap = QPixmap(self.size())
            self.render(pixmap)
            drag.setPixmap(pixmap)
            
            drag.exec_(Qt.MoveAction)
            self.show() # Show this widget again, if it's dropped outside.

            
class DragWidget(QWidget):
    """
    Generic list sorting handler.
    """

    orderChanged = pyqtSignal(list)

    def __init__(self, *args, orientation=Qt.Orientation.Horizontal, **kwargs):
        super().__init__()
        self.setAcceptDrops(True)

        # Store the orientation for drag checks later.
        self.orientation = orientation

        if self.orientation == Qt.Orientation.Vertical:
            self.blayout = QVBoxLayout()
        else:
            self.blayout = QHBoxLayout()

        # Add the drag target indicator. This is invisible by default,
        # we show it and move it around while the drag is active.
        self._drag_target_indicator = DragTargetIndicator()
        self.blayout.addWidget(self._drag_target_indicator)
        self._drag_target_indicator.hide()

        self.setLayout(self.blayout)

    def dragEnterEvent(self, e):
        e.accept()

    def dragLeaveEvent(self, e):
        self._drag_target_indicator.hide()
        e.accept()

    def dragMoveEvent(self, e):
        # Find the correct location of the drop target, so we can move it there.
        index = self._find_drop_location(e)
        if index is not None:
            # Inserting moves the item if its alreaady in the layout.
            self.blayout.insertWidget(index, self._drag_target_indicator)
            # Hide the item being dragged.
            e.source().hide()
            # Show the target.
            self._drag_target_indicator.set_size(e.source().size())
            self._drag_target_indicator.show()
        e.accept()

    def dropEvent(self, e):
        widget = e.source()
        # Use drop target location for destination, then remove it.
        self._drag_target_indicator.hide()
        index = self.blayout.indexOf(self._drag_target_indicator)
        if index is not None:
            self.blayout.insertWidget(index, widget)
            self.orderChanged.emit(self.get_item_data())
            widget.show()
            self.blayout.activate()
        e.accept()

    def _find_drop_location(self, e):
        pos = e.pos()
        spacing = self.blayout.spacing() / 2

        for n in range(self.blayout.count()):
            # Get the widget at each index in turn.
            w = self.blayout.itemAt(n).widget()

            if self.orientation == Qt.Orientation.Vertical:
                # Drag drop vertically.
                drop_here = (
                    pos.y() >= w.y() - spacing
                    and pos.y() <= w.y() + w.size().height() + spacing
                )
            else:
                # Drag drop horizontally.
                drop_here = (
                    pos.x() >= w.x() - spacing
                    and pos.x() <= w.x() + w.size().width() + spacing
                )

            if drop_here:
                # Drop over this target.
                break

        return n

    def add_item(self, item):
        self.blayout.addWidget(item)

    def get_item_data(self):
        data = []
        for n in range(self.blayout.count()):
            # Get the widget at each index in turn.
            w = self.blayout.itemAt(n).widget()
            if hasattr(w, "data"):
                # The target indicator has no data.
                data.append(w.data)
        return data
