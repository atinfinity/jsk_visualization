from threading import Lock

import cv2
import numpy as np
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
import python_qt_binding.QtCore as QtCore
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QImage
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QDialog
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from image_view2.msg import MouseEvent
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import Image


class ComboBoxDialog(QDialog):
    def __init__(self, parent=None):
        super(ComboBoxDialog, self).__init__()
        self.number = 0
        vbox = QVBoxLayout(self)
        self.combo_box = QComboBox(self)
        self.combo_box.activated.connect(self.onActivated)
        vbox.addWidget(self.combo_box)
        button = QPushButton()
        button.setText("Done")
        button.clicked.connect(self.buttonCallback)
        vbox.addWidget(button)
        self.setLayout(vbox)

    def buttonCallback(self, event):
        self.close()

    def onActivated(self, number):
        self.number = number


class ImageView2Plugin(Plugin):
    """
    rqt wrapper for image_view2
    """
    def __init__(self, context):
        super(ImageView2Plugin, self).__init__(context)
        self.setObjectName("ImageView2Plugin")
        self._widget = ImageView2Widget(context.node)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._widget.trigger_configuration()


class ScaledLabel(QLabel):
    def __init__(self, *args, **kwargs):
        QLabel.__init__(self)
        self._pixmap = QPixmap(self.pixmap())

    def resizeEvent(self, event):
        self.setPixmap(self._pixmap.scaled(
            self.width(), self.height(),
            QtCore.Qt.KeepAspectRatio))


class ImageView2Widget(QWidget):
    """
    Qt widget to communicate with image_view2
    """
    cv_image = None
    pixmap = None
    repaint_trigger = pyqtSignal()

    def __init__(self, node):
        super(ImageView2Widget, self).__init__()
        self._node = node
        self.left_button_clicked = False

        self.repaint_trigger.connect(self.redraw)
        self.lock = Lock()
        self.need_to_rewrite = False
        self.bridge = CvBridge()
        self.image_sub = None
        self.event_pub = None
        self.label = ScaledLabel()
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setSizePolicy(
            QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored))
        # self.label.installEventFilter(self)
        vbox = QVBoxLayout(self)
        vbox.addWidget(self.label)
        self.setLayout(vbox)

        self._image_topics = []
        # In ROS 1 this was a (one-shot) background thread; a periodic
        # QTimer poll of the ROS graph gives the same behavior and keeps
        # all GUI updates in the Qt thread.
        self._update_topic_timer = QTimer(self)
        self._update_topic_timer.timeout.connect(self.updateTopics)
        self._update_topic_timer.start(1000)

        self._active_topic = None
        self.setMouseTracking(True)
        self.label.setMouseTracking(True)
        self._dialog = ComboBoxDialog()
        self.show()

    def trigger_configuration(self):
        self._dialog.exec_()
        self.setupSubscriber(self._image_topics[self._dialog.number])

    def setupSubscriber(self, topic):
        if self.image_sub:
            self._node.destroy_subscription(self.image_sub)
        self._node.get_logger().info(
            "Subscribing %s" % (topic + "/marked"))
        self.image_sub = self._node.create_subscription(
            Image, topic + "/marked", self.imageCallback, 1)
        self.event_pub = self._node.create_publisher(
            MouseEvent, topic + "/event", 1)
        self._active_topic = topic

    def onActivated(self, number):
        self.setupSubscriber(self._image_topics[number])

    def imageCallback(self, msg):
        with self.lock:
            if msg.width == 0 or msg.height == 0:
                self._node.get_logger().debug(
                    "Looks input images is invalid")
                return
            cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if msg.encoding == "bgr8":
                self.cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif msg.encoding == "rgb8":
                self.cv_image = cv_image
            self.numpy_image = np.asarray(self.cv_image)
            self.need_to_rewrite = True
            self.repaint_trigger.emit()

    def updateTopics(self):
        if not self._node.context.ok():
            # the Qt timer can fire while rclpy is shutting down
            return
        need_to_update = False
        for (topic, topic_types) in self._node.get_topic_names_and_types():
            if 'sensor_msgs/msg/Image' in topic_types:
                with self.lock:
                    if not topic in self._image_topics:
                        self._image_topics.append(topic)
                        need_to_update = True
        if need_to_update:
            with self.lock:
                self._image_topics = sorted(self._image_topics)
                self._dialog.combo_box.clear()
                for topic in self._image_topics:
                    self._dialog.combo_box.addItem(topic)
                if self._active_topic:
                    self._dialog.combo_box.setCurrentIndex(
                        self._image_topics.index(self._active_topic))

    @pyqtSlot()
    def redraw(self):
        with self.lock:
            if not self.need_to_rewrite:
                return
            if np.all(self.cv_image) is not None:
                size = self.cv_image.shape
                img = QImage(self.cv_image.data,
                             size[1], size[0], size[2] * size[1],
                             QImage.Format_RGB888)
                # convert to QPixmap
                self.pixmap = QPixmap(size[1], size[0])
                self.pixmap.convertFromImage(img)
                self.label.setPixmap(self.pixmap.scaled(
                    self.label.width(), self.label.height(),
                    QtCore.Qt.KeepAspectRatio))
                # self.label.setPixmap(self.pixmap)

    def mousePosition(self, e):
        label_x = self.label.x()
        label_y = self.label.y()
        label_width = self.label.width()
        label_height = self.label.height()
        pixmap_width = self.label.pixmap().width()
        pixmap_height = self.label.pixmap().height()
        x_offset = (label_width - pixmap_width) / 2.0 + label_x
        y_offset = (label_height - pixmap_height) / 2.0 + label_y
        return (e.x() - x_offset, e.y() - y_offset)

    def mouseMoveEvent(self, e):
        msg = MouseEvent()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.type = MouseEvent.MOUSE_MOVE
        msg.width = self.label.pixmap().width()
        msg.height = self.label.pixmap().height()
        x, y = self.mousePosition(e)
        msg.x = int(x)
        msg.y = int(y)
        if self.event_pub:
            self.event_pub.publish(msg)

    def mousePressEvent(self, e):
        msg = MouseEvent()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        if e.button() == Qt.LeftButton:
            msg.type = MouseEvent.MOUSE_LEFT_DOWN
            self.left_button_clicked = True
        elif e.button() == Qt.RightButton:
            msg.type = MouseEvent.MOUSE_RIGHT_DOWN
        msg.width = self.label.pixmap().width()
        msg.height = self.label.pixmap().height()
        x, y = self.mousePosition(e)
        msg.x = int(x)
        msg.y = int(y)
        if self.event_pub:
            self.event_pub.publish(msg)

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.left_button_clicked = False
            msg = MouseEvent()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.width = self.label.pixmap().width()
            msg.height = self.label.pixmap().height()
            msg.type = MouseEvent.MOUSE_LEFT_UP
            x, y = self.mousePosition(e)
            msg.x = int(x)
            msg.y = int(y)
            if self.event_pub:
                self.event_pub.publish(msg)

    def save_settings(self, plugin_settings, instance_settings):
        if self._active_topic:
            instance_settings.set_value("active_topic", self._active_topic)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("active_topic"):
            topic = instance_settings.value("active_topic")
            self._dialog.combo_box.addItem(topic)
            self.setupSubscriber(topic)
