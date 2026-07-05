#!/usr/bin/env python3

import argparse
import collections.abc
import os
import time

from io import BytesIO as StringIO
import cv2
from cv_bridge import CvBridge
from matplotlib.figure import Figure
import numpy as np
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget

from ament_index_python.packages import get_package_share_directory
from rqt_gui_py.plugin import Plugin
from rqt_plot.rosplot import ROSData as _ROSData
from rqt_plot.rosplot import RosPlotException
from rqt_py_common.topic_completer import TopicCompleter
from sensor_msgs.msg import Image

from jsk_recognition_msgs.msg import HistogramWithRange

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg \
    as FigureCanvas
try:
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QTAgg \
        as NavigationToolbar
except ImportError:
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT \
        as NavigationToolbar


class ROSData(_ROSData):
    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                return val
            for f in self.field_evals:
                val = f(val)
            return val
        except IndexError:
            self.error = RosPlotException(
                "{0} index error for: {1}".format(
                    self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException(
                "{0} value was not numeric: {1}".format(
                    self.name, val))


class HistogramPlot(Plugin):
    def __init__(self, context):
        super(HistogramPlot, self).__init__(context)
        self.setObjectName('HistogramPlot')
        self._args = self._parse_args(context.argv())
        self._widget = HistogramPlotWidget(context.node, self._args.topics)
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(
            prog='rqt_histogram_plot', add_help=False)
        HistogramPlot.add_arguments(parser)
        args = parser.parse_args(argv)
        return args

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_histogram plugin')
        group.add_argument(
            'topics', nargs='?', default=[], help='Topics to plot')


class HistogramPlotWidget(QWidget):
    _redraw_interval = 40

    def __init__(self, node, topics):
        super(HistogramPlotWidget, self).__init__()
        self.setObjectName('HistogramPlotWidget')
        self._node = node
        ui_file = os.path.join(
            get_package_share_directory('jsk_rqt_plugins'),
            'resource', 'plot_histogram.ui')
        loadUi(ui_file, self)
        self.cv_bridge = CvBridge()
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = MatHistogramPlot(self)
        self.data_plot_layout.addWidget(self.data_plot)
        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics(self._node)
        self.topic_edit.setCompleter(self._topic_completer)
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent
        self._start_time = time.time()
        self._rosdata = None
        self.pub_image = None
        if len(topics) != 0:
            # Right after node creation the ROS graph may not be
            # discovered yet, so resolving the topic type of a topic
            # passed on the command line can fail (in ROS 1 the master
            # always knew it). Poll until the topic appears.
            self._initial_topic = topics
            self._initial_topic_deadline = time.time() + 30.0
            self._initial_topic_timer = QTimer(self)
            self._initial_topic_timer.timeout.connect(
                self._subscribe_initial_topic)
            self._initial_topic_timer.start(500)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._update_plot_timer.start(self._redraw_interval)

    def _subscribe_initial_topic(self):
        names = [n for n, _ in self._node.get_topic_names_and_types()]
        if any(self._initial_topic == n or
               self._initial_topic.startswith(n + '/') for n in names):
            self._initial_topic_timer.stop()
            self.subscribe_topic(self._initial_topic)
        elif time.time() > self._initial_topic_deadline:
            self._initial_topic_timer.stop()
            self._node.get_logger().warn(
                'initial topic %s did not appear within 30s; '
                'subscribe via the GUI' % self._initial_topic)

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.subscribe_topic(topic_name)

    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.subscribe_topic(str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.subscribe_topic(str(self.topic_edit.text()))

    def subscribe_topic(self, topic_name):
        self.topic_with_field_name = topic_name
        try:
            self.pub_image = self._node.create_publisher(
                Image, topic_name + "/histogram_image", 1)
        except Exception as e:
            self._node.get_logger().warn(
                'cannot advertise %s/histogram_image: %s' % (topic_name, e))
            self.pub_image = None
        if not self._rosdata:
            self._rosdata = ROSData(self._node, topic_name, self._start_time)
        else:
            if self._rosdata != topic_name:
                self._rosdata.close()
                self.data_plot.clear()
                self._rosdata = ROSData(
                    self._node, topic_name, self._start_time)
            else:
                self._node.get_logger().warn(
                    "%s is already subscribed" % topic_name)

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()

    @Slot()
    def on_clear_button_clicked(self):
        self.data_plot.clear()

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    def update_plot(self):
        if not self._rosdata:
            return
        try:
            data_x, data_y = self._rosdata.next()
        except RosPlotException as e:
            self._node.get_logger().error("Exception in subscribing topic")
            self._node.get_logger().error(str(e))
            return

        if len(data_y) == 0:
            return
        axes = self.data_plot._canvas.axes
        axes.cla()
        if isinstance(data_y[-1], HistogramWithRange):
            xs = [y.count for y in data_y[-1].bins]
            pos = [y.min_value for y in data_y[-1].bins]
            widths = [y.max_value - y.min_value for y in data_y[-1].bins]
            axes.set_xlim(xmin=pos[0], xmax=pos[-1] + widths[-1])
        elif isinstance(data_y[-1], (collections.abc.Sequence, np.ndarray)):
            xs = data_y[-1]
            pos = np.arange(len(xs))
            widths = [1] * len(xs)
            axes.set_xlim(xmin=0, xmax=len(xs))
        else:
            self._node.get_logger().error(
                "Topic/Field name '%s' has unsupported '%s' type."
                "List of float values and "
                "jsk_recognition_msgs/HistogramWithRange are supported."
                % (self.topic_with_field_name,
                   self._rosdata.sub.msg_type))
            return
        # axes.xticks(range(5))
        for p, x, w in zip(pos, xs, widths):
            axes.bar(p, x, color='r', align='center', width=w)
        axes.legend([self.topic_with_field_name], prop={'size': '8'})
        self.data_plot._canvas.draw()
        if self.pub_image is None:
            return
        try:
            # read the Agg buffer that draw() above just rendered;
            # savefig() on a live Qt canvas races expose/resize events
            # (PIL "tile cannot extend outside image") and an exception
            # escaping this Qt slot would abort the process
            buf = np.asarray(self.data_plot._canvas.buffer_rgba(),
                             dtype=np.uint8)
            img = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)
            self.pub_image.publish(
                self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        except Exception as e:
            self._node.get_logger().debug(
                'skipped histogram_image frame: %s' % e)


class MatHistogramPlot(QWidget):
    class Canvas(FigureCanvas):
        def __init__(self, parent=None):
            super(MatHistogramPlot.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

        def resizeEvent(self, event):
            super(MatHistogramPlot.Canvas, self).resizeEvent(event)
            try:
                self.figure.tight_layout()
            except Exception:
                # tight_layout raises LinAlgError on degenerate
                # (zero-area) canvases, e.g. under a WM-less Xvfb
                pass

    def __init__(self, parent=None):
        super(MatHistogramPlot, self).__init__(parent)
        self._canvas = MatHistogramPlot.Canvas()
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)

    def redraw(self):
        pass

    def clear(self):
        self._canvas.axes.cla()
        self._canvas.draw()
