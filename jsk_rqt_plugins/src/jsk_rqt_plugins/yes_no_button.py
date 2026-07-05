#!/usr/bin/env python3

import os
import time

from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin

from jsk_gui_msgs.srv import YesNo


class YesNoButtonWidget(QWidget):
    # emitted from the ROS spinner thread; handled in the Qt GUI thread
    _question = Signal(str)
    _answered = Signal()

    def __init__(self, node):
        super(YesNoButtonWidget, self).__init__()
        self._node = node
        ui_file = os.path.join(
            get_package_share_directory('jsk_rqt_plugins'),
            'resource', 'yes_no_button.ui')
        loadUi(ui_file, self)
        self.setObjectName('YesNoButtonUi')
        self.yes_button.clicked[bool].connect(self._clicked_yes_button)
        self.no_button.clicked[bool].connect(self._clicked_no_button)
        self.yes_button.setEnabled(False)
        self.no_button.setEnabled(False)
        self._question.connect(self._show_question)
        self._answered.connect(self._disable_buttons)
        self.yes = None
        service_name = self._node.resolve_service_name('rqt_yn_btn')
        advertised_services = [
            name for name, _ in self._node.get_service_names_and_types()]
        if service_name in advertised_services:
            self._node.get_logger().warn(
                '{} is already advertised'.format(service_name))
            return
        self.srv = self._node.create_service(
            YesNo, 'rqt_yn_btn', self._handle_yn_btn)

    def _clicked_yes_button(self):
        """Handle events of being clicked yes button."""
        self.yes = True

    def _clicked_no_button(self):
        """Handle events of being clicked no button."""
        self.yes = False

    def _show_question(self, message):
        self.message.setText(message)
        self.yes_button.setEnabled(True)
        self.no_button.setEnabled(True)

    def _disable_buttons(self):
        self.yes_button.setEnabled(False)
        self.no_button.setEnabled(False)

    def _handle_yn_btn(self, req, res):
        """Callback function of service,

        and handle enable/disable of the buttons.
        """
        # Blocking in this callback is acceptable here: it runs on rqt's
        # background spinner thread, which is distinct from the Qt GUI
        # thread, so the buttons stay clickable while we wait. GUI updates
        # are marshalled to the Qt thread via signals. This is the only
        # service served by this plugin, but the wait still aborts after
        # 60s so the (single-threaded) executor cannot get stuck forever.
        self.yes = None  # initialize
        self._question.emit(req.message)
        timeout = time.time() + 60.0
        while self.yes is None:  # wait for user input
            if time.time() > timeout:
                self._node.get_logger().warn(
                    'rqt_yn_btn got no user input within 60s; '
                    'returning yes=False')
                break
            time.sleep(0.1)
        self._answered.emit()
        res.yes = bool(self.yes) if self.yes is not None else False
        return res

    def __del__(self):
        if hasattr(self, 'srv'):
            self._node.destroy_service(self.srv)


class YesNoButton(Plugin):
    def __init__(self, context):
        super(YesNoButton, self).__init__(context)
        self.setObjectName('YesNoButton')
        self._widget = YesNoButtonWidget(context.node)
        context.add_widget(self._widget)
