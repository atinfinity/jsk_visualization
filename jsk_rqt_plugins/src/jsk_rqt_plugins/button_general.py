import os

import python_qt_binding.QtCore as QtCore
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtCore import QTranslator
from python_qt_binding.QtCore import Signal
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QMessageBox
from python_qt_binding.QtWidgets import QRadioButton
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QToolButton
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
import yaml

from rclpy.exceptions import ParameterAlreadyDeclaredException
from resource_retriever import get_filename
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


class ServiceButtonGeneralWidget(QWidget):
    """
    Qt widget to visualize multiple buttons
    """
    # emitted from the ROS spinner thread when an asynchronous service call
    # failed; handled in the Qt GUI thread (queued connection)
    _call_failed = Signal(str, object, bool)

    def __init__(self, node, button_type="push"):
        super(ServiceButtonGeneralWidget, self).__init__()
        self._node = node
        self.button_type = button_type
        self._layout_param = None
        self._service_clients = {}
        self._translator = QTranslator()
        self._dialog = QFileDialog()
        self._dialog.setFileMode(QFileDialog.ExistingFile)
        self._call_failed.connect(self._on_call_failed)

        try:
            self._node.declare_parameter('layout_yaml_file', '')
        except ParameterAlreadyDeclaredException:
            pass
        if self._node.get_parameter('layout_yaml_file').value:
            self.loadLayoutYaml(None)

        self.show()

    def showError(self, message):
        QMessageBox.about(self, "ERROR", message)

    def loadLayoutYaml(self, layout_param):
        # Initialize layout of the buttons from yaml file
        # The yaml file can be specified by ROS parameter
        layout_yaml_file = (
            self._node.get_parameter('layout_yaml_file').value
            or layout_param)
        if not layout_yaml_file:
            self.showError("No yaml file is specified for layout")
            return False
        resolved_layout_yaml_file = get_filename(layout_yaml_file)
        if (resolved_layout_yaml_file is not None
                and resolved_layout_yaml_file.startswith("file://")):
            resolved_layout_yaml_file = resolved_layout_yaml_file[len("file://"):]
        # check file exists
        if os.path.exists(resolved_layout_yaml_file):
            self.setupButtons(resolved_layout_yaml_file)
            self.show()
            return True
        else:
            self.showError("Cannot find %s (%s)" % (
                           layout_yaml_file, resolved_layout_yaml_file))
            return False

    def setupButtons(self, yaml_file):
        """
        Parse yaml file and setup Buttons.
        """
        with open(yaml_file) as f:
            yaml_data = yaml.safe_load(f)
            self.setupButtons_with_yaml_data(yaml_data)

    def setupButtons_with_yaml_data(self, yaml_data, namespace=None):
        """
        Setup Buttons with yaml_data which is loaded in setupButtons. Format of the yaml file should be:
        - name: 'button name' (required)
          image: 'path to image for icon' (optional)
          image_size: 'width and height of icon' (optional)
          service: 'service' (required)
          column: 'column index' (optional, defaults to 0)
        """
        self.buttons = []
        if True:
            # lookup colum direction
            direction = 'vertical'
            for d in yaml_data:
                if 'direction' in d:
                    if d['direction'] == 'horizontal':
                        direction = 'horizontal'
                    else: # d['direction'] == 'vertical':
                        direction = 'vertical'
                    yaml_data.remove(d)
                    break
            # lookup column num
            column_indices = [d['column'] for d in yaml_data]
            if len(column_indices) > 1:
                max_column_index = max(*column_indices)
            else:
                max_column_index = column_indices[0]
            if direction == 'vertical':
                self.layout = QHBoxLayout()
                self.layout_boxes = [QVBoxLayout()
                                     for i in range(max_column_index + 1)]
            else:  # direction == 'horizontal'
                self.layout = QVBoxLayout()
                self.layout_boxes = [QHBoxLayout()
                                     for i in range(max_column_index + 1)]
            self.button_groups = [QGroupBox()
                                  for i in range(max_column_index + 1)]
            for button_data in yaml_data:
                # check if all the field is available
                if "name" not in button_data:
                    self.showError("name field is missed in yaml")
                    raise Exception("name field is missed in yaml")
                if "service" not in button_data:
                    self.showError("service field is missed in yaml")
                    raise Exception("service field is missed in yaml")
                if self.button_type == "push":
                    button = QToolButton()
                else:  # self.button_type == "radio":
                    button = QRadioButton()
                button.setSizePolicy(
                    QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred))
                if "image" in button_data:
                    image_file = get_filename(
                        button_data["image"])[len("file://"):]
                    if os.path.exists(image_file):
                        icon = QtGui.QIcon(image_file)
                        button.setIcon(icon)
                        if "image_size" in button_data:
                            button.setIconSize(QSize(
                                button_data["image_size"][0],
                                button_data["image_size"][1]))
                        else:
                            button.setIconSize(QSize(100, 100))
                if "name" in button_data:
                    name = button_data['name']
                    button.setText(name)
                if 'service_type' in button_data:
                    if button_data['service_type'] == 'Trigger':
                        service_type = Trigger
                    elif button_data['service_type'] == 'Empty':
                        service_type = Empty
                    elif button_data['service_type'] == 'SetBool':
                        service_type = SetBool
                    else:
                        raise Exception("Unsupported service type: {}".format(
                            button_data['service_type']))
                else:
                    service_type = Empty
                if service_type == SetBool:
                    button.setCheckable(True)
                if namespace:
                    sname = button_data['service']
                    if sname[0] == '/' or sname[0] == '~':
                        sname = sname
                    else:
                        sname = namespace + '/' + sname
                    button.clicked.connect(
                        self.buttonCallback(sname, service_type, button))
                else:
                    button.clicked.connect(
                        self.buttonCallback(button_data['service'], service_type, button))
                if self.button_type == "push":
                    button.setToolButtonStyle(
                        QtCore.Qt.ToolButtonTextUnderIcon)
                if ((self.button_type == "radio" or service_type == SetBool)
                        and ("default_value" in button_data
                             and button_data['default_value'])):
                    button.setChecked(True)
                self.layout_boxes[button_data['column']].addWidget(button)
                self.buttons.append(button)
            for i in range(len(self.button_groups)):
                self.button_groups[i].setLayout(self.layout_boxes[i])
            for group in self.button_groups:
                self.layout.addWidget(group)
            self.setLayout(self.layout)

    def buttonCallback(self, service_name, service_type, button):
        """
        return function as callback
        """
        return lambda checked: self.buttonCallbackImpl(checked, service_name, service_type, button)

    def buttonCallbackImpl(self, checked, service_name, service_type=Empty, button=None):
        key = (service_name, service_type)
        client = self._service_clients.get(key)
        if client is None:
            client = self._node.create_client(service_type, service_name)
            self._service_clients[key] = client
        if (not client.service_is_ready()
                and not client.wait_for_service(timeout_sec=1.0)):
            self._on_call_failed(
                "Failed to call %s" % service_name,
                button if service_type == SetBool else None, checked)
            return
        req = service_type.Request()
        if service_type == SetBool:
            req.data = checked
        future = client.call_async(req)
        # NOTE: the done callback runs on the rqt spinner thread. GUI updates
        # are marshalled back to the Qt thread via the _call_failed signal.
        future.add_done_callback(
            lambda f: self._doneCallback(
                f, service_name, service_type, button, checked))

    def _doneCallback(self, future, service_name, service_type, button, checked):
        try:
            res = future.result()
        except Exception as e:
            self._node.get_logger().error(
                'Failed to call {}: {}'.format(service_name, e))
            self._call_failed.emit(
                "Failed to call %s" % service_name,
                button if service_type == SetBool else None, checked)
            return
        if hasattr(res, 'success') and not res.success:
            self._call_failed.emit(
                "Succeeded to call {}, but service response is res.success=False"
                .format(service_name),
                button if service_type == SetBool else None, checked)

    def _on_call_failed(self, message, button, checked):
        self.showError(message)
        if button is not None:
            button.setChecked(not checked)

    def save_settings(self, plugin_settings, instance_settings):
        if self._layout_param:
            instance_settings.set_value("layout_param", self._layout_param)
            self._node.get_logger().info(
                "save setting is called. %s" % self._layout_param)

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("layout_param"):
            self._layout_param = instance_settings.value("layout_param")
            self._node.get_logger().info(
                "restore setting is called. %s" % self._layout_param)
            updated = self.loadLayoutYaml(self._layout_param)
            if updated:
                self._node.get_logger().info(
                    "succeeded to restore. %s" % self._layout_param)
            else:
                self._node.get_logger().error(
                    "failed to restore. %s" % self._layout_param)

    def trigger_configuration(self):
        self._layout_param = self._dialog.getOpenFileName(
            None, self._translator.tr("Open File"), "",
            self._translator.tr("YAML files (*.yaml *.yml)"))[0]

        if self._layout_param:
            updated = self.loadLayoutYaml(self._layout_param)
            self._node.get_logger().info(
                "trigger configuration is called. %s" % self._layout_param)
            if updated:
                self._node.get_logger().info(
                    "succeeded to configure. %s" % self._layout_param)
            else:
                self._node.get_logger().error(
                    "failed to configure. %s" % self._layout_param)
                self.trigger_configuration()
