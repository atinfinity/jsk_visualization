from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QMessageBox
from python_qt_binding.QtWidgets import QTabWidget
from python_qt_binding.QtWidgets import QWidget
import yaml

from rclpy.exceptions import ParameterAlreadyDeclaredException
from resource_retriever import get_filename

from jsk_rqt_plugins.button_general import ServiceButtonGeneralWidget


class ServiceTabbedButtonGeneralWidget(QWidget):
    def __init__(self, node):
        super(ServiceTabbedButtonGeneralWidget, self).__init__()
        self._node = node
        self._tab_settings = None

        # ROS 2 parameters cannot hold nested dicts like the ROS 1
        # ~tabbed_layout parameter, so the layout is given as a yaml file
        # (parameter 'tabbed_layout_yaml_file') with the same content as the
        # ROS 1 ~tabbed_layout parameter.
        tabbed_layout = self._load_tabbed_layout()
        if tabbed_layout is None:
            self.showError(
                "Cannot find parameter tabbed_layout_yaml_file")
            return
        self._tab_list = []
        if not 'tab_list' in tabbed_layout:
            self.showError("Cannot find tab_list in %s"%(tabbed_layout))
            return
        tab_list = tabbed_layout['tab_list']
        for tb in tab_list:
            if tb in tabbed_layout:
                param_settings = tabbed_layout[tb]
                settings = {}
                ##
                if 'type' in param_settings:
                    settings['type'] = param_settings['type']
                ##
                if not 'name' in param_settings:
                    settings['name'] = tb
                else:
                    settings['name'] = param_settings['name']
                ##
                if 'yaml_file' in param_settings:
                    settings['yaml_file'] =  param_settings['yaml_file']
                else:
                    self.showError("Cannot find yaml_file in %s"%(tb))
                    settings = None
                ##
                if 'namespace' in param_settings:
                    settings['namespace'] =  param_settings['namespace']

                if settings:
                    self._tab_list.append(settings)
            else:
                self.showError("Cannot find key %s in %s"%(tb, tabbed_layout))

        if len(self._tab_list) == 0:
            self.showError("there is no valid param in tabbed_layout")
            return

        qtab = QTabWidget()
        for tb in self._tab_list:
            wg = ServiceButtonGeneralWidget_in_tab(self._node, tb)
            qtab.addTab(wg, tb['name'] )

        #self.setWindowTitle('Tab Layout')
        hbox = QHBoxLayout()
        hbox.addWidget(qtab)
        self.setLayout(hbox)
        self.show()

    def _load_tabbed_layout(self):
        try:
            self._node.declare_parameter('tabbed_layout_yaml_file', '')
        except ParameterAlreadyDeclaredException:
            pass
        layout_yaml_file = self._node.get_parameter(
            'tabbed_layout_yaml_file').value
        if not layout_yaml_file:
            return None
        resolved_yaml = get_filename(layout_yaml_file)
        if resolved_yaml is not None and resolved_yaml.startswith("file://"):
            resolved_yaml = resolved_yaml[len("file://"):]
        with open(resolved_yaml) as f:
            return yaml.safe_load(f)

    def showError(self, message):
        QMessageBox.about(self, "ERROR", message)

    def save_settings(self, plugin_settings, instance_settings):
        ## ignore settings
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        ## ignore settings
        pass
    def trigger_configuration(self):
        ## ignore settings
        pass

class ServiceButtonGeneralWidget_in_tab(ServiceButtonGeneralWidget):
    """
    Qt widget to visualize multiple buttons
    """
    def __init__(self, node, settings):
        super(ServiceButtonGeneralWidget_in_tab, self).__init__(node)
        yaml_file = settings['yaml_file']
        namespace = None
        if 'type' in settings:
            self.button_type = settings['type']
        else:
            self.button_type = 'push'

        if 'namespace' in settings:
            namespace = settings['namespace']

        self._layout_param = None
        self._dialog = QFileDialog()
        self._dialog.setFileMode(QFileDialog.ExistingFile)
        self._dialog.setNameFilter(
            self._translator.tr("YAML files (*.yaml *.yml)"))

        resolved_yaml = get_filename(yaml_file)
        if "file://" == resolved_yaml[0:7]:
            resolved_yaml = resolved_yaml[len("file://"):]

        with open(resolved_yaml) as f:
            yaml_data = yaml.safe_load(f)
            self.setupButtons_with_yaml_data(yaml_data=yaml_data, namespace=namespace)

        self.show()
