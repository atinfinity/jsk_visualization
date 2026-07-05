from glob import glob

from setuptools import setup

package_name = 'jsk_rqt_plugins'

setup(
    name=package_name,
    version='3.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        ('share/' + package_name + '/resource', [
            'resource/plot3d.ui',
            'resource/plot_histogram.ui',
            'resource/yes_no_button.ui',
            'resource/rqt_image_view2_button.perspective',
            'resource/rqt_service_buttons.perspective',
            'resource/rqt_service_radio_buttons.perspective',
            'resource/service_button_layout.yaml',
            'resource/service_radio_button_layout.yaml',
        ]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/sample',
         glob('sample/*.launch.py') + ['sample/kiva_pod_image_color.jpg']),
        ('lib/' + package_name, glob('sample_scripts/*.py') + [
            'bin/rqt_2d_plot',
            'bin/rqt_3d_plot',
            'bin/rqt_drc_mini_maxwell',
            'bin/rqt_histogram_plot',
            'bin/rqt_image_view2',
            'bin/rqt_service_buttons',
            'bin/rqt_status_light',
            'bin/rqt_string_label',
            'bin/rqt_tabbed_buttons',
            'bin/rqt_yn_btn',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Yuto Inagaki',
    maintainer='Kei Okada',
    maintainer_email='k-okada@jsk.t.u-tokyo.ac.jp',
    description='rqt plugins for jsk (ROS 2)',
    license='BSD',
    tests_require=['pytest'],
)
