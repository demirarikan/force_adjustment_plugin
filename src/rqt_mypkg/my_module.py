import os
import rospy
import rospkg
from force_adjustment import force_z_control_loop as fz
import anatomy_limits

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtWidgets
from geometry_msgs.msg import PoseStamped 
from iiwa_msgs.msg import CartesianPose
from std_msgs.msg import Bool


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print( 'arguments: ', args)
            print( 'unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ForceAdjustment')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.anatomySelection.currentIndexChanged.connect(self._handle_anatomy_selection_index_changed)

        self._widget.setStartPoseButton.clicked.connect(self._handle_set_start_pose_button_clicked)
        self._widget.setStartPoseToolButton.clicked.connect(self._handle_set_start_pose_tool_button_clicked)
        self._widget.setGoalPoseButton.clicked.connect(self._handle_set_goal_pose_button_clicked)
        self._widget.setGoalPoseToolButton.clicked.connect(self._handle_set_goal_pose_tool_button_clicked)
        self._widget.resetSensorButton.clicked.connect(self._handle_reset_sensor_button_clicked)

        self._widget.resetButton.clicked[bool].connect(self._handle_reset_button_clicked)
        self._widget.startButton.clicked[bool].connect(self._handle_start_button_clicked)

        self.robot = fz.RobotInstance()


    def _handle_anatomy_selection_index_changed(self, index):
        print("anatomy selection changed to index: ", index)
        min_str = "min. force: "
        max_str = "max. force: "
        if(index >= 1):
            min_str += str(anatomy_limits.ANATOMY_LIMITS[index][0])
            self.robot.lower_threshold = anatomy_limits.ANATOMY_LIMITS[index][0]
            max_str += str(anatomy_limits.ANATOMY_LIMITS[index][1])
            self.robot.upper_threshold = anatomy_limits.ANATOMY_LIMITS[index][1]
        self._widget.maxForceLabel.setText(max_str)
        self._widget.minForceLabel.setText(min_str)
            
    
    def _handle_set_start_pose_button_clicked(self):
        self.robot.start_pose = self.robot.current_pose


    def _handle_set_start_pose_tool_button_clicked(self):
        if not self.robot.start_pose == None:
            QtWidgets.QMessageBox.information(self._widget, "Current start pose", str(self.robot.start_pose.pose))
        else:
            QtWidgets.QMessageBox.critical(self._widget, "Starting pose not set", "Please set a valid starting pose")


    def _handle_set_goal_pose_button_clicked(self):
        self.robot.goal_pose = self.robot.current_pose


    def _handle_set_goal_pose_tool_button_clicked(self):
        if not self.robot.goal_pose == None:
            QtWidgets.QMessageBox.information(self._widget, "Current goal pose", str(self.robot.goal_pose.pose))
        else:
            QtWidgets.QMessageBox.critical(self._widget, "Goal pose not set", "Please set a valid goal pose")


    def _handle_reset_sensor_button_clicked(self):
        print("Clicked reset sensor button")
        force_sensor_reset_pub = rospy.Publisher("force_sensor_reset", Bool, queue_size=2)
        force_sensor_reset_pub.publish(True)


    def _handle_start_button_clicked(self):
        print("clicked start button")   
        # Show error modal if all fields do not have valid values
        if self._widget.anatomySelection.currentIndex() == 0 or\
           self._widget.desiredForceSpinBox.value() <= 0 or\
           self.robot.start_pose == None or\
           self.robot.goal_pose == None:
            QtWidgets.QMessageBox.critical(self._widget, "Error", "Please set a valid value for all fields ")
        elif self._widget.desiredForceSpinBox.value() < anatomy_limits.ANATOMY_LIMITS[self._widget.anatomySelection.currentIndex()][0] or\
            self._widget.desiredForceSpinBox.value() > anatomy_limits.ANATOMY_LIMITS[self._widget.anatomySelection.currentIndex()][1]:
                QtWidgets.QMessageBox.critical(self._widget, "Error", "Desired force has to be between min and max forces")
        else: 
            self.robot.desired_force = self._widget.desiredForceSpinBox.value()
            self.robot.lower_deviation, self.robot.upper_deviation = calculate_force_limits(self._widget.deviationSpinBox.value(), self.robot.desired_force,\
                 self.robot.lower_threshold, self.robot.upper_threshold)
            #TODO: Send robot to starting position and wait until its reached
            fz.start_control_loop(self.robot)


    def _handle_reset_button_clicked(self):
        print("clicked reset button")
        self._widget.anatomySelection.setCurrentIndex(0)
        self._widget.desiredForceSpinBox.setValue(0)
        self._widget.deviationSpinBox.setValue(0)
        self.robot = fz.RobotInstance()


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

def calculate_force_limits(deviation, desired_force, min_force, max_force):
    min_value = abs(desired_force - min_force) * deviation/100
    max_value = abs(desired_force - max_force) * deviation/100
    return [desired_force-min_value, desired_force+max_value]
