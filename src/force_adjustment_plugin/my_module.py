import os
from time import sleep
import rospy
import rospkg
from force_adjustment import force_z_control_loop as fz
import anatomy_limits

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog
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
        ui_file = os.path.join(rospkg.RosPack().get_path('force_adjustment_plugin'), 'resource', 'MyPlugin.ui')
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

        # Handle interactions with the widgets
        self._widget.anatomySelection.currentIndexChanged.connect(self._handle_anatomy_selection_index_changed)
        # Hide the min max force selectors until user selects the "custom" option in anatomy selection
        self._widget.minForceSpinBox.setHidden(True)
        self._widget.maxForceSpinBox.setHidden(True)
        self._widget.minForceSpinBox.valueChanged.connect(self._handle_min_force_spin_box_value_changed)
        self._widget.maxForceSpinBox.valueChanged.connect(self._handle_max_force_spin_box_value_changed)

        self._widget.setStartPoseButton.clicked.connect(self._handle_set_start_pose_button_clicked)
        self._widget.setStartPoseToolButton.clicked.connect(self._handle_set_start_pose_tool_button_clicked)
        self._widget.setGoalPoseButton.clicked.connect(self._handle_set_goal_pose_button_clicked)
        self._widget.setGoalPoseToolButton.clicked.connect(self._handle_set_goal_pose_tool_button_clicked)

        self._widget.resetSensorButton.clicked.connect(self._handle_reset_sensor_button_clicked)

        self._widget.desiredForceSpinBox.valueChanged.connect(self._handle_desired_force_spinbox_value_changed)

        self._widget.deviationSpinBox.valueChanged.connect(self._handle_deviation_spin_box_value_changed)

        self._widget.customStepSizeCheckbox.stateChanged.connect(self._handle_custom_step_size_checkbox_state_changed)
        self._widget.customStepSizeSpinBox.valueChanged.connect(self._handle_custom_step_size_spin_box_value_changed)
        self._widget.customStepSizeSpinBox.setDisabled(True)

        self._widget.helpButton.clicked.connect(self._handle_help_button_clicked)
        self._widget.resetButton.clicked.connect(self._handle_reset_button_clicked)
        self._widget.startButton.clicked.connect(self._handle_start_button_clicked)

        self.robot = fz.RobotInstance()


    def _handle_anatomy_selection_index_changed(self, index):
        print("anatomy selection changed to index: ", index)
        min_str = "min. force: "
        max_str = "max. force: "
        if(str(self._widget.anatomySelection.itemText(index)) == "Custom"):
            print("Set custom force values")
            self._widget.minForceSpinBox.setVisible(True)
            self._widget.maxForceSpinBox.setVisible(True)
        elif(index >= 1):
            self._widget.minForceSpinBox.setHidden(True)
            self._widget.maxForceSpinBox.setHidden(True)
            min_str += str(anatomy_limits.ANATOMY_LIMITS[index][0])
            self.robot.lower_threshold = anatomy_limits.ANATOMY_LIMITS[index][0]
            max_str += str(anatomy_limits.ANATOMY_LIMITS[index][1])
            self.robot.upper_threshold = anatomy_limits.ANATOMY_LIMITS[index][1]
        self._widget.maxForceLabel.setText(max_str)
        self._widget.minForceLabel.setText(min_str)


    def _handle_min_force_spin_box_value_changed(self, value): 
        self.robot.lower_threshold = value


    def _handle_max_force_spin_box_value_changed(self, value): 
        self.robot.upper_threshold = value


    def _handle_set_start_pose_button_clicked(self):
        print("Starting pose set.")
        self.robot.start_pose = self.robot.current_pose


    def _handle_set_start_pose_tool_button_clicked(self):
        print("Showing starting pose.")
        if not self.robot.start_pose == None:
            QtWidgets.QMessageBox.information(self._widget, "Current start pose", str(self.robot.start_pose.pose))
        else:
            QtWidgets.QMessageBox.critical(self._widget, "Starting pose not set", "Please set a valid starting pose")


    def _handle_set_goal_pose_button_clicked(self):
        print("Goal pose set.")
        self.robot.goal_pose = self.robot.current_pose


    def _handle_set_goal_pose_tool_button_clicked(self):
        print("Showing goal pose.")
        if not self.robot.goal_pose == None:
            QtWidgets.QMessageBox.information(self._widget, "Current goal pose", str(self.robot.goal_pose.pose))
        else:
            QtWidgets.QMessageBox.critical(self._widget, "Goal pose not set", "Please set a valid goal pose")


    def _handle_reset_sensor_button_clicked(self):
        print("Resetting sensor")
        force_sensor_reset_pub = rospy.Publisher("force_sensor_reset", Bool, queue_size=2)
        force_sensor_reset_pub.publish(True)


    def _handle_desired_force_spinbox_value_changed(self, value):
        self.robot.desired_force = value


    def _handle_deviation_spin_box_value_changed(self, value):
        (min, max) = calculate_force_limits(self._widget.deviationSpinBox.value(), self.robot.desired_force,\
                 self.robot.lower_threshold, self.robot.upper_threshold)
        self._widget.deviationForces.setText(str(min) + "-" + str(max)+"N")


    def _handle_custom_step_size_checkbox_state_changed(self, value):
        if value == 0:
            self._widget.customStepSizeSpinBox.setDisabled(True)
        else:
            self._widget.customStepSizeSpinBox.setEnabled(True)


    def _handle_custom_step_size_spin_box_value_changed(self, value):
        self.robot.step_size = value


    def _handle_start_button_clicked(self):
        print("clicked start button")   
        # Show error modal if all fields do not have valid values
        if self._widget.anatomySelection.currentIndex() == 0 or\
           self._widget.desiredForceSpinBox.value() <= 0 or\
           self.robot.start_pose == None or self.robot.goal_pose == None or\
           self.robot.lower_threshold > self.robot.upper_threshold or\
           self.robot.lower_threshold == 0 or self.robot.upper_threshold == 0:
            QtWidgets.QMessageBox.critical(self._widget, "Error", "Please set valid values for all fields ")

        elif self._widget.desiredForceSpinBox.value() < anatomy_limits.ANATOMY_LIMITS[self._widget.anatomySelection.currentIndex()][0] or\
            self._widget.desiredForceSpinBox.value() > anatomy_limits.ANATOMY_LIMITS[self._widget.anatomySelection.currentIndex()][1]:
                QtWidgets.QMessageBox.critical(self._widget, "Error", "Desired force has to be between min and max forces")
        else: 
            self.robot.lower_deviation, self.robot.upper_deviation = calculate_force_limits(self._widget.deviationSpinBox.value(), self.robot.desired_force,\
                 self.robot.lower_threshold, self.robot.upper_threshold)

            print("Current settings:\n", str(self.robot.upper_threshold) + "\n",
                str(self.robot.lower_threshold) + "\n",
                str(self.robot.upper_deviation) + "\n",
                str(self.robot.lower_deviation) + "\n",
                str(self.robot.desired_force) + "\n")
            self.robot.new_pose_publisher.publish(self.robot.start_pose)
            while not self.robot.pose_reached(self.robot.start_pose):
                print("Going to starting pose",)
                sleep(0.5)
            print("Reached starting pose")
            self.robot.start_control_loop()
            print("Reached goal pose")


    def _handle_reset_button_clicked(self):
        print("Reset")
        self._widget.minForceSpinBox.setValue(0)
        self._widget.maxForceSpinBox.setValue(0)
        self._widget.minForceSpinBox.setHidden(True)
        self._widget.maxForceSpinBox.setHidden(True)
        self._widget.anatomySelection.setCurrentIndex(0)
        self._widget.desiredForceSpinBox.setValue(0)
        self._widget.deviationSpinBox.setValue(0)
        self._widget.customStepSizeSpinBox.setDisabled(True)
        self.robot = fz.RobotInstance()
        self._widget.customStepSizeSpinBox.setValue(self.robot.step_size)


    def _handle_help_button_clicked(self):
        QtWidgets.QMessageBox.information(self._widget, "Help", 
        "         Anatomy selection: Select the anatomy you want to scan from the menu. The min. and max. forces will be adjusted accordingly. Select the custom option if you want define your own min. max. forces.\n\n \
        Setting the start and goal poses: Move the robot to the desired location and press the respective buttons to set the poses. Poses can be viewed with the info button.\n\n\
        Resetting the sensor: To be used after attaching or removing from the sensor. Resets the values to 0 similar to the tare function on a scale\n\n\
        Desired force: The force that will be applied during the scan. Has to be between min. and max. forces\n\n\
        Deviation from desired force: Sets the force limits for the scan. 100 allows for all values between the desired force and min./max. forces.\n\n\
        Custom step size: Distance that the robot will move in order to increase or decrease the force at each correction. \n\n\
        Reset: Resets all fields.\n\n\
        Start: Moves the robot to the starting position and starts the scan.")


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
