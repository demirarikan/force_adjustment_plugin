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
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.resetButton.clicked[bool].connect(self._handle_reset_button_clicked)
        self._widget.startButton.clicked[bool].connect(self._handle_start_button_clicked)
        self._widget.anatomySelection.currentIndexChanged.connect(self._handle_anatomy_selection_index_changed)

        self._widget.setStartPoseButton.clicked.connect(self._handle_set_start_pose_button_clicked)
        self._widget.setStartPoseToolButton.clicked.connect(self._handle_set_start_pose_tool_button_clicked)
        self._widget.setGoalPoseButton.clicked.connect(self._handle_set_goal_pose_button_clicked)
        self._widget.setGoalPoseToolButton.clicked.connect(self._handle_set_goal_pose_tool_button_clicked)
        self._widget.resetSensorButton.clicked.connect(self._handle_reset_sensor_button_clicked)

    def _handle_reset_button_clicked(self):
        print("clicked reset button")
        self._widget.anatomySelection.setCurrentIndex(0)
        self._widget.desiredForceSpinBox.setValue(0)
        #TODO: Reset start and goal poses


    def _handle_anatomy_selection_index_changed(self, index):
        print("anatomy selection changed to index: ", index)
        #TODO: Change strings to variables. Set label test at the end of if statement. Inside the if statements just add the numbers to the final string
        if(index == 0):
            self._widget.maxForceLabel.setText("max. force:")
            self._widget.minForceLabel.setText("min. force:")
            return
        elif(index == 1):
            self._widget.maxForceLabel.setText("max. force: t")
            self._widget.minForceLabel.setText("min. force: t")
            return
        elif(index == 2):
            self._widget.maxForceLabel.setText("max. force: a")
            self._widget.minForceLabel.setText("min. force: a")
            return
        elif(index == 3):
            self._widget.maxForceLabel.setText("max. force: arm")
            self._widget.minForceLabel.setText("min. force: arm")
            return
        elif(index == 4):
            self._widget.maxForceLabel.setText("max. force: l")
            self._widget.minForceLabel.setText("min. force: l")
            return
        elif(index == 5):
            self._widget.maxForceLabel.setText("max. force: s")
            self._widget.minForceLabel.setText("min. force: s")
            return
        else:
            self._widget.maxForceLabel.setText("max. force: ?")
            self._widget.minForceLabel.setText("min. force: ?")
            return
    
    def _handle_set_start_pose_button_clicked():
        # TODO: Set start pose to current pose of robot
        pass

    def _handle_set_start_pose_tool_button_clicked():
        #TODO: Open modal and show the start pose, no pose is set inform the user about this
        pass

    def _handle_set_goal_pose_button_clicked():
        #TODO: set goal pose to currect pose of robot
        pass

    def _handle_set_goal_pose_tool_button_clicked():
        #TODO: open modal and show goal pose
        pass

    def _handle_reset_sensor_button_clicked():
        #TODO: Close sensor node and restart it
        pass

    def _handle_start_button_clicked(self):
        print( self._widget.coordinateButtons.checkedId())
        print("clicked start button")   
        # Show error modal if all fields do not have valid values
        if self._widget.anatomySelection.currentIndex() == 0 or\
           self._widget.desiredForceSpinBox.value() <= 0 or\
           self._widget.goalDistance.value() == 0 or\
           self._widget.coordinateButtons.checkedId() == -1:
            QtWidgets.QMessageBox.critical(self._widget, "Error", "Please set a valid value for all fields ")
        else: 
            robot = fz.RobotInstance()
            robot.current_pose = rospy.wait_for_message("/iiwa/state/CartesianPose", CartesianPose, timeout=5)
            robot.goal_pose = fz.create_goal_pose_message(robot, self._widget.coordinateButtons.checkedId(), self._widget.goalDistance.value())
            robot.lower_threshold = anatomy_limits.ANATOMY_LIMITS[self._widget.anatomySelection.currentIndex()][0]
            robot.upper_threshold = anatomy_limits.ANATOMY_LIMITS[self._widget.anatomySelection.currentIndex()][1]
            robot.desired_force = self._widget.desiredForceSpinBox.value()
            # TODO:force z control loop, x yonu icin ayarlanmis. en basta baslangic pozisyonunu kaydedip, her degisimden sonra start pozisyonuna kiyasla goal poz ayarlayip yeni koordinat gondermen lazim.
            print(robot.goal_pose)
            fz.start_control_loop(robot)


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
