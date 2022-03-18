package frc.robot.constants;

import com.arctos6135.robotpathfinder.core.RobotSpecs;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
	// Drivetrain Motors 
	public static final int RIGHT_CANSPARKMAX = 1;
	public static final int LEFT_CANSPARKMAX = 2;
	public static final int RIGHT_CANSPARKMAX_FOLLOWER = 3;
	public static final int LEFT_CANSPARKMAX_FOLLOWER = 4;

	// Shooter Motors 
	public static final int MAIN_SHOOTER_MOTOR = 5;
	public static final int AUXILLIARY_SHOOTER_MOTOR = 6;

	// Intake Motors
	public static final int MECANUM_INTAKE_MOTOR = 7;
	public static final int INTAKE_ARM_MOTOR = 8;

	// Shooter Feeder Motors
	public static final int TOP_ROLLER_MOTOR = 7;
	public static final int BOTTOM_ROLLER_MOTOR = 10; 

	// Climb Motors 
	public static final int HOOK_DEPLOYMENT_MOTOR = 11; 
	public static final int LEFT_CLIMB_MOTOR = 12; 
	public static final int RIGHT_CLIMB_MOTOR = 13; 

	// Indexer Constants
	public static final double ROLL_SPEED = 0.5;
	public static final double ROLL_SPEED_SENSING = 0.25;
  
	// Climb Related Constants
	public static final double START_CLIMB_TIME = 30.0; // seconds
	public static final double CLIMB_DRIVE_TIME = 0.1; 
	public static final double RAISE_HALFWAY = 4.1; 
	public static final double CLIMB_SPEED = 0.5; 

	public static final double INTAKE_ARM_LOWERED = 1.2; 
	public static final double INTAKE_ARM_RAISED = 0; 

	// SPARK MAX Encoders (in inches)
	public static final double WHEEL_DIAMETER = 4.0;
	public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	public static final double GEARBOX_RATIO = 1/6.1;
	public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
	public static final double POSITION_CONVERSION_FACTOR_METERS = Units.inchesToMeters(POSITION_CONVERSION_FACTOR); 
	public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO / 60;
	public static final double VELOCITY_CONVERSION_FACTOR_METERS = Units.inchesToMeters(VELOCITY_CONVERSION_FACTOR); 
	public static final int COUNTS_PER_REVOLUTION = 42;
	
	// Warning and Shutoff Temperatures (in centigrade, for inconsistency)
	public static double MOTOR_WARNING_TEMP = 70;
	public static double MOTOR_SHUTOFF_TEMP = 90;
	
	// Xbox Controller
	public static final double CONTROLLER_DEADZONE = 0.15;

	// Driver Controller
	public static final int XBOX_DRIVER = 0;
	public static final int DRIVE_FWD_REV = XboxController.Axis.kLeftY.value;
	public static final int DRIVE_LEFT_RIGHT = XboxController.Axis.kRightX.value;
	public static final int REVERSE_DRIVE_DIRECTION = XboxController.Button.kLeftStick.value;
	public static final int OVERRIDE_MOTOR_PROTECTION = XboxController.Button.kB.value;
	public static final int PRECISION_DRIVE_TOGGLE = XboxController.Button.kX.value;
	public static final int DEPLOY_CLIMB_HOOK = XboxController.Button.kY.value; 
	public static final int RETRACT_CLIMB_HOOK = XboxController.Button.kA.value; 
	public static final int PRECISION_DRIVE_HOLD = XboxController.Axis.kLeftTrigger.value;
	public static final int INTAKE_FORWARD_BUTTON = XboxController.Button.kLeftBumper.value;
	public static final int INTAKE_REVERSE_BUTTON = XboxController.Button.kRightBumper.value;
	
	// Operator Controller 
	public static final int XBOX_OPERATOR = 1;
	public static final int DEPLOY_SHOOTER_LOWER_BUTTON = XboxController.Axis.kRightTrigger.value;
	public static final int DEPLOY_SHOOTER_UPPER_BUTTON = XboxController.Axis.kLeftTrigger.value; 
	public static final int SHOOT_LOW_RPM_BUTTON = XboxController.Button.kX.value;
	public static final int SHOOT_HIGH_RPM_BUTTON = XboxController.Button.kY.value;
	public static final int SENSORED_ROLL = XboxController.Button.kA.value;
	public static final int STOP_SHOOTER_BUTTON = XboxController.Button.kB.value;
	public static final int OVERRIDE_SHOOTER_PROTECTION_BUTTON = XboxController.Button.kStart.value; 
	public static final int CLIMB_TIME_OVERRIDE_BUTTON = XboxController.Button.kBack.value; 
	public static final int CLIMB_RUNG_AXIS = XboxController.Axis.kLeftY.value;
	public static final int TOGGLE_CLIMB_PRECISION = XboxController.Button.kLeftStick.value;
	public static final int INTAKE_ARM_ROTATE = XboxController.Axis.kRightY.value;
	public static final int INTAKE_ARM_REVERSE_BUTTON = XboxController.Button.kRightStick.value; 
	public static final int TELEOP_ROLL_UP_TRIGGER = XboxController.Button.kRightBumper.value;
	public static final int TELEOP_ROLL_DOWN_TRIGGER = XboxController.Button.kLeftBumper.value; 
	
	// Robot Dimensions
	// TODO: change these to match robot
	public static final double ROBOT_MAX_VELOCITY = 0;
	public static final double ROBOT_MAX_ACCELERATION = 0;
	public static final double ROBOT_BASE_WIDTH = 0; // inches
	public static final RobotSpecs ROBOT_SPECS = new RobotSpecs(ROBOT_MAX_VELOCITY, ROBOT_MAX_ACCELERATION,
			ROBOT_BASE_WIDTH);

	// Drive
	public static final double COLLISION_THRESHOLD = .5; // TODO: tune the collision threshold

	public static final int COLOR_MOTOR_OK = 0x00FF00FF;
	public static final int COLOR_MOTOR_WARNING = 0xFFFF00FF;
	public static final int COLOR_MOTOR_SHUTOFF = 0xFF0000FF;
	public static final int COLOR_MOTOR_OVERRIDDEN = 0xA72DFFFF;

	// hub heights 
	public static final double LOWER_HUB = 48.;
	public static final double UPPER_HUB = 120.;

	public static final double BALL_MASS = 9.5;

	// Colors 
	// TODO: change if necessary
	public static final Color OUR_ALLIANCE = Color.kBlue; 
	public static final Color OPPOSING_ALLIANCE = Color.kRed; 

	// TODO: test values for shooter RPM
	public static final double LOW_HUB_RPM = 1000.0; 
	public static final double HIGH_HUB_RPM = 1.0; 

	public static final double MAX_BALLS = 2; 
	public static final double PRELOADED_BALLS = 1; 
}
