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

	// Shooter Feeder Motors
	public static final int TOP_ROLLER_MOTOR = 7;

	// Intake Motors
	public static final int BOTTOM_ROLLER_MOTOR = 8; 
	public static final int MECANUM_INTAKE_MOTOR = 9;

	// Climb Motors 
	public static final int LEFT_CLIMB_MOTOR = 10; 
	public static final int RIGHT_CLIMB_MOTOR = 11; 

	// Indexer Constants
	public static final double MECANUM_INTAKE_SPEED = 0.5; 
	public static final double BOTTOM_INTAKE_ROLLER_SPEED = 1.0;
	public static final double ROLL_SPEED = 0.5;
	public static final double SHOOT_ROLL_SPEED = 0.25;
	public static final double ROLL_SPEED_SENSING = 0.25;
	public static final double OUTTAKE_TIME = 0.002;
  
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
	public static final double FWD_REV_DAMPENING = 0.75; 
	public static final double LEFT_RIGHT_DAMPENING = 0.75; 
	public static final double LEFT_RIGHT_BOOST_DAMPENING = 0.5; 

	// Driver Controller
	public static final int XBOX_DRIVER = 0;
	public static final int DRIVE_FWD_REV = XboxController.Axis.kLeftY.value;
	public static final int DRIVE_LEFT_RIGHT = XboxController.Axis.kRightX.value;
	public static final int REVERSE_DRIVE_DIRECTION = XboxController.Button.kLeftStick.value;
	public static final int OVERRIDE_MOTOR_PROTECTION = XboxController.Button.kB.value;
	public static final int PRECISION_DRIVE_TOGGLE = XboxController.Button.kX.value;
	public static final int ADJUST_SHOOTER_RPM_HIGHER = XboxController.Button.kY.value; 
	public static final int ADJUST_SHOOTER_RPM_LOWER = XboxController.Button.kA.value; 
	public static final int PRECISION_DRIVE_HOLD = XboxController.Axis.kLeftTrigger.value;
	public static final int INTAKE_BUTTON = XboxController.Button.kRightBumper.value;
	public static final int OUTTAKE_BUTTON = XboxController.Button.kLeftBumper.value;
	
	// Operator Controller 
	public static final int XBOX_OPERATOR = 1;
	public static final int ADJUST_SHOOTER_HIGHER_BUTTON = XboxController.Axis.kRightTrigger.value;
	public static final int ADJUST_SHOOTER_LOWER_BUTTON = XboxController.Axis.kLeftTrigger.value; 
	public static final int SHOOT_LOW_RPM_BUTTON = XboxController.Button.kX.value;
	public static final int SHOOT_HIGH_RPM_BUTTON = XboxController.Button.kY.value;
	public static final int SHOOT_ROLL_BUTTON = XboxController.Button.kA.value;
	public static final int STOP_SHOOTER_BUTTON = XboxController.Button.kB.value;
	public static final int OVERRIDE_SHOOTER_PROTECTION_BUTTON = XboxController.Button.kBack.value; 
	public static final int STOP_SHOOTER_FEEDER_BUTTON = XboxController.Button.kStart.value; 
	public static final int TELEOP_ROLL_UP_TRIGGER = XboxController.Button.kRightBumper.value;
	public static final int TELEOP_ROLL_DOWN_TRIGGER = XboxController.Button.kLeftBumper.value; 
	public static final int CLIMB_RUNG_AXIS = XboxController.Axis.kLeftY.value;
	public static final int OVERRIDE_CLIMB_TIME_BUTTON = XboxController.Button.kLeftStick.value;
	public static final int INTAKE_ARM_ROTATE = XboxController.Axis.kRightY.value;
	public static final int INTAKE_ARM_REVERSE_BUTTON = XboxController.Button.kRightStick.value;

	// Robot Dimensions
	public static final double ROBOT_MAX_VELOCITY = 0;
	public static final double ROBOT_MAX_ACCELERATION = 0;
	public static final double ROBOT_BASE_WIDTH = 0; // inches
	public static final RobotSpecs ROBOT_SPECS = new RobotSpecs(ROBOT_MAX_VELOCITY, ROBOT_MAX_ACCELERATION,
			ROBOT_BASE_WIDTH);

	// Drive
	public static final double COLLISION_THRESHOLD = .5;

	public static final int COLOR_MOTOR_OK = 0x00FF00FF;
	public static final int COLOR_MOTOR_WARNING = 0xFFFF00FF;
	public static final int COLOR_MOTOR_SHUTOFF = 0xFF0000FF;
	public static final int COLOR_MOTOR_OVERRIDDEN = 0xA72DFFFF;

	// hub heights 
	public static final double LOWER_HUB = 48.;
	public static final double UPPER_HUB = 120.;

	public static final double BALL_MASS = 9.5;

	// Colors 
	public static final Color OUR_ALLIANCE = Color.kRed; 
	public static final Color OPPOSING_ALLIANCE = Color.kBlue; 

	public static final double LOW_HUB_RPM = 2000.0; 
	public static final double HIGH_HUB_RPM = 4000.0; 
	public static final double SHOOTER_ANGLE_ADJUSTMENT = 1000.0;

	// Direct Shooter Velocity 
	public static final double LOW_HUB_RPM_DIRECT = 0.5; 
	public static final double HIGH_HUB_RPM_DIRECT = 0.9; 

	public static final double MAX_BALLS = 2; 
	public static final double PRELOADED_BALLS = 1; 

	// Vision Targets and Settings (Inches and Degrees) 
	public static final double TARGET_HEIGHT = 104; 
	public static final double LIMELIGHT_HEIGHT = 37; 
	public static final double LIMELIGHT_ANGLE = 25; 
	public static final double TARGET_DISTANCE = 96;
	public static final double TARGET_DISTANCE_TOLERANCE = 12;
}
