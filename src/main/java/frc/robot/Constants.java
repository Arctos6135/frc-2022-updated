// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.arctos6135.robotpathfinder.core.RobotSpecs;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // SPARK MAX Motors 
    // TODO: Placeholder values as of now 
    public static final int RIGHT_CANSPARKMAX = 1;
    public static final int LEFT_CANSPARKMAX = 2;
    public static final int RIGHT_CANSPARKMAX_FOLLOWER = 3;
    public static final int LEFT_CANSPARKMAX_FOLLOWER = 4;

    // Shooter Motors 
    // TODO: Write constants for shooter motors 

    // SPARK MAX Encoders 
    // TODO: change gearbox ratio 
    public static final double WHEEL_DIAMETER = 4.0; // inches 
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // inches 
    public static final double GEARBOX_RATIO = 0;
    public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO; // inches 
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO / 60; // inches / second
    public static final int COUNTS_PER_REVOLUTION = 42;
    
    // Warning and Shutoff Temperatures 
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
    public static final int PRECISION_DRIVE_HOLD = XboxController.Axis.kLeftTrigger.value; 
    
    // Operator Controller 
    public static final int XBOX_OPERATOR = 1;

    // Robot Dimensions 
    // TODO: change these to match robot 
    public static final double ROBOT_MAX_VELOCITY = 0;
    public static final double ROBOT_MAX_ACCELERATION = 0;
    public static final double ROBOT_BASE_WIDTH = 0; // inches 
    public static final RobotSpecs ROBOT_SPECS = new RobotSpecs(ROBOT_MAX_VELOCITY, ROBOT_MAX_ACCELERATION,
            ROBOT_BASE_WIDTH); 
}
