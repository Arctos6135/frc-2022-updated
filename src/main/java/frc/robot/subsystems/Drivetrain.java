// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MonitoredCANSparkMaxGroup;

// TODO: do we want to set speed using voltage directly? 
// If we set the motors using direct voltage, we can use 
// TankDrive and Odometry, which is useful for autonomous 
// commands. 
// Use the following equation: 
// V = kS sgn(d) + kV d1 + kA d2 
// V -> applied voltage 
// kS -> voltage needed to overcome static friction 
// d -> displacement of motor 
// kV -> voltage needed to hold constant velocity 
// d1 -> velocity 
// kA -> voltage needed to accelerate 
// d2 -> acceleration 
public class Drivetrain extends SubsystemBase {

  // Motor Controllers 
  private final CANSparkMax rightMotor;
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightFollowerMotor;
  private final CANSparkMax leftFollowerMotor;

  private final MotorControllerGroup m_leftMotors; 
  private final MotorControllerGroup m_rightMotors; 

  private final DifferentialDrive m_differentialDrive; 
  private final DifferentialDriveOdometry m_differentialOdometry; 

  // Encoders 
  private final RelativeEncoder rightEncoder;
  private final RelativeEncoder leftEncoder;
  
  // Motor Controller Monitors 
  private final MonitoredCANSparkMaxGroup motorMonitorGroup;
  
  // Whether to protect against overheating. 
  private boolean protectionOverridden = false;
  
  // Acceleration of Motors
  private double leftMotorLastRate, rightMotorLastRate = 0;
  private double lastTime = 0; // TODO: does this have to be initialized?  

  // Robot Navigation System 
  private final AHRS ahrs; 

  // Break Mode: motors are brought to a quick stop (motor wires are shorted together). 
  // Coast Mode: motors can spin at their own rates (motor wires disconnected).
  private IdleMode idleMode = IdleMode.kCoast;

  // Adjusts speeds.
  private double speedMultiplier = 1.0;
  
  /**
   * Set the speed multiplier. 
   * 
   * <p>
   * Speeds passed to the motors are multiplied by this.
   * </p> 
   * 
   * @param speedMultiplier the speed multiplier.
   */
  public void setSpeedMultiplier(double speedMultiplier) {
    this.speedMultiplier = speedMultiplier;
  }
  
  /**
   * Get the speed multiplier. 
   * 
   * @return the speed multiplier.
   */
  public double getSpeedMultiplier() {
    return this.speedMultiplier;
  }

  /**
   * Set percentage output of the right motor.
   * 
   * @param motorOutput the motor output.
   */
  public void setRightMotor(double motorOutput) {
    rightMotor.set(motorMonitorGroup.getOverheatShutoff() && !protectionOverridden ? 0 : motorOutput * speedMultiplier);
  }
  
  /**
   * Set percentage output of the left motor. 
   * 
   * @param motorOutput the motor output.
   */
  public void setLeftMotor(double motorOutput) {
    leftMotor.set(motorMonitorGroup.getOverheatShutoff() && !protectionOverridden ? 0 : motorOutput * speedMultiplier);
  }
  
  /**
   * Set the left and right side motors of the drivetrain.
   * 
   * Inputs to the motors are multiplied by the speed multiplier. 
   * Motor outputs are constrained to [-1, 1]. 
   * 
   * @param leftOutput (inverted) percent output of left side motors.
   * @param rightOutput percent output of right side motors. 
   */
  public void setMotors(double leftOutput, double rightOutput) {
    setLeftMotor(leftOutput);
    setRightMotor(rightOutput);
  }

  /**
   * Set the ramping rate of the motors. 
   * The ramping rate is the number of seconds it takes to go from 0 to maximum speed. 
   * 
   * @param rampRate number of seconds to go from 0 to full throttle. 
   */
  public void setRamping(double rampRate) {
    leftMotor.setOpenLoopRampRate(rampRate);
    rightMotor.setOpenLoopRampRate(rampRate);
  }

  /**
   * Set the overheat shutoff. 
   * 
   * When overridden, motors are still active after shutoff limits. 
   * Callbacks will be called and the motors will be in a "shutoff" or "warning" state. 
   * 
   * @param override
   */
  public void setOverheatShutoffOverride(boolean override) {
    this.protectionOverridden = override;
  }
  
  /**
   * Return the overheat shutoff override boolean.
   * 
   * When overridden, motors are still active after shutoff limits.
   * Callbacks will be called and the motors will be in a "shutoff" or "warning"
   * state.
   * 
   * @return whether the overheat shutoff is overridden.
   */
  public boolean getOverheatShutoffOverride() {
    return this.protectionOverridden;
  }
  
  // Drivetrain Encoder Methods
  /**
   * Reset the left and right encoders. 
   */
  public void resetEncoders() {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }
  
  /**
   * Get the distance that the right encoder has travelled.
   * Position conversion factors are already considered.
   * 
   * @return distance travelled by right encoder (inches).
   */
  public double getRightDistance() {
    return rightEncoder.getPosition();
  }

  /**
   * Get the distance that the left encoder has travelled. 
   * Position conversion factors are already considered. 
   * 
   * @return distance travelled by left encoder (inches). 
   */
  public double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  /**
   * Get velocity of the right encoder. 
   * 
   * @return velocity of the right encoder (inches / second). 
   */
  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }
  
  /**
   * Get velocity of the left encoder. 
   * 
   * @return velocity of the left encoder (inches / second). 
   */
  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }
  
  public double[] getAccelerations() {
    double dt = Timer.getFPGATimestamp() - lastTime;
    double rightRate = getRightVelocity();
    double leftRate = getLeftVelocity();

    double rightAccel = (rightRate - rightMotorLastRate) / dt;
    double leftAccel = (leftRate - leftMotorLastRate) / dt;

    rightMotorLastRate = rightRate;
    leftMotorLastRate = leftRate;

    lastTime = Timer.getFPGATimestamp();

    return new double[] { leftAccel, rightAccel };
  }
  
  /**
   * Get the Idle Mode of the Drivetrain.
   * 
   * @return the idle mode. 
   */
  public IdleMode getIdleMode() {
    return this.idleMode;
  }
  
  /**
   * Set the Idle Mode (brake/coast) of the drivetrain motors. 
   * @param mode
   */
  public void setMotorMode(IdleMode mode) {
    idleMode = mode;
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode); 
  }
  
  // Robot Navigation 

  /**
   * Return the heading of the robot.
   * 
   * @return the heading of the robot in degrees.
   */
  public double getHeading() {
    return ahrs.getFusedHeading();
  }
  
  /**
   * Reset the heading of the robot.
   */
  public void zeroHeading() {
    ahrs.reset(); 
  }

  // Autonomous Mode

  /**
   * Set the motor controllers using direct voltage. 
   * 
   * @param leftVolts the voltage passed to the left motors.
   * @param rightVolts the voltage passed to the right motors. 
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts); 
    m_rightMotors.setVoltage(rightVolts);
    m_differentialDrive.feed(); 
  }

  public void setMaxOutput(double maxOutput) {
    m_differentialDrive.setMaxOutput(maxOutput);
  }

  /**
   * Get the estimated pose of the robot in its current state. 
   * 
   * @return the pose as a Pose2d object. 
   */
  public Pose2d getPose() {
    return m_differentialOdometry.getPoseMeters(); 
  }

  /**
   * Get the robot's Atttude and Heading Reference System.
   * 
   * @return the robot's AHRS.
   */
  public AHRS getAHRS() {
    return this.ahrs;
  }

  /**
   * Get the current wheel speeds of the robot (in m/s). 
   * 
   * @return the current wheel speeds. 
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity()); 
  }

  /**
   * Resets the odometry to the specified pose. 
   * 
   * @param pose the pose to which to set the odometry. 
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_differentialOdometry.resetPosition(pose, ahrs.getRotation2d());
  }

  /**
   * Drives the robot with forwards/backwards translation and rotation.
   * 
   * Speed scaling factor will be applied before setting the motor outputs.
   * A position rotation value will turn the robot clockwise (leftwards), as
   * the left output will be larger than the right output.
   * 
   * A negative rotation value will turn the robot counterclockwise (rightwards),
   * as the right output will be larger than the left output.
   * 
   * @param translation the translation, constrained in the interval [-1.0, 1.0].
   * @param rotation the rotation, constrained in the interval [-1.0, 1.0].
   * @param scalingFactor scales the motor output.
   */
  public void arcadeDrive(double translation, double rotation, double scalingFactor) {
    double left = (translation + rotation) * scalingFactor;
    double right = (translation - rotation) * scalingFactor;
    
    setMotors(left, right); 
  }

  /**
   * Drives robot with forwards/backwards translation + rotation.
   * 
   * The overloaded arcadeDrive performs turning.
   * 
   * @param translation the translation, constrained in the interval [-1.0, 1.0]
   * @param rotation the rotation, constrained in the interval [-1.0, 1.0]
   */
  public void arcadeDrive(double translation, double rotation) {
    arcadeDrive(translation, rotation, 1.0);
  }

  /**
   * Writes all settings of SPARK MAX motor controllers to flash. 
   * 
   * This allows SPARK MAX controllers to remember their configuration throughout a power cycle. 
   */
  public void burnFlash() {
    rightMotor.burnFlash();
    leftMotor.burnFlash();
    rightFollowerMotor.burnFlash();
    leftFollowerMotor.burnFlash(); 
  }

  /**
   * Get the monitor group for the drivetrain. 
   * 
   * @return the drivetrain's motor monitor group. 
   */
  public MonitoredCANSparkMaxGroup getMonitorGroup() {
    return this.motorMonitorGroup; 
  }

  /**
   * Creates a new drivetrain.
   * 
   * @param rightMaster   the corresponding PDP port for the right motor
   *                      controller.
   * @param leftMaster    the corresponding PDP port for the left motor
   *                      controller.
   * @param rightFollower the corresponding PDP port for the right follower motor
   *                      controller.
   * @param leftFollower  the corresponding PDP port for the left follower motor
   *                      controller.
   */
  public Drivetrain(int rightMaster, int leftMaster, int rightFollower, int leftFollower) {
    // Robot Navigation 
    ahrs = new AHRS(I2C.Port.kOnboard); 

    // Motor Instantiation
    rightMotor = new CANSparkMax(rightMaster, MotorType.kBrushless);
    leftMotor = new CANSparkMax(leftMaster, MotorType.kBrushless);
    rightFollowerMotor = new CANSparkMax(rightFollower, MotorType.kBrushless);
    leftFollowerMotor = new CANSparkMax(leftFollower, MotorType.kBrushless);

    // Speed Group Initialization 
    m_rightMotors = new MotorControllerGroup(rightMotor, rightFollowerMotor); 
    m_leftMotors = new MotorControllerGroup(leftMotor, leftFollowerMotor); 

    // Differential Drive and Odometry
    m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    m_differentialOdometry = new DifferentialDriveOdometry(ahrs.getRotation2d()); 
    
    // Encoder Instantiation 
    rightEncoder = rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.COUNTS_PER_REVOLUTION);
    leftEncoder = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.COUNTS_PER_REVOLUTION);

    // Motor Monitor Group 
    motorMonitorGroup = new MonitoredCANSparkMaxGroup("Drivetrain", Constants.MOTOR_WARNING_TEMP, Constants.MOTOR_SHUTOFF_TEMP, 
      rightMotor, leftMotor, rightFollowerMotor, leftFollowerMotor); 

    rightFollowerMotor.follow(rightMotor);
    leftFollowerMotor.follow(leftMotor);
    
    rightMotor.stopMotor();
    leftMotor.stopMotor();
    
    // Invert master motors to drive in the correct direction. 
    rightMotor.setInverted(false);
    leftMotor.setInverted(true);
    
    rightEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR_METERS);
    leftEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR_METERS);
    rightEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR_METERS);
    leftEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR_METERS);
  }

  @Override
  public void periodic() {
    motorMonitorGroup.monitorOnce();
    
    if (motorMonitorGroup.getOverheatShutoff()) {
      setMotors(0, 0); 
    }

    m_differentialOdometry.update(
      ahrs.getRotation2d(), 
      Units.inchesToMeters(leftEncoder.getPosition() * Constants.WHEEL_CIRCUMFERENCE), 
      Units.inchesToMeters(rightEncoder.getPosition() * Constants.WHEEL_CIRCUMFERENCE));
  }

  @Override 
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("DifferentialDrive");
    builder.setActuator(true);
    builder.setSafeState(() -> {
      setMotors(0, 0); 
    });
    
    // Adds getter and setter properties for the left and right motor speeds.
    builder.addDoubleProperty("Left Motor Speed", leftMotor::get, this::setLeftMotor);
    builder.addDoubleProperty("Right Motor Speed", rightMotor::get, this::setRightMotor); 
  }
}