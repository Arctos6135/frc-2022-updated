package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MonitoredCANSparkMaxGroup;

public class Shooter extends SubsystemBase {

	private final CANSparkMax masterShooterMotor;
	private final CANSparkMax followerShooterMotor;
	private final RelativeEncoder shooterEncoder;
	private final SparkMaxPIDController pidController;

	public static final double BASE_SPEED = 0;
	public double shooterDist;
	private double velocity = 0;

	private final MonitoredCANSparkMaxGroup monitorGroup;
	
	boolean protectionOverridden = false;

	static final double kP = 0, kI = 0, kD = 0, kF = 0;

	/**
	 * Creates new instance of the shooter subsystem. 

	 * @param masterMotor the PDP pin of the motor for the top shooter wheel.
	 * @param followerMotor the PDP pin of the motor for the bottom shooter wheel. 
	 */
	public Shooter(int masterMotor, int followerMotor) {
		this.masterShooterMotor = new CANSparkMax(masterMotor, MotorType.kBrushless);
		this.followerShooterMotor = new CANSparkMax(followerMotor, MotorType.kBrushless);

		monitorGroup = new MonitoredCANSparkMaxGroup("Shooter", Constants.MOTOR_WARNING_TEMP, Constants.MOTOR_SHUTOFF_TEMP,
		this.masterShooterMotor, this.followerShooterMotor);

		this.shooterEncoder = masterShooterMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.COUNTS_PER_REVOLUTION);
		// this.shooterEncoder.setVelocityConversionFactor(2 * Math.PI / 60); 

		this.pidController = masterShooterMotor.getPIDController();

		// Spin Motors in the Same Direction
		this.masterShooterMotor.setInverted(false);
		this.followerShooterMotor.setInverted(true);

		this.followerShooterMotor.follow(this.masterShooterMotor);

		// Set Constants for the Shooter PID Controller
		pidController.setP(kP);
		pidController.setI(kI);
		pidController.setD(kD);
		pidController.setFF(kF);

		pidController.setOutputRange(-1.0, 1.0);
		pidController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
	}

	/**
	 * Get the encoder used to measure velocity (in rpm). 
	 *
	 * @return the built in motor encoder on the shooter motor. 
	 */
	public RelativeEncoder getEncoder() {
		return this.shooterEncoder;
	}
	
	/**
	 * Get the velocity that the shooter wheels are spinning at. 
	 *
	 * @return the shooter velocity measured by the encoder. 
	 */
	public double getActualVelocity() {
		return this.shooterEncoder.getVelocity();
	}

	/**
	 * Get the desired velocity of the shooter wheels. 
	 *
	 * @return the desired velocity that the PID controller will use.
	 */
	public double getVelocitySetpoint() {
		return this.velocity;
	}

	/**
	 * Set the velocity of the shooter using PID control. 

	 * @param rpm the desired velocity of the shooter. 
	 */
	public void setVelocity(double rpm) {
		this.pidController.setReference(monitorGroup.getOverheatShutoff() && !protectionOverridden
		? 0 : rpm, CANSparkMax.ControlType.kVelocity);
		this.velocity = rpm;
	}

	/**
	 * Get the PID controller of the shooter. 
	 *
	 * @return the PID controller of the shooter motor. 
	 */
	public SparkMaxPIDController getPIDController() {
		return this.pidController;
	}

	/**
	 * Get the monitor group for the shooter motors. 
	 *
	 * @return the monitor group for the shooter motors. 
	 */
	public MonitoredCANSparkMaxGroup getMonitorGroup() {
		return this.monitorGroup;
	}

	/**
	 * Get whether the overheating protection has been overridden. 
	 *
	 * @return whether the overheating protection has been overridden. 
	 */
	public boolean getOverheatShutoffOverride() {
		return this.protectionOverridden;
	}

	/**
	 * Set whether the overheating protection has been overridden. 
	 *
	 * @param protectionOverridden whether the overheating protection has been overridden. 
	 */
	public void setOverheatShutoffOverride(boolean protectionOverridden) {
		this.protectionOverridden = protectionOverridden;
	}

	@Override
	public void periodic() {
		this.monitorGroup.monitorOnce();

		if (this.monitorGroup.getOverheatShutoff()) {
			setVelocity(0);
			masterShooterMotor.stopMotor();
		}
	}

	// Settings of the shooter SPARK MAX motors.
	public void burnFlash() {
		masterShooterMotor.burnFlash();
		followerShooterMotor.burnFlash();
	}
	
	public double shooterDistToPower(double x, double y) {
		// angle is 1.222, sec(1.222) = 1.926, csc2(1.222) = 1.132
		// requires x and y in metres; others can try to rewrite
		return 1.132*9.807*x*x/(1.926*x-y)*Constants.BALL_MASS;
	}

	/**
	 * A custom exception that is thrown when the shooter motors cannot handle 
	 * the amount of power passed to it. 
	 */
	public static class PowerException extends Exception {
		public PowerException() {
			super(); 
		}

		public PowerException(String msg) {
			super(msg); 
		}

		public PowerException(Throwable cause) {
			super(cause); 
		}

		public PowerException(String msg, Throwable cause) {
			super(msg, cause); 
		}
	}

	public void fire(boolean upper) throws PowerException {
		double power = shooterDistToPower(shooterDist, upper ? Constants.UPPER_HUB : Constants.LOWER_HUB);
		
		/* fire at power calculated
		 * maximum power of NEO is 55/s
		 * Attached to an 8 inch 4.6 pound mecanum wheel,
		 * that's ~33 Joules
		 * feel free to change from 33;
		 * it's just a rough estimate based on a uniform wheel
		 */
		if (power > 33) throw new PowerException("NEO cannot support a throw that far");
		setVelocity(power/33);
	}
}
