package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The intake subsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
    // No Pneumatics

    // Motors to Spin Mecanum Wheels 
    private final CANSparkMax mecanumWheelMotor;
    private final CANSparkMax intakeArmMotor;
    private final SparkMaxPIDController intakeArmPIDController; 
 
    private IdleMode idleMode;

    private boolean intakeArmLowered = false; 
    private double intakeArmPosition = 0; 

    // Intake PID Controller Constants
    // TODO: tune PIDF constants 
    public final static double kP = 0, kI = 0, kD = 0, kF = 0; 

    /**
     * Get whether the intake arm is raised or lowered. 
     * 
     * @return whether the intake arm is raised or lowered. 
     */
    public boolean getIntakeArmLowered() {
        return this.intakeArmLowered; 
    }

    /**
     * Get the intake arm position. 
     * 
     * @return the intake arm position. 
     */
    public double getIntakeArmPosition() {
        return this.intakeArmPosition; 
    }

    /**
     * Set the speed of the intake motors.
     *
     * @param scale the speed of the motors.
     */
    public void setMecanumWheelMotor(double scale) {
        mecanumWheelMotor.set(scale); 
    }

    /**
     * Rotate the intake arm to a position.
     * 
     * @param setpoint the position of the intake arm, restricted to [-1.0, 1.0]. 
     */
    public void setIntakeArmPosition(double setpoint) {
        this.intakeArmPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition); 
        this.intakeArmPosition = setpoint; 

        if (setpoint == Constants.INTAKE_ARM_LOWERED) {
            this.intakeArmLowered = true; 
        } else {
            this.intakeArmLowered = false; 
        }
    }

    /**
     * Get the Idle Mode of the Intake motors (brake/coast).
     *
     * @return the intake motor idle mode.
     */
    public IdleMode getIntakeIdleMode() {
        return this.idleMode;
    }
   
    /**
     * Set the Idle Mode of the Intake motors (brake/coast).
     *
     * @param idleMode the idle mode.
     */
    public void setMecanumWheelMotorMode(IdleMode idleMode) {
        this.idleMode = idleMode;
        this.mecanumWheelMotor.setIdleMode(this.idleMode); 
    }
   
    /**
     * Create a new intake subsystem.
     * 
     * The PID controller used by the intake arm is built in to the Spark Max motor used for 
     * the intake arm. 
     *
     * @param leftIntake the CAN ID of the left intake motor controller.
     * @param rightIntake the CAN ID of the right intake motor controller.
     */
    public IntakeSubsystem(int mecanumWheelMotor, int intakeArmMotor) {
        this.mecanumWheelMotor = new CANSparkMax(mecanumWheelMotor, MotorType.kBrushless);
        this.intakeArmMotor = new CANSparkMax(intakeArmMotor, MotorType.kBrushless);
        this.intakeArmPIDController = this.intakeArmMotor.getPIDController(); 
       
        // Invert the left motor to spin in the same direction.
        this.mecanumWheelMotor.setInverted(true);
        this.intakeArmMotor.setInverted(false);

        this.mecanumWheelMotor.stopMotor();
        this.intakeArmMotor.stopMotor(); 

        intakeArmPIDController.setP(kP); 
        intakeArmPIDController.setI(kI); 
        intakeArmPIDController.setD(kD); 
        intakeArmPIDController.setFF(kF); 

        intakeArmPIDController.setOutputRange(-1.0, 1.0); 
        intakeArmPIDController.setReference(0.0, CANSparkMax.ControlType.kPosition); 
    }
}
