package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/**
 * The intake arm for the intake subsystem. The arm is rotated up and down 
 * to intake balls from the ground, using mecanum wheels.
 */
public class IntakeArm extends SubsystemBase {
    private final CANSparkMax intakeArmMotor;
    private final SparkMaxPIDController intakeArmPIDController;
    
    private boolean intakeArmLowered = false;
    private double intakeArmPosition = 0;
    
    // TODO: tune these constants
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
     * Set the intake arm motor to a certain speed. Use this for rotation.
     * 
     * @param speed the desired speed of the intake arm in the interval [-1.0, 1.0]. 
     */
    public void setIntakeArmMotor(double speed) {
        this.intakeArmMotor.set(speed);
    }

    public void stopIntakeArmMotor() {
        this.intakeArmMotor.stopMotor();
    }

    public IntakeArm(int intakeArmMotor) {
        this.intakeArmMotor = new CANSparkMax(intakeArmMotor, MotorType.kBrushless);
        this.intakeArmPIDController = this.intakeArmMotor.getPIDController();

        this.intakeArmMotor.setInverted(false);

        this.intakeArmMotor.stopMotor();
        
        intakeArmPIDController.setP(kP);
        intakeArmPIDController.setI(kI);
        intakeArmPIDController.setD(kD);
        intakeArmPIDController.setFF(kF);

        intakeArmPIDController.setOutputRange(-1.0, 1.0);
        intakeArmPIDController.setReference(0.0, CANSparkMax.ControlType.kPosition);
    }
}
