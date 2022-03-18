package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/**
 * The intake arm for the intake subsystem. The arm is rotated up and down 
 * to intake balls from the ground, using mecanum wheels. 
 * The default command for this subsystem is {@link frc.robot.commands.intake.RotateArm}. 
 */
public class IntakeArm extends SubsystemBase {
    private final VictorSPX intakeArmMotor;
    
    private boolean intakeArmLowered = false;
    private double intakeArmPosition = 0;

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
        this.intakeArmMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Stop the intake arm motor. 
     */
    public void stopIntakeArmMotor() {
        this.intakeArmMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Create a new intake arm subsystem. 
     * 
     * @param intakeArmMotor the CAN ID of the intake arm motor. 
     */
    public IntakeArm(int intakeArmMotor) {
        this.intakeArmMotor = new VictorSPX(intakeArmMotor);
        this.intakeArmMotor.setInverted(false);
        stopIntakeArmMotor();
    }
}
