package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The hook deployment system for rung climbing. 
 * Note that this subsystem is separated from {@link ClimbSubsystem} due to 
 * the requirement of default commands in {@link frc.robot.RobotContainer}.
 */
public class HookSubsystem extends SubsystemBase {

    private final TalonSRX climbMotorHook; 

    /**
     * Creates a new Hook subsystem. 
     * 
     * @param climbHookMotor the CAN ID of the hook motor. 
     */
    public HookSubsystem(int climbHookMotor) {
        this.climbMotorHook = new TalonSRX(climbHookMotor); 
        this.climbMotorHook.setNeutralMode(NeutralMode.Brake); 
    }

    /**
     * Set the idlemode of the hook deployment motor. 
     * 
     * @param idleMode the idlemode of the hook deployment motor. 
     */
    public void setIdleModeHook(NeutralMode neutralMode) {
        this.climbMotorHook.setNeutralMode(neutralMode); 
    }

    /**
     * Set the speed of the hook motors. 
     * 
     * @param hookSpeed the speed of the hook motor.
     */
    public void setHookMotorSpeed(double hookSpeed) {
        this.climbMotorHook.set(ControlMode.PercentOutput, hookSpeed); 
    }

    /**
     * Stop the hook motor. 
     */
    public void stopHookMotor() {
        this.climbMotorHook.set(ControlMode.PercentOutput, 0); 
    }

}
