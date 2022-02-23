package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
    // Motor for Hook Deployment 
    private final CANSparkMax climbMotorHook;

    // Motors for Pulling Robot Up 
    private final CANSparkMax climbMotorUpLeft;
    private final CANSparkMax climbMotorUpRight;  

    /**
     * Creates new instance of the climbing subsystem. 
     *
     * @param climbMotorHook the PDP pin of the hook deployment motor. 
     * @param climbMotorUpLeft the PDP pin of the leftset climbing motor. 
     * @param climbMotorUpRight the PDP pin of the right climbing motor. 
     */
    public ClimbSubsystem(int climbMotorHook, int climbMotorUpLeft, int climbMotorUpRight) {
        this.climbMotorHook = new CANSparkMax(climbMotorHook, MotorType.kBrushless); 
        this.climbMotorUpLeft = new CANSparkMax(climbMotorUpLeft, MotorType.kBrushless); 
        this.climbMotorUpRight = new CANSparkMax(climbMotorUpRight, MotorType.kBrushless); 

        this.climbMotorUpLeft.follow(this.climbMotorUpRight); 

        setIdleModeHook(IdleMode.kBrake); 
        setIdleModeClimb(IdleMode.kBrake);
    }

    /**
     * Set the idlemode of the hook deployment motor. 
     * 
     * @param idleMode the idlemode of the hook deployment motor. 
     */
    public void setIdleModeHook(IdleMode idleMode) {
        this.climbMotorHook.setIdleMode(idleMode); 
    }

    /**
     * Set the idlemode of both the climbing motors. 
     * 
     * @param idleMode the idlemode of both the climbing motors. 
     */
    public void setIdleModeClimb(IdleMode idleMode) {
        this.climbMotorUpLeft.setIdleMode(idleMode); 
        this.climbMotorUpRight.setIdleMode(idleMode); 
    }

    /**
     * Set the speed of the hook motors. 
     * 
     * @param hookSpeed the speed of the hook motor.
     */
    public void setHookMotorSpeed(double hookSpeed) {
        this.climbMotorHook.set(hookSpeed); 
    }

    /**
     * Set the speed of the climbing motors. 
     * 
     * @param climbSpeed the speed of the robot during climbing.
     */
    public void setClimbMotorSpeed(double climbSpeed) {
        this.climbMotorUpRight.set(climbSpeed); 
    }

    /**
     * Stop the climbing motors. 
     */
    public void stopClimbMotors() {
        this.climbMotorUpRight.stopMotor();
        this.climbMotorUpLeft.stopMotor(); 
    }

    public void stopHookMotor() {
        this.climbMotorHook.stopMotor(); 
    }
}
