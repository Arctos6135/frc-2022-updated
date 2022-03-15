package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The climb subsystem consists of two motors (brushed) and two TALON SRX motor
 * controllers. The default command for this subsystem is {@link frc.robot.commands.climbing.Climb}. 
 * 
 * @see {@link HookSubsystem}
 */
public class ClimbSubsystem extends SubsystemBase {
    // Motors for Pulling Robot Up 
    private final TalonSRX climbMotorUpLeft;
    private final TalonSRX climbMotorUpRight;  

    public static boolean overrideClimbTime = false; 

    /**
     * Creates new instance of the climbing subsystem. 
     *
     * @param climbMotorHook the PDP pin of the hook deployment motor. 
     * @param climbMotorUpLeft the PDP pin of the leftset climbing motor. 
     * @param climbMotorUpRight the PDP pin of the right climbing motor. 
     */
    public ClimbSubsystem(int climbMotorUpLeft, int climbMotorUpRight) {
        this.climbMotorUpLeft = new TalonSRX(climbMotorUpLeft); 
        this.climbMotorUpRight = new TalonSRX(climbMotorUpRight); 

        this.climbMotorUpLeft.follow(this.climbMotorUpRight); 

        setNeutralModeClimb(NeutralMode.Brake);
    }

    /**
     * Toggle time override.
     */
    public static void toggleClimbTimeOverride() {
        ClimbSubsystem.overrideClimbTime = !ClimbSubsystem.overrideClimbTime;
    }

    /**
     * Get whether climb time protection has been overrided.
     * 
     * @return whether climb time has been overrided.
     */
    public static boolean getClimbTimeOverride() {
        return ClimbSubsystem.overrideClimbTime;
    }

    
    /**
     * Set the idlemode of both the climbing motors. 
     * 
     * @param neutralMode the neutral mode of both the climbing motors. 
     */
    public void setNeutralModeClimb(NeutralMode neutralMode) {
        this.climbMotorUpLeft.setNeutralMode(neutralMode); 
        this.climbMotorUpRight.setNeutralMode(neutralMode); 
    }

    /**
     * Set the speed of the climbing motors. 
     * 
     * @param climbSpeed the speed of the robot during climbing.
     */
    public void setClimbMotorSpeed(double climbSpeed) {
        this.climbMotorUpRight.set(ControlMode.PercentOutput, climbSpeed); 
    }

    /**
     * Stop the climbing motors. 
     */
    public void stopClimbMotors() {
        this.climbMotorUpRight.set(ControlMode.PercentOutput, 0);
        this.climbMotorUpLeft.set(ControlMode.PercentOutput, 0); 
    }
}
