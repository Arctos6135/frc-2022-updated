package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.Constants;


/**
 * The climb subsystem consists of two motors (brushed) and two TALON SRX motor
 * controllers. The default command for this subsystem is {@link frc.robot.commands.climbing.Climb}. 
 * 
 * @see {@link HookSubsystem}
 */
public class Elevator extends SubsystemBase {
    // Motors for Pulling Robot Up 
    private final TalonSRX climbMotorUpLeft;
    private final TalonSRX climbMotorUpRight;  
    
    private static boolean overrideClimbTime;

    /**
     * Creates new instance of the climbing subsystem. 
     *
     * @param climbMotorUpLeft the CAN ID of the left climbing motor. 
     * @param climbMotorUpRight the CAN ID of the right climbing motor. 
     */
    public Elevator(int climbMotorUpLeft, int climbMotorUpRight) {
        this.climbMotorUpLeft = new TalonSRX(climbMotorUpLeft); 
        this.climbMotorUpRight = new TalonSRX(climbMotorUpRight); 

        this.climbMotorUpLeft.setInverted(false);
        this.climbMotorUpRight.setInverted(false); 

        setNeutralModeClimb(NeutralMode.Brake);
        
        this.stopClimbMotors();
    }

    /**
     * Toggle the climb timer override
     */
    public static void toggleClimbTimeOverride() {
        Elevator.overrideClimbTime = !Elevator.overrideClimbTime;
    }
    
    /**
     * Get whether climb time protection has been overrided.
     * 
     * @return whether climb time has been overrided.
     */
    public static boolean getClimbTimeOverride() {
        return Elevator.overrideClimbTime;
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
     * Set the speed of the climbing motors. This does nothing if the Elevator is not enabled.
     * 
     * @param climbSpeed the speed of the robot during climbing.
     */
    public void setClimbMotorSpeed(double climbSpeed) {
        if (this.enabled()) {
            this.climbMotorUpRight.set(ControlMode.PercentOutput, climbSpeed); 
            this.climbMotorUpLeft.set(ControlMode.PercentOutput, climbSpeed);
        } else {
            this.stopClimbMotors();
        }
    }
    
    /**
     * Check if the elevator should be able to move based off of the timer and the timer override.
     */
    private boolean enabled() {
        if (Elevator.overrideClimbTime) {
            return true;
        } else if (DriverStation.getMatchTime() <= Constants.START_CLIMB_TIME) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Stop the climbing motors. 
     */
    public void stopClimbMotors() {
        this.climbMotorUpRight.set(ControlMode.PercentOutput, 0);
        this.climbMotorUpLeft.set(ControlMode.PercentOutput, 0); 
    }
}
