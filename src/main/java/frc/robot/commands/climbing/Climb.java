package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;

/**
 * Pulls the robot up using two motors on the climb subsystem to roll the winch.    
 * This is the default command for 
 * @see Elevator.java 
 */
public class Climb extends CommandBase {
    
    private final Elevator climbSubsystem;
    private final XboxController operatorController;
    // the Xbox axis that controls the elevator
    private final int deployClimbAxisRight;
    private final int deployClimbAxisLeft; 

    // whether precision drive is enabled. Precision drive causes the elevator to climb at half speed
    public static boolean precisionClimb = false; 
    // the normal elevator speed
    public static double normalPrecision = 0.6;
    // the elevator speed when precision drive is enabled
    public static double precisionFactor = 0.025; 
    // whether to climb even if the game is not in endgame
    public static boolean overrideClimbTime = false;


    /**
     * Create an instance of the climb command. 
     * 
     * @param climbSubsystem the climb subsystem with the climb and hook motors. 
     * @param operatorController the controller used to interact with the climb commands. 
     */
    public Climb(Elevator climbSubsystem, XboxController operatorController, int deployClimbAxisRight, int deployClimbAxisLeft) {
        this.climbSubsystem = climbSubsystem; 
        this.operatorController = operatorController;
        this.deployClimbAxisRight = deployClimbAxisRight;
        this.deployClimbAxisLeft = deployClimbAxisLeft; 

        addRequirements(climbSubsystem);
    }

    /**
     * Toggle whether to enter precision mode for climbing. 
     */
    public static void togglePrecisionClimb() {
        Climb.precisionClimb = !Climb.precisionClimb;
    }

    /**
     * Get whether climbing is in precision mode. 
     * 
     * @return whether the climb is in precision mode.
     */
    public static boolean isPrecisionClimb() {
        return Climb.precisionClimb;
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        double climbSpeedRight = operatorController.getRawAxis(this.deployClimbAxisRight);
        double climbSpeedLeft = operatorController.getRawAxis(this.deployClimbAxisLeft);

        climbSpeedRight = TeleopDrive.applyDeadband(climbSpeedRight, Constants.CONTROLLER_DEADZONE);
        climbSpeedLeft = TeleopDrive.applyDeadband(climbSpeedLeft, Constants.CONTROLLER_DEADZONE);
        
        if (Climb.precisionClimb) {
            climbSpeedRight = climbSpeedRight * precisionFactor;
            climbSpeedLeft = climbSpeedLeft * precisionFactor;
        } else {
            climbSpeedRight = climbSpeedRight * normalPrecision;
            climbSpeedLeft = climbSpeedLeft * normalPrecision;
        }

        if (!DriverStation.isAutonomous()) {
            // this will only activate the elevator if the match is in endgame or if Elevator.overrideClimbTime is true
            this.climbSubsystem.setClimbMotorSpeed(climbSpeedRight, climbSpeedLeft);
        }
    }

    @Override 
    public void end(boolean interrupted) {
        this.climbSubsystem.stopClimbMotors();
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}
