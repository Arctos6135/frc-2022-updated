package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.routines.OneBallAuto;
import frc.robot.commands.auto.routines.ThreeBallAuto;
import frc.robot.commands.auto.routines.ThreeBallTerminalAuto;
import frc.robot.commands.auto.routines.TwoBallAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * A number of autonomous commands for autonomous mode. 
 * This includes driving, shooting, and intake. 
 */
public class Autonomous {

    private SendableChooser<AutoMode> chooser; 

    public enum AutoMode {
        /**
         * No autonomous.
         *
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Same Location</li> 
         * <li>Scores: Nothing</li> 
         * <li>Preload: Any</li>
         * </ul>   
         */
        NONE("None"), 
        /**
         * Drive forwards, shoot, drive off tarmac. 
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Off Tarmac</li> 
         * <li>Scores: 1 Ball</li> 
         * <li>Preload: 1 Ball</li>
         * </ul>
         */
        ONE_BALL_AUTO("One Ball Auto (Low Hub)"),
        /**
         * Shoot high hub and drive off the tarmac. 
         * 
         * <ul> 
         * <li>Starts: Edge of Tarmac</li> 
         * <li>Ends: Off Tarmac</li> 
         * <li>Scores: 10 Points</li> 
         * <li>Preload: 1 Ball</li>
         * </ul>
         */
        TWO_BALL_AUTO("Two Ball Auto (High Hub)"), 
        /**
         * Shoot high hub and drive off the tarmac. 
         * 
         * <ul> 
         * <li>Starts: Edge of Tarmac</li> 
         * <li>Ends: Off Tarmac</li> 
         * <li>Scores: 10 Points</li> 
         * <li>Preload: 1 Ball</li>
         * </ul>
         */
        TWO_BALL_FAST_AUTO("Two Ball Fast Auto (High Hub)"), 
        /**
         * Shoot high hub, intake two cargo ring balls.
         * 
         * <ul> 
         * <li>Starts: Edge of Tarmac</li> 
         * <li>Ends: Forwards</li> 
         * <li>Scores: 14 Points</li> 
         * <li>Preload: 1 Ball</li>
         * </ul> 
         */
        THREE_BALL_CARGO_RING_AUTO("Three Ball Auto (Cargo Rings)"),
        /**
         * Shoot high hub, intake one cargo ring and one terminal ball.
         *  
         * <ul> 
         * <li>Starts: Edge of Tarmac</li> 
         * <li>Ends: Forwards</li> 
         * <li>Scores: 14 Points</li> 
         * <li>Preload: 1 Ball</li> 
         */
        THREE_BALL_TERMINAL_AUTO("Three Ball Auto (Terminal)");
        
        String autoName; 

        AutoMode(String autoName) {
            this.autoName = autoName; 
        }
    }

    /**
     * Get the autonomous chooser with all possible auto modes. 
     * 
     * @return the autonomous chooser.
     */
    public SendableChooser<AutoMode> getChooser() {
        return this.chooser; 
    }

    /**
     * Get the autonomous routine. 
     * 
     * @param mode the autonomous mode. 
     * @param drivetrain the robot driving subsystem. 
     * @param intake the intake system.
     * @param intakeArm the intake arm. 
     * @param shooter the shooter subsystem. 
     * @param shooterFeeder the essie subsystem. 
     * @return the autonomous routine, as a {@link edu.wpi.first.wpilibj2.command.Command}.
     */
    public Command getAuto(AutoMode mode, Drivetrain drivetrain, IntakeSubsystem intakeSubsystem,
        Shooter shooter, ShooterFeederSubsystem shooterFeeder) {
        // Measurements in Meters
        switch(mode) {
            case ONE_BALL_AUTO: 
                return new OneBallAuto(drivetrain, shooter, shooterFeeder).getAutoCommand(); 
            case TWO_BALL_AUTO: 
                return new TwoBallAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem).getAutoCommand();
            case TWO_BALL_FAST_AUTO:
                return new TwoBallAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem).getFastAutoCommand(); 
            case THREE_BALL_CARGO_RING_AUTO: 
                return new ThreeBallAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem).getAutoCommand();
            case THREE_BALL_TERMINAL_AUTO: 
                return new ThreeBallTerminalAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem).getAutoCommand();
            default:
                return null;
        }
    }

    /**
     * Creates a new sendable chooser for autonomous mode. 
     * Autonomous can be selected prior to the match. 
     */
    public Autonomous() {
        chooser = new SendableChooser<>(); 

        for (AutoMode mode : AutoMode.class.getEnumConstants()) {
            chooser.addOption(mode.autoName, mode); 
        }

        chooser.setDefaultOption(AutoMode.TWO_BALL_AUTO.autoName, AutoMode.TWO_BALL_AUTO); 
    }
}
