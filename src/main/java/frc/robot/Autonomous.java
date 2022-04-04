package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.routines.DriveDistance;
import frc.robot.commands.auto.routines.NonTrajectory;
import frc.robot.commands.auto.routines.TerminalAuto;
import frc.robot.commands.auto.routines.TwoBallAuto;
import frc.robot.commands.auto.routines.TwoBallNonTrajectory;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeArm;
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
        NON_TRAJECTORY("No Trajectory"),
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
        TWO_BALL_AUTO_NON_TRAJECTORY("Two Ball Auto (No Trajectory)"), 
        /**
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Forwards</li> 
         * <li>Scores: Nothing</li> 
         * <li>Preload: Any</li>
         * </ul>   
         */
        TESTING("Test & Debug"), 
        /**
         * Drive towards hub. 
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Forwards</li> 
         * <li>Scores: Nothing</li> 
         * <li>Preload: Any</li>
         */
        INIT_FORWARD("Drive Forwards (Towards Hub)"), 
        /**
         * Drive away from hub and off the tarmac. 
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Backwards</li> 
         * <li>Scores: 2 Match Points</li> 
         * <li>Preload: Any</li>
         */
        INIT_REVERSE("Drive Backwards (Off Tarmac)"), 
        /**
         * Intake a ball from the ground. 
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Same Location</li> 
         * <li>Scores: Nothing</li> 
         * <li>Preload: Any</li>
         */
        INTAKE("Intake Ball"), 
        /**
         * Intake 2 balls and shoot them.
         * 
         * <ul> 
         * <li>Starts: Top Blue Tarmac</li> 
         * <li>Ends: Near Hub</li> 
         * <li>Scores: 2 Balls</li> 
         * <li>Preload: 1 Ball</li>
         */
        TWO_BALL_AUTO_1("Two Ball Auto 1"), 
        /**
         * Intake 2 balls and shoot them.
         * 
         * <ul> 
         * <li>Starts: Bottom Blue Tarmac</li> 
         * <li>Ends: Near Hub</li> 
         * <li>Scores: 2 Balls</li> 
         * <li>Preload: 1 Ball</li>
         */
        TWO_BALL_AUTO_2("Two Ball Auto 2"),
        /**
         * Intake 2 balls and shoot them.
         * 
         * <ul> 
         * <li>Starts: Bottom Red Tarmac</li> 
         * <li>Ends: Near Hub</li> 
         * <li>Scores: 2 Balls</li> 
         * <li>Preload: 1 Ball</li>
         */
        TWO_BALL_AUTO_3("Two Ball Auto 3"), 
        /**
         * Intake 2 balls and shoot them.
         * 
         * <ul> 
         * <li>Starts: Top Red Tarmac</li> 
         * <li>Ends: Near Hub</li> 
         * <li>Scores: 2 Balls</li> 
         * <li>Preload: 1 Ball</li>
         */
        TWO_BALL_AUTO_4("Two Ball Auto 4"),
        /**
         * Intake 2 balls and shoot them.
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Near Hub</li> 
         * <li>Scores: 2 Balls</li> 
         * <li>Preload: 0 Balls</li>
         */
        TWO_BALL_AUTO("Two Ball Auto"),
        /**
         * Intake balls from the terminal.
         * 
         * <ul> 
         * <li>Starts: Top Blue Tarmac</li> 
         * <li>Ends: N/A </li> 
         * <li>Scores: x Balls</li> 
         * <li>Preload: 1 Balls</li>
         */
        // TERMINAL_AUTO("Terminal Auto"), 
        TERMINAL_AUTO_1("Terminal Auto 1"), 
        /**
         * Intake balls from the terminal.
         * 
         * <ul> 
         * <li>Starts: Bottom Blue Tarmac</li> 
         * <li>Ends: N/A </li> 
         * <li>Scores: x Balls</li> 
         * <li>Preload: 1 Balls</li>
         */
        TERMINAL_AUTO_2("Terminal Auto 2"), 
        /**
         * Intake balls from the terminal.
         * 
         * <ul> 
         * <li>Starts: Bottom Red Tarmac</li> 
         * <li>Ends: N/A </li> 
         * <li>Scores: x Balls</li> 
         * <li>Preload: 1 Balls</li>
         */
        TERMINAL_AUTO_3("Terminal Auto 3"), 
        /**
         * Intake balls from the terminal.
         * 
         * <ul> 
         * <li>Starts: Top Red Tarmac</li> 
         * <li>Ends: N/A </li> 
         * <li>Scores: x Balls</li> 
         * <li>Preload: 1 Balls</li>
         */
        TERMINAL_AUTO_4("Terminal Auto 4"); 

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
        IntakeArm intakeArm, Shooter shooter, ShooterFeederSubsystem shooterFeeder) {
        // Measurements in Meters
        switch(mode) {
            case NONE:
                return null;
            case NON_TRAJECTORY: 
                return new NonTrajectory(drivetrain, shooter, shooterFeeder).getAutoCommand(); 
            case TWO_BALL_AUTO_NON_TRAJECTORY: 
                return new TwoBallNonTrajectory(drivetrain, shooter, shooterFeeder, intakeSubsystem).getAutoCommand(); 
            case INIT_FORWARD:
                return new DriveDistance(drivetrain, 2).getAutoCommand();
            case INIT_REVERSE:
                return new DriveDistance(drivetrain, -2).getAutoCommand(); 
            case INTAKE:
                return new AutoIntake(intakeSubsystem, AutoConstants.AUTO_INTAKE_SPEED);
            case TWO_BALL_AUTO_1:
                return new TwoBallAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 1, true).getAutoCommand();
            case TWO_BALL_AUTO_2: 
                return new TwoBallAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 2, true).getAutoCommand(); 
            case TWO_BALL_AUTO_3:
                return new TwoBallAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 3, true).getAutoCommand();
            case TWO_BALL_AUTO_4:
                return new TwoBallAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 4, true).getAutoCommand();
            case TERMINAL_AUTO_1: 
                return new TerminalAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 1, true).getAutoCommand();
            case TERMINAL_AUTO_2:
                return new TerminalAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 2, true).getAutoCommand(); 
            case TERMINAL_AUTO_3:
                return new TerminalAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 3, true).getAutoCommand(); 
            case TERMINAL_AUTO_4: 
                return new TerminalAuto(drivetrain, shooter, shooterFeeder, intakeSubsystem, 4, true).getAutoCommand();  
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

        chooser.setDefaultOption(AutoMode.NON_TRAJECTORY.autoName, AutoMode.NON_TRAJECTORY); 
    }
}
