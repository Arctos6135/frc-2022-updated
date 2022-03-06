package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.driving.DriveDistance;
import frc.robot.commands.intake.AutoIntake;
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
         * Drive to the hub and shoot the ball in the lower hub. 
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Forwards</li> 
         * <li>Scores: 2 Match Points (One Ball, Lower Hub)</li> 
         * <li>Preload: 1 or 2 Balls</li>
         */
        FORWARD_SHOOT_LOW_HUB("Drive Forwards & Shoot"), 
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
         * Drive towards a ball and intake.
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: The ball's location</li> 
         * <li>Scores: None</li> 
         * <li>Preload: No Balls</li>
         */
        DRIVE_INTAKE("Drive & Intake"), 
        /**
         * Uses path weaver and trajectory to drive along a path. 
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: N/A</li> 
         * <li>Scores: N/A</li> 
         * <li>Preload: No Balls</li>
         */
        PATH_WEAVER("Path Weaver Auto");  

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

    public Command getAuto(AutoMode mode, Drivetrain drivetrain, IntakeSubsystem intake,
        IntakeArm intakeArm, Shooter shooter, ShooterFeederSubsystem shooterFeeder) {
        
        // Measurements in Inches 
        switch(mode) {
            case NONE:
                return null;
            case DRIVE_INTAKE:
                return null; // TODO: change 
            case FORWARD_SHOOT_LOW_HUB:
                return new DriveDistance(drivetrain, 60);
            case INIT_FORWARD:
                return new DriveDistance(drivetrain, 72); 
            case INIT_REVERSE:
                return new DriveDistance(drivetrain, -72);
            case INTAKE:
                return new AutoIntake(intake, intakeArm, Constants.AUTO_INTAKE_SPEED, false);
            case PATH_WEAVER:
                return null;
            default:
                return null;
        }
    }

    public Autonomous() {
        chooser = new SendableChooser<>(); 

        for (AutoMode mode : AutoMode.class.getEnumConstants()) {
            chooser.addOption(mode.autoName, mode); 
        }

        chooser.setDefaultOption(AutoMode.NONE.autoName, AutoMode.NONE); 
    }
}
