package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.commands.auto.PathFinder;
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
         * Intake 2 balls and shoot them.
         * 
         * <ul> 
         * <li>Starts: Anywhere</li> 
         * <li>Ends: Near Hub</li> 
         * <li>Scores: 2 Balls</li> 
         * <li>Preload: 1 Ball</li>
         */
        TWO_BALL_AUTO("Two Ball Auto");  

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
            case FORWARD_SHOOT_LOW_HUB:
                return new SequentialCommandGroup(
                    new PathFinder(drivetrain, new Pose2d(0, 0, new Rotation2d(0)),
                        null, new Pose2d(2, 0, new Rotation2d(0)))
                    .andThen(new Shoot(shooter, shooterFeeder, true))
                    .andThen(PathFinder.resetInitialPosition())
                );
            case INIT_FORWARD:
                return new DriveDistance(drivetrain, 72); 
            case INIT_REVERSE:
                return new DriveDistance(drivetrain, -72);
            case INTAKE:
                return new AutoIntake(intake, intakeArm, Constants.AUTO_INTAKE_SPEED, false);
            case TWO_BALL_AUTO:
                return new SequentialCommandGroup(
                    new PathFinder(
                        drivetrain,
                        new Pose2d(0, 0, new Rotation2d(0)), 
                        null,
                        new Pose2d(0, 2, new Rotation2d(0)))
                    .andThen(new Shoot(shooter, shooterFeeder, true))
                    .andThen(new PathFinder(
                        drivetrain,
                        new Pose2d(0, 0, new Rotation2d(0)), 
                        List.of(new Translation2d(0, -2), new Translation2d(1.5, -3)),
                        new Pose2d(3, -6, new Rotation2d(0))
                    ))
                    .andThen(new AutoIntake(intake, intakeArm, 0.75, false))
                    .andThen(new PathFinder(
                        drivetrain, 
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(new Translation2d(-3, 4)), 
                        new Pose2d(-3, 6, new Rotation2d(0))
                    ))
                    .andThen(new Shoot(shooter, shooterFeeder, true))
                );
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
