package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathFinder;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.shooting.AutoShoot;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
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
        TWO_BALL_AUTO_PRELOAD("Two Ball Auto Preload"), 
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
         * <li>Starts: Anywhere</li> 
         * <li>Ends: N/A </li> 
         * <li>Scores: x Balls</li> 
         * <li>Preload: 1 Balls</li>
         */
        TERMINAL_AUTO("Terminal Auto"); 

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
                    // Drive Forwards
                    new PathFinder(drivetrain, new Pose2d(0, 0, new Rotation2d()),
                        null, new Pose2d(2, 0, new Rotation2d())).getAutoCommand()
                    // Shoot 
                    .andThen(new AutoShoot(shooter, shooterFeeder, true, Constants.PRELOADED_BALLS))
                    // Drive Backwards (Exit Tarmac)
                    .andThen(new PathFinder(drivetrain, new Pose2d(0, 0, new Rotation2d()), 
                        null, new Pose2d(-4, 0, new Rotation2d())).getAutoCommand())
                    .andThen(PathFinder.resetInitialPosition())
                );
            case INIT_FORWARD:
                return new PathFinder(drivetrain, new Pose2d(0, 0, new Rotation2d(0)), 
                    null, new Pose2d(2, 0, new Rotation2d(0))).getAutoCommand();
            case INIT_REVERSE:
                return new PathFinder(drivetrain, new Pose2d(0, 0, new Rotation2d(0)),
                null, new Pose2d(-2, 0, new Rotation2d(0))).getAutoCommand();
            case INTAKE:
                return new AutoIntake(intake, intakeArm, AutoConstants.AUTO_INTAKE_SPEED, false);
            case TWO_BALL_AUTO_PRELOAD:
                return new ParallelRaceGroup(
                    // Intake for all of autonomous
                    new AutoIntake(intake, intakeArm, AutoConstants.AUTO_INTAKE_SPEED, false),
                    new SequentialCommandGroup(
                        // Drive towards center of fender
                        new PathFinder(
                            drivetrain,
                            new Pose2d(0, 0, new Rotation2d(0)), 
                            null, 
                            FieldConstants.FENDER_2)
                        .getAutoCommand()
                        // Shoot preloads
                        .andThen(new AutoShoot(shooter, shooterFeeder, true, Constants.PRELOADED_BALLS))
                        // Drive to the bottom left blue ball
                        .andThen(new PathFinder(
                            drivetrain,
                            List.of(FieldConstants.FENDER_2, 
                                    FieldConstants.TARMAC_BOTTOM_LEFT_2_REFERENCE, 
                                    FieldConstants.BOTTOM_CARGO_BLUE)
                        ).getAutoCommand())
                        // Drive to the center of the fender
                        .andThen(new PathFinder(
                            drivetrain, 
                            List.of(FieldConstants.BOTTOM_CARGO_BLUE, 
                                    FieldConstants.TARMAC_BOTTOM_LEFT_2_REFERENCE, 
                                    FieldConstants.FENDER_2)
                        ).getAutoCommand())
                        // Shoot 
                        .andThen(new AutoShoot(shooter, shooterFeeder, true, Constants.PRELOADED_BALLS)))
                );
            case TWO_BALL_AUTO: 
                return new ParallelRaceGroup(
                    new AutoIntake(intake, intakeArm, AutoConstants.AUTO_INTAKE_SPEED, false), 
                    new SequentialCommandGroup(
                        // Drive to the bottom left ball
                        new PathFinder(
                            drivetrain, new Pose2d(0, 0, new Rotation2d(0)), 
                            null, 
                            FieldConstants.BOTTOM_CARGO_BLUE
                        ).getAutoCommand()
                        // Drive to the center of the fender 
                        .andThen(new PathFinder(
                            drivetrain, List.of(FieldConstants.BOTTOM_CARGO_BLUE, 
                            FieldConstants.TARMAC_BOTTOM_LEFT_2_REFERENCE, 
                            FieldConstants.FENDER_2)).getAutoCommand())
                        // Shoot the ball
                        .andThen(new AutoShoot(shooter, shooterFeeder, true, Constants.PRELOADED_BALLS))
                        // Drive to the middle ball
                        .andThen(new PathFinder(
                            drivetrain, List.of(
                            FieldConstants.FENDER_2, 
                            FieldConstants.TARMAC_BOTTOM_LEFT_1_REFERENCE, 
                            FieldConstants.MIDDLE_CARGO_BLUE)).getAutoCommand())
                        // Drive back to center of fender
                        .andThen(new PathFinder(
                            drivetrain, List.of(
                                FieldConstants.MIDDLE_CARGO_BLUE, 
                                FieldConstants.TARMAC_BOTTOM_LEFT_1_REFERENCE, 
                                FieldConstants.FENDER_2
                            )).getAutoCommand())
                        // Shoot the ball
                        .andThen(new AutoShoot(shooter, shooterFeeder, true, Constants.PRELOADED_BALLS))
                    )
                );
            case TERMINAL_AUTO: 
                return new ParallelRaceGroup(
                    new AutoIntake(intake, intakeArm, AutoConstants.AUTO_INTAKE_SPEED, false), 
                    new SequentialCommandGroup(
                        // Drive to the center of fender
                        new PathFinder(drivetrain, 
                            new Pose2d(), 
                            null, 
                            FieldConstants.FENDER_2).getAutoCommand()
                        // Shoot the preloaded ball
                        .andThen(new AutoShoot(shooter, shooterFeeder, true, Constants.PRELOADED_BALLS))
                        // Drive to terminal and intake 2 balls 
                        .andThen(new PathFinder(
                            drivetrain, 
                            List.of(FieldConstants.FENDER_2, 
                            FieldConstants.TARMAC_BOTTOM_LEFT_1_REFERENCE, 
                            FieldConstants.MIDDLE_CARGO_BLUE, 
                            FieldConstants.TERMINAL_CARGO)))
                        // Drive back to fender
                        .andThen(new PathFinder(
                            drivetrain, 
                            List.of(FieldConstants.TERMINAL_CARGO, 
                            FieldConstants.MIDDLE_CARGO_BLUE, 
                            FieldConstants.TARMAC_BOTTOM_LEFT_1_REFERENCE, 
                            FieldConstants.FENDER_2)).getAutoCommand())
                        // Shoot both balls 
                        .andThen(new AutoShoot(shooter, shooterFeeder, true, Constants.MAX_BALLS))
                    )
                );
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

        chooser.setDefaultOption(AutoMode.NONE.autoName, AutoMode.NONE); 
    }
}
