package frc.robot.commands.auto.routines;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathFinder;
import frc.robot.commands.indexer.AutoFeed;
import frc.robot.commands.indexer.SensoredRoll;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.shooting.PrepareShooterPID;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class TwoBallAuto {
    private Pose2d startPosition;  

    private Pose2d cargo; 
    private List<Translation2d> waypoints; 
    
    private final Drivetrain drivetrain; 
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeeder; 
    private final IntakeSubsystem intakeSubsystem; 

    private double shooterRPM;

    // Driving Trajectories 
    private PathFinder retrieveFirstBall; 

    public TwoBallAuto(Drivetrain drivetrain, Shooter shooter, ShooterFeederSubsystem shooterFeeder, IntakeSubsystem intakeSubsystem,
        int tarmac, boolean lowHub) {

        this.drivetrain = drivetrain;
        this.shooter = shooter; 
        this.shooterFeeder = shooterFeeder; 
        this.intakeSubsystem = intakeSubsystem; 

        if (lowHub) {
            shooterRPM = Constants.LOW_HUB_RPM;
        } else {
            shooterRPM = Constants.HIGH_HUB_RPM; 
        }

        switch (tarmac) {
            case FieldConstants.TOP_BLUE_TARMAC: 
                this.startPosition = FieldConstants.FENDER_1; 
                this.cargo = FieldConstants.TOP_LEFT_CARGO_BLUE;
                this.waypoints.add(FieldConstants.TARMAC_TOP_LEFT_1_REFERENCE.getTranslation()); 
                break; 
            case FieldConstants.BOTTOM_BLUE_TARMAC:
                this.startPosition = FieldConstants.FENDER_2; 
                this.cargo = FieldConstants.BOTTOM_LEFT_CARGO_BLUE; 
                this.waypoints.add(FieldConstants.TARMAC_BOTTOM_LEFT_2_REFERENCE.getTranslation());
                break; 
            case FieldConstants.BOTTOM_RED_TARMAC:
                this.startPosition = FieldConstants.FENDER_3; 
                this.cargo = FieldConstants.BOTTOM_RIGHT_CARGO_RED; 
                this.waypoints.add(FieldConstants.TARMAC_BOTTOM_RIGHT_1_REFERENCE.getTranslation()); 
                break; 
            case FieldConstants.TOP_RED_TARMAC: 
                this.startPosition = FieldConstants.FENDER_4;
                this.cargo = FieldConstants.TOP_RIGHT_CARGO_RED; 
                this.waypoints.add(FieldConstants.TARMAC_TOP_RIGHT_2_REFERENCE.getTranslation()); 
                break;
        }

        this.retrieveFirstBall = new PathFinder(
            this.drivetrain, 
            this.startPosition, 
            this.waypoints, 
            this.cargo
        ); 
    }

    public ParallelRaceGroup getAutoCommand() {
        return new ParallelRaceGroup(
            new AutoIntake(intakeSubsystem, AutoConstants.AUTO_INTAKE_SPEED), 
            new PrepareShooterPID(shooter, shooterRPM), 
            new SequentialCommandGroup(
                // Shoot from in front of the fender. 
                new AutoFeed(shooterFeeder), 
                // Drive to retrieve cargo.
                this.retrieveFirstBall.getAutoCommand(),
                // Intake the ball and store it at the color sensor. 
                new SensoredRoll(shooterFeeder),
                // Drive back to fender.  
                PathFinder.invertPathfinderCommand(this.retrieveFirstBall), 
                // Shoot the ball.
                new AutoFeed(shooterFeeder)
            )
        );
    }
}