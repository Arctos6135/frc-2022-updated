package frc.robot.commands.auto.routines;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathFinder;
import frc.robot.commands.indexer.AutoFeed;
import frc.robot.commands.indexer.AutoLoad;
import frc.robot.commands.indexer.SensoredRoll;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.shooting.PrepareShooterPID;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class TerminalAuto {
    private List<Pose2d> cargo; 
    private final Pose2d fenderPosition; 

    private final Drivetrain drivetrain; 
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeeder; 
    private final IntakeSubsystem intakeSubsystem; 

    private double shooterRPM;

    public TerminalAuto(Drivetrain drivetrain, Shooter shooter, ShooterFeederSubsystem shooterFeeder, IntakeSubsystem intakeSubsystem, 
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
                this.fenderPosition = FieldConstants.FENDER_1;
                this.cargo = List.of(
                    FieldConstants.MIDDLE_LEFT_CARGO_BLUE, 
                    FieldConstants.TERMINAL_CARGO_BLUE); 
                break; 
            case FieldConstants.BOTTOM_BLUE_TARMAC:
                this.fenderPosition = FieldConstants.FENDER_2;
                this.cargo = List.of(
                    FieldConstants.TARMAC_BOTTOM_LEFT_1_REFERENCE,
                    FieldConstants.MIDDLE_LEFT_CARGO_BLUE, 
                    FieldConstants.TERMINAL_CARGO_BLUE); 
                break; 
            case FieldConstants.BOTTOM_RED_TARMAC:
                this.fenderPosition = FieldConstants.FENDER_3;
                this.cargo = List.of(
                    FieldConstants.MIDDLE_RIGHT_CARGO_RED, 
                    FieldConstants.TERMINAL_CARGO_RED
                );
                break; 
            case FieldConstants.TOP_RED_TARMAC:
                this.fenderPosition = FieldConstants.FENDER_4;
                this.cargo = List.of(
                    FieldConstants.TARMAC_TOP_RIGHT_1_REFERENCE,
                    FieldConstants.MIDDLE_RIGHT_CARGO_RED,
                    FieldConstants.TERMINAL_CARGO_RED 
                ); 
                break;
            default: 
                this.fenderPosition = null; 
                this.cargo = List.of(); 
        }
    } 

    public ParallelRaceGroup getAutoCommand() {
        Pose2d terminalCargo; 
        // Drive to first ball. 
        List<Pose2d> firstDrive; 
        // Drive from terminal to fender.
        List<Translation2d> secondDrive; 

        if (this.cargo.size() == 3) {
            terminalCargo = this.cargo.get(2); 
            firstDrive = List.of(this.cargo.get(0), this.cargo.get(1));
            secondDrive = List.of(this.cargo.get(1).getTranslation());
        } else {
            terminalCargo = this.cargo.get(1); 
            firstDrive = List.of(this.cargo.get(0));
            secondDrive = List.of(this.cargo.get(0).getTranslation()); 
        }

        if (this.fenderPosition != null) {
            return new ParallelRaceGroup(
            new AutoIntake(intakeSubsystem, AutoConstants.AUTO_INTAKE_SPEED),  
            new SequentialCommandGroup(
                // Spin the shooter flywheels. 
                new PrepareShooterPID(shooter, shooterRPM),
                // Shoot preload. 
                new AutoFeed(shooterFeeder), 
                // Stop shooter flywheels. 
                new PrepareShooterPID(shooter, 0),
                // Drive to retrieve first cargo. 
                new PathFinder(drivetrain, firstDrive).getAutoCommand(), 
                // Roll the ball and store it. 
                new SensoredRoll(shooterFeeder), 
                // Roll the ball to the top of the roller without shooting. 
                new AutoFeed(shooterFeeder), 
                // Drive to retrieve terminal cargo.
                new PathFinder(drivetrain, List.of(terminalCargo)).getAutoCommand(), 
                // Roll the ball and store it. 
                new SensoredRoll(shooterFeeder),
                // Drive back to fender.  
                new PathFinder(drivetrain, terminalCargo, secondDrive, fenderPosition), 
                // Load balls to shooter.                     
                new AutoLoad(shooterFeeder), 
                // Spin shooter flywheels. 
                new PrepareShooterPID(shooter, shooterRPM), 
                // Shoot the balls. 
                new AutoFeed(shooterFeeder)
                )
            );
        } else {
            return null; 
        }
    }
}
