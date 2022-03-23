package frc.robot.commands.auto.routines;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathFinder;
import frc.robot.commands.indexer.AutoFeed;
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
                this.cargo = List.of(
                    FieldConstants.MIDDLE_LEFT_CARGO_BLUE, 
                    FieldConstants.TERMINAL_CARGO_BLUE); 
                break; 
            case FieldConstants.BOTTOM_BLUE_TARMAC:
                this.cargo = List.of(
                    FieldConstants.TARMAC_BOTTOM_LEFT_1_REFERENCE,
                    FieldConstants.MIDDLE_LEFT_CARGO_BLUE, 
                    FieldConstants.TERMINAL_CARGO_BLUE); 
                break; 
            case FieldConstants.BOTTOM_RED_TARMAC:
                this.cargo = List.of(
                    FieldConstants.MIDDLE_RIGHT_CARGO_RED, 
                    FieldConstants.TERMINAL_CARGO_RED
                );
                break; 
            case FieldConstants.TOP_RED_TARMAC:
                this.cargo = List.of(
                    FieldConstants.TARMAC_TOP_RIGHT_1_REFERENCE,
                    FieldConstants.MIDDLE_RIGHT_CARGO_RED,
                    FieldConstants.TERMINAL_CARGO_RED 
                ); 
                break;
        }
    } 

    public ParallelRaceGroup getAutoCommand() {
        return new ParallelRaceGroup(
            new AutoIntake(intakeSubsystem, AutoConstants.AUTO_INTAKE_SPEED), 
            new PrepareShooterPID(shooter, shooterRPM), 
            new SequentialCommandGroup(
                // Shoot preload. 
                new AutoFeed(shooterFeeder), 
                // Drive to retrieve two cargo. TODO: implement command for two ball shooting 
                new PathFinder(drivetrain, this.cargo).getAutoCommand() 
                
            )
        );
    }
}