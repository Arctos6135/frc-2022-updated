package frc.robot.commands.auto.routines;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathFinder;
import frc.robot.commands.indexer.AutoFeed;
import frc.robot.commands.indexer.SensoredRoll;
import frc.robot.commands.shooting.PrepareShooterPID;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class TerminalAuto {
    private Pose2d startingPosition; 
    private List<Pose2d> cargo; 
    
    private final Drivetrain drivetrain; 
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeeder; 
    private final IntakeSubsystem intakeSubsystem; 

    private double shooterRPM;

    private Command terminalAuto; 
    private Command autoIntake; 

    /**
     * Create a new autonomous routine with one process going to the terminal. 
     * 
     * @param drivetrain the driving subsystem. 
     * @param shooter the shooting subsystem.
     * @param shooterFeeder the roller motor subsystem.
     * @param intakeSubsystem the intake subsystem. 
     * @param tarmac the starting tarmac position.
     * @param lowHub whether to shoot low or high hub. 
     */
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
                this.startingPosition = FieldConstants.FENDER_1;
                this.cargo = List.of(
                    FieldConstants.MIDDLE_LEFT_CARGO_BLUE, 
                    FieldConstants.TERMINAL_CARGO_BLUE); 
                break; 
            case FieldConstants.BOTTOM_BLUE_TARMAC:
                this.startingPosition = FieldConstants.FENDER_2; 
                this.cargo = List.of(
                    FieldConstants.TARMAC_BOTTOM_LEFT_1_REFERENCE,
                    FieldConstants.MIDDLE_LEFT_CARGO_BLUE, 
                    FieldConstants.TERMINAL_CARGO_BLUE); 
                break; 
            case FieldConstants.BOTTOM_RED_TARMAC:
                this.startingPosition = FieldConstants.FENDER_3; 
                this.cargo = List.of(
                    FieldConstants.MIDDLE_RIGHT_CARGO_RED, 
                    FieldConstants.TERMINAL_CARGO_RED
                );
                break; 
            case FieldConstants.TOP_RED_TARMAC:
                this.startingPosition = FieldConstants.FENDER_4;
                this.cargo = List.of(
                    FieldConstants.TARMAC_TOP_RIGHT_1_REFERENCE,
                    FieldConstants.MIDDLE_RIGHT_CARGO_RED,
                    FieldConstants.TERMINAL_CARGO_RED 
                ); 
                break;
        }

        this.autoIntake = new FunctionalCommand(() -> {
            this.intakeSubsystem.runIntake(AutoConstants.AUTO_INTAKE_ROLLER_SPEED, AutoConstants.AUTO_MECANUM_SPEED);
        }, () -> {

        }, (interrupted) -> {
            this.intakeSubsystem.runIntake(0, 0);
        }, () -> false, 
        this.intakeSubsystem);

        this.terminalAuto = new SequentialCommandGroup(
            // Shoot preload.
            new AutoFeed(this.shooterFeeder), 
            // Drive to retrieve first cargo from fender. 
            new PathFinder(this.drivetrain, this.startingPosition, null, this.cargo.get(1)).getAutoCommand(), 
            // Intake the ball, roll up the shooter feeder. 
            new SensoredRoll(this.shooterFeeder), 
            // Drive to fender for shooting. 
            new PathFinder(this.drivetrain, this.cargo.get(1), null, this.startingPosition).getAutoCommand(),
            // Shoot the ball.
            new AutoFeed(this.shooterFeeder), 
            // Drive to retrieve second cargo from terminal.
            new PathFinder(this.drivetrain, this.cargo), 
            // Intake the ball, roll up the shooter feeder. 
            new SensoredRoll(this.shooterFeeder), 
            // Drive to fender from terminal.
            new PathFinder(this.drivetrain, this.cargo.get(2), List.of(this.cargo.get(1).getTranslation()), this.startingPosition).getAutoCommand(), 
            // Shoot the ball. 
            new AutoFeed(this.shooterFeeder)
        );
    } 

    /**
     * Get the terminal autonomous routine. 
     * 
     * @return the autonomous routine, as a {@link edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup}. 
     */
    public Command getAutoCommand() {
        return new ParallelDeadlineGroup(this.terminalAuto, this.autoIntake,
            new PrepareShooterPID(shooter, shooterRPM)
        );
    }
}
