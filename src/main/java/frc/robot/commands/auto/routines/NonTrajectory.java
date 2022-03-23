package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

/**
 * Drive forwards, shoot preloaded ball, drive off tarmac. 
 */
public class NonTrajectory {
    private final Drivetrain drivetrain;
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeeder;
    
    private double initialDriveForwardsTime; 
    private boolean driveForwardsFinished = false;

    private double targetShooterRPM;
    private boolean shootingFinished = false; 

    private double initialRollUpTime; 
    private boolean rollUpFinished = false; 

    private double initialDriveBackwardsTime; 
    private boolean driveBackwardsFinished = false; 

    public Command driveForwards; 
    public Command setShooterRPM; 
    public Command rollBallUp; 
    public Command driveBackwards;

    public NonTrajectory(Drivetrain drivetrain, Shooter shooter, ShooterFeederSubsystem shooterFeeder) {
        this.drivetrain = drivetrain; 
        this.shooter = shooter; 
        this.shooterFeeder = shooterFeeder; 

        this.targetShooterRPM = Constants.LOW_HUB_RPM; 

        // Drive towards fender. 
        driveForwards = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(1, 0, 0.5);
            this.initialDriveForwardsTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveForwardsTime >= 2.5) {
                this.driveForwardsFinished = true; 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0, 0); 
        }, () -> this.driveForwardsFinished, drivetrain); 

        // Set the RPM of the shooter for all of autonomous. 
        setShooterRPM = new FunctionalCommand(() -> {
            this.shooter.setVelocity(this.targetShooterRPM);
        }, () -> {
            if (Math.abs(this.targetShooterRPM - shooter.getActualVelocity()) <= Shooter.VELOCITY_TOLERANCE) {
                this.shootingFinished = true; 
            } 
        }, (interrupted) -> {
            shooter.setVelocity(0);
        }, () -> this.shootingFinished, this.shooter); 

        // Roll balls up to shooter. 
        rollBallUp = new FunctionalCommand(() -> {
            this.initialRollUpTime = Timer.getFPGATimestamp();
            this.shooterFeeder.setRollSpeed(Constants.ROLL_SPEED); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialRollUpTime > 1.00) {
                this.rollUpFinished = true; 
            }
        }, (interrupted) -> {
            this.shooterFeeder.setRollSpeed(0); 
        }, () -> this.rollUpFinished, this.shooterFeeder);
        
        // Drive backwards off tarmac. 
        driveBackwards = new FunctionalCommand(() -> {
            this.initialDriveBackwardsTime = Timer.getFPGATimestamp(); 
            this.drivetrain.arcadeDrive(-1.0, 0, 0.5); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveBackwardsTime >= 4.00) {
                this.driveBackwardsFinished = true; 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0, 0); 
        }, () -> this.driveBackwardsFinished, this.drivetrain); 
    }
    
    public ParallelRaceGroup getAutoCommand() {
        return new ParallelRaceGroup(
            this.setShooterRPM, 
            new SequentialCommandGroup(
                this.driveForwards, 
                this.rollBallUp, 
                this.driveBackwards
            )
        ); 
    }
    
}