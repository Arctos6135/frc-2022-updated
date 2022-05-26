package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class TwoBallExitAuto {
    private final Drivetrain drivetrain; 
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeeder; 
    private final IntakeSubsystem intakeSubsystem; 

    public static final double shooterTargetRPM = 4250.0;
    public static final double driveForwardSpeed = 0.75;  
    public static final double moveArmSpeed = 0.4; 

    // Intake ball for all of autonomous. 
    public Command intakeBall;
    public boolean intakeBallFinished = false;

    // Drive forwards to move the arm down. 
    public Command moveArm; 
    public double initialMoveArmTime; 
    public static double moveArmTime = 0.20; 
    public boolean moveArmFinished = false; 

    // Drive backwards to reset position. 
    public Command resetPosition; 
    public double initialResetPositionTime; 
    public static double resetPositionTime = 0.35; 
    public boolean resetPositionFinished = false;
    public static double resetPositionSpeed = -0.25;

    // Set shooter RPM. 
    public Command setShooterRPM;
    public boolean setShooterRPMFinished = false; 
    public static double setShootRPMTime = 15.0; 
    public double initialSetShooterRPMTime; 

    public Command pause; 
    public boolean pauseFinished = false;
    public static double pauseTime = 1.0;
    public double initialPause;

    // Roll ball up to shooter. 
    public Command feedShooter; 
    public double initialFeedShooterTime; 
    public static double feedShooterTime = 2.5;
    public boolean feedShooterFinished = false; 

    // Roll ball up to shooter. 
    public Command feedSecondShooter; 
    public double initialFeedSecondShooterTime; 
    public static double feedSecondShooterTime = 4.0;
    public boolean feedSecondShooterFinished = false; 

    // Drive backwards off the tarmac to retrieve ball. 
    public Command driveBackwards; 
    public double initialDriveBackwardsTime; 
    public boolean driveBackwardsFinished = false; 
    public static double driveBackwardsTime = 1.5;
    public static double driveBackwardsSpeed = -0.125;
    public static double driveBackwardsRotation = 0.125; 

    public Command pauseDrive; 
    public double initialPauseDriveTime; 
    public boolean pauseDriveFinished = false; 
    public static double pauseDriveTime = 1.0;
    public double pauseDriveRollSpeed = 0.5;

    // Drive back to shooting spot. 
    public Command driveToShoot; 
    public double initialDriveToShootTime; 
    public boolean driveToShootFinished = false; 
    public static double driveToShootTime = 1.75; 
    public static double driveToShootSpeed = 0.125; 
    public static double driveToShootRotation = -0.125;

    public Command pauseToShoot; 
    public double initialPauseToShoot; 
    public boolean pauseToShootFinished = false; 
    public static double pauseToShootTime = 1.0;

    public Command stopShooter; 
    public boolean stopShooterFinished = false; 
    public double initialStopShooter;
    
    /**
     * Creates a new instance of the Two Ball Auto Command. 
     * 
     * @param drivetrain the driving subsystem. 
     * @param shooter the shooter. 
     * @param shooterFeeder the shooter rollers. 
     * @param intakeSubsystem the intake subsystem. 
     */
    public TwoBallExitAuto(Drivetrain drivetrain, Shooter shooter, ShooterFeederSubsystem shooterFeeder, 
        IntakeSubsystem intakeSubsystem) {
        this.drivetrain = drivetrain; 
        this.shooter = shooter;
        this.shooterFeeder = shooterFeeder; 
        this.intakeSubsystem = intakeSubsystem; 

        this.intakeBall = new FunctionalCommand(() -> {
            this.intakeSubsystem.runIntake(AutoConstants.AUTO_INTAKE_ROLLER_SPEED, AutoConstants.AUTO_MECANUM_SPEED);
        }, () -> {

        }, (interrupted) -> {
            this.intakeSubsystem.runIntake(0, 0);
        }, () -> this.intakeBallFinished, this.intakeSubsystem);

        this.moveArm = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(moveArmSpeed, 0);
            this.initialMoveArmTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialMoveArmTime >= moveArmTime) {
                this.moveArmFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(moveArmSpeed, 0);
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.moveArmFinished, this.drivetrain);

        this.resetPosition = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(resetPositionSpeed, 0); 
            this.initialResetPositionTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialResetPositionTime >= resetPositionTime) {
                this.resetPositionFinished = true;
            } else {
                this.drivetrain.arcadeDrive(resetPositionSpeed, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0); 
        }, () -> this.resetPositionFinished, this.drivetrain);

        this.pause = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(0, 0);
            this.initialPause = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialPause >= pauseTime) {
                this.pauseFinished = true;
            } else {
                this.drivetrain.arcadeDrive(0, 0);
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.pauseFinished, this.drivetrain); 

        this.setShooterRPM = new FunctionalCommand(() -> {
            this.shooter.setVelocity(shooterTargetRPM);
            this.initialSetShooterRPMTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialSetShooterRPMTime >= setShootRPMTime) {
                this.setShooterRPMFinished = true;
            } 
        }, (interrupted) -> {
            this.shooter.stopShooter();
        }, () -> this.setShooterRPMFinished, this.shooter); 

        this.driveBackwards = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(driveBackwardsSpeed, driveBackwardsRotation);
            this.initialDriveBackwardsTime = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveBackwardsTime >= driveBackwardsTime) {
                this.driveBackwardsFinished = true; 
            } else { 
                this.drivetrain.arcadeDrive(driveBackwardsSpeed, driveBackwardsRotation); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.driveBackwardsFinished, this.drivetrain); 

        this.pauseDrive = new FunctionalCommand(() -> {
            this.initialPauseDriveTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialPauseDriveTime >= pauseDriveTime) {
                this.pauseDriveFinished = true; 
                this.shooterFeeder.setRollSpeed(pauseDriveRollSpeed);
            } else {
                this.drivetrain.arcadeDrive(0, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0); 
            this.shooterFeeder.setRollSpeed(0);
        } , () -> this.pauseDriveFinished, this.drivetrain); 

        this.driveToShoot = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(driveToShootSpeed, driveToShootRotation); 
            this.initialDriveToShootTime = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveToShootTime >= driveToShootTime) {
                this.driveToShootFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(driveToShootSpeed, driveToShootRotation); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0); 
        }, () -> this.driveToShootFinished, this.drivetrain); 

        this.pauseToShoot = new FunctionalCommand(() -> {
            this.initialPauseToShoot = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialPauseToShoot >= pauseToShootTime) {
                this.pauseToShootFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(0, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        } , () -> this.pauseToShootFinished, this.drivetrain); 

        this.feedShooter = new FunctionalCommand(() -> {
            this.shooterFeeder.setRollSpeed(AutoConstants.AUTO_ROLL_SPEED);
            this.initialFeedShooterTime = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialFeedShooterTime >= feedShooterTime) {
                this.feedShooterFinished = true; 
            } else {
                this.shooterFeeder.setRollSpeed(AutoConstants.AUTO_ROLL_SPEED);
            }
        }, (interrupted) -> {
            this.shooterFeeder.setRollSpeed(0); 
        }, () -> this.feedShooterFinished, this.shooterFeeder);

        this.feedSecondShooter = new FunctionalCommand(() -> {
            this.shooterFeeder.setRollSpeed(AutoConstants.AUTO_ROLL_SPEED);
            this.initialFeedSecondShooterTime = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialFeedSecondShooterTime >= feedSecondShooterTime) {
                this.feedSecondShooterFinished = true; 
            } else {
                this.shooterFeeder.setRollSpeed(AutoConstants.AUTO_ROLL_SPEED);
            }
        }, (interrupted) -> {
            this.shooterFeeder.setRollSpeed(0); 
            this.shooter.setVelocity(0);
        }, () -> this.feedSecondShooterFinished, this.shooterFeeder);

    }

    public Command getAutoCommand() {
        return new ParallelRaceGroup(
            this.intakeBall, 
            this.setShooterRPM,
            new SequentialCommandGroup(
                this.moveArm,
                this.resetPosition,
                this.pause,
                this.feedShooter,
                this.driveBackwards, 
                this.pauseDrive,
                this.driveToShoot,
                this.pauseToShoot, 
                this.feedSecondShooter
            )
        ); 
    }
}
