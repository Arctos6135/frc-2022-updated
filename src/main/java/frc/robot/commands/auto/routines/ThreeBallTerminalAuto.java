package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;

public class ThreeBallTerminalAuto {
    private final Drivetrain drivetrain; 
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeeder; 
    private final IntakeSubsystem intakeSubsystem; 

    public static final double shooterTargetRPM = 5000.0;
    public static final double shooterTargetRPMHigh = 6000.0; 
    public static final double driveForwardSpeed = 0.75;  

    // Intake ball for all of autonomous. 
    public Command intakeBall;
    public boolean intakeBallFinished = false;

    // Drive backwards off the tarmac to move the arm down. 
    public Command moveArm; 
    public double initialMoveArmTime; 
    public static double moveArmTime = 0.20; 
    public boolean moveArmFinished = false; 

    // Drive forwards to reset position. 
    public Command resetPosition; 
    public double initialResetPositionTime; 
    public static double resetPositionTime = 0.20; 
    public boolean resetPositionFinished = false;

    // Set shooter RPM. 
    public Command setShooterRPM;
    public boolean setShooterRPMFinished = false; 
    public static double setShootRPMTime = 1.00; 
    public double initialSetShooterRPMTime; 

    // Roll ball up to shooter. 
    public Command feedShooter; 
    public double initialFeedShooterTime; 
    public static double feedShooterTime = 2.0;
    public boolean feedShooterFinished = false; 

    // Drive backwards off the tarmac to retrieve ball. 
    public Command driveBackwards; 
    public double initialDriveBackwardsTime; 
    public boolean driveBackwardsFinished = false; 
    public static double driveBackwardsTime = 1.0;
    public static double driveBackwardsSpeed = -0.5;
    public static double driveBackwardsRotation = -0.325; 

    public Command pauseDrive; 
    public double initialPauseDriveTime; 
    public boolean pauseDriveFinished = false; 
    public static double pauseDriveTime = 0.25;

    // Drive back to shooting spot. 
    public Command driveToShoot; 
    public double initialDriveToShootTime; 
    public boolean driveToShootFinished = false; 
    public static double driveToShootTime = 1.0; 
    public static double driveToShootSpeed = 0.5;

    // Set shooter RPM. 
    public Command setSecondShooterRPM;
    public boolean setSecondShooterRPMFinished = false; 
    public static double setSecondShootRPMTime = 2.75; 
    public double initialSetSecondShooterRPMTime; 

    // Roll ball up to shooter. 
    public Command feedSecondBall; 
    public double initialFeedSecondBall;
    public boolean feedSecondBallFinished = false; 
    public static double feedSecondBallTime = 2.0; 

    // Rotate for third cargo (-90 degrees). 
    public Command rotateDrive; 
    public double initialRotateDrive; 
    public boolean rotateDriveFinished = false; 
    public static double rotateDriveTime = 1.0; 
    public double rotationFactor = -0.25;

    public Command driveThirdCargo; 
    public double initialDriveThirdCargo; 
    public boolean driveThirdCargoFinished = false; 
    public static double driveThirdCargoTime = 3.0;
    public double driveThirdCargoSpeed = -0.75;

    // Drive back towards hub.
    public Command driveShootThird; 
    public double initialDriveShootThird; 
    public boolean driveShootThirdFinished = false; 
    public static double driveShootThirdTime = 2.75;

    // Rotate to align with high hub. 
    public Command rotateAlign; 
    public double initialRotateAlign; 
    public boolean rotateAlignFinished = false; 
    public static double rotateAlignTime = 0.5;
    public double rotateAlignSpeed = 0.25;

    public Command setThirdShooterRPM; 
    public boolean setThirdShooterRPMFinished = false; 
    public static double setThirdShooterRPMTime = 3.25;
    public double initialSetThirdShooterRPM;
 
    public Command feedThirdBall; 
    public double initialFeedThirdBall; 
    public static double feedThirdBallTime = 2.0;
    public boolean feedThirdBallFinished = false; 

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
    public ThreeBallTerminalAuto(Drivetrain drivetrain, Shooter shooter, ShooterFeederSubsystem shooterFeeder, 
        IntakeSubsystem intakeSubsystem) {
        this.drivetrain = drivetrain; 
        this.shooter = shooter;
        this.shooterFeeder = shooterFeeder; 
        this.intakeSubsystem = intakeSubsystem; 

        this.intakeBall = new FunctionalCommand(() -> {
            this.intakeSubsystem.runIntake(AutoConstants.AUTO_INTAKE_SPEED, AutoConstants.AUTO_INTAKE_SPEED);
        }, () -> {

        }, (interrupted) -> {
            this.intakeSubsystem.runIntake(0, 0);
        }, () -> this.intakeBallFinished, this.intakeSubsystem);

        this.moveArm = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(driveForwardSpeed, 0);
            this.initialMoveArmTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialMoveArmTime >= moveArmTime) {
                this.moveArmFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(driveForwardSpeed, 0);
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.moveArmFinished, this.drivetrain);

        this.resetPosition = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(driveBackwardsSpeed, 0); 
            this.initialResetPositionTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialResetPositionTime >= resetPositionTime) {
                this.resetPositionFinished = true;
            } else {
                this.drivetrain.arcadeDrive(driveBackwardsSpeed, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0); 
        }, () -> this.resetPositionFinished, this.drivetrain);

        this.setShooterRPM = new FunctionalCommand(() -> {
            this.shooter.setVelocity(shooterTargetRPM);
            this.initialSetShooterRPMTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialSetShooterRPMTime >= setShootRPMTime) {
                this.setShooterRPMFinished = true;
            } 
        }, (interrupted) -> {

        }, () -> this.setShooterRPMFinished, this.shooter); 

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
            this.shooter.setVelocity(0);
        }, () -> this.feedShooterFinished, this.shooterFeeder);

        this.driveBackwards = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(driveBackwardsSpeed, 0);
            this.initialDriveBackwardsTime = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveBackwardsTime >= driveBackwardsTime) {
                this.driveBackwardsFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(driveBackwardsSpeed, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.driveBackwardsFinished, this.drivetrain); 

        this.driveToShoot = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(driveToShootSpeed, 0); 
            this.initialDriveToShootTime = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveToShootTime >= driveToShootTime) {
                this.driveToShootFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(driveToShootSpeed, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0); 
        }, () -> this.driveToShootFinished, this.drivetrain); 

        this.pauseDrive = new FunctionalCommand(() -> {
            this.initialPauseDriveTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialPauseDriveTime >= pauseDriveTime) {
                this.pauseDriveFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(0, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0); 
        } , () -> this.pauseDriveFinished, this.drivetrain); 

        this.setSecondShooterRPM = new FunctionalCommand(() -> {
            this.shooter.setVelocity(shooterTargetRPM);
            this.initialSetSecondShooterRPMTime = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialSetSecondShooterRPMTime >= setSecondShootRPMTime) {
                this.setSecondShooterRPMFinished = true;
            } 

        }, (interrupted) -> {

        }, () -> this.setSecondShooterRPMFinished, this.shooter); 

        this.feedSecondBall = new FunctionalCommand(() -> {
            this.shooterFeeder.setRollSpeed(AutoConstants.AUTO_ROLL_SPEED);
            this.initialFeedSecondBall = Timer.getFPGATimestamp();
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialFeedSecondBall >= feedShooterTime) {
                this.feedSecondBallFinished = true; 
            } else {
                this.shooterFeeder.setRollSpeed(AutoConstants.AUTO_ROLL_SPEED);
            }
        }, (interrupted) -> {
            this.shooterFeeder.setRollSpeed(0); 
            this.shooter.setVelocity(0);
        }, () -> this.feedSecondBallFinished, this.shooterFeeder);

        this.rotateDrive = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(0, this.rotationFactor);
            this.initialRotateDrive = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialRotateDrive >= rotateDriveTime) {
                this.rotateDriveFinished = true;
            } else {
                this.drivetrain.arcadeDrive(0, this.rotationFactor); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.rotateDriveFinished, this.drivetrain);

        this.driveThirdCargo = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(this.driveThirdCargoSpeed, 0);
            this.initialDriveThirdCargo = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveThirdCargo >= driveThirdCargoTime) {
                this.driveThirdCargoFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(this.driveThirdCargoSpeed, 0); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.driveThirdCargoFinished, this.drivetrain); 

        this.driveShootThird = new FunctionalCommand(() -> {
            this.initialDriveShootThird = Timer.getFPGATimestamp(); 
            this.drivetrain.arcadeDrive(driveToShootSpeed, 0);
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialDriveShootThird >= driveThirdCargoTime) {
                this.driveShootThirdFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(driveToShootSpeed, 0);
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0);
        }, () -> this.driveShootThirdFinished, this.drivetrain);

        this.rotateAlign = new FunctionalCommand(() -> {
            this.drivetrain.arcadeDrive(0, this.rotateAlignSpeed); 
            this.initialRotateAlign = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialRotateAlign >= rotateAlignTime) {
                this.rotateAlignFinished = true; 
            } else {
                this.drivetrain.arcadeDrive(0, this.rotateAlignSpeed); 
            }
        }, (interrupted) -> {
            this.drivetrain.arcadeDrive(0, 0); 
        }, () ->  this.rotateAlignFinished, this.drivetrain); 

        this.setThirdShooterRPM = new FunctionalCommand(() -> {
            this.shooter.setVelocity(shooterTargetRPMHigh);
            this.initialSetThirdShooterRPM = Timer.getFPGATimestamp(); 
        }, () -> {
            if (Timer.getFPGATimestamp() - this.initialSetThirdShooterRPM > setThirdShooterRPMTime) {
                this.setThirdShooterRPMFinished = true; 
            }
        }, (interrupted) -> {

        }, () -> this.setThirdShooterRPMFinished, this.shooter); 

        this.feedThirdBall = this.feedSecondBall; 

        this.stopShooter = new InstantCommand(() -> {
            this.shooter.setVelocity(0); 
        }, this.shooter);
    }

    public Command getAutoCommand() {
        return new ParallelCommandGroup(
            this.intakeBall, 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        this.moveArm, 
                        this.resetPosition
                    ), 
                    this.setShooterRPM
                ),
                this.feedShooter, 
                this.driveBackwards, 
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        this.pauseDrive,
                        this.driveToShoot
                    ),
                    this.setSecondShooterRPM
                ),
                this.feedSecondBall,
                this.rotateDrive, 
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        this.driveThirdCargo,
                        this.rotateAlign
                    ),
                    this.setThirdShooterRPM
                ), 
                this.feedThirdBall,
                this.stopShooter
            )
        ); 
    }
}