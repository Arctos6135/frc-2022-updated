package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.util.Limelight;

public class AutoPIDShoot extends CommandBase {
    private final Shooter shooter; 
    private final ShooterFeederSubsystem shooterFeeder; 
    private final Drivetrain drivetrain; 
    private final Limelight limelight; 

    private boolean upperHub; 
    private boolean rpmReached; 
    private double targetVelocity; 
    private boolean aligned; 
    private boolean firstShot = true; 
    private boolean finished = false; 

    private double initialShootingTime; 
    private final double shootingDuration = 2.00; 

    // PID control for aiming. 
    private final double drivetrainKp = 0.1; 
    private final double minPower = 0.05; 

    public AutoPIDShoot(Shooter shooter, ShooterFeederSubsystem shooterFeeder, Drivetrain drivetrain, Limelight limelight, boolean upperHub) {
        this.shooter = shooter; 
        this.shooterFeeder = shooterFeeder; 
        this.drivetrain = drivetrain; 
        this.limelight = limelight; 

        this.upperHub = upperHub; 
        this.rpmReached = false; 

        addRequirements(shooter, shooterFeeder, drivetrain);
    }   

    @Override 
    public void initialize() {
        if (this.upperHub) { 
            shooter.setVelocity(Constants.HIGH_HUB_RPM);
            this.targetVelocity = Constants.HIGH_HUB_RPM; 
        } else {
            shooter.setVelocity(Constants.LOW_HUB_RPM);
            this.targetVelocity = Constants.LOW_HUB_RPM; 
        }
    }

    @Override 
    public void execute() {
        if (!rpmReached) {
            if (Math.abs(shooter.getActualVelocity() - this.targetVelocity) <= Shooter.VELOCITY_TOLERANCE) {
                rpmReached = true; 
            }
        }
        
        if (!aligned) {
            double headingError = limelight.getHorizontalAngle();
            if (headingError > 1.0) {
                drivetrain.setMotors(
                    drivetrainKp * headingError - minPower, 
                    drivetrainKp * headingError + minPower);
            } else if (headingError < 1.0) {
                drivetrain.setMotors(
                    drivetrainKp * headingError + minPower, 
                    drivetrainKp * headingError - minPower);
            } else {
                aligned = true; 
            }
        }

        if (firstShot && (aligned && rpmReached)) {
            shooterFeeder.setRollSpeed(Constants.ROLL_SPEED);
            initialShootingTime = Timer.getFPGATimestamp(); 
            firstShot = false; 
        } else if (aligned && rpmReached) {
            if (Timer.getFPGATimestamp() - initialShootingTime >= shootingDuration) {
                finished = true; 
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished; 
    }

    @Override 
    public void end(boolean interrupted) {
        this.shooterFeeder.setRollSpeed(0);
        this.shooter.setVelocity(0);
        this.drivetrain.arcadeDrive(0, 0);
    }
}
