package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

/**
 * Use Limelight vision system to align with the vision tape on the high hub. 
 * Distance is dealt with separately, in the {@link DistanceAim} command.
 */
public class AutoAim extends CommandBase {
    private final Drivetrain drivetrain; 
    private final Shooter shooter; 

    public static double kP = -0.1; 
    public static double min_command = 0.05; 
    
    public double tx;

    /**
     * Create a new Limelight aiming command. 
     * 
     * @param drivetrain the robot drivetrain. 
     * @param shooter the robot shooter. 
     */
    public AutoAim(Drivetrain drivetrain, Shooter shooter) {
        this.drivetrain = drivetrain; 
        this.shooter = shooter;
        addRequirements(drivetrain, shooter);
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        this.tx = shooter.getLimelight().getHorizontalAngle();
        double heading_error = -this.tx; 
        double steering_adjust = 0;

        if (this.tx > 1.0) {
            steering_adjust = kP * heading_error - min_command;
        } else if (this.tx < 1.0) {
            steering_adjust = kP * heading_error + min_command;
        }

        this.drivetrain.setMotors(steering_adjust, -steering_adjust);
    }

    @Override 
    public void end(boolean interrupted) {
        this.drivetrain.arcadeDrive(0, 0); 
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}
