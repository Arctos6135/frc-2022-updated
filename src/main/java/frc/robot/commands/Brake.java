package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Brake extends CommandBase {
    
    private final Drivetrain drivetrain;
    private IdleMode startMode;
    
    public Brake(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    
    @Override 
    public void initialize() {
        startMode = drivetrain.getIdleMode();
        drivetrain.setMotorMode(IdleMode.kBrake);
    }
    
    @Override 
    public void execute() {

    }

    @Override 
    public void end(boolean interrupted) {
        drivetrain.setMotorMode(startMode);
    }
    
    // Returns true when command should end. 
    @Override 
    public boolean isFinished() {
        // Command ends when robot is moving very slow.
        return Math.abs(drivetrain.getLeftVelocity()) < 1.0 &&
                Math.abs(drivetrain.getRightVelocity()) < 1.0;
    }
    
}
