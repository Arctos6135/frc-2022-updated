package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Performs ball intake without any driver controller input. 
 */
public class AutoIntake extends CommandBase {
    
    private final IntakeSubsystem intakeSubsystem; 
    private final double speed; 

    public double lastIntakeRPM = 0; 
    public double lastTime = 0; 

    // Whether the intake subsystem dipped in acceleration. 
    public boolean ballIntake = false; 
    public boolean finished = false; 

    /**
     * Creates a new autonomous intake command. 
     * Unlike {@link Intake.java}, this command does not use controller input.
     *
     * @param intakeSubsystem the intake system with motors controlling mecanum wheels.
     * @param speed the speed of the rollers.
     * @param raiseArmEnd whether to raise the intake arm at the end of the command.
     */ 
    public AutoIntake(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed; 

        addRequirements(intakeSubsystem);
    }

    @Override 
    public void initialize() {
        intakeSubsystem.setMecanumWheelMotor(speed);
    }

    @Override 
    public void execute() {
        /* double currentIntakeRPM = intakeSubsystem.getMecanumWheelVelocity();
        double currentTime = Timer.getFPGATimestamp();

        // Change in velocity divided by change in time.
        double acceleration = (currentIntakeRPM - lastIntakeRPM) / (currentTime - lastTime); 

        // Ball is rubbing against the mecanum wheels. 
        if (acceleration < 0) {
            ballIntake = true; 
        } 
        // Mecanum wheels are returning to regular speed. 
        else if (ballIntake && acceleration > 0) {
            finished = true; 
        }

        lastIntakeRPM = currentIntakeRPM; 
        lastTime = currentTime; */ 
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSubsystem.setMecanumWheelMotor(0);
    }

    @Override 
    public boolean isFinished() {
        return finished; 
    }
}
