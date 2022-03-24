package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Performs ball intake without any driver controller input. 
 * Intake will go for 3 seconds. 
 */
public class AutoIntake extends CommandBase {
    
    private final IntakeSubsystem intakeSubsystem; 
    private final double speed; 

    public double initialIntakeTime = 0; 
    public static double autoIntakeTime = 3.0;

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
        intakeSubsystem.runIntake(this.speed, this.speed); 
        this.initialIntakeTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute() {
        if (Timer.getFPGATimestamp() - this.initialIntakeTime >= AutoIntake.autoIntakeTime) {
            this.finished = true; 
        }
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSubsystem.runIntake(0, 0);
    }

    @Override 
    public boolean isFinished() {
        return finished; 
    }
}
