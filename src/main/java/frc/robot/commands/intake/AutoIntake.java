package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Performs ball intake without any driver controller input. 
 */
public class AutoIntake extends CommandBase {
    
    private final IntakeSubsystem intakeSubsystem; 
    private final double speed; 
    private final boolean raiseArmEnd;  

    /**
     * Creates a new autonomous intake command. 
     * Unlike {@link Intake.java}, this command does not use controller input.
     *
     * @param intakeSubsystem the intake system with motors controlling mecanum wheels.
     * @param speed the speed of the rollers.
     * @param raiseArmEnd whether to raise the intake arm at the end of the command.
     */
    public AutoIntake(IntakeSubsystem intakeSubsystem, double speed, boolean raiseArmEnd) {
        this.intakeSubsystem = intakeSubsystem; 
        this.speed = speed; 
        this.raiseArmEnd = raiseArmEnd; 

        addRequirements(intakeSubsystem);
    }

    @Override 
    public void initialize() {
        intakeSubsystem.setMecanumWheelMotor(speed);
        intakeSubsystem.setIntakeArmPosition(Constants.INTAKE_ARM_LOWERED);
    }

    @Override 
    public void execute() {

    }

    @Override 
    public void end(boolean interrupted) {
        intakeSubsystem.setMecanumWheelMotor(0);

        if (raiseArmEnd) {
            intakeSubsystem.setIntakeArmPosition(Constants.INTAKE_ARM_RAISED);
        } 
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}
