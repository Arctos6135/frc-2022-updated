package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Spins the motor controlling the mecanum wheel to intake or outtake a ball.
 * The intake arm is controlled separately by a joystick on the operator controller. 
 */
public class Intake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final XboxController controller; 
    private final int forwardButton;
    private final int reverseButton;
   
    public Intake(IntakeSubsystem intakeSubsystem, XboxController controller, int forwardButton, int reverseButton) {
        this.intakeSubsystem = intakeSubsystem;
        this.controller = controller;
        this.forwardButton = forwardButton;
        this.reverseButton = reverseButton;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        boolean forward = controller.getRawButton(forwardButton);
        boolean reverse = controller.getRawButton(reverseButton);

        if (forward & !reverse) {
            intakeSubsystem.setMecanumWheelMotor(1.0);
        } else if (!forward & reverse) {
            intakeSubsystem.setMecanumWheelMotor(-1.0);
        } else {
            intakeSubsystem.setMecanumWheelMotor(0);
        }
    }

    @Override
    public void initialize() {
        
    }
   
    @Override
    public void end(boolean interrupted) { 
        intakeSubsystem.setMecanumWheelMotor(0);
    }
   
    @Override
    public boolean isFinished() {
        return false;
    }
   
}
