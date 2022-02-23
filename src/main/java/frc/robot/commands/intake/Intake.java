package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Drops the intake arm and spins the motor controlling the mecanum wheel to intake a ball. 
 */
public class Intake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final XboxController controller; // TODO: may need to change to GenericHID
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
            intakeSubsystem.setIntakeArmPosition(Constants.INTAKE_ARM_LOWERED);
        } else if (!forward & reverse) {
            intakeSubsystem.setMecanumWheelMotor(-1.0);
            intakeSubsystem.setIntakeArmPosition(Constants.INTAKE_ARM_RAISED); 
        } else {
            intakeSubsystem.setMecanumWheelMotor(0);
            intakeSubsystem.setIntakeArmPosition(Constants.INTAKE_ARM_RAISED); 
        }
    }

    @Override
    public void initialize() {
        
    }
   
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeArmPosition(Constants.INTAKE_ARM_RAISED); 
        intakeSubsystem.setMecanumWheelMotor(0);
    }
   
    @Override
    public boolean isFinished() {
        return false;
    }
   
}
