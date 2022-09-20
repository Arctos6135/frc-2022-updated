package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Spins the motor controlling the mecanum wheel to intake or outtake a ball.
 * The intake arm is controlled separately by a joystick on the operator controller. 
 */
public class Intake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final XboxController controller; 
    private final int intakeButton;
    private final int outtakeButton;
   
    public Intake(IntakeSubsystem intakeSubsystem, XboxController controller, int intakeButton, int outtakeButton) {
        this.intakeSubsystem = intakeSubsystem;
        this.controller = controller;
        this.intakeButton = intakeButton;
        this.outtakeButton = outtakeButton;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        boolean intake = controller.getRawButton(intakeButton);
        boolean outtake = controller.getRawButton(outtakeButton);

        if (intake & !outtake) {
            intakeSubsystem.runIntake(Constants.BOTTOM_INTAKE_ROLLER_SPEED, Constants.MECANUM_INTAKE_SPEED); 
        } else if (!intake & outtake) {
            intakeSubsystem.runIntake(-Constants.BOTTOM_INTAKE_ROLLER_SPEED, -Constants.MECANUM_INTAKE_SPEED);
        } else {
            intakeSubsystem.runIntake(0, 0);
        }
    }

    @Override
    public void initialize() {
        
    }
   
    @Override
    public void end(boolean interrupted) { 
        intakeSubsystem.setMecanumWheelMotor(0);
        intakeSubsystem.runIntake(0, 0);
    }
   
    @Override
    public boolean isFinished() {
        return false;
    }
   
}
