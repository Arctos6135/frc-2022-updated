package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeArm;

/**
 * Rotate the arm based on the operator controller input. 
 * Limit Switches on the intake arm restricts the rotation. 
 */
public class RotateArm extends CommandBase {
    private final IntakeArm intakeArm;
    private final XboxController operatorController; 

    private final int Y_AXIS;
    
    private static boolean reverseRotation = false;
    private static boolean precisionRotation = false;
    private static double precisionFactor = 0.5; 
    
    /**
     * Creates a new RotateArm command. 
     * 
     * @param intakeArm the intake arm subsystem. 
     * @param operatorController the operator controller.
     * @param upDownAxis the Y component of the joystick.
     */
    public RotateArm(IntakeArm intakeArm, XboxController operatorController, int upDownAxis) {
        this.intakeArm = intakeArm;
        this.operatorController = operatorController;

        this.Y_AXIS = upDownAxis;

        addRequirements(intakeArm);
    }
    
    /**
     * Get whether the rotation has been reversed. 
     * 
     * @return whether the rotation has been reversed.
     */
    public static boolean isRotationReversed() {
        return reverseRotation;
    }
    
    /**
     * Set whether the rotation should be reversed. 
     * 
     * @param reverseRotation whether to reverse the rotation. 
     */
    public static void setReverseRotation(boolean reverseRotation) {
        RotateArm.reverseRotation = reverseRotation; 
    }

    /**
     * Toggle the intake arm reversal. 
     */
    public static void toggleReverseRotation() {
        RotateArm.reverseRotation = !RotateArm.reverseRotation;
    }
    
    /**
     * Get whether the intake arm is in precision mode. 
     * 
     * @return the precision mode for rotation. 
     */
    public static boolean isPrecisionRotation() {
        return precisionRotation; 
    }

    /**
     * Set whether the intake arm is in precision mode or not. 
     * 
     * @param precisionRotation whether the intake arm is in precision mode. 
     */
    public static void setPrecisionDrive(boolean precisionRotation) {
        RotateArm.precisionRotation = precisionRotation; 
    }

    /**
     * Toggle the intake arm precision mode. 
     */
    public static void togglePrecisionRotation() {
        RotateArm.precisionRotation = !RotateArm.precisionRotation; 
    }

    /**
     * Set the precision factor of the intake arm. 
     * 
     * @param rotationPrecision the precision factor of the intake arm. 
     */
    public static void setPrecisionFactor(double rotationPrecision) {
        RotateArm.precisionFactor = rotationPrecision;
    }
    
    /**
     * Get the precision factor of the intake arm. 
     * 
     * @return the precision factor of the intake arm. 
     */
    public static double getPrecisionFactor() {
        return RotateArm.precisionFactor; 
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        double rotation = TeleopDrive.applyDeadband(
                reverseRotation ? operatorController.getRawAxis(Y_AXIS) : -operatorController.getRawAxis(Y_AXIS),
                Constants.CONTROLLER_DEADZONE);

        rotation = Math.copySign(rotation * rotation, rotation);

        intakeArm.setIntakeArmMotor(precisionRotation ? rotation * precisionFactor : rotation);
    }
    
    @Override 
    public void end(boolean interrupted) {
        intakeArm.stopIntakeArmMotor();
    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
}
