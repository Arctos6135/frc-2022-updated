package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/**
 * The intake subsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
    // No Pneumatics

    // Motors to Spin Mecanum Wheels 
    private final CANSparkMax mecanumWheelMotor;
    private final RelativeEncoder mecanumWheelEncoder; 
 
    private IdleMode idleMode;

    /**
     * Set the speed of the intake motors.
     *
     * @param scale the speed of the motors.
     */
    public void setMecanumWheelMotor(double scale) {
        mecanumWheelMotor.set(scale); 
    }

    /**
     * Get the speed of the intake motors.
     * 
     * @return the RPM of the intake mecanum wheels.
     */
    public double getMecanumWheelVelocity() {
        return this.mecanumWheelEncoder.getVelocity();
    }

    /**
     * Get the relative encoder of the intake subsystem. 
     * 
     * @return the intake encoder. 
     */
    public RelativeEncoder getMecanumWheelEncoder() {
        return this.mecanumWheelEncoder;
    }

    /**
     * Get the Idle Mode of the Intake motors (brake/coast).
     *
     * @return the intake motor idle mode.
     */
    public IdleMode getIntakeIdleMode() {
        return this.idleMode;
    }
   
    /**
     * Set the Idle Mode of the Intake motors (brake/coast).
     *
     * @param idleMode the idle mode.
     */
    public void setMecanumWheelMotorMode(IdleMode idleMode) {
        this.idleMode = idleMode;
        this.mecanumWheelMotor.setIdleMode(this.idleMode); 
    }
   
    /**
     * Create a new intake subsystem.
     * 
     * The PID controller used by the intake arm is built in to the Spark Max motor used for 
     * the intake arm. 
     *
     * @param leftIntake the CAN ID of the left intake motor controller.
     * @param rightIntake the CAN ID of the right intake motor controller.
     */
    public IntakeSubsystem(int mecanumWheelMotor) {
        this.mecanumWheelMotor = new CANSparkMax(mecanumWheelMotor, MotorType.kBrushless);
        this.mecanumWheelEncoder = this.mecanumWheelMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.COUNTS_PER_REVOLUTION); 
       
        // Invert the left motor to spin in the same direction.
        this.mecanumWheelMotor.setInverted(true);

        this.mecanumWheelMotor.stopMotor();
    }
}
