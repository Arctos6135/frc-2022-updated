package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/**
 * The intake subsystem, with a {@link com.revrobotics.CANSparkMax} controlling the mecanum wheels. 
 * The default command for this subsystem is {@link frc.robot.commands.intake.Intake}. 
 */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX bottomIntakeMotor; 
    private final CANSparkMax mecanumWheelMotor;
    private final RelativeEncoder mecanumWheelEncoder; 
 
    private IdleMode idleMode;

    /**
     * Create a new intake subsystem.
     * 
     * The PID controller used by the intake arm is built in to the Spark Max motor used for 
     * the intake arm. 
     *
     * @param leftIntake the CAN ID of the left intake motor controller.
     * @param rightIntake the CAN ID of the right intake motor controller.
     */
    public IntakeSubsystem(int bottomIntakeMotor, int mecanumWheelMotor) {
        this.bottomIntakeMotor = new TalonSRX(bottomIntakeMotor); 
        this.mecanumWheelMotor = new CANSparkMax(mecanumWheelMotor, MotorType.kBrushless);
        this.mecanumWheelEncoder = this.mecanumWheelMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.COUNTS_PER_REVOLUTION); 
       
        this.bottomIntakeMotor.setNeutralMode(NeutralMode.Brake);
        this.mecanumWheelMotor.setIdleMode(IdleMode.kCoast);

        this.bottomIntakeMotor.setInverted(false);
        this.mecanumWheelMotor.setInverted(true);
    }

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

    public void setIntakeMotor(double speed) {
        this.bottomIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Run intake of the rollers and mecanum wheels.
     * 
     * @param intakeSpeed speed of the bottom intake motor.
     * @param mecanumSpeed speed of the mecanum wheel motors. 
     */
    public void runIntake(double intakeSpeed, double mecanumSpeed) {
        setIntakeMotor(intakeSpeed);
        setMecanumWheelMotor(mecanumSpeed);
    }
}
