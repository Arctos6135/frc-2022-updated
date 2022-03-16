package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The shooter feeder subsystem (essie) is composed of belts controlled by 
 * a motor and a color sensor approximately halfway up. The shooter feeder
 * subsystem is able to roll balls upwards or store them. 
 * 
 * The default command for this subsystem is {@link frc.robot.commands.indexer.TeleopRoll}. 
 */
public class ShooterFeederSubsystem extends SubsystemBase {
    // Roller Motors 
    private final CANSparkMax topRollerMotor;
    private final TalonSRX bottomRollerMotor;  
    
    private final ColorSensorV3 colorSensor; 

    private double rollSpeed = 0.5; 
    private boolean rollUpwards = true; 
    private boolean ballInShotPosition = false; 
    private int ballCount = 0; 
    public static boolean constantRollSpeed = true; 

    /**
     * Creates a new shooter feeder subsystem (essie). 
     * 
     * @param rollerMotor the roller motor of the belts.
     */
    public ShooterFeederSubsystem(int topRollerMotor, int bottomRollerMotor) {
        this.topRollerMotor = new CANSparkMax(topRollerMotor, MotorType.kBrushless); 
        this.bottomRollerMotor = new TalonSRX(bottomRollerMotor); 

        this.topRollerMotor.setIdleMode(IdleMode.kBrake); 
        this.bottomRollerMotor.setNeutralMode(NeutralMode.Coast); 

        this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    /**
     * Get the color sensor used by the shooter feeder subsystem. 
     * 
     * @return the color sensor used by the shooter feeder subsystem. 
     */
    public ColorSensorV3 getColorSensor() {
        return this.colorSensor; 
    }

    /**
     * Get the color detected by the color sensor. 
     *
     * @return the color detected by the color sensor. 
     */
    public Color getColorDetected() {
        return this.colorSensor.getColor();
    }

    /**
     * Stops rolling the flat belt motors. 
     */
    public void stopRoller() {
        topRollerMotor.stopMotor();
        bottomRollerMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Get the speed that the belts are rolling at. 
     * 
     * @return the speed of the roller motor. 
     */
    public double getRollSpeed() {
        return rollUpwards ? rollSpeed : -rollSpeed; 
    }

    /**
     * Set the speed that the belts are rolling at. 
     * 
     * @param rollSpeed the desired speed of the roller motor. 
     */
    public void setRollSpeed(double rollSpeed) {
        this.rollSpeed = rollSpeed; 

        if (rollUpwards) {
            topRollerMotor.set(this.rollSpeed); 
            bottomRollerMotor.set(ControlMode.PercentOutput, this.rollSpeed);
        } else {
            topRollerMotor.set(-this.rollSpeed); 
            bottomRollerMotor.set(ControlMode.PercentOutput, -this.rollSpeed);
        }
    }

    /**
     * Get the direction the belts are rolling. 
     * 
     * @return the direction of the roller belts. 
     */
    public boolean getRollDirection() {
        return this.rollUpwards; 
    }

    /**
     * Set whether the belts should roll upwards or downwards. 
     * 
     * @param rollUpwards whether the belts should roll upwards.
     */
    public void setRollDirection(boolean rollUpwards) {
        this.rollUpwards = rollUpwards; 
    }

    /**
     * Toggle between rotating the belts upwards and downwards. 
     */
    public void toggleRollDirection() {
        this.rollUpwards = !this.rollUpwards;
    }

    /**
     * Get whether the ball has been sensed by the color sensor and is ready to shoot. 
     * 
     * @return whether there is a ball ready to shoot. 
     */
    public boolean getBallInShotPosition() {
        return this.ballInShotPosition; 
    }

    /**
     * Set whether the ball has been sensed by the color sensor and is ready to shoot. 
     * 
     * @param ballInShotPosition whether there is a ball ready to shoot. 
     */
    public void setBallInShotPosition(boolean ballInShotPosition) {
        this.ballInShotPosition = ballInShotPosition; 
    }

    /**
     * Get the number of balls currently in the roll shaft. 
     * The robot can hold a maximum of 2 balls at a time. 
     * 
     * @return the current number of balls in the roll shaft. 
     */
    public int getBallCount() {
        return this.ballCount; 
    }

    /**
     * Add one ball to the number of balls counted in the roll shaft. 
     */
    public void incrementBallCount() {
        this.ballCount++; 
    }

    /**
     * Subtract one ball to the number of balls counted in the roll shaft. 
     */
    public void decrementBallCount() {
        this.ballCount--; 
    }

}
