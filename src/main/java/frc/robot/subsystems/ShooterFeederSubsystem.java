package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFeederSubsystem extends SubsystemBase {
    
    private final CANSparkMax rollerMotor;
    private final ColorSensorV3 colorSensor; 

    private double rollSpeed = 0; 
    private boolean rollUpwards = true; 
    private boolean ballInShotPosition = false; 
    private int ballCount = 0; 

    public ShooterFeederSubsystem(int rollerMotor) {
        this.rollerMotor = new CANSparkMax(rollerMotor, MotorType.kBrushless);

        this.rollerMotor.setIdleMode(IdleMode.kBrake); 

        this.colorSensor = new ColorSensorV3(I2C.Port.kOnboard); 
        this.rollSpeed = Constants.ROLL_SPEED; 
        this.ballCount = 0; 
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
     * Start rolling the flat belts to move balls upwards. 
     * 
     * @param rollSpeed speed of the roller motors. 
     */
    public void startRoller() {
        if (rollUpwards) {
            rollerMotor.set(this.rollSpeed); 
        } else {
            rollerMotor.set(-this.rollSpeed); 
        }
    }

    /**
     * Stops rolling the flat belt motors. 
     */
    public void stopRoller() {
        rollerMotor.stopMotor();
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
