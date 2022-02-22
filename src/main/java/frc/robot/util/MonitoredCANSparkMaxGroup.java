package frc.robot.util;

import java.util.function.BiConsumer;

import com.revrobotics.CANSparkMax;

import frc.robot.RobotContainer;

public class MonitoredCANSparkMaxGroup {
    
    private final double SHUTOFF_TEMPERATURE;
    private final double WARNING_TEMPERATURE;
    
    private CANSparkMax[] motors;
    private final short[] motorFaults;
    
    private String name;
    
    private boolean overheatShutoff = false;
    private boolean overheatWarning = false;
    private BiConsumer<CANSparkMax, Double> overheatShutoffCallback;
    private BiConsumer<CANSparkMax, Double> overheatWarningCallback;
    private Runnable normalTempCallback;
    
    public MonitoredCANSparkMaxGroup(String name, double warningTemp,
            double shutoffTemp, CANSparkMax... motors) {
        this.name = name;
        this.WARNING_TEMPERATURE = warningTemp;
        this.SHUTOFF_TEMPERATURE = shutoffTemp;
        this.motors = motors;
        this.motorFaults = new short[motors.length];
    }
    
    /**
     * Get whether any motors are overheating.
     * 
     * When motors overheat, setMotor will be disabled. 
     * 
     * @return boolean indicating if the motors are overheating.
     */
    public boolean getOverheatShutoff() {
        return overheatShutoff;
    }
    
    /**
     * Get whether any motors have given overheating warnings. 
     * 
     * @return boolean indicating if the motors are overheating. 
     */
    public boolean getOverheatWarning() {
        return overheatWarning;
    }
    
    /**
     * Set the overheat shutoff callback. 
     * 
     * <p>
     * In the BiConsumer, the first argument is the overheating motor, and the
     * second is the temperature of the motor.  
     * </p>
     * 
     * @param callback a callback with the overheating motor and its temperature.
     */
    public void setOverheatShutoffCallback(BiConsumer<CANSparkMax, Double> callback) {
        this.overheatShutoffCallback = callback;
    }

    /**
     * Set the overheat warning callback.
     * 
     * <p>
     * In the BiConsumer, the first argument is the overheating motor, and the
     * second is the temperature of the motor.
     * </p>
     * 
     * @param callback a callback with the overheating motor and its temperature.
     */
    public void setOverheatWarningCallback(BiConsumer<CANSparkMax, Double> callback) {
        this.overheatWarningCallback = callback;
    }

    /**
     * Set the callback for when the motor temperatures are normal again. 
     * 
     * @param callback a callback, Runnable, which can be run within a thread when
     *                 the temperature goes back to normal.
     */
    public void setNormalTempCallback(Runnable callback) {
        this.normalTempCallback = callback; 
    }
    
    /**
     * Monitor the monitors once.
     */
    public void monitorOnce() {
        boolean shutoff = false;
        boolean warning = false;
        
        // Check each motor. 
        for (var i = 0; i < motors.length; i++) {
            CANSparkMax motor = motors[i];

            // Check each motor for faults.
            short faults = motor.getFaults();

            if (faults != motorFaults[i] && faults != 0) {
                RobotContainer.getLogger().logError(name + " motor " + motor.getDeviceId() + " had faults " + faults);
            }

            motorFaults[i] = faults;

            // Check all motors for temperature.
            double temp = motor.getMotorTemperature();

            if (temp >= SHUTOFF_TEMPERATURE) {
                // Call the callback if the motor was not overheating prior to the check.
                if (!overheatShutoff && overheatShutoffCallback != null) {
                    overheatShutoffCallback.accept(motor, temp);
                }
                shutoff = true;
            } else if (temp >= WARNING_TEMPERATURE) {
                // Call the callback if the motor was not overheating prior to the check.
                if (!overheatWarning && overheatWarningCallback != null) {
                    overheatWarningCallback.accept(motor, temp);
                }
                warning = true;
            }
        }
        
        // Run the normal callback upon return to normal temperatures.
        if ((overheatShutoff || overheatWarning) && (!shutoff && !warning) && normalTempCallback != null) {
            normalTempCallback.run();
        }
        
        overheatShutoff = shutoff;
        overheatWarning = warning; 
    }
}
