package frc.robot.util;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A CANPIDController that implements Sendable. Appears as a PID Controller on Shuffleboard. 
 * 
 * Allows for tuning values in the PIDF controller and PID setpoint values to be modified 
 * via the Shuffleboard. 
 */
public class SendableCANPIDController implements Sendable {
    
    private SparkMaxPIDController controller;
    private double setpoint = 0; 
    private boolean enabled = false; 

    public SendableCANPIDController(SparkMaxPIDController controller) {
        this.controller = controller; 
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        
        builder.addDoubleProperty("p", controller::getP, controller::setP);
        builder.addDoubleProperty("i", controller::getI, controller::setI); 
        builder.addDoubleProperty("d", controller::getD, controller::setD);
        builder.addDoubleProperty("f", controller::getFF, controller::setFF); 
        
        builder.addDoubleProperty("setpoint", () -> {
            return this.setpoint; 
        }, (setpoint) -> {
            this.setpoint = setpoint; 

            if (enabled) {
                this.controller.setReference(setpoint, CANSparkMax.ControlType.kVelocity); 
            }
        });

        builder.addBooleanProperty("enabled", () -> {
            return this.enabled; 
        }, (enabled) -> {
            this.enabled = enabled; 

            if (enabled) {
                this.controller.setReference(setpoint, CANSparkMax.ControlType.kVelocity); 
            } else {
                controller.setReference(0, CANSparkMax.ControlType.kVelocity);
            }
        }); 
    } 
}
