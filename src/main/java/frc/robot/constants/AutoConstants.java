package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class AutoConstants {
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0;

    public static final double kPDriveVel = 0;
    public static final double kDDriveVel = 0; 

    public static final double maxVoltage = 10; 

    // Horizontal distance between wheels
    public static final double kTrackWidthMeters = 0;
    
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);
            
    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
    
    public static final double kRamseteB = 2; 
    public static final double kRamseteZeta = 0.7;

    public static final double AUTO_MECANUM_SPEED = 0.5; 
    public static final double AUTO_INTAKE_ROLLER_SPEED = 0.75;
    public static final double AUTO_ROLL_SPEED = 0.5;
    public static final double THREE_BALL_AUTO_ROLL_SPEED = 1.00; 
}
