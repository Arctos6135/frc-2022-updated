package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    // Field Dimensions 
    public static final double FIELD_LENGTH = Units.feetToMeters(54); 
    public static final double FIELD_WIDTH = Units.feetToMeters(27); 

    public static final Rotation2d CENTER_LINE_ANGLE = Rotation2d.fromDegrees(66.0); 
    public static final Translation2d CENTER_OF_HUB = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2); 

    // Measurements in Inches
    public static final double TARMAC_DEPTH = 84.75; 
    public static final double TARMAC_WIDTH = 153.0; 
    public static final double TARMAC_FULL_WIDTH = Math.sqrt(Math.pow(TARMAC_WIDTH, 2) / 2.0); // ~ 108.19 inches
    public static final double FENDER_WIDTH = 46.5; 
    public static final double LOWER_EXIT_WIDTH = 13.25; 
    public static final double HUB_WIDTH = 107.0; 
    public static final double HUB_SQUARE_DIAGONAL = HUB_WIDTH - 2 * LOWER_EXIT_WIDTH; // 80.5 inches 
    public static final double HUB_SQUARE_EDGE = Math.sqrt(Math.pow(HUB_SQUARE_DIAGONAL, 2) / 2.0); // ~ 56.92 inches 
    public static final double TARMAC_EDGE = Math.sqrt(2 * Math.pow(TARMAC_DEPTH, 2) - 2 * Math.pow(TARMAC_DEPTH, 2) * Math.cos(45)); // 64.86 inches 
    public static final Rotation2d TARMAC_OCTAGON_ANGLE = Rotation2d.fromDegrees(135.0); 

    // Tarmac and Hub Measurements
    public static final double TARMAC_FULL_WIDTH_METERS = Units.inchesToMeters(TARMAC_FULL_WIDTH); 
    public static final double TARMAC_EDGE_TO_FENDER = Units.inchesToMeters(TARMAC_DEPTH); 
    public static final double TARMAC_DIAMETER = Units.inchesToMeters(2 * TARMAC_FULL_WIDTH + LOWER_EXIT_WIDTH); // along the middle line 
    public static final double TARMAC_DIAMETER_CORNER = Units.inchesToMeters(2 * TARMAC_DEPTH + HUB_SQUARE_EDGE); // ~ 226.42 inches 

    // Rotation References (from hubs and fenders)
    // Tarmac Centers (from top to bottom)
    // Blue Tarmacs
    public static final Rotation2d TARMAC_TOP_LEFT_1_ROTATION = 
        Rotation2d.fromDegrees(180.0).minus(CENTER_LINE_ANGLE).plus(Rotation2d.fromDegrees(360.0 / 16.0));

    public static final Rotation2d TARMAC_TOP_LEFT_2_ROTATION = 
        TARMAC_TOP_LEFT_1_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0)); 

    public static final Rotation2d TARMAC_BOTTOM_LEFT_1_ROTATION = 
        TARMAC_TOP_LEFT_2_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0)); 

    public static final Rotation2d TARMAC_BOTTOM_LEFT_2_ROTATION = 
        TARMAC_BOTTOM_LEFT_1_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0)); 

    // Red Tarmacs
    public static final Rotation2d TARMAC_BOTTOM_RIGHT_1_ROTATION = 
        TARMAC_BOTTOM_LEFT_2_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));

    public static final Rotation2d TARMAC_BOTTOM_RIGHT_2_ROTATION = 
        TARMAC_BOTTOM_RIGHT_1_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));  

    public static final Rotation2d TARMAC_TOP_RIGHT_1_ROTATION = 
        TARMAC_BOTTOM_RIGHT_2_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0)); 

    public static final Rotation2d TARMAC_TOP_RIGHT_2_ROTATION = 
        TARMAC_TOP_RIGHT_1_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0)); 

    // Reference Points (Blue Tarmacs)
    public static final Pose2d TARMAC_TOP_LEFT_1_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_TOP_LEFT_1_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    );

    public static final Pose2d TARMAC_TOP_LEFT_2_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_TOP_LEFT_2_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    ); 

    public static final Pose2d TARMAC_BOTTOM_LEFT_1_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_BOTTOM_LEFT_1_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    ); 

    public static final Pose2d TARMAC_BOTTOM_LEFT_2_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_BOTTOM_LEFT_2_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    );
        
    // Reference Points (Red Tarmacs)
    public static final Pose2d TARMAC_BOTTOM_RIGHT_1_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_BOTTOM_RIGHT_1_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    ); 

    public static final Pose2d TARMAC_BOTTOM_RIGHT_2_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_BOTTOM_RIGHT_2_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    ); 
    
    public static final Pose2d TARMAC_TOP_RIGHT_1_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_TOP_RIGHT_1_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    ); 

    public static final Pose2d TARMAC_TOP_RIGHT_2_REFERENCE = 
        new Pose2d(CENTER_OF_HUB, TARMAC_TOP_RIGHT_2_ROTATION)
        .transformBy(new Transform2d(new Translation2d(TARMAC_DIAMETER / 2.0, 0.0), new Rotation2d())
    ); 
    
    // Fender Rotations (from top to bottom)
    public static final Rotation2d FENDER_1_ROTATION = 
        TARMAC_TOP_LEFT_1_ROTATION.rotateBy(Rotation2d.fromDegrees(360.0 / 16.0)); 

    public static final Rotation2d FENDER_2_ROTATION = 
        FENDER_1_ROTATION.rotateBy(Rotation2d.fromDegrees(90.0)); 

    public static final Rotation2d FENDER_3_ROTATION = 
        FENDER_1_ROTATION.rotateBy(Rotation2d.fromDegrees(180.0)); 

    public static final Rotation2d FENDER_4_ROTATION = 
        FENDER_1_ROTATION.rotateBy(Rotation2d.fromDegrees(270.0)); 

    // Fender Centers 
    public static final Pose2d FENDER_1 = 
        new Pose2d(CENTER_OF_HUB, FENDER_1_ROTATION)
        .transformBy(new Transform2d(new Translation2d(HUB_SQUARE_EDGE / 2.0, 0.0), new Rotation2d())
    ); 

    public static final Pose2d FENDER_2 = 
        new Pose2d(CENTER_OF_HUB, FENDER_2_ROTATION)
        .transformBy(new Transform2d(new Translation2d(HUB_SQUARE_EDGE / 2.0, 0.0), new Rotation2d())
    ); 

    public static final Pose2d FENDER_3 = 
        new Pose2d(CENTER_OF_HUB, FENDER_3_ROTATION)
        .transformBy(new Transform2d(new Translation2d(HUB_SQUARE_EDGE / 2.0, 0.0), new Rotation2d())
    ); 

    public static final Pose2d FENDER_4 = 
        new Pose2d(CENTER_OF_HUB, FENDER_4_ROTATION)
        .transformBy(new Transform2d(new Translation2d(HUB_SQUARE_EDGE / 2.0, 0.0), new Rotation2d())
    ); 

    // Cargo References
    // Cargo are always a set distance from the center of tarmac edges. 
    public static final double CORNER_TO_CARGO = Units.inchesToMeters(15.56); 
    public static final double REFERENCE_TO_CARGO_Y = (TARMAC_FULL_WIDTH_METERS / 2.0) - CORNER_TO_CARGO; 
    public static final double REFERENCE_TO_CARGO_X = Units.inchesToMeters(40.44);

    public static final Pose2d TOP_LEFT_CARGO_RED = TARMAC_TOP_LEFT_1_REFERENCE.transformBy(
        new Transform2d(new Translation2d(REFERENCE_TO_CARGO_X, -REFERENCE_TO_CARGO_Y), new Rotation2d())
    ); 

    public static final Pose2d TOP_LEFT_CARGO_BLUE = TARMAC_TOP_LEFT_1_REFERENCE.transformBy(
        new Transform2d(new Translation2d(REFERENCE_TO_CARGO_X, REFERENCE_TO_CARGO_Y), new Rotation2d())
    );

    public static final Pose2d MIDDLE_CARGO_RED = TARMAC_TOP_LEFT_2_REFERENCE.transformBy(
        new Transform2d(new Translation2d(REFERENCE_TO_CARGO_X, REFERENCE_TO_CARGO_Y), new Rotation2d())
    ); 

    public static final Pose2d MIDDLE_CARGO_BLUE = TARMAC_BOTTOM_LEFT_1_REFERENCE.transformBy(
        new Transform2d(new Translation2d(REFERENCE_TO_CARGO_X, -REFERENCE_TO_CARGO_Y), new Rotation2d())
    ); 

    public static final Pose2d BOTTOM_CARGO_BLUE = TARMAC_BOTTOM_LEFT_2_REFERENCE.transformBy(
        new Transform2d(new Translation2d(REFERENCE_TO_CARGO_X, -REFERENCE_TO_CARGO_Y), new Rotation2d())
    ); 

    public static final Pose2d BOTTOM_CARGO_RED = TARMAC_BOTTOM_LEFT_2_REFERENCE.transformBy(
        new Transform2d(new Translation2d(REFERENCE_TO_CARGO_X, REFERENCE_TO_CARGO_Y), new Rotation2d())
    );

    // Terminal and Cargo Line 
    public static final double DRIVER_STATION_WIDTH = 69.0; // inches 
    public static final double HANGAR_WIDTH = 108.25; // inches
    public static final double TERMINAL_WIDTH_Y = Units.inchesToMeters(324.0 - 2 * DRIVER_STATION_WIDTH - HANGAR_WIDTH); // field length - non-terminal side length
    public static final Rotation2d TERMINAL_ANGLE = Rotation2d.fromDegrees(135.0);
    public static final double TERMINAL_WIDTH_X = Math.tan(
        Rotation2d.fromDegrees(180.0).minus(TERMINAL_ANGLE).getRadians()) * TERMINAL_WIDTH_Y; 
    public static final Pose2d TERMINAL_CENTER = new Pose2d(
        new Translation2d(TERMINAL_WIDTH_X / 2.0, TERMINAL_WIDTH_Y / 2.0), TERMINAL_ANGLE.minus(Rotation2d.fromDegrees(90.0))
    ); 
    public static final double TERMINAL_CARGO_DISTANCE = Units.inchesToMeters(10.43); 
    public static final Pose2d TERMINAL_CARGO = TERMINAL_CENTER.transformBy(
        new Transform2d(new Translation2d(TERMINAL_CARGO_DISTANCE, 0.0), new Rotation2d())
    );
}
