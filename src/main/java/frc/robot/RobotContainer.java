// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.logging.Level;

import com.arctos6135.robotlib.logging.RobotLogger;
import com.arctos6135.robotlib.newcommands.triggers.AnalogTrigger;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain;

  private static final XboxController driverController = new XboxController(Constants.XBOX_DRIVER); 
  private static final XboxController operatorController = new XboxController(Constants.XBOX_OPERATOR);
  
  // Shuffleboard Tabs
  private final ShuffleboardTab configTab;
  private final ShuffleboardTab driveTab;
  private final ShuffleboardTab prematchTab;
  private final ShuffleboardTab debugTab;

  // Network Tables for Smart Dashboard 
  private NetworkTableEntry driveReversedEntry;
  private NetworkTableEntry precisionDriveEntry;
  private NetworkTableEntry overrideModeEntry; 
  
  // Logging Related 
  private NetworkTableEntry lastError;
  private NetworkTableEntry lastWarning; 

  private static final RobotLogger logger = new RobotLogger();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain = new Drivetrain(Constants.RIGHT_CANSPARKMAX, Constants.LEFT_CANSPARKMAX,
        Constants.RIGHT_CANSPARKMAX_FOLLOWER, Constants.LEFT_CANSPARKMAX_FOLLOWER);
    drivetrain.setDefaultCommand(
        new TeleopDrive(drivetrain, driverController, Constants.DRIVE_FWD_REV, Constants.DRIVE_LEFT_RIGHT));
    
    // Shuffle Board Tabs 
    configTab = Shuffleboard.getTab("Config");
    driveTab = Shuffleboard.getTab("Drive");
    prematchTab = Shuffleboard.getTab("Pre-match");
    debugTab = Shuffleboard.getTab("Debug");

    configureDashboard();

    while (!DriverStation.isDSAttached()) {
      try {
        Thread.sleep(500);
      } catch (InterruptedException e) {
        e.printStackTrace();
        // NetworkTableEntry for last error or log the error 
      }
    }

    initLogger();
    
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureDashboard() {
    // Configure Tabs 
    configTab.add("Precision Drive Factor", TeleopDrive.getPrecisionFactor()).withPosition(0, 0).withSize(9, 4)
    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
    .addListener(notif -> {
      TeleopDrive.setPrecisionFactor(notif.value.getDouble()); 
        }, EntryListenerFlags.kUpdate);
    
    configTab.add("Motor Ramping Rate", TeleopDrive.getRampingRate()).withPosition(9, 0).withSize(9, 4)
    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
    .addListener(notif -> {
      TeleopDrive.setRampingRate(notif.value.getDouble()); 
        }, EntryListenerFlags.kUpdate);
        
    configTab.add("Motor Warning Temp", Constants.MOTOR_WARNING_TEMP).withWidget(BuiltInWidgets.kNumberSlider)
    .withPosition(18, 0).withSize(9, 4).withProperties(Map.of("min", 0.0, "max", 150.0)).getEntry()
    .addListener(notif -> {
      Constants.MOTOR_WARNING_TEMP = notif.value.getDouble(); 
        }, EntryListenerFlags.kUpdate);
        
    configTab.add("Motor Shutoff Temp", Constants.MOTOR_SHUTOFF_TEMP).withWidget(BuiltInWidgets.kNumberSlider)
    .withPosition(27, 0).withSize(9, 4).withProperties(Map.of("min", 0.0, "max", 150.0)).getEntry()
    .addListener(notif -> {
      Constants.MOTOR_SHUTOFF_TEMP = notif.value.getDouble(); 
        }, EntryListenerFlags.kUpdate);
        
    // Drive Tabs  
    driveTab.add("Gyro", drivetrain.getAHRS()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 4).withSize(9, 10);

    // Driving Related Entries
    driveReversedEntry = driveTab.add("Reversed", TeleopDrive.isReversed()).withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0).withSize(4, 4).getEntry();
    
    precisionDriveEntry = driveTab.add("Precision", TeleopDrive.isPrecisionDrive()).withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(4, 0).withSize(4, 4).getEntry();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button reverseDriveButton = new JoystickButton(driverController, Constants.REVERSE_DRIVE_DIRECTION);
    Button dtOverheatOverrideButton = new JoystickButton(driverController, Constants.OVERRIDE_MOTOR_PROTECTION);
    Button precisionDriveButton = new JoystickButton(driverController, Constants.PRECISION_DRIVE_TOGGLE);
    AnalogTrigger precisionDriveTrigger = new AnalogTrigger(driverController, Constants.PRECISION_DRIVE_HOLD, 0.5);

    // Driver Button Bindings 
    reverseDriveButton.whenPressed(() -> {
      TeleopDrive.toggleReverseDrive();
      getLogger().logInfo("Drive reverse set to " + TeleopDrive.isReversed());
    });

    precisionDriveButton.whenPressed(() -> {
      TeleopDrive.togglePrecisionDrive();
    });
    
    precisionDriveTrigger.setMinTimeRequired(0.05);
    precisionDriveTrigger.whileActiveOnce(new FunctionalCommand(() -> {
        TeleopDrive.togglePrecisionDrive();
      }, () -> {
      }, (interrupted) -> {
        TeleopDrive.togglePrecisionDrive();
    }, () -> false));

    dtOverheatOverrideButton.whenPressed(() -> {
      // Toggle between overriding and not overriding overheat shutoff.
      boolean override = !drivetrain.getOverheatShutoffOverride();
      drivetrain.setOverheatShutoffOverride(override);
      
      if (override) {
        getLogger().logWarning("Drivetrain motor temperature protection overridden.");
      } else {
        getLogger().logInfo("Drivetrain motor temperature protection re-enabled."); 
      }
    });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: implement an autonomous command 
    return new InstantCommand();
  }

  private void initLogger() {
	try {
		logger.init(Robot.class, new File(Filesystem.getOperatingDirectory().getCanonicalPath() + "/frc-robot-logs"));

		logger.setLevel(Level.FINE);
		
		logger.setLogHandler((level, message) -> {
			if (level == Level.SEVERE) {
				lastError.setString(message);
			} else if (level == Level.WARNING) {
				lastWarning.setString(message);
			}
		});

		logger.cleanLogs(72);
		logger.logInfo("Logger initialized");
		
	} catch (IOException e) {
		e.printStackTrace();
		lastError.setString("Failed to initialize logger");
	}
}
  
  /**
   * Get the robot logger. 
   * 
   * @return the robot logger.
   */
  public static RobotLogger getLogger() {
    return logger; 
  }
  
}
