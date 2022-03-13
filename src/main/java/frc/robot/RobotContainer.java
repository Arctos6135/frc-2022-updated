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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.climbing.Climb;
import frc.robot.commands.climbing.DriveRaise;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.indexer.TeleopRoll;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.RotateArm;
import frc.robot.commands.shooting.Shoot;
import frc.robot.constants.Autonomous;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.util.SendableCANPIDController;
import frc.robot.subsystems.ShooterFeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Contains all commands and subsystems.
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final Drivetrain drivetrain;
	private final IntakeSubsystem intakeSubsystem;
	private final IntakeArm intakeArm; 
	private final ShooterFeederSubsystem shooterFeederSubsystem; 
	private final Shooter shooterSubsystem;
	private final ClimbSubsystem climbSubsystem; 

	private static final XboxController driverController = new XboxController(Constants.XBOX_DRIVER);
	private static final XboxController operatorController = new XboxController(Constants.XBOX_OPERATOR);

	// Shuffleboard Tabs
	final ShuffleboardTab configTab; 
	final ShuffleboardTab driveTab;
	final ShuffleboardTab shooterTab;
	final ShuffleboardTab colorTab; 
	final ShuffleboardTab prematchTab;
	final ShuffleboardTab debugTab;

	// Network Tables for Smart Dashboard
	NetworkTableEntry driveReversedEntry;
	NetworkTableEntry precisionDriveEntry;
	NetworkTableEntry overrideModeEntry;
	NetworkTableEntry shooterRPMEntry; 
	
	// Logging Related
	NetworkTableEntry lastError;
	NetworkTableEntry lastWarning;
	
	// Drivetrain Status
	SimpleWidget drivetrainMotorStatus;
	SimpleWidget shooterMotorStatus;
	SimpleWidget shooterFeederStatus;
	SimpleWidget climbStatus;

	private SendableChooser<Integer> preloadedBalls = new SendableChooser<>(); 

	public static final RobotLogger logger = new RobotLogger();

	private Autonomous autonomous; 

	// Contains subsystems, OI devices, and commands
	public RobotContainer() {
		drivetrain = new Drivetrain(Constants.RIGHT_CANSPARKMAX, Constants.LEFT_CANSPARKMAX,
			Constants.RIGHT_CANSPARKMAX_FOLLOWER, Constants.LEFT_CANSPARKMAX_FOLLOWER);
		drivetrain.setDefaultCommand(
			new TeleopDrive(drivetrain, driverController, Constants.DRIVE_FWD_REV, Constants.DRIVE_LEFT_RIGHT));

		intakeSubsystem = new IntakeSubsystem(Constants.MECANUM_INTAKE_MOTOR);
		intakeSubsystem.setDefaultCommand( 
			new Intake(intakeSubsystem, driverController, Constants.INTAKE_FORWARD_BUTTON, Constants.INTAKE_REVERSE_BUTTON)
		);

		intakeArm = new IntakeArm(Constants.INTAKE_ARM_MOTOR);
		intakeArm.setDefaultCommand(
			new RotateArm(intakeArm, operatorController, Constants.INTAKE_ARM_ROTATE)
		);

		shooterFeederSubsystem = new ShooterFeederSubsystem(Constants.ROLLER_MOTOR); 
		shooterFeederSubsystem.setDefaultCommand(
			new TeleopRoll(shooterFeederSubsystem, operatorController, Constants.ROLL_AXIS) 
		);

		shooterSubsystem = new Shooter(Constants.MAIN_SHOOTER_MOTOR, Constants.AUXILLIARY_SHOOTER_MOTOR);
		shooterSubsystem.setDefaultCommand(
			// Shoot for the lower hub
			new Shoot(shooterSubsystem, shooterFeederSubsystem, true)
		);

		climbSubsystem = new ClimbSubsystem(Constants.HOOK_DEPLOYMENT_MOTOR, Constants.LEFT_CLIMB_MOTOR, Constants.RIGHT_CLIMB_MOTOR); 
		climbSubsystem.setDefaultCommand(
			new Climb(climbSubsystem, operatorController, Constants.DEPLOY_CLIMB_BUTTON)
		);

		autonomous = new Autonomous(); 

		// Shuffle Board Tabs
		configTab = Shuffleboard.getTab("Config");
		driveTab = Shuffleboard.getTab("Drive");
		shooterTab = Shuffleboard.getTab("Shoot"); 
		colorTab = Shuffleboard.getTab("Color"); 
		prematchTab = Shuffleboard.getTab("Pre-match");
		debugTab = Shuffleboard.getTab("Debug");

		configureDashboard();

		while (!DriverStation.isDSAttached()) {
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		initLogger();
		
		// Configure the button bindings
		configureButtonBindings();
	}

	private void configureDashboard() {
		// Configure Tabs
		configTab.add("Precision Drive Factor", TeleopDrive.getPrecisionFactor()).withPosition(0, 0).withSize(6, 4)
		.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
		.addListener(notif -> {
			TeleopDrive.setPrecisionFactor(notif.value.getDouble());
				}, EntryListenerFlags.kUpdate);
		
		configTab.add("Motor Ramping Rate", TeleopDrive.getRampingRate()).withPosition(6, 0).withSize(6, 4)
		.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
		.addListener(notif -> {
			TeleopDrive.setRampingRate(notif.value.getDouble());
				}, EntryListenerFlags.kUpdate);
				
		configTab.add("Motor Warning Temp", Constants.MOTOR_WARNING_TEMP).withWidget(BuiltInWidgets.kNumberSlider)
		.withPosition(12, 0).withSize(6, 4).withProperties(Map.of("min", 0.0, "max", 150.0)).getEntry()
		.addListener(notif -> {
			Constants.MOTOR_WARNING_TEMP = notif.value.getDouble();
				}, EntryListenerFlags.kUpdate);
				
		configTab.add("Motor Shutoff Temp", Constants.MOTOR_SHUTOFF_TEMP).withWidget(BuiltInWidgets.kNumberSlider)
		.withPosition(18, 0).withSize(6, 4).withProperties(Map.of("min", 0.0, "max", 150.0)).getEntry()
		.addListener(notif -> {
			Constants.MOTOR_SHUTOFF_TEMP = notif.value.getDouble();
				}, EntryListenerFlags.kUpdate);

		configTab.add("Shooter PID", new SendableCANPIDController(shooterSubsystem.getPIDController()))
		.withWidget(BuiltInWidgets.kPIDController).withPosition(0, 4).withSize(6, 12);
		
		configTab.add("Intake Arm Precision", RotateArm.getPrecisionFactor()).withPosition(6, 4).withSize(6, 4)
		.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
		.addListener(notif -> {
			RotateArm.setPrecisionFactor(notif.value.getDouble());
			}, EntryListenerFlags.kUpdate); 
				
		// Write Settings of Spark Max Motors on Drivetrain and Shooter 
		InstantCommand burnFlashCommand = new InstantCommand(() -> {
			drivetrain.burnFlash(); 
			shooterSubsystem.burnFlash(); 
		}); 

		burnFlashCommand.setName("Burn Flash");
		configTab.add("Burn Spark Motors", burnFlashCommand).withWidget(BuiltInWidgets.kCommand).withPosition(12, 4); 

		// Drive Tabs
		driveTab.add("Gyro", drivetrain.getAHRS()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 4).withSize(6, 8);

		// Driving Related Entries
		driveReversedEntry = driveTab.add("Reversed", TeleopDrive.isReversed()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0)
		.withSize(4, 4).getEntry();
		
		precisionDriveEntry = driveTab.add("Precision", TeleopDrive.isPrecisionDrive()).withWidget(BuiltInWidgets.kBooleanBox)
		.withPosition(4, 0).withSize(4, 4).getEntry();

		shooterTab.add("Shooter RPM", shooterSubsystem.getActualVelocity()).withWidget(BuiltInWidgets.kDial).withPosition(0, 0)
		.withSize(6, 6).withProperties(Map.of("min", 0, "max", 5000)).getEntry()
		.addListener(notif -> {
			shooterSubsystem.setVelocity(notif.value.getDouble());
		}, EntryListenerFlags.kUpdate);
				
		InstantCommand climbOverrideCommand = new InstantCommand(() -> {
			Climb.toggleOverride();
		});
		climbOverrideCommand.setName("Override");
		driveTab.add("Override Climb Time", climbOverrideCommand).withWidget(BuiltInWidgets.kCommand).withPosition(6, 4).withSize(6, 6);
		
		// Color Detection of Balls 
		colorTab.add("Red Color", shooterFeederSubsystem.getColorSensor().getRed());
		colorTab.add("Blue Color", shooterFeederSubsystem.getColorSensor().getBlue());
		colorTab.add("Unknown Color", shooterFeederSubsystem.getColorSensor().getIR()); 

		// Overheating Warnings
		drivetrain.getMonitorGroup().setOverheatShutoffCallback((motor, temp) -> {
			if (!drivetrain.getOverheatShutoffOverride()) {
				drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_SHUTOFF)).getEntry().setBoolean(false);
			}
			getLogger().logError("Drivetrain motor " + motor.getDeviceId() + " reached overheat shutoff limit at " + temp + "C!");
		});

		drivetrain.getMonitorGroup().setOverheatWarningCallback((motor, temp) -> {
			if (!drivetrain.getOverheatShutoffOverride())
				drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_WARNING)).getEntry().setBoolean(false);
			getLogger().logWarning("Drivetrain motor " + motor.getDeviceId() + " reached overheat warning at " + temp + " C!");
		});

		drivetrain.getMonitorGroup().setNormalTempCallback(() -> drivetrainMotorStatus.getEntry().setBoolean(true));

		shooterSubsystem.getMonitorGroup().setOverheatShutoffCallback((motor, temp) -> {
			if (!shooterSubsystem.getOverheatShutoffOverride())
				shooterMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_SHUTOFF)).getEntry().setBoolean(false);
			getLogger().logError("Shooter motor " + motor.getDeviceId() + " reched overheat shutoff limit at " + temp + "C!");
		});

		shooterSubsystem.getMonitorGroup().setOverheatWarningCallback((motor, temp) -> {
			if (!shooterSubsystem.getOverheatShutoffOverride())
				shooterMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_WARNING)).getEntry().setBoolean(false);
			getLogger().logWarning("Shooter motor " + motor.getDeviceId() + " reached overheat warning at " + temp + " C!");
		});

		prematchTab.add("Autonomous Mode", autonomous.getChooser()).withPosition(0, 0).withSize(9, 5);
		preloadedBalls.setDefaultOption("0", 0);
		preloadedBalls.addOption("1", 1);
		prematchTab.add("Preloaded Balls", preloadedBalls).withPosition(9, 0).withSize(5, 5); 
		
		lastError = driveTab.add("Last Error", "").withPosition(0, 12).withSize(20, 4).getEntry();
		lastWarning = driveTab.add("Last Warning", "").withPosition(4, 12).withSize(20, 4).getEntry();
		
		debugTab.add(drivetrain).withPosition(0, 0).withSize(19, 15); 
	}

	public void updateDashboard() {
		shooterRPMEntry.setNumber(shooterSubsystem.getActualVelocity());
	}

	private void configureButtonBindings() {
		// Driving Related 
		Button reverseDriveButton = new JoystickButton(driverController, Constants.REVERSE_DRIVE_DIRECTION);
		Button dtOverheatOverrideButton = new JoystickButton(driverController, Constants.OVERRIDE_MOTOR_PROTECTION);
		Button precisionDriveButton = new JoystickButton(driverController, Constants.PRECISION_DRIVE_TOGGLE);
		AnalogTrigger precisionDriveTrigger = new AnalogTrigger(driverController, Constants.PRECISION_DRIVE_HOLD, 0.5);
		
		// Shooter Related 
		Button deployShooterLowerButton = new JoystickButton(operatorController, Constants.DEPLOY_SHOOTER_LOWER_BUTTON);
		Button deployShooterUpperButton = new JoystickButton(operatorController, Constants.DEPLOY_SHOOTER_UPPER_BUTTON);
		Button constantRollSpeedButton = new JoystickButton(operatorController, Constants.CONSTANT_ROLL_SPEED_BUTTON);
		Button shootLowHubRPMButton = new JoystickButton(operatorController, Constants.SHOOT_LOW_RPM_BUTTON);
		Button shootHighHubRPMButton = new JoystickButton(operatorController, Constants.SHOOT_HIGH_RPM_BUTTON); 

		// Climb Related
		Button overrideClimbTimeButton = new JoystickButton(operatorController, Constants.CLIMB_TIME_OVERRIDE_BUTTON); 
		Button driveRaiseHalfway = new JoystickButton(operatorController, Constants.DRIVE_RAISE_HALFWAY); 
		Button driveRaiseFully = new JoystickButton(operatorController, Constants.DRIVE_RAISE_FULLY);
		
		// Intake Related 
		Button reverseIntakeArmButton = new JoystickButton(operatorController, Constants.INTAKE_ARM_REVERSE_BUTTON); 

		// Driver Button Bindings
		reverseDriveButton.whenPressed(() -> {
			TeleopDrive.toggleReverseDrive();
			getLogger().logInfo("Drive reverse set to " + TeleopDrive.isReversed());
		});

		precisionDriveButton.whenPressed(() -> {
			TeleopDrive.togglePrecisionDrive();
			precisionDriveEntry.setBoolean(TeleopDrive.isPrecisionDrive()); 
  		});
    
		precisionDriveTrigger.setMinTimeRequired(0.05);
		precisionDriveTrigger.whileActiveOnce(new FunctionalCommand(() -> {
		TeleopDrive.togglePrecisionDrive();
	      	}, () -> {
	        }, (interrupted) -> {
			TeleopDrive.togglePrecisionDrive();
			precisionDriveEntry.setBoolean(TeleopDrive.isPrecisionDrive()); 
	    }, () -> false));

		dtOverheatOverrideButton.whenPressed(() -> {
			// Toggle overheat shutoff override
			boolean override = !drivetrain.getOverheatShutoffOverride();
			drivetrain.setOverheatShutoffOverride(override);
			
			if (override) {
				getLogger().logWarning("Drivetrain motor temperature protection overridden.");
			} else {
				getLogger().logInfo("Drivetrain motor temperature protection re-enabled.");
			}
		});

		// Shooting 
		deployShooterLowerButton.whenActive(() -> {
			new Shoot(shooterSubsystem, shooterFeederSubsystem, true); 
		});

		deployShooterUpperButton.whenActive(() -> {
			new Shoot(shooterSubsystem, shooterFeederSubsystem, false); 
		});

		// Setting RPM 
		shootLowHubRPMButton.whenPressed(() -> {
			shooterSubsystem.setVelocity(Constants.LOW_HUB_RPM);
		});

		shootHighHubRPMButton.whenPressed(() -> {
			shooterSubsystem.setVelocity(Constants.HIGH_HUB_RPM); 
		});

		constantRollSpeedButton.whenPressed(() -> {
			ShooterFeederSubsystem.toggleConstantRollSpeed();
		});  

		// Climber Button Bindings 
		overrideClimbTimeButton.whenPressed(() -> {
			Climb.toggleOverride();
		});

		driveRaiseHalfway.whenPressed(
			new DriveRaise(climbSubsystem, drivetrain) 
		); 

		driveRaiseFully.whenPressed(
			new DriveRaise(climbSubsystem, drivetrain) // TODO: change to a similar drive raise command 
		);
		
		// Intake Button Bindings 
		reverseIntakeArmButton.whenPressed(() -> {
			RotateArm.toggleReverseRotation();
			getLogger().logInfo("Intake arm direction set to " + RotateArm.isRotationReversed());
		});


	}

	public Command getAutonomousCommand() {
		return autonomous.getAuto(autonomous.getChooser().getSelected(), drivetrain, intakeSubsystem, intakeArm, shooterSubsystem, shooterFeederSubsystem); 
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
	
	public static RobotLogger getLogger() {
		return logger;
	}
}
