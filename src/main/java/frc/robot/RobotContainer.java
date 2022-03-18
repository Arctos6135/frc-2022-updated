package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.logging.Level;

import com.arctos6135.robotlib.logging.RobotLogger;
import com.arctos6135.robotlib.newcommands.triggers.AnalogTrigger;
import com.arctos6135.robotlib.oi.Rumble;

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
import frc.robot.commands.climbing.DeployHook;
import frc.robot.commands.driving.TeleopDrive;
import frc.robot.commands.indexer.SensoredRoll;
import frc.robot.commands.indexer.TeleopRoll;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.RotateArm;
import frc.robot.commands.shooting.PrepareShooter;
import frc.robot.commands.shooting.PrepareShooterPID;
import frc.robot.commands.shooting.Shoot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.util.SendableCANPIDController;
import frc.robot.subsystems.ShooterFeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is w here the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final Drivetrain drivetrain;
	// private final IntakeSubsystem intakeSubsystem;
	// private final IntakeArm intakeArm; 
	private final ShooterFeederSubsystem shooterFeederSubsystem; 
	private final Shooter shooterSubsystem;
	// private final ClimbSubsystem climbSubsystem; 
	// private final HookSubsystem hookSubsystem; 

	// Controllers
	private static final XboxController driverController = new XboxController(Constants.XBOX_DRIVER);
	private static final XboxController operatorController = new XboxController(Constants.XBOX_OPERATOR);

	// Controller Rumbling 
	public static final Rumble infoRumbleDriver = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 200, 1); 
	public static final Rumble warningRumbleDriver = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 300, 2); 
	public static final Rumble errorRumbleDriver = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 400, 3);
	public static final Rumble infoRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 200, 1); 
	public static final Rumble shooterRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 200, 2);
	public static final Rumble warningRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 300, 2); 
	public static final Rumble errorRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 400, 3); 

	// Shuffleboard Tabs
	public final ShuffleboardTab configTab; 
	public final ShuffleboardTab driveTab;
	public final ShuffleboardTab shooterTab;
	public final ShuffleboardTab climbTab; 
	public final ShuffleboardTab colorTab; 
	public final ShuffleboardTab prematchTab;
	public final ShuffleboardTab debugTab;

	// Network Tables for Smart Dashboard
	public NetworkTableEntry driveReversedEntry;
	public NetworkTableEntry precisionDriveEntry;
	public NetworkTableEntry overrideModeEntry;
	public NetworkTableEntry shooterRPMEntry; 
	
	// Logging Related
	public NetworkTableEntry lastError;
	public NetworkTableEntry lastWarning;
	
	// Drivetrain Status
	public SimpleWidget drivetrainMotorStatus;
	public SimpleWidget shooterMotorStatus;
	public SimpleWidget shooterFeederStatus;
	public SimpleWidget climbStatus;

	// Autonomous Mode 
	// private SendableChooser<Integer> preloadedBalls = new SendableChooser<>(); 
	// private Autonomous autonomous;

	public static final RobotLogger logger = new RobotLogger(); 

	/**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
	public RobotContainer() {
		drivetrain = new Drivetrain(Constants.RIGHT_CANSPARKMAX, Constants.LEFT_CANSPARKMAX,
			Constants.RIGHT_CANSPARKMAX_FOLLOWER, Constants.LEFT_CANSPARKMAX_FOLLOWER);
		drivetrain.setDefaultCommand(
			new TeleopDrive(drivetrain, driverController, Constants.DRIVE_FWD_REV, Constants.DRIVE_LEFT_RIGHT));

		/* intakeSubsystem = new IntakeSubsystem(Constants.MECANUM_INTAKE_MOTOR);
		intakeSubsystem.setDefaultCommand( 
			new Intake(intakeSubsystem, driverController, Constants.INTAKE_FORWARD_BUTTON, Constants.INTAKE_REVERSE_BUTTON)
		); */ 

		/* intakeArm = new IntakeArm(Constants.INTAKE_ARM_MOTOR);
		intakeArm.setDefaultCommand(
			new RotateArm(intakeArm, operatorController, Constants.INTAKE_ARM_ROTATE)
		); */

		shooterFeederSubsystem = new ShooterFeederSubsystem(Constants.TOP_ROLLER_MOTOR, Constants.BOTTOM_ROLLER_MOTOR); 
		shooterFeederSubsystem.setDefaultCommand(
			new TeleopRoll(shooterFeederSubsystem, operatorController, Constants.TELEOP_ROLL_UP_TRIGGER, Constants.TELEOP_ROLL_DOWN_TRIGGER) 
		); 

		shooterSubsystem = new Shooter(Constants.MAIN_SHOOTER_MOTOR, Constants.AUXILLIARY_SHOOTER_MOTOR);

		/* climbSubsystem = new ClimbSubsystem(Constants.LEFT_CLIMB_MOTOR, Constants.RIGHT_CLIMB_MOTOR);
		climbSubsystem.setDefaultCommand(
			new Climb(climbSubsystem, operatorController, Constants.CLIMB_RUNG_AXIS)
		);
		
		hookSubsystem = new HookSubsystem(Constants.HOOK_DEPLOYMENT_MOTOR); 
		hookSubsystem.setDefaultCommand(
			new DeployHook(hookSubsystem, driverController)
		); */ 

		// autonomous = new Autonomous(); 

		// Shuffle Board Tabs
		configTab = Shuffleboard.getTab("Config");
		driveTab = Shuffleboard.getTab("Drive");
		shooterTab = Shuffleboard.getTab("Shoot"); 
		climbTab = Shuffleboard.getTab("Climb"); 
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

	/**
	 * Add tabs and widgets to the {@link edu.wpi.first.wpilibj.shuffleboard.Shuffleboard}. 
	 */
	private void configureDashboard() {
		// Motor Configuration Settings 
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
		
		/* configTab.add("Intake Arm Precision", RotateArm.getPrecisionFactor()).withPosition(6, 4).withSize(6, 4)
		.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
		.addListener(notif -> {
			RotateArm.setPrecisionFactor(notif.value.getDouble());
				}, EntryListenerFlags.kUpdate); */ 
				
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

		// Shooting Configurations
		shooterTab.add("Shooter RPM (LOW)", shooterSubsystem.getActualVelocity()).withWidget(BuiltInWidgets.kDial).withPosition(0, 0)
		.withSize(6, 6).withProperties(Map.of("min", 0, "max", 5000)).getEntry()
		.addListener(notif -> {
			shooterSubsystem.setVelocity(notif.value.getDouble());
		}, EntryListenerFlags.kUpdate);

		shooterTab.add("Shooter RPM (HIGH)", shooterSubsystem.getActualVelocity()).withWidget(BuiltInWidgets.kDial).withPosition(6, 0)
		.withSize(6, 6).withProperties(Map.of("min", 0, "max", 5000)).getEntry()
		.addListener(notif -> {
			shooterSubsystem.setVelocity(notif.value.getDouble());
		}, EntryListenerFlags.kUpdate);

		shooterTab.add("Shooter Roller Speed", shooterFeederSubsystem.getRollSpeed()).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 6)
		.withSize(6, 6).withProperties(Map.of("min", 0, "max", 1.0)).getEntry()
		.addListener(notif -> {
			shooterFeederSubsystem.setRollSpeed(notif.value.getDouble());
		}, EntryListenerFlags.kUpdate); 
		
		/* 
		// Climbing Configurations
		InstantCommand climbOverrideCommand = new InstantCommand(() -> {
			Climb.toggleOverride();
		});
		climbOverrideCommand.setName("Override");
		climbTab.add("Override Climb Time", climbOverrideCommand).withWidget(BuiltInWidgets.kCommand).withPosition(0, 0).withSize(6, 6);

		climbTab.add("Precision Climb", Climb.isPrecisionClimb()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 0).withSize(4, 4).getEntry(); */ 
		
		// Color Detection of Balls 
		colorTab.add("Red Color", shooterFeederSubsystem.getColorSensor().getRed());
		colorTab.add("Blue Color", shooterFeederSubsystem.getColorSensor().getBlue());
		colorTab.add("Unknown Color", shooterFeederSubsystem.getColorSensor().getIR()); 

		// Overheating Warnings
		drivetrain.getMonitorGroup().setOverheatShutoffCallback((motor, temp) -> {
			if (!drivetrain.getOverheatShutoffOverride()) {
				drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_SHUTOFF)).getEntry().setBoolean(false);
				errorRumbleDriver.execute(); 
			}
			String error = "Drivetrain motor" + Integer.toString(motor.getDeviceId()) + " reached overheat shutoff limit at " + Double.toString(temp) + "C!"; 
			getLogger().logError(error);
			DriverStation.reportError(error, true); 
		});

		drivetrain.getMonitorGroup().setOverheatWarningCallback((motor, temp) -> {
			if (!drivetrain.getOverheatShutoffOverride()) {
				drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_WARNING)).getEntry().setBoolean(false);
				warningRumbleDriver.execute();
			}	
			String error = "Drivetrain motor" + Integer.toString(motor.getDeviceId()) + " reached overheat warning at " + Double.toString(temp) + "C!"; 
			getLogger().logWarning(error);
			DriverStation.reportError(error, true); 
		});

		drivetrain.getMonitorGroup().setNormalTempCallback(() -> drivetrainMotorStatus.getEntry().setBoolean(true));

		shooterSubsystem.getMonitorGroup().setOverheatShutoffCallback((motor, temp) -> {
			if (!shooterSubsystem.getOverheatShutoffOverride()) {
				shooterMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_SHUTOFF)).getEntry().setBoolean(false);
				errorRumbleOperator.execute(); 
			}
			String error = "Shooter motor " + Integer.toString(motor.getDeviceId()) + " reched overheat shutoff limit at " + Double.toString(temp) + "C!";
			getLogger().logError(error);
			DriverStation.reportError(error, true); 
		});

		shooterSubsystem.getMonitorGroup().setOverheatWarningCallback((motor, temp) -> {
			if (!shooterSubsystem.getOverheatShutoffOverride()) {
				shooterMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_WARNING)).getEntry().setBoolean(false);
				warningRumbleOperator.execute(); 
			}
			String error = "Shooter motor " + Integer.toString(motor.getDeviceId()) + " reached overheat warning at " + Double.toString(temp) + " C!";
			getLogger().logWarning(error);
			DriverStation.reportError(error, true);
		});
		
		// Autonomous Mode 
		/* prematchTab.add("Autonomous Mode", autonomous.getChooser()).withPosition(0, 0).withSize(12, 5);
		preloadedBalls.setDefaultOption("0", 0);
		preloadedBalls.addOption("1", 1);
		prematchTab.add("Preloaded Balls", preloadedBalls).withPosition(12, 0).withSize(5, 5); */ 
		
		lastError = driveTab.add("Last Error", "").withPosition(0, 12).withSize(20, 4).getEntry();
		lastWarning = driveTab.add("Last Warning", "").withPosition(4, 12).withSize(20, 4).getEntry();
		
		debugTab.add(drivetrain).withPosition(0, 0).withSize(19, 15); 
	}

	public void updateDashboard() {
		// shooterRPMEntry.setNumber(shooterSubsystem.getActualVelocity());
	}

	/**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
	private void configureButtonBindings() {
		// Driving Related 
		Button reverseDriveButton = new JoystickButton(driverController, Constants.REVERSE_DRIVE_DIRECTION);
		Button dtOverheatOverrideButton = new JoystickButton(driverController, Constants.OVERRIDE_MOTOR_PROTECTION);
		Button precisionDriveButton = new JoystickButton(driverController, Constants.PRECISION_DRIVE_TOGGLE);
		AnalogTrigger precisionDriveTrigger = new AnalogTrigger(driverController, Constants.PRECISION_DRIVE_HOLD, 0.5);
		
		// Shooter Related 
		AnalogTrigger deployShooterLowerButton = new AnalogTrigger(operatorController, Constants.DEPLOY_SHOOTER_LOWER_BUTTON, 0.5);
		AnalogTrigger deployShooterUpperButton = new AnalogTrigger(operatorController, Constants.DEPLOY_SHOOTER_UPPER_BUTTON, 0.5);
		Button stopShooterButton = new JoystickButton(operatorController, Constants.STOP_SHOOTER_BUTTON); 

		// TESTING BUTTONS
		Button shootLowHubRPMButton = new JoystickButton(operatorController, Constants.SHOOT_LOW_RPM_BUTTON);
		Button shootHighHubRPMButton = new JoystickButton(operatorController, Constants.SHOOT_HIGH_RPM_BUTTON); 

		Button shooterOverheatOverrideButton = new JoystickButton(operatorController, Constants.OVERRIDE_SHOOTER_PROTECTION_BUTTON); 
		Button sensoredRollButton = new JoystickButton(operatorController, Constants.SENSORED_ROLL); 
		Button stopShooterFeederButton = new JoystickButton(operatorController, Constants.STOP_SHOOTER_FEEDER_BUTTON); 

		// Climb Related 
		Button toggleClimbPrecision = new JoystickButton(operatorController, Constants.TOGGLE_CLIMB_PRECISION); 
		
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
			// Toggle overheat shutoff override.
			boolean override = !drivetrain.getOverheatShutoffOverride();
			drivetrain.setOverheatShutoffOverride(override);
			
			if (override) {
				getLogger().logWarning("Drivetrain motor temperature protection overridden.");
				DriverStation.reportWarning("Drivetrain motor temperature protection overridden.", true);
			} else {
				getLogger().logInfo("Drivetrain motor temperature protection re-enabled.");
			}
			infoRumbleDriver.execute(); 
		});

		shooterOverheatOverrideButton.whenPressed(() -> {
			// Toggle overheat shutoff override.
			boolean override = !shooterSubsystem.getOverheatShutoffOverride(); 
			shooterSubsystem.setOverheatShutoffOverride(override);

			if (override) {
				getLogger().logWarning("Shooter motor temperature protection overridden.");
				// DriverStation.reportWarning("Shooter motor temperature protection overridden.", true);
			} else {
				getLogger().logInfo("Shooter motor temperature protection re-enabled.");
			}
			infoRumbleOperator.execute(); 
		}); 

		// Shooting 
		deployShooterLowerButton.whenActive(
			new Shoot(shooterSubsystem, shooterFeederSubsystem, true)
		);

		deployShooterUpperButton.whenActive(
			new Shoot(shooterSubsystem, shooterFeederSubsystem, false)
		); 

		// Setting RPM TODO: test PrepareShooterPID
		stopShooterButton.whenPressed(
			new PrepareShooter(shooterSubsystem, 0)
		); 

		shootLowHubRPMButton.whenPressed(
			// new PrepareShooter(shooterSubsystem, Constants.LOW_HUB_RPM_DIRECT) 
			// TODO: tune PID constant values
			new PrepareShooterPID(shooterSubsystem, Constants.LOW_HUB_RPM)
		);

		shootHighHubRPMButton.whenPressed(
			// new PrepareShooter(shooterSubsystem, Constants.HIGH_HUB_RPM_DIRECT)
			// TODO: tune PID constant values
			new PrepareShooterPID(shooterSubsystem, Constants.HIGH_HUB_RPM)
		);

		sensoredRollButton.whenPressed(
			new SensoredRoll(shooterFeederSubsystem)
		);

		stopShooterFeederButton.whenHeld(new InstantCommand(() -> {
			shooterFeederSubsystem.stopRoller();
		}, shooterFeederSubsystem));

		/* 
		// Climber Button Bindings 
		overrideClimbTimeButton.whenPressed(() -> {
			ClimbSubsystem.toggleClimbTimeOverride();
		});

		toggleClimbPrecision.whenPressed(() -> {
			Climb.togglePrecisionClimb();
		}); 
		
		// Intake Button Bindings 
		reverseIntakeArmButton.whenPressed(() -> {
			RotateArm.toggleReverseRotation();
			getLogger().logInfo("Intake arm direction set to " + RotateArm.isRotationReversed());
		}); */ 
	}

	/**
	 * Return the autonomous command of the robot. 
	 * 
	 * @return the autonomous command for the match.
	 */
	public Command getAutonomousCommand() {
		// return autonomous.getAuto(autonomous.getChooser().getSelected(), drivetrain, intakeSubsystem, intakeArm, shooterSubsystem, shooterFeederSubsystem); 
		return null;
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
