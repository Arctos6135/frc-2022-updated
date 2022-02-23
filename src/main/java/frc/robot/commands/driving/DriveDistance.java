// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class DriveDistance extends FollowTrajectory {

    /**
     * Instantiate a DriveDistance command.
     *
     * @param drivetrain the drivetrain.
     * @param distance the distance to drive (inches).
     */
    public DriveDistance(Drivetrain drivetrain, double distance) {
        super(drivetrain, new TrapezoidalTankDriveProfile(Constants.ROBOT_SPECS, distance));
    }
}
