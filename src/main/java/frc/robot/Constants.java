// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Drivetrain constants for run Ramsete commands
    // These numbers are supposed to be in inches (not meters like the big robot)

    // The romi larry has
    public static final double turncomp = -.25; //compensate for unequal motors

    // FEEDFORWARD AND FEEDBACK GAINS
    public static final double ksVolts = 1.32;   // kS in characterization tool
    public static final double kvVoltSecondsPerMeter = 6.13; // kV
    public static final double kaVoltSecondsSquaredPerMeter = 0.0189; // kA
    public static final double kPDriveVel = 0.00343;   // kP in tool, I think
    public static final double kTrackwidthMeters = 0.14284467176823962;

    // DIFFERENTIAL DRIVE KINEMATICS
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // RAMSETE PARAMETERS
    public static final double kRamseteB = 2; // generic ramsete values
    public static final double kRamseteZeta = .7; // generic ramsete values
}
