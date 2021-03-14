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

    // Romi WPILibPi-aeb223bc  March 12, 2021
    // FEEDFORWARD AND FEEDBACK GAINS
    // public static final double ksVolts = 0.29;   // kS in characterization tool
    // public static final double kvVoltSecondsPerInch = 0.262; // kV
    // public static final double kaVoltSecondsSquaredPerInch = 0.00473; // kA
    // public static final double kPDriveVel = 0.2;   // kP in tool, I think

    // Team 195 numbers
    public static final double ksVolts = 0.929;   // kS in characterization tool
    public static final double kvVoltSecondsPerMeter = 6.33; // kV
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389; // kA
    public static final double kPDriveVel = 0.085;   // kP in tool, I think

    // DIFFERENTIAL DRIVE KINEMATICS
    public static final double kTrackwidthMeters = 0.142; 
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // RAMSETE PARAMETERS
    public static final double kRamseteB = 2; // generic ramsete values
    public static final double kRamseteZeta = 0.7; // generic ramsete values    
}
