// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kvVoltSecondsPerMeter = 2.64; 
    public static final double kaVoltSecondsSquaredPerMeter = 0.324; 
    // Simulation parameters  FAKE!!
    public static final double kvVoltSecondsPerRadian = 3.0;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
            LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
    public static final double kDriveGearing = 8;

}
