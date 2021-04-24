// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousFetch extends SequentialCommandGroup {
  /** Creates a new AutonomousFetch. */
  public AutonomousFetch(Drivetrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( // With Time
                 new DriveTime(0.6, 1.205, driveTrain),
                 new TurnTime(-0.6, 0.478, driveTrain),
                 new DriveTime(0.6, 1.15, driveTrain),// get first ball
                 new DriveTime(-0.6, 0.88, driveTrain),
                 new TurnTime(-0.6, 0.187, driveTrain),
                 new DriveTime(-0.6, 2.256, driveTrain),// bottom
                 new TurnTime(0.6, 0.355, driveTrain),
                 new DriveTime(0.6, 0.87, driveTrain),
                 new TurnTime(-0.6, 0.189, driveTrain),
                 new DriveTime(0.6, 1.735, driveTrain),// get the second ball
                 new DriveTime(-0.6, 2.485, driveTrain),// bottom
                 new TurnTime(0.6, 0.397, driveTrain),

                 new DriveTime(0, 0.5, driveTrain),

                 new DriveTime(0.6, 2.06, driveTrain),
                 new TurnTime(-0.6, 0.456, driveTrain),
                 new DriveTime(0.6, 2.2, driveTrain), //get the third ball
                 new DriveTime(-0.6, 1.45, driveTrain),
                 new TurnTime(0.6, 0.35, driveTrain),
                 new DriveTime(0.6, 1.37, driveTrain)

                 // With Measure
                 /*

                 new DriveDistance()
                 */
                 );
  }
}