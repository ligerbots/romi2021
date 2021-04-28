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
    addCommands( 

                 new DriveDistance(0.7, 7.5, driveTrain)
                 
                 //new TurnTime(-0.7, 0.145, driveTrain),

                /* new DriveTime(0.7, 0.92, driveTrain),
                 new TurnTime(-0.7, 0.31, driveTrain),
                 new DriveTime(0.7, 0.675, driveTrain),// get first ball
                 new TurnTime(-0.7, 0.105, driveTrain),
                 new DriveTime(-0.7, 1.831, driveTrain),//bottom

                 new TurnTime(0.7, 0.23, driveTrain),
                 new DriveTime(0.7, 0.84, driveTrain),
                 new TurnTime(-0.7, 0.21798, driveTrain),
                 new DriveTime(0.7, 1.294, driveTrain), // get the second ball
                 new DriveTime(-0.7, 1.786, driveTrain), //bottom
                 new TurnTime(0.7, 0.28, driveTrain),

                 new DriveTime(0.7, 1.644, driveTrain),
                 new TurnTime(-0.7, 0.3428, driveTrain),
                 new DriveTime(0.7, 1.62, driveTrain), // get the third ball

                 new DriveTime(-0.7, 0.8, driveTrain),
                 new TurnTime(0.7, 0.3, driveTrain),
                 new DriveTime(0.7, 0.812, driveTrain)*/

                 );
  }
}