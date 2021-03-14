// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousTime extends SequentialCommandGroup implements AutoCommandInterface {
  private final Pose2d m_initialPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  public AutonomousTime(Drivetrain drivetrain) {
    addCommands(
        new DriveTime(-0.6, 2.0, drivetrain),
        new TurnTime(-0.5, 1.3, drivetrain),
        new DriveTime(-0.6, 2.0, drivetrain),
        new TurnTime(0.5, 1.3, drivetrain));
  }

  // Allows the system to get the initial pose of this command
  public Pose2d getInitialPose() {
    return m_initialPose;
  }
}
