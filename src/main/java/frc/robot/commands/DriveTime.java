// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTime extends CommandBase {
  private final double m_duration;
  private final double m_speed;
  private final Drivetrain m_drive;
  private long m_startTime;

  /**
   * Creates a new DriveTime. This command will drive your robot for a desired speed and time.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param time How much time to drive in seconds
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveTime(double speed, double time, Drivetrain drive) {
    m_speed = speed;
    m_duration = time * 1000;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_drive.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_speed > 0){
        m_drive.arcadeDrive(m_speed, -0.15);
    } else{
        m_drive.arcadeDrive(m_speed, 0.15);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(System.currentTimeMillis() - m_startTime) >= m_duration){
      return (System.currentTimeMillis() - m_startTime) >= m_duration;
    }
  while (m_drive.getLeftEncoderCount() >= m_drive.getRightEncoderCount()) {
    if (m_drive.getLeftEncoderCount() > m_drive.getRightEncoderCount()) {
      if (m_speed > 0) {
        m_drive.arcadeDrive(m_speed, -0.25);
      } else {
        m_drive.arcadeDrive(m_speed, 0.25);
      }
    }
  }
    return (System.currentTimeMillis() - m_startTime) >= m_duration;
  }
}
