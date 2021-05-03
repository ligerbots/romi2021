// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnGyro extends CommandBase {
  /** Creates a new TurnGyro. */
  private final Drivetrain m_drive;
  private final double m_speed;
  private final double m_degrees;
  // Simply input the desired degrees increased by 35 when the speed is 0.7
  private double m_zaxisStart;
  public TurnGyro(double speed, double degrees, Drivetrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = speed;
    m_degrees = degrees;
    m_drive = driveTrain;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_zaxisStart = m_drive.getGyroAngleZ();
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(m_drive.getGyroAngleZ() - m_zaxisStart) >= m_degrees;
    return Math.abs(m_drive.getGyroAngleZ()) >= m_degrees;
  }
}
