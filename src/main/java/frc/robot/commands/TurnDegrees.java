// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrees extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_speed;

  int total = 0;
  int calL = 0;
  int calR = 0;
  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double speed, double degrees, Drivetrain drive) {
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_speed);
    total++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    //System.out.println("Done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    double inchPerDegree = Math.PI * 5.551 / 360;
    // Compare distance travelled from start to distance based on degree turn
    //System.out.println("Left: " + m_drive.getLeftDistanceInch());
    //System.out.println("Right: " + m_drive.getRightDistanceInch());
    double error = 0; 
    for(int i = 0; i < 5; i++){
      error += Math.abs(m_drive.getLeftEncoderCount()) - Math.abs(m_drive.getRightEncoderCount());
    }

    while (error <= 0) {
      if(getAverageTurningDistance() >= (inchPerDegree * m_degrees)) return getAverageTurningDistance() >= (inchPerDegree * m_degrees);
      //error = 0;
      if(m_speed < 0){
        m_drive.arcadeDrive(0, -0.4);
        calR++;
      } else{
        m_drive.arcadeDrive(0, 0.4);
        calL++;
      }
      System.out.println("L: " + calL + " R: " + calR + " out of " + total);
    for(int i = 0; i < 5; i++){
      error += Math.abs(m_drive.getLeftEncoderCount()) - Math.abs(m_drive.getRightEncoderCount());
    }
  }
    return getAverageTurningDistance() >= (inchPerDegree * m_degrees);
}

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceInch());
    double rightDistance = Math.abs(m_drive.getRightDistanceInch());
    double i = 0;

    for(; i < 10; i++){
       leftDistance += Math.abs(m_drive.getLeftDistanceInch());
       rightDistance += Math.abs(m_drive.getRightDistanceInch());
    }
    //return (leftDistance + rightDistance) / 2.0;
    return (leftDistance + rightDistance) / i;
  }
}
