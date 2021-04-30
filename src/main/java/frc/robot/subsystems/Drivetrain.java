// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.*;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.function.Consumer;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = 0.070;  // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private DifferentialDrivePoseEstimator m_odometry;
  //private DifferentialDriveOdometry m_odometry;

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  final Field2d m_field2d = new Field2d();


  static final Matrix<N5,N1> stateStdDevs = new Matrix<N5,N1>(
          new SimpleMatrix(5,1,true,new double[]{1.5,1.5,1.5,1.5,1.5})
  );
  static final Matrix<N3,N1> localMeasurementStdDevs = new Matrix<N3,N1>(
          new SimpleMatrix(3,1,true,new double[]{.05,.05,.3})
  );
  static final Matrix<N3,N1> visionMeasurementStdDevs = new Matrix<N3,N1>(
          new SimpleMatrix(3,1,true,new double[]{.05,.05,.01})
  );


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use Meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();


    m_odometry = new DifferentialDrivePoseEstimator(m_gyro.getRotation2d(),
            new Pose2d(0,0,new Rotation2d(0)),
            stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs);

    //m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    SmartDashboard.putData("field", m_field2d);
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
    //return m_odometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    // The left and right encoders MUST be reset when odometry is reset
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public double getHeading() {
    return m_odometry.getEstimatedPosition().getRotation().getDegrees();
    //return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate, false);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVolts);  // ???make sure right is negative because sides are opposite
    m_diffDrive.feed();
  }

  public void tankDrive(double left, double right) {
    m_leftMotor.set(left);
    m_rightMotor.set(-right);  // ???make sure right is negative because sides are opposite
    m_diffDrive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds () {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  public Field2d getField2d(){
    return m_field2d;
  }
  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public boolean firstSample = true;
  ArrayList<Consumer<Pose2d>> visionListeners = new ArrayList<>();
  public void addVisionSample(Pose2d pose, double timestamp){
    for(Consumer<Pose2d> visionListener:visionListeners){
      visionListener.accept(pose);
    }
    visionListeners.clear();

    if(firstSample){

      resetEncoders();
      m_odometry = new DifferentialDrivePoseEstimator(m_gyro.getRotation2d(),
              pose,
              stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs);
      firstSample=false;
    }else {
      m_odometry.addVisionMeasurement(pose, timestamp/1000.);
    }
  }
  public class WaitForVision extends CommandBase {
    Pose2d result;
    Consumer<Pose2d> doWithResult;
    public WaitForVision(Consumer<Pose2d> doWithResult) {
      this.doWithResult=doWithResult;
    }

    @Override
    public void initialize() {
      result=null;
      visionListeners.add((Pose2d visionMeasurement)->{
        doWithResult.accept(visionMeasurement);
        result = visionMeasurement;
      });
    }

    @Override
    public boolean isFinished() {
      return result!=null;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(),getWheelSpeeds(),  m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    //m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    // Also update the Field2D object (so that we can visualize this in sim)
    Pose2d pose = getPose();

    m_field2d.setRobotPose(pose);
    SmartDashboard.putNumber("x position", pose.getX());
    SmartDashboard.putNumber("y position", pose.getY());
    SmartDashboard.putNumber("heading", pose.getRotation().getDegrees());
  }


}
