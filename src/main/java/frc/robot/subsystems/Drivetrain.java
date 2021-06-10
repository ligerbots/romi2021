// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpiutil.math.Pair;
import frc.robot.sensors.RomiGyro;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

class DelayedVisionOdometry{
  DifferentialDriveOdometry odometry;
  OdometrySample[] pastOdometryPoses; //circular buffer for poses
  ArrayList<VisionSample> pastVisionMeasurements; //millis
  Supplier<Double> speedSupplier;
  int nextIndex;
  double updatePeriod;
  Pose2d pose;
  static class OdometrySample {
    Pose2d pose;

    double speed;

    OdometrySample(Pose2d pose, double speed){
      this.pose=pose;
      this.speed=speed;

    }
  }
  static class VisionSample{
    double timeStampMillis;
    Pose2d pastVisionPose;
    Pose2d pastOdometryPose;
    VisionSample(double timeStampMillis, Pose2d pastVisionPose, Pose2d pastOdometryPose){
      this.timeStampMillis=timeStampMillis;
      this.pastVisionPose=pastVisionPose;
      this.pastOdometryPose=pastOdometryPose;
    }
  }
  DelayedVisionOdometry(Rotation2d gyroAngle, double maxDelaySeconds, double updatePeriod, Supplier<Double> speedSupplier){
    odometry= new DifferentialDriveOdometry(gyroAngle);
    pastVisionMeasurements = new ArrayList<>();
    this.updatePeriod=updatePeriod;
    pastOdometryPoses = new OdometrySample[(int)(maxDelaySeconds/updatePeriod)];
    pose = new Pose2d();
    this.speedSupplier=speedSupplier;
  }
  public Pose2d update(Rotation2d gyroAngle, double leftDistanceMeters, double rightDistanceMeters) {
    Pose2d currentOdometryPose = odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters);

    pastOdometryPoses[nextIndex]=new OdometrySample(currentOdometryPose, speedSupplier.get());
    System.out.println("SPEED "+pastOdometryPoses[nextIndex].speed);
    nextIndex++;
    if(nextIndex>=pastOdometryPoses.length)nextIndex=0;
    pose = estimatePosition();
    return(getPoseMeters());
  }
  public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle) {
    odometry.resetPosition(poseMeters, gyroAngle);
    pose = poseMeters;
    Arrays.fill(pastOdometryPoses, null);
    pastVisionMeasurements.clear();
  }
  public Pose2d getPoseMeters() {
    return pose;
  }
  public void addVisionMeasurement(double timeStampMillis, Pose2d pose){
    OdometrySample pastOdometry = getPastOdometryPose(
            (RobotController.getFPGATime() / 1000.0-timeStampMillis)/1000
    );
    if(pastOdometry!=null && pastOdometry.speed<0.2){
      pastVisionMeasurements.add(new VisionSample(timeStampMillis, pose, pastOdometry.pose));
      if(pastVisionMeasurements.size()>10){
        pastVisionMeasurements.remove(0);
      }
    }
  }
  OdometrySample getPastOdometryPose(double secondsAgo){
    if(secondsAgo > updatePeriod* pastOdometryPoses.length) return null;
    int updatesAgo = (int) (secondsAgo/updatePeriod);
    if(updatesAgo >= pastOdometryPoses.length) return null; //too old, not stored
    int index = nextIndex -1 - updatesAgo;
    if(index<0) index+=pastOdometryPoses.length;
    return(pastOdometryPoses[index]);
  }
  Pose2d estimatePosition(){
    int numberSamples =0;
    double x = 0;
    double y = 0;
    double rotx = 0;
    double roty = 0;
    Pose2d currentOdometryPose = odometry.getPoseMeters();

    for (VisionSample pastVisionMeasurement : pastVisionMeasurements) {

      Pose2d pastOdometryPose=pastVisionMeasurement.pastOdometryPose;
      Transform2d pastToCurrent = new Transform2d(pastOdometryPose, currentOdometryPose);
      Pose2d currentEstimatedVision = pastVisionMeasurement.pastVisionPose.transformBy(pastToCurrent);
      x+= currentEstimatedVision.getX();
      y+=currentEstimatedVision.getY();
      rotx+=currentEstimatedVision.getRotation().getCos();
      roty+=currentEstimatedVision.getRotation().getSin();
      numberSamples++;
      //System.out.println("EST: " +currentEstimatedVision);

    }
    //System.out.println("EST POS: " +numberSamples);

    if(numberSamples > 0){
      x/=numberSamples;
      y/=numberSamples;
      rotx/=numberSamples;
      roty/=numberSamples;
      Pose2d res = new Pose2d(x,y,new Rotation2d(rotx,roty));
      System.out.println("uSING VISION ");

      return res;
    }else{
      System.out.println("uSING NONE ");

      return currentOdometryPose;

    }
  }
}

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

  private DelayedVisionOdometry m_odometry;
  //private DifferentialDriveOdometry m_odometry;

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  final Field2d m_field2d = new Field2d();

  Pose2d resetNextTick= null;
  public Pose2d lastVisionPosition = null;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use Meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();

    //m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    m_odometry=new DelayedVisionOdometry(m_gyro.getRotation2d(),5,0.02,()->{
      return(Math.abs(m_leftEncoder.getRate())+Math.abs(m_rightEncoder.getRate()));
    });
    SmartDashboard.putData("field", m_field2d);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();

  }

  public void setPose(Pose2d pose) {
    // The left and right encoders MUST be reset when odometry is reset
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public double getHeading() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    //m_diffDrive.setDeadband(0.001);
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


  ArrayList<Pair<Consumer<Pose2d>,Double>> visionListeners = new ArrayList<>();
  public void addVisionSample(Pose2d pose, double timestamp){
    ListIterator<Pair<Consumer<Pose2d>,Double>> iter = visionListeners.listIterator();
    while(iter.hasNext()){
      Pair<Consumer<Pose2d>,Double> listenerEntry = iter.next();
      if(listenerEntry.getSecond()<=timestamp){
        listenerEntry.getFirst().accept(pose);
        iter.remove();
      }else{
        //System.out.println("Vision measurement too old need:"+listenerEntry.getSecond()+" it " +timestamp);
      }
    }
    m_odometry.addVisionMeasurement(timestamp,pose);
    lastVisionPosition=pose;
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
      visionListeners.add(new Pair<>((Pose2d visionMeasurement) -> {
        doWithResult.accept(visionMeasurement);
        result = visionMeasurement;
      }, (double) (RobotController.getFPGATime() / 1000)));
    }

    @Override
    public boolean isFinished() {
      return result!=null;
    }

    @Override
    public boolean runsWhenDisabled(){
      return true;
    }
  }
  public Command getVisionResetCommand(){
    return new WaitForVision((Pose2d pose)->{
      resetNextTick=pose;
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    // Also update the Field2D object (so that we can visualize this in sim)
    Pose2d pose = getPose();

    m_field2d.setRobotPose(pose);
    SmartDashboard.putNumber("x position", pose.getX());
    SmartDashboard.putNumber("y position", pose.getY());
    SmartDashboard.putNumber("heading", pose.getRotation().getDegrees());

    m_field2d.getObject("line").setPose(pose);

    if(resetNextTick != null){
      setPose(resetNextTick);
      resetNextTick=null;
    }
  }
}
