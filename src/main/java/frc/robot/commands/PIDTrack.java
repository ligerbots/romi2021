
package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivetrain;

public class PIDTrack extends CommandBase implements AutoCommandInterface {
    Drivetrain drivetrain;
    PIDController pidLeft;
    PIDController pidRight;
    final static double callLength = 0.02;
    double targetLeft;
    double targetRight;
    static double totalLeftVolt;
    static double totalRightVolt;
    static double totalLeftDistance;
    static double totalRightDistance;
    double leftStartDistance;
    double rightStartDistance;
    double lastRightDistance;
    double lastLeftDistance;
    Translation2d target;
    public PIDTrack(Drivetrain drivetrain, Translation2d target){
        this.drivetrain=drivetrain;
        addRequirements(drivetrain);
        pidLeft = new PIDController(10,2,0);
        pidRight = new PIDController(10,2,0);
        this.target=target;
    }
    double angleError(){
        Pose2d currentPose = drivetrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());

        double targetangle = Math.atan2(delta.getY(),delta.getX());

        double currentangle = currentPose.getRotation().getRadians();
        double angleerror = targetangle - currentangle;

        if(angleerror>Math.PI) angleerror-=2*Math.PI;
        if(angleerror<-Math.PI) angleerror+=2*Math.PI;
        return(angleerror);
    }
    double distanceError(){
        Pose2d currentPose = drivetrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());

        return(delta.getNorm());
    }
    @Override
    public void initialize(){
        drivetrain.resetEncoders();
        pidLeft.reset();
        pidRight.reset();
        targetLeft=0;
        targetRight=0;
        totalLeftVolt=0;
        totalRightVolt=0;
        leftStartDistance=drivetrain.getLeftDistanceMeter();
        rightStartDistance=drivetrain.getRightDistanceMeter();
        lastLeftDistance = 0;
        lastRightDistance = 0;
    }
    @Override
    public void execute(){
        double angleError = angleError();

        double speed = 0.2*Math.max(Math.pow(Math.cos(angleError),5), 0);
        double angle = -MathUtil.clamp(angleError*0.2,-0.2,0.2);
        System.out.println("left: "+(speed + angle)+" right: "+(speed - angle));

        targetLeft +=  + angle;
        targetRight +=  - angle;

        double leftDistance = drivetrain.getLeftDistanceMeter()-leftStartDistance;
        double rightDistance = drivetrain.getRightDistanceMeter()-rightStartDistance;

        double leftVolts = pidLeft.calculate(leftDistance,targetLeft);
        double rightVolts = pidRight.calculate(rightDistance,targetRight);

        //System.out.println(drivetrain.getLeftDistanceMeter()+", "+drivetrain.getRightDistanceMeter()+", "+leftVolts+", "+rightVolts);
        if(Math.abs(leftDistance)>0.5&&Math.abs(rightDistance)>0.5){
            double leftDistancePerVolt = drivetrain.getLeftDistanceMeter() / totalLeftVolt;
            double rightDistancePerVolt = drivetrain.getRightDistanceMeter() / totalRightVolt;
            double ratio = rightDistancePerVolt/leftDistancePerVolt;
            leftVolts*=ratio;
            //System.out.println("RATIO "+ratio);
        }
        totalLeftVolt+=leftVolts;
        totalRightVolt+=rightVolts;
        totalLeftDistance += leftDistance - lastLeftDistance;
        totalRightDistance += rightDistance - lastRightDistance;
        lastLeftDistance = leftDistance;
        lastRightDistance = rightDistance;
        drivetrain.tankDriveVolts(leftVolts, rightVolts);
    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
