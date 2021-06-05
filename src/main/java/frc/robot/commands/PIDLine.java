
package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PIDLine extends CommandBase implements AutoCommandInterface {
    Drivetrain drivetrain;
    PIDController pidLeft;
    PIDController pidRight;
    final static double callLength = 0.02;
    double targetPosition;
    double metersPerSecond;
    static double totalLeftVolt;
    static double totalRightVolt;
    static double totalLeftDistance;
    static double totalRightDistance;
    double leftStartDistance;
    double rightStartDistance;
    double lastRightDistance;
    double lastLeftDistance;
    public PIDLine(Drivetrain drivetrain, double metersPerSecond){
        this.drivetrain=drivetrain;
        addRequirements(drivetrain);
        pidLeft = new PIDController(10,2,0);
        pidRight = new PIDController(10,2,0);
        this.metersPerSecond=metersPerSecond;
    }
    @Override
    public void initialize(){
        drivetrain.resetEncoders();
        pidLeft.reset();
        pidRight.reset();
        targetPosition=0;
        totalLeftVolt=0;
        totalRightVolt=0;
        leftStartDistance=drivetrain.getLeftDistanceMeter();
        rightStartDistance=drivetrain.getRightDistanceMeter();
        lastLeftDistance = 0;
        lastRightDistance = 0;
    }
    @Override
    public void execute(){
        targetPosition+=callLength*metersPerSecond;
        double leftDistance = drivetrain.getLeftDistanceMeter()-leftStartDistance;
        double rightDistance = drivetrain.getRightDistanceMeter()-rightStartDistance;

        double leftVolts = pidLeft.calculate(leftDistance,targetPosition);
        double rightVolts = pidRight.calculate(rightDistance,targetPosition);

        System.out.println(drivetrain.getLeftDistanceMeter()+", "+drivetrain.getRightDistanceMeter()+", "+leftVolts+", "+rightVolts);
        if(Math.abs(leftDistance)>0.5&&Math.abs(rightDistance)>0.5){
            double leftDistancePerVolt = drivetrain.getLeftDistanceMeter() / totalLeftVolt;
            double rightDistancePerVolt = drivetrain.getRightDistanceMeter() / totalRightVolt;
            double ratio = rightDistancePerVolt/leftDistancePerVolt;
            leftVolts*=ratio;
            System.out.println("RATIO "+ratio);
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
