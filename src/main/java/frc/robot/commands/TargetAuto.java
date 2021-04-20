package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class TargetAuto extends CommandBase implements AutoCommandInterface {

    private final Pose2d m_initialPose = null;

    Translation2d target;
    Drivetrain driveTrain;

    double onTargetTime;
    double lastReset = 0;
    public TargetAuto(Drivetrain driveTrain, Translation2d target){
        this.driveTrain=driveTrain;
        this.target=target;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetEncoders();
    }
    public static double clamp(double val, double max) {
        return Math.max(-max, Math.min(max, val));
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d currentPose = driveTrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());
        double targetangle = Math.atan2(delta.getY(),delta.getX());
        double targetdistance = Math.hypot(delta.getX(),delta.getY());
        double currentangle = currentPose.getRotation().getRadians();
        double angleerror = targetangle - currentangle;
        if(angleerror>Math.PI)angleerror-=2*Math.PI;
        if(angleerror<-Math.PI)angleerror+=2*Math.PI;
        driveTrain.arcadeDrive(clamp(targetdistance*Math.cos(angleerror)+.05,1), clamp(-angleerror/4,1));

        boolean onTarget = targetdistance<.02;
        if(onTargetTime>=0&&(!onTarget)){
            onTargetTime=-1;
        }else if(onTargetTime<0 && onTarget){
            onTargetTime = System.currentTimeMillis();
        }
        if(lastReset + 1000<System.currentTimeMillis()){
            lastReset = System.currentTimeMillis();
            driveTrain.firstSample=true;
        }
        System.out.println("angleerror: "+ angleerror+ " targetdistance: "+targetdistance);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.arcadeDrive(0,0);
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return m_initialPose;
    }

    @Override
    public boolean isFinished() {
        return onTargetTime>=0 && (System.currentTimeMillis()-onTargetTime)>1000;
    }

}