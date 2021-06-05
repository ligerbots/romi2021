package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivetrain;

public class Pursuit extends CommandBase implements AutoCommandInterface{
    Drivetrain drivetrain;
    Translation2d target;
    public Pursuit(Drivetrain drivetrain,Translation2d target){
        this.drivetrain=drivetrain;
        this.target=target;
        addRequirements(drivetrain);
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
    public void initialize() {

    }

    @Override
    public void execute(){
        double angleError = angleError();
        double distance = distanceError();

        double speed = 0.5*Math.max(Math.pow(Math.cos(angleError),5), 0);
        double angle = -MathUtil.clamp(angleError*0.5,-0.2,0.2);
        if(distance<0.5&&speed>0.2)angle = 0;
        drivetrain.arcadeDrive(speed,angle);
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        double angleError = angleError();
        double distance = distanceError();
        return(distance< 0.5 &&Math.cos(angleError)< 0);
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
