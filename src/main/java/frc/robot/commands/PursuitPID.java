package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivetrain;

public class PursuitPID extends CommandBase implements AutoCommandInterface{
    Drivetrain drivetrain;
    Translation2d target;
    PIDController anglematch;
    public PursuitPID(Drivetrain drivetrain, Translation2d target){
        this.drivetrain=drivetrain;
        this.target=target;
        anglematch=new PIDController(0.4,0,0.2,0.02);
        anglematch.enableContinuousInput(-Math.PI,Math.PI);
        addRequirements(drivetrain);
    }

    double distanceError(){
        Pose2d currentPose = drivetrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());

        return(delta.getNorm());
    }

    @Override
    public void initialize() {
        anglematch.reset();
    }

    @Override
    public void execute(){
        Translation2d toTarget = target.minus(drivetrain.getPose().getTranslation());
        double output = anglematch.calculate(drivetrain.getPose().getRotation().getRadians(),Math.atan2(toTarget.getY(),toTarget.getX()));
        double distance = distanceError();

        double speed = 0.5*Math.max(Math.pow(Math.cos(anglematch.getPositionError()),5), 0);
        if(distance<0.5&&speed>0.2)output = 0;
        System.out.println(distance+" + "+anglematch.getPositionError()+": "+output);

        drivetrain.arcadeDrive(speed,-output);
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        double angleError = anglematch.getPositionError();
        double distance = distanceError();
        return(distance< 0.5 &&Math.cos(angleError)< 0);
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
