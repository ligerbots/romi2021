package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivetrain;

public class TurnMove extends SequentialCommandGroup implements AutoCommandInterface {
    Drivetrain driveTrain;
    Translation2d target;

    double acceptableAngleError=.1;
    double acceptableDistanceError=.03;

    double expectedRotRad;

    public TurnMove(Drivetrain driveTrain, Translation2d target) {
        this.driveTrain=driveTrain;
        this.target=target;

        addCommands(
                new Turn(),
                new Move()
        );

    }



    double calcExpectedRotRad(){
        Pose2d currentPose = driveTrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());

        double targetangle = Math.atan2(delta.getY(),delta.getX());
        return(targetangle);
    }
    double angleError(){
        Pose2d currentPose = driveTrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());

        double targetangle = Math.atan2(delta.getY(),delta.getX());

        double currentangle = currentPose.getRotation().getRadians();
        double angleerror = targetangle - currentangle;

        if(angleerror>Math.PI) angleerror-=2*Math.PI;
        if(angleerror<-Math.PI) angleerror+=2*Math.PI;
        return(angleerror);
    }

    double distanceError(){
        Pose2d currentPose = driveTrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());
        return(Math.hypot(delta.getX(),delta.getY()));
    }

    class Turn extends CommandBase {
        int doneticks = 0;
        double maxSpeed;
        Turn(){
            addRequirements(driveTrain);
        }



        @Override
        public void initialize() {
            maxSpeed=2.5;
        }

        @Override
        public void execute(){
            double angle = angleError();
            if(Math.abs(angle)<0.1){
                doneticks++;
                maxSpeed*=0.9;
            }else{
                doneticks=0;
                if(maxSpeed<0.65){
                    maxSpeed=0.65;
                }
            }
            if(Math.abs(angle)<1) {
                maxSpeed=Math.min(maxSpeed,1.5);

            }
            if(Math.abs(angle)<.5){
                maxSpeed=Math.min(maxSpeed,0.9);
            }
            if(Math.abs(angle)<.3){
                maxSpeed=Math.min(maxSpeed,0.7);
            }
            if(Math.abs(angle)<.1){
                maxSpeed=Math.min(maxSpeed,0.65);
            }
            double turnSpeed = -maxSpeed*Math.signum(angle);
            driveTrain.tankDriveVolts(turnSpeed,-turnSpeed);
            System.out.println(angle+", "+turnSpeed);

        }
        @Override
        public void end(boolean interrupted) {
            driveTrain.tankDriveVolts(0,0);
        }
        @Override
        public boolean isFinished() {
            return doneticks>30;
        }
    }
    class Move extends CommandBase {
        boolean done;
        Move(){
            addRequirements(driveTrain);

        }

        @Override
        public void initialize() {
            done=false;
        }
        @Override
        public void execute(){
            double distance = distanceError();
            double angle = angleError();
            if(distance<acceptableDistanceError || Math.abs(angle)>Math.PI/2){
                driveTrain.arcadeDrive(0, 0);
                done=true;
            }else{
                double currentangle = driveTrain.getPose().getRotation().getRadians();

                double error = currentangle - expectedRotRad;
                if(error>Math.PI) error-=2*Math.PI;
                if(error<-Math.PI) error+=2*Math.PI;

                double speed;

                //System.out.println(currentangle+" "+expectedRotRad+" "+error);

                driveTrain.arcadeDrive(MathUtil.clamp(distance+.2,-.9,.9), 0/*clamp(error,.3)*/);

            }
        }
        @Override
        public void end(boolean interrupted) {
            driveTrain.arcadeDrive(0, 0);
        }
        @Override
        public boolean isFinished() {
            return done;
        }
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}