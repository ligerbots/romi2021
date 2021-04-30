package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.Drivetrain;

public class TurnMoveSeq extends CommandBase implements AutoCommandInterface {
    Drivetrain driveTrain;
    Translation2d target;

    static final double meterPerRad = Constants.kTrackwidthMeters / 2;

    double acceptableAngleError=.1;
    double acceptableDistanceError=.03;
    boolean done = false;
    public TurnMoveSeq(Drivetrain driveTrain, Translation2d target) {
        this.driveTrain=driveTrain;
        this.target=target;


    }
    @Override
    public void initialize() {
        double angleError = angleError();
        done=false;
        TurnDegFast.getTurnCommand(new Rotation2d(-angleError), driveTrain)
                .andThen(new TurnChar.DelaySeconds(.3))
                .andThen(new FineTurn())
                .andThen(new Move())
                .andThen(()-> done=true).schedule();
    }

    public static double clamp(double val, double max) {
        return Math.max(-max, Math.min(max, val));
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


    class FineTurn extends CommandBase {
        boolean done;
        FineTurn(){

        }

        @Override
        public void initialize() {

        }
        @Override
        public void execute(){
            double angle = angleError();
            if(Math.abs(angle)<acceptableAngleError){
                done=true;
            }else if(angle>0){
                driveTrain.arcadeDrive(0, -.1);
            }else{
                driveTrain.arcadeDrive(0, .1);
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
    class Move extends CommandBase {
        boolean done;
        Move(){

        }

        @Override
        public void initialize() {

        }
        @Override
        public void execute(){
            double distance = distanceError();
            double angle = angleError();
            if(distance<acceptableDistanceError || Math.abs(angle)>Math.PI/2){
                driveTrain.arcadeDrive(0, 0);
                done=true;
            }else{
                double speed;
                if(distance>.5){
                    speed=1;
                }else{
                    speed=distance+.2;
                }

                driveTrain.arcadeDrive(speed, -clamp(angle/3,.5));

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
    public boolean isFinished() {
        return done;
    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
