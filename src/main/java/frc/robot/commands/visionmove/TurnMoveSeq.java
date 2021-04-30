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

public class TurnMoveSeq extends SequentialCommandGroup implements AutoCommandInterface {
    Drivetrain driveTrain;
    Translation2d target;

    double acceptableAngleError=.1;
    double acceptableDistanceError=.03;

    public TurnMoveSeq(Drivetrain driveTrain, Translation2d target, boolean doMove) {
        this.driveTrain=driveTrain;
        this.target=target;

        addCommands(
                new InstantSuppliedCommand(()->{
                    double angleError = angleError();
                    return(TurnDegFast.getTurnCommand(new Rotation2d(-angleError), driveTrain));
                }, driveTrain),
                new TurnChar.DelaySeconds(.3),
                new FineTurn()
        );
        if(doMove)addCommands(new Move());
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
        int doneticks = 0;
        FineTurn(){
            addRequirements(driveTrain);
        }

        @Override
        public void initialize() {

        }
        @Override
        public void execute(){
            double angle = angleError();
            if(Math.abs(angle)<acceptableAngleError){
                doneticks++;
            }else{
                doneticks=0;
            }
            driveTrain.arcadeDrive(0, clamp(-angle,.1));


        }
        @Override
        public void end(boolean interrupted) {
            driveTrain.arcadeDrive(0, 0);
        }
        @Override
        public boolean isFinished() {
            return doneticks>10;
        }
    }
    class Move extends CommandBase {
        boolean done;
        Move(){
            addRequirements(driveTrain);

        }

        @Override
        public void initialize() {
            System.out.println("MOVE START");

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

                driveTrain.arcadeDrive(speed, 0);

            }
        }
        @Override
        public void end(boolean interrupted) {
            driveTrain.arcadeDrive(0, 0);
            System.out.println("MOVE END");

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
