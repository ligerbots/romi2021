package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpiutil.math.Pair;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.Drivetrain;

public class TurnMoveFast extends SequentialCommandGroup implements AutoCommandInterface {
    Drivetrain driveTrain;
    Translation2d target;

    double acceptableAngleError=.1;

    double targetAngle;
    int tickMove;

    public TurnMoveFast(Drivetrain driveTrain, Translation2d target) {
        this.driveTrain=driveTrain;
        this.target=target;

        addCommands(
                new TurnChar.DelaySeconds(.3),
                new InstantSuppliedCommand(()->{
                    targetAngle = calcExpectedRotRad();
                    Pair<Integer, Double> fastMoveParam = MoveFast.calib_0p8.getTicksAndAngleForDistance(distanceError());
                    targetAngle-= fastMoveParam.getSecond();
                    tickMove=fastMoveParam.getFirst();
                    System.out.println("tickMove "+tickMove);
                    return(TurnDegFast.getTurnCommand(new Rotation2d(-angleError(targetAngle)), driveTrain));
                }, driveTrain),
                new FineTurn(),
                new InstantSuppliedCommand(()-> (new MoveTicks(tickMove,.8,driveTrain)))
        );

    }


    public static double clamp(double val, double max) {
        return Math.max(-max, Math.min(max, val));
    }

    double calcExpectedRotRad(){
        Pose2d currentPose = driveTrain.getPose();
        Translation2d delta = target.minus(currentPose.getTranslation());

        double targetangle = Math.atan2(delta.getY(),delta.getX());
        return(targetangle);
    }
    double angleError(double targetAngle){
        Pose2d currentPose = driveTrain.getPose();

        double currentangle = currentPose.getRotation().getRadians();
        double angleerror = targetAngle - currentangle;

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
            double angle = angleError(targetAngle);
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

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
