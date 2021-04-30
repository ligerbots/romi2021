package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnMove extends CommandBase implements AutoCommandInterface {
    Drivetrain driveTrain;
    Translation2d target;

    enum Mode {
        COARSE_TURN,
        FINE_TURN,
        MOVE,
        DONE,
    };
    Mode mode;
    static final double meterPerRad = Constants.kTrackwidthMeters / 2;

    double coarseTurnTarget;
    double acceptableAngleError=.1;
    double acceptableDistanceError=.1;

    double leftEncoderStart, rightEncoderStart;
    public TurnMove(Drivetrain driveTrain, Translation2d target) {
        this.driveTrain=driveTrain;
        this.target=target;
        addRequirements(driveTrain);
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
    @Override
    public void initialize() {
        mode=Mode.COARSE_TURN;
        double angle = angleError();
        coarseTurnTarget = meterPerRad * angle;
        leftEncoderStart=driveTrain.getLeftDistanceMeter();
        rightEncoderStart=driveTrain.getRightDistanceMeter();
    }

    @Override
    public void execute() {
        switch (mode){
            case COARSE_TURN:
                if(getAverageTurningDistance() >= Math.abs(coarseTurnTarget)/10){
                    mode=Mode.MOVE;
                    driveTrain.arcadeDrive(0, 0);

                }else{
                    if(coarseTurnTarget>0){
                        driveTrain.arcadeDrive(0, -1);
                    }else{
                        driveTrain.arcadeDrive(0, 1);
                    }
                }
                break;
            case FINE_TURN:
                double angle = angleError();
                if(Math.abs(angle)<acceptableAngleError){
                    driveTrain.arcadeDrive(.1, 0);
                    mode=Mode.MOVE;
                }else if(angle>0){
                    driveTrain.arcadeDrive(0, -.1);
                }else{
                    driveTrain.arcadeDrive(0, .1);
                }
                break;
            case MOVE:
                double distance = distanceError();
                if(distance<acceptableDistanceError){
                    driveTrain.arcadeDrive(0, 0);
                    mode=Mode.DONE;
                }else{
                    driveTrain.arcadeDrive(.1, 0);
                }
                break;
        }
        System.out.println(mode+" "+coarseTurnTarget+" "+getAverageTurningDistance());
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return mode==Mode.DONE;
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }

    double getAverageTurningDistance() {
        double leftDistance = Math.abs(leftEncoderStart-driveTrain.getLeftDistanceMeter());
        double rightDistance = Math.abs(rightEncoderStart-driveTrain.getRightDistanceMeter());
        return (leftDistance + rightDistance) / 2.0;
    }
}
