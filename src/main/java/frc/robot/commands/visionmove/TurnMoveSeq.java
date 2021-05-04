package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.Drivetrain;

public class TurnMoveSeq extends SequentialCommandGroup implements AutoCommandInterface {
    Drivetrain driveTrain;
    Translation2d target;

    double acceptableAngleError=.1;
    double acceptableDistanceError=.03;

    double expectedRotRad;

    public TurnMoveSeq(Drivetrain driveTrain, Translation2d target, boolean doMove) {
        this.driveTrain=driveTrain;
        this.target=target;

        addCommands(
                /*new InstantSuppliedCommand(()->{
                    expectedRotRad= calcExpectedRotRad();
                    double angleError = angleError();
                    return(TurnDegFast.getTurnCommand(new Rotation2d(-angleError), driveTrain));
                }, driveTrain),*/
                new FineTurn()
        );

        if(doMove)addCommands(new Move());
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

    /*
    class FineTurn extends CommandBase {
        PIDController pid;
        int cnt;
        FineTurn(){
            addRequirements(driveTrain);
            pid = new PIDController(.2,.1,.5, 0.020);
            pid.setTolerance(.1,.07);
            pid.enableContinuousInput(-Math.PI, Math.PI);

        }

        @Override
        public void initialize() {
            cnt=0;
        }
        @Override
        public void execute(){
            Pose2d currentPose = driveTrain.getPose();
            Translation2d delta = target.minus(currentPose.getTranslation());

            double targetangle = Math.atan2(delta.getY(),delta.getX());

            double currentAngle =  currentPose.getRotation().getRadians();

            double out = pid.calculate(currentAngle, targetangle);
            out+=.03*Math.signum(out);
            System.out.println("ANGLE ERROR "+pid.getPositionError()+" VEL ERROR "+ pid.getVelocityError()+ " currentangle "+currentAngle );
            driveTrain.tankDrive(-out,out);
            cnt++;
        }
        @Override
        public void end(boolean interrupted) {
            System.out.println("FINETURN " + interrupted);
            driveTrain.arcadeDrive(0, 0);
        }
        @Override
        public boolean isFinished() {

            DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.getWheelSpeeds();
            double avgWheelSpeeds = (Math.abs(wheelSpeeds.leftMetersPerSecond)+Math.abs(wheelSpeeds.rightMetersPerSecond))/2;
            return ((pid.atSetpoint()&&avgWheelSpeeds<.02)||cnt>45);
        }
    }*/
    class FineTurn extends CommandBase {
        int doneticks = 0;
        double maxSpeed;

        FineTurn(){
            addRequirements(driveTrain);
        }

        @Override
        public void initialize() {
            maxSpeed=.35;
        }
        @Override
        public void execute(){
            double angle = angleError();
            if(Math.abs(angle)<acceptableAngleError){
                doneticks++;
            }else{
                doneticks=0;
            }
            if(Math.abs(angle)<1) {
                maxSpeed=Math.min(maxSpeed,.25);

            }
            if(Math.abs(angle)<.5){
                maxSpeed=Math.min(maxSpeed,.2);
            }
            if(Math.abs(angle)<.3){
                maxSpeed=Math.min(maxSpeed,.1);
            }
            driveTrain.arcadeDrive(0, -maxSpeed*Math.signum(angle));


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

                driveTrain.arcadeDrive(clamp(distance+.2,.9), 0/*clamp(error,.3)*/);

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
