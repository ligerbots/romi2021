package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Kick extends SequentialCommandGroup implements AutoCommandInterface {
    double backuplength= Units.inchesToMeters(8);
    Drivetrain drivetrain;
    public Kick(Drivetrain drivetrain, Translation2d ballPos, Translation2d target){
        this.drivetrain=drivetrain;
        Translation2d toTarget = target.minus(ballPos);
        toTarget=toTarget.div(toTarget.getNorm());

        Translation2d beginPos = ballPos.plus(toTarget.times(-backuplength));
        addCommands(
                drivetrain.new WaitForVision(drivetrain::setPose),
                new TurnTowardsTarget(target)
        );
    }


    class TurnTowardsTarget extends CommandBase {
        int doneticks = 0;
        double maxSpeed;
        Translation2d target;
        TurnTowardsTarget(Translation2d target){
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
            drivetrain.tankDriveVolts(turnSpeed,-turnSpeed);
            System.out.println(angle+", "+turnSpeed);

        }
        @Override
        public void end(boolean interrupted) {
            drivetrain.tankDriveVolts(0,0);
        }
        @Override
        public boolean isFinished() {
            return doneticks>30;
        }
    }

    /*
    class TurnTowardsTarget extends CommandBase {
        PIDController pid;
        Translation2d target;
        int c =0;
        boolean atPoint = false;
        TurnTowardsTarget(Translation2d target){
            addRequirements(drivetrain);
            this.target=target;
            pid = new PIDController(2,3,1, 0.060);
            pid.setTolerance(.05,.07);
            pid.enableContinuousInput(-Math.PI, Math.PI);
        }
        @Override
        public void initialize() {
            System.out.println("EXEC");
            pid.reset();
            atPoint=false;
        }
        @Override
        public void execute(){
            c++;
            System.out.println(c);

            if(c>=3) {
                Pose2d currentPose = drivetrain.getPose();
                Translation2d delta = target.minus(currentPose.getTranslation());
                double targetangle = Math.atan2(delta.getY(), delta.getX());
                double currentAngle = currentPose.getRotation().getRadians();
                double out = pid.calculate(currentAngle, targetangle);
                out += .03 * Math.signum(out);
                System.out.println("ANGLE ERROR " + pid.getPositionError() + " VEL ERROR " + pid.getVelocityError() + " currentangle " + currentAngle);
                drivetrain.tankDriveVolts(-out, out);


                DifferentialDriveWheelSpeeds wheelSpeeds = drivetrain.getWheelSpeeds();
                double avgWheelSpeeds = (Math.abs(wheelSpeeds.leftMetersPerSecond)+Math.abs(wheelSpeeds.rightMetersPerSecond))/2;
                atPoint = pid.atSetpoint()&&avgWheelSpeeds<.02;

                c=0;
            }
            System.out.println(atPoint);
        }
        @Override
        public void end(boolean interrupted) {
            drivetrain.arcadeDrive(0, 0);
        }
        @Override
        public boolean isFinished() {
            return atPoint;
        }
    }*/
    @Override
    public Pose2d getInitialPose() {
        return null;
    }

}
