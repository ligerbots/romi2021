package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;

public class EncoderChar extends CommandBase {
    static class Sample {
        double leftSpeed, rightSpeed, leftEncoder, rightEncoder;
        Sample(double leftSpeed, double rightSpeed, double leftEncoder, double rightEncoder){
            this.leftSpeed=leftSpeed;
            this.rightSpeed=rightSpeed;
            this.leftEncoder=leftEncoder;
            this.rightEncoder=rightEncoder;
        }

        @Override
        public String toString() {
            return "Sample{" +
                    "leftSpeed=" + leftSpeed +
                    ", rightSpeed=" + rightSpeed +
                    ", leftEncoder=" + leftEncoder +
                    ", rightEncoder=" + rightEncoder +
                    '}';
        }
    }
    static int seconds = 2;
    static ArrayList<Sample> samples=new ArrayList<>();
    double left;
    double right;
    int leftEncoderStart;
    int rightEncoderStart;
    double startTime;
    Drivetrain drivetrain;
    EncoderChar(Drivetrain drivetrain, double left, double right){
        this.left=left;
        this.right=right;
        this.drivetrain=drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("ENCODER CHAR");
        leftEncoderStart=drivetrain.getLeftEncoderCount();
        rightEncoderStart=drivetrain.getRightEncoderCount();
        startTime=System.currentTimeMillis()/1000.;
        drivetrain.tankDrive(left,right);
    }
    @Override
    public void execute(){
        drivetrain.tankDrive(left,right);
    }
    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0,0);
        Sample sample = new Sample(
                left,
                right,
                drivetrain.getLeftEncoderCount()-leftEncoderStart,
                drivetrain.getRightEncoderCount()-rightEncoderStart
        );
        System.out.println(sample);
        samples.add(sample);
    }
    @Override
    public boolean isFinished() {
        return(startTime+seconds<System.currentTimeMillis()/1000.);
    }
}