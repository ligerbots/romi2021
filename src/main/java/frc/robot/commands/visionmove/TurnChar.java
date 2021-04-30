package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;

public class TurnChar extends SequentialCommandGroup {
    static class Sample {
        double deltaRad;
        int ticks;
        double turn;
        Sample(double deltaAngle,int ticks,double turn){
            this.deltaRad=deltaAngle;
            this.ticks=ticks;
            this.turn=turn;
        }

        @Override
        public String toString() {
            return "Sample{" +
                    "deltaDeg=" + Units.radiansToDegrees(deltaRad) +
                    ", ticks=" + ticks +
                    ", turn=" + turn +
                    '}';
        }
    }
    static ArrayList<Sample> samples=new ArrayList<>();
    Rotation2d startingRotation;
    Drivetrain driveTrain;

    TurnChar(Drivetrain driveTrain, int ticks, double turn){
        this.driveTrain=driveTrain;
        addCommands(
                new DelaySeconds(1),

                driveTrain.new WaitForVision((Pose2d visionMeasurement)->{
                    startingRotation = visionMeasurement.getRotation();
                }),
                new TurnTicks(ticks, turn,driveTrain),

                new DelaySeconds(1),
                driveTrain.new WaitForVision((Pose2d visionMeasurement)->{
                    Rotation2d currentRotation = visionMeasurement.getRotation();
                    Rotation2d diff = startingRotation.minus(currentRotation);
                    double rotRad = diff.getRadians();
                    if(turn>0 && rotRad<0&&ticks>5){
                        rotRad+=Math.PI*2;
                    }else if(turn<0 && rotRad>0&&ticks>5){
                        rotRad-=Math.PI*2;
                    }
                    Sample sample = new Sample(rotRad, ticks, turn);
                    samples.add(sample);
                    System.out.println("Sample: "+ sample);
                })
        );
    }
    public static class DelaySeconds extends CommandBase {
        double seconds;
        double start;
        DelaySeconds(double seconds){
            this.seconds=seconds;
        }
        @Override
        public void initialize() {
            start=System.currentTimeMillis()/1000.;
        }
        @Override
        public boolean isFinished() {
            return(start+seconds<System.currentTimeMillis()/1000.);
        }
    }


}