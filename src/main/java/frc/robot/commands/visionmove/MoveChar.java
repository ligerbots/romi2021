package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;

public class MoveChar extends SequentialCommandGroup {
    static class Sample {
        Pose2d movement;

        int ticks;
        double speed;
        Sample(Pose2d movement,int ticks,double speed){
            this.movement=movement;
            this.ticks=ticks;
            this.speed=speed;
        }

        @Override
        public String toString() {
            return "Sample{" +
                    "movementx=" + movement.getX() +
                    ", movementy=" + movement.getY() +
                    ", movementdeg=" + movement.getRotation().getDegrees() +
                    ", ticks=" + ticks +
                    ", speed=" + speed +
                    '}';
        }
    }
    static ArrayList<Sample> samples=new ArrayList<>();
    Pose2d startingPosition;
    Drivetrain driveTrain;

    MoveChar(Drivetrain driveTrain, int ticks, double speed){
        this.driveTrain=driveTrain;
        addCommands(
                new DelaySeconds(1),

                driveTrain.new WaitForVision((Pose2d visionMeasurement)->{
                    startingPosition = visionMeasurement;
                }),
                new MoveTicks(ticks, speed,driveTrain),

                new DelaySeconds(1),
                driveTrain.new WaitForVision((Pose2d visionMeasurement)->{
                    Pose2d diff = visionMeasurement.relativeTo(startingPosition);

                    Sample sample = new Sample(diff, ticks, speed);
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