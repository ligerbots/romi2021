package frc.robot.commands.visionmove;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

public class MoveList extends SequentialCommandGroup implements AutoCommandInterface {
    public MoveList(List<Translation2d> points, Drivetrain drivetrain){
        for(Translation2d point: points){
            addCommands(new TurnMoveSeq(drivetrain,point,true),
                        new InstantCommand(()->{drivetrain.firstSample=true;}),
                        drivetrain.new WaitForVision((Pose2d res)->{}));
        }

    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
