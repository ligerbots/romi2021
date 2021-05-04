package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.FranticFetch;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class EncoderChars extends SequentialCommandGroup implements AutoCommandInterface {
    public EncoderChars(Drivetrain drivetrain){

        ArrayList<List<Command>> commands = new ArrayList<>();
        for(double s =0;s<=1;s+=.03){
            commands.add(List.of(
                    new InstantCommand(()->{drivetrain.firstSample=true;}),
                    drivetrain.new WaitForVision((Pose2d res)->{}),
                    new TurnMoveSeq(drivetrain,FranticFetch.grid(1,3),true),
                    new TurnChar.DelaySeconds(1),


                    new TurnMoveSeq(drivetrain,FranticFetch.grid(12,3),false),
                    new TurnChar.DelaySeconds(1),

                    new EncoderChar(drivetrain, s, s)
            ));
        }
        Collections.shuffle(commands);
        for(List<Command> cmds:commands){
            for(Command cmd:cmds){
                addCommands(cmd);
            }
        }
        addCommands(new InstantCommand(() -> {
            System.out.println(EncoderChar.samples);
        }));
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
