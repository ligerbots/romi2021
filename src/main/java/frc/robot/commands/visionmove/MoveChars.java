package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.FranticFetch;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MoveChars extends SequentialCommandGroup implements AutoCommandInterface {
    public MoveChars(Drivetrain drivetrain){

        ArrayList<List<Command>> commands = new ArrayList<>();
        for(int ticks =0;ticks<100;ticks+=2){
            commands.add(List.of(
                    drivetrain.getVisionResetCommand(),
                    new TurnChar.DelaySeconds(.5),

                    new TurnMoveSeq(drivetrain,FranticFetch.grid(1,1),true),
                    new TurnChar.DelaySeconds(.5),


                    new TurnMoveSeq(drivetrain,FranticFetch.grid(5,6),false),
                    new TurnChar.DelaySeconds(.5),

                    new MoveChar(drivetrain,ticks,.8)
            ));
        }
        Collections.shuffle(commands);
        for(List<Command> cmds:commands){
            for(Command cmd:cmds){
                addCommands(cmd);
            }
        }
        addCommands(new InstantCommand(() -> {
            System.out.println(MoveChar.samples);
        }));
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
