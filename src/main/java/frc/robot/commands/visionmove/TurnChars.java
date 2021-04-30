package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.Drivetrain;

public class TurnChars extends SequentialCommandGroup implements AutoCommandInterface {
    public TurnChars(Drivetrain drivetrain){

        for(int i =0;i<25;i+=1){
            for(int j=0;j<2;j++) {
                addCommands(new TurnChar(drivetrain, i, 1), new TurnChar(drivetrain, i, -1));
            }
        }
        addCommands(new InstantCommand(() -> {
            System.out.println(TurnChar.samples);
        }));
    }

    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
