package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.FranticFetch;
import frc.robot.subsystems.Drivetrain;

public class EncoderChars extends SequentialCommandGroup implements AutoCommandInterface {
    public EncoderChars(Drivetrain drivetrain){

        for(double s =0;s<=1;s+=.03){
            addCommands(
                    new InstantCommand(()->{drivetrain.firstSample=true;}),
                    drivetrain.new WaitForVision((Pose2d res)->{}),
                    new TurnMoveSeq(drivetrain,FranticFetch.grid(1,3),true),
                    new TurnChar.DelaySeconds(1),


                    new TurnMoveSeq(drivetrain,FranticFetch.grid(12,3),false),
                    new TurnChar.DelaySeconds(1),

                    new EncoderChar(drivetrain, s, s)
            );

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
