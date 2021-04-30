package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.Drivetrain;

public class TestTurnDegFast extends SequentialCommandGroup implements AutoCommandInterface {
    public TestTurnDegFast(Drivetrain drivetrain){
        addCommands(
                TurnDegFast.getTurnCommand(Rotation2d.fromDegrees(90),drivetrain),
                new TurnChar.DelaySeconds(.5),
                TurnDegFast.getTurnCommand(Rotation2d.fromDegrees(-90),drivetrain)
                );
    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
