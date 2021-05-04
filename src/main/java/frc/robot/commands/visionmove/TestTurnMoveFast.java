package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.FranticFetch;
import frc.robot.subsystems.Drivetrain;

public class TestTurnMoveFast extends SequentialCommandGroup implements AutoCommandInterface {
    public TestTurnMoveFast(Drivetrain drivetrain){
        addCommands(
                new TurnMoveFast(drivetrain, FranticFetch.grid(1,3)),
                new TurnChar.DelaySeconds(.5),
                new TurnMoveFast(drivetrain, FranticFetch.grid(11,3))
        );
    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
