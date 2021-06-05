package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.commands.AllianceAnticsAuto.grid;

public class PursuitTest extends SequentialCommandGroup implements AutoCommandInterface {
    public PursuitTest(Drivetrain drivetrain){
        addCommands(new Pursuit(drivetrain,grid(9,5)));
        addCommands(new Pursuit(drivetrain,grid(6,5)));
        addCommands(new Pursuit(drivetrain,grid(3,5)));
    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
