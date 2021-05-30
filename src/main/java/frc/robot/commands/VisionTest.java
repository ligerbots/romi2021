package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class VisionTest extends CommandBase implements AutoCommandInterface {
    Drivetrain drivetrain;
    VisionTest(Drivetrain drivetrain){
        this.drivetrain=drivetrain;
    }
    @Override
    public void execute(){
    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
