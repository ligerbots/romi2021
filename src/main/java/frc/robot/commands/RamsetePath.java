package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class RamsetePath extends SequentialCommandGroup implements AutoCommandInterface {
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    // Start at the origin facing the +X direction
    private final Pose2d m_initialPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    public RamsetePath(Drivetrain driveTrain) {
        
        // Define these here, but we may override them within the case statement so we can tune each
        // path individually
        double maxSpeed = 0.8;
        double maxAccel = 0.8;

        // This will make the robot slow down around turns
        TrajectoryConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(1.5);
        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                10);

        TrajectoryConfig configForward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .addConstraint(centripetalAccelerationConstraint);

        Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
                m_initialPose,
                List.of(), 
                new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(0.0)), 
                configForward);

        RamseteCommand ramseteForward = new RamseteCommand(
            forwardTrajectory,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                    Constants.kvVoltSecondsPerMeter,
                                    Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts,
            driveTrain
        );

        addCommands(
            ramseteForward.andThen( () -> driveTrain.tankDriveVolts(0, 0) )
        );
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return m_initialPose;
    }
}