package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FranticFetch extends SequentialCommandGroup implements AutoCommandInterface, Plottable {
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    // Start at the origin facing the +X direction

    public static Translation2d grid(double x, double y){
        return(new Translation2d(Units.inchesToMeters(x*15/2),Units.inchesToMeters(y*15/2)));
    }
    private final Pose2d m_initialPose = new Pose2d(grid(1,3),new Rotation2d(0));
    Trajectory forwardTrajectory;

    public FranticFetch(Drivetrain driveTrain) {

        // Define these here, but we may override them within the case statement so we can tune each
        // path individually
        double maxSpeed = 0.5;
        double maxAccel = 0.5;

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

        forwardTrajectory = TrajectoryGenerator.generateTrajectory(
                m_initialPose,
                List.of(
                        grid(3,3)
                ),
                new Pose2d(grid(3,5), Rotation2d.fromDegrees(90)),
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
    public void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.plotTrajectory(0, forwardTrajectory);
    }
    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return m_initialPose;
    }
}