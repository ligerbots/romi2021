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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.commands.visionmove.InstantSuppliedCommand;
import frc.robot.commands.visionmove.TurnChar;
import frc.robot.subsystems.Drivetrain;

public class FranticFetch extends SequentialCommandGroup implements AutoCommandInterface {
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    // Start at the origin facing the +X direction

    public static Translation2d grid(double x, double y){
        return(new Translation2d(Units.inchesToMeters(x*15/2),Units.inchesToMeters(y*15/2)));
    }
    private final Pose2d m_initialPose = new Pose2d(grid(1,3),new Rotation2d(0));
    Trajectory forwardTrajectory;
    Drivetrain driveTrain;
    public FranticFetch(Drivetrain driveTrain) {

        this.driveTrain=driveTrain;

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

        TrajectoryConfig backForward = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .addConstraint(centripetalAccelerationConstraint).setReversed(true);




        addCommands(
                new TurnChar.DelaySeconds(2),
                driveTrain.new WaitForVision((Pose2d result)->{driveTrain.setPose(result);}),
                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(
                                    grid(2.5,3.5)
                            ),
                            new Pose2d(grid(2.8,5), Rotation2d.fromDegrees(90)),
                            configForward);
                    return(generateRamseteCommand(trajectory));
                }, driveTrain),
                driveTrain.new WaitForVision((Pose2d result)->{driveTrain.setPose(result);}),
                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(
                                    grid(3.9,3),
                                    grid(4.3,1.6),
                                    grid(6,1.6)
                            ),
                            new Pose2d(grid(6.5,5), Rotation2d.fromDegrees(270)),
                            backForward);
                    return(generateRamseteCommand(trajectory));
                }, driveTrain),
                driveTrain.new WaitForVision((Pose2d result)->{driveTrain.setPose(result);}),
                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(
                                    grid(6.3,2),
                                    grid(7.1,1),
                                    grid(8.5,1),
                                    grid(8.6,2)
                            ),
                            new Pose2d(grid(8.9,4.7), Rotation2d.fromDegrees(90)),
                            configForward);
                    return(generateRamseteCommand(trajectory));
                }, driveTrain),
                driveTrain.new WaitForVision((Pose2d result)->{driveTrain.setPose(result);}),
                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(
                                    grid(9.3,3.7)
                            ),
                            new Pose2d(grid(11,3.2), Rotation2d.fromDegrees(180)),
                            backForward);
                    return(generateRamseteCommand(trajectory));
                }, driveTrain),
                new InstantCommand(() -> driveTrain.tankDriveVolts(0, 0) )
        );
    }
    RamseteCommand generateRamseteCommand(Trajectory trajectory){
        return(new RamseteCommand(
                trajectory,
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
        ));

    }
    /*
    public void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.plotTrajectory(0, forwardTrajectory);
    }*/
    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return null;
    }
}