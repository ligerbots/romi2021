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
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;

public class AlliananceAnticsAuto extends SequentialCommandGroup implements AutoCommandInterface {
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    // Start at the origin facing the +X direction

    public static Translation2d grid(double x, double y){
        return(new Translation2d(Units.inchesToMeters(x*15/2),Units.inchesToMeters(y*15/2)));
    }
    private final Pose2d m_initialPose = null;
    Drivetrain driveTrain;
    OnBoardIO onBoardIO;
    public AlliananceAnticsAuto(Drivetrain driveTrain, OnBoardIO onBoardIO) {

        this.driveTrain=driveTrain;
        this.onBoardIO=onBoardIO;
        double maxSpeed = 0.3;
        double maxAccel = 0.3;

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

        TrajectoryConfig configBackwards = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .addConstraint(centripetalAccelerationConstraint).setReversed(true);




        this.addRequirements(driveTrain);
        addCommands(
                driveTrain.new WaitForVision(this::offsetVisionPose),
                addIntakeCommands(
                        new InstantSuppliedCommand(()->{
                            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                    driveTrain.getPose(),
                                    List.of(),
                                    new Pose2d(grid(9, 5), Rotation2d.fromDegrees(180)),
                                    configForward);
                            return(generateRamseteCommand(trajectory));
                        }),
                        0.5
                ),
                driveTrain.new WaitForVision(this::offsetVisionPose),
                addIntakeCommands(
                        new InstantSuppliedCommand(()->{
                            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                    driveTrain.getPose(),
                                    List.of(),
                                    new Pose2d(grid(6, 5), Rotation2d.fromDegrees(180)),
                                    configForward);
                            return(generateRamseteCommand(trajectory));
                        }),
                        1
                ),
                driveTrain.new WaitForVision(this::offsetVisionPose),
                addIntakeCommands(
                        new InstantSuppliedCommand(()->{
                            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                    driveTrain.getPose(),
                                    List.of(),
                                    new Pose2d(grid(3, 5), Rotation2d.fromDegrees(180)),
                                    configForward);
                            return(generateRamseteCommand(trajectory));
                        }),
                        1
                ),
                driveTrain.new WaitForVision(this::offsetVisionPose),

                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(),
                            new Pose2d(grid(2, 5.5), Rotation2d.fromDegrees(180)),
                            configForward);
                    return(generateRamseteCommand(trajectory));
                }),

                driveTrain.new WaitForVision(this::offsetVisionPose),

                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(),
                            new Pose2d(grid(1.2, 5.5), Rotation2d.fromDegrees(180)),
                            configForward);
                    return(generateRamseteCommand(trajectory));
                }),
                getKickCommand(),
                driveTrain.new WaitForVision(this::offsetVisionPose),

                new InstantSuppliedCommand(()-> {
                    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                            new Pose2d(grid(1, 5.5), Rotation2d.fromDegrees(180)),
                            List.of(
                                    grid(2, 5.2),
                                    grid(3, 4.5),
                                    grid(2.8, 3.6),
                                    grid( 2, 3)

                            ),
                            new Pose2d(grid(1, 2.9), Rotation2d.fromDegrees(0)),
                            configBackwards);
                    return(generateRamseteCommand(trajectory2));
                }),
                new InstantCommand(() -> driveTrain.tankDriveVolts(0, 0) )
        );
    }
    void offsetVisionPose(Pose2d original){
        driveTrain.setPose(new Pose2d(original.getX(),original.getY()-.05,original.getRotation()));
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
    Command addIntakeCommands(Command command, double delayIntake){
        return(new ParallelDeadlineGroup(
                command,
                new WaitCommand(delayIntake).andThen(()->onBoardIO.setIntakeServo(true))
        ).andThen(()->onBoardIO.setIntakeServo(false)));
    }
    Command getKickCommand(){
        return(new SequentialCommandGroup(
                new InstantCommand(()->onBoardIO.setIntakeServo(true)),
                new WaitCommand(0.2),
                new InstantCommand(()->onBoardIO.setKickerServo(true)),
                new WaitCommand(0.2),
                new InstantCommand(()->onBoardIO.setKickerServo(false)),
                new WaitCommand(0.4),
                new InstantCommand(()->onBoardIO.setIntakeServo(false))
        ));
    }
    /*
    public void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.plotTrajectory(0, forwardTrajectory);
    }*/
    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return m_initialPose;
    }
}