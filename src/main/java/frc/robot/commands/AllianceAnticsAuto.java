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

public class AllianceAnticsAuto extends SequentialCommandGroup implements AutoCommandInterface {
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    // Start at the origin facing the +X direction

    public static Translation2d grid(double x, double y){
        return(new Translation2d(Units.inchesToMeters(x*15/2),Units.inchesToMeters(y*15/2)));
    }
    public static double grid2m(double x){
        return(Units.inchesToMeters(x*15/2));
    }
    private final Pose2d m_initialPose = null;
    Drivetrain driveTrain;
    OnBoardIO onBoardIO;
    public AllianceAnticsAuto(Drivetrain driveTrain, OnBoardIO onBoardIO, TrajectoryPlotter plotter) {

        this.driveTrain=driveTrain;
        this.onBoardIO=onBoardIO;
        double maxSpeed = 0.5;
        double maxSpeedSlow = 0.3;
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

        TrajectoryConfig configBackwards = new TrajectoryConfig(maxSpeed, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .addConstraint(centripetalAccelerationConstraint).setReversed(true);

        TrajectoryConfig configForwardSlow = new TrajectoryConfig(maxSpeedSlow, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .addConstraint(centripetalAccelerationConstraint);

        TrajectoryConfig configBackwardsSlow = new TrajectoryConfig(maxSpeedSlow, maxAccel)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .addConstraint(centripetalAccelerationConstraint).setReversed(true);

        this.addRequirements(driveTrain);
        double ballforward = 1.4;
        addCommands(
                driveTrain.new WaitForVision(driveTrain::setPose),

                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(),
                            new Pose2d(grid(9, 5), Rotation2d.fromDegrees(180)),
                            configForward);
                    plotter.clear();
                    plotter.plotTrajectory(trajectory);
                    return(addIntakeCommands(generateRamseteCommand(trajectory,true),grid2m(9+ballforward), trajectory.getTotalTimeSeconds()));
                }),

                driveTrain.new WaitForVision(driveTrain::setPose),

                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(),
                            new Pose2d(grid(6, 5), Rotation2d.fromDegrees(180)),
                            configForward);
                    plotter.addTrajectory(trajectory);
                    return(addIntakeCommands(generateRamseteCommand(trajectory,true),grid2m(6+ballforward), trajectory.getTotalTimeSeconds()));
                }),

                driveTrain.new WaitForVision(driveTrain::setPose),
                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(),
                            new Pose2d(grid(3, 5), Rotation2d.fromDegrees(180)),
                            configForward);
                    plotter.addTrajectory(trajectory);

                    return(addIntakeCommands(generateRamseteCommand(trajectory,true),grid2m(3+ballforward), trajectory.getTotalTimeSeconds()));
                }),

                driveTrain.new WaitForVision(driveTrain::setPose),

                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(),
                            new Pose2d(grid(2, 5.5), Rotation2d.fromDegrees(180)),
                            configForwardSlow);
                    plotter.addTrajectory(trajectory);

                    return(generateRamseteCommand(trajectory,false));
                }),
                /*
                driveTrain.new WaitForVision(driveTrain::setPose),
                
                new InstantSuppliedCommand(()->{
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(),
                            new Pose2d(grid(1.5, 5.5), Rotation2d.fromDegrees(180)),
                            configForwardSlow);
                    return(generateRamseteCommand(trajectory,false));
                }),*/
                getKickCommand(),
                driveTrain.new WaitForVision(driveTrain::setPose),

                new InstantSuppliedCommand(()-> {
                    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            driveTrain.getPose(),
                            List.of(
                                    grid(2.7, 4),
                                    grid(2.3, 3.3),
                                    grid( 2, 3.2)

                            ),
                            new Pose2d(grid(1, 3.1), Rotation2d.fromDegrees(0)),
                            configBackwardsSlow);
                    plotter.addTrajectory(trajectory);

                    return(generateRamseteCommand(trajectory,false));
                }),
                new InstantCommand(() -> driveTrain.tankDriveVolts(0, 0) )
        );
    }

    RamseteCommand generateRamseteCommand(Trajectory trajectory, boolean isLine){
        return(new RamseteCommand(
                trajectory,
                driveTrain::getPose,
                new RamseteController(isLine?Constants.kRamseteBLine:Constants.kRamseteB, isLine?Constants.kRamseteZetaLine : Constants.kRamseteZeta),
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
    Command addIntakeCommands(Command command, double ballStartX, double ramseteLength){
        return(new ParallelDeadlineGroup(
                command,
                new FunctionalCommand(()->{},()->{
                    Pose2d currentRobotPose = driveTrain.getPose();
                    System.out.println(Math.abs(currentRobotPose.getX()-ballStartX));
                    if(Math.abs(currentRobotPose.getX()-ballStartX)<0.1){
                        onBoardIO.setIntakeServo(true);
                    }else{
                        onBoardIO.setIntakeServo(false);

                    }
                }, (Boolean interrupted)->{}, ()->false))
                .andThen(()->onBoardIO.setIntakeServo(false))

                /*
                new WaitCommand(delayIntake)
                        .andThen(()->onBoardIO.setIntakeServo(true))
                        .andThen(new WaitCommand(ramseteLength-delayIntake-0.5))
                        .andThen(()->onBoardIO.setIntakeServo(false)))*/
        );
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