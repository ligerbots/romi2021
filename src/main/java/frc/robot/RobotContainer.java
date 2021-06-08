// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import edu.wpi.first.wpilibj2.command.button.Button;


import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

import frc.robot.subsystems.OnBoardIO;

import static frc.robot.commands.AllianceAnticsAuto.grid;
// import frc.robot.subsystems.OnBoardIO.ChannelMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(OnBoardIO.ChannelMode.INPUT, OnBoardIO.ChannelMode.INPUT);
  private final Vision m_vision = new Vision(m_drivetrain);

  // Assumes a game controller plugged into channnel 0
  private final XboxController m_xbox = new XboxController(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<AutoCommandInterface> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> input_mode = new SendableChooser<>();
  TrajectoryPlotter m_plotter = new TrajectoryPlotter(m_drivetrain.getField2d());


  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    // Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    // onboardButtonA
    //     .whenActive(new PrintCommand("Button A Pressed"))
    //     .whenInactive(new PrintCommand("Button A Released"));


    // Setup SmartDashboard options
    m_chooser.setDefaultOption("PIDTrack", new PIDTrack(m_drivetrain,grid(3,5)));
    m_chooser.addOption("PIDLine", new PIDLine(m_drivetrain,0.5));
    m_chooser.addOption("TurnMoveTest", new TurnMoveTest(m_drivetrain));
    m_chooser.addOption("Pursuittest", new PursuitTest(m_drivetrain));

    m_chooser.addOption("Pursuit", new Pursuit(m_drivetrain,grid(3,5)));
    m_chooser.addOption("Kick", new Kick(m_drivetrain, grid(9,5), grid(1,5.5)));
    m_chooser.addOption("Alliance Antics Auto", new AllianceAnticsAuto(m_drivetrain, m_onboardIO));
    m_chooser.addOption("Alliance Antics Auto Jack", new AllianceAnticsAutoJack(m_drivetrain, m_onboardIO));
    m_chooser.addOption("Auto Reset Ramsete", new TargetAutoRamsete(
            m_drivetrain,
            new Pose2d(Units.inchesToMeters(15./2),Units.inchesToMeters(15+15./2),new Rotation2d(0))
    ));
    m_chooser.addOption("Vision test", new VisionTest(m_drivetrain));
    m_chooser.addOption("Vision Calib", new VisionCalib(m_drivetrain));

    input_mode.setDefaultOption("Keyboard","keyboard");
    input_mode.addOption("xbox","xbox");
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData(input_mode);
  }

  /*public XboxController getXbox(){
    return m_xbox;
  }*/
  public boolean kickerPressed(){
    if(input_mode.getSelected().equals("keyboard")){
      return(m_xbox.getAButton());
    }else if(input_mode.getSelected().equals("xbox")){
      return(m_xbox.getTriggerAxis(Hand.kRight)>0.5);
    }
    return false;
  }
  public boolean intakePressed(){
    if(input_mode.getSelected().equals("keyboard")){
      return(m_xbox.getBButton());
    }else if(input_mode.getSelected().equals("xbox")){
      return(m_xbox.getTriggerAxis(Hand.kLeft)>0.5);
    }
    return false;
  }
  public boolean batteryPressed(){
    if(input_mode.getSelected().equals("keyboard")){
      return(m_xbox.getXButton());
    }else if(input_mode.getSelected().equals("xbox")){
      return(m_xbox.getXButton());
    }
    return false;
  }
  public Drivetrain getDriveTrain() {
    return m_drivetrain;
  }
  public OnBoardIO getOnBoardIO() {
    return m_onboardIO;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public AutoCommandInterface getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
            m_drivetrain,
            () -> {
              if(input_mode.getSelected().equals("keyboard")) {
                return (-m_xbox.getY(Hand.kLeft));
              }else if(input_mode.getSelected().equals("xbox")){
                return (-m_xbox.getY(Hand.kLeft));
              }
              return 0.;
            },
            () -> {
              if(input_mode.getSelected().equals("keyboard")) {
                return (m_xbox.getX(Hand.kLeft));
              }else if(input_mode.getSelected().equals("xbox")){
                return (m_xbox.getX(Hand.kRight));
              }
              return 0.;
            }
    );
  }
}
