// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.TimedBasedAuto.ChargeStationNotEngaged;
//import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationNotEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.test;
import frc.robot.commands.PIDCommand.ElevatorPIDCommand;
import frc.robot.commands.PIDCommand.ManipulatorPIDCommand;
// import frc.robot.commands.PIDCommand.TiltPID;
import frc.robot.commands.PIDCommand.TiltPID;

public class RobotContainer {
  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static Manipulator manipulator = new Manipulator();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static TiltSubsystem tiltSubsystem = new TiltSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Command chargedStationNotEngaged = new ChargeStationNotEngaged();
  private final Command scoreMobilityChargeStationEngaged = new ScoreMobilityChargeStationEngaged();
  private final Command scoreMobilityChargeStationNotEngaged = new ScoreMobilityChargeStationNotEngaged();
  private final Command test = new test();

  public RobotContainer() {
    configureBindings();
    // Calling Arcade Drive Comment
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Charge Station Not Engaged", chargedStationNotEngaged);
    m_chooser.addOption("Score Mobility Charge Station Engaged", scoreMobilityChargeStationEngaged);
    m_chooser.addOption("Score Mobility Charge Station Not Engaged", scoreMobilityChargeStationNotEngaged);
    m_chooser.addOption("test", test);

    // Put the chooser on the dashboard
    SmartDashboard.putData("Autonomous Modes", m_chooser);

  }

  private void configureBindings() {
    m_CommandXboxController.button(Constants.LEFT_BUMPER).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(Constants.LEFT_STICK_PRESS).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.Y_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT));
    m_CommandXboxController.button(Constants.X_BUTTON).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT2));
    m_CommandXboxController.button(Constants.RIGHT_BUMPER).whileTrue(new TiltPID(Constants.TILT_UP_SETPOINT));
    m_CommandXboxController.button(Constants.RIGHT_STICK_PRESS).whileTrue(new TiltPID(Constants.TILT_DOWN_SETPOINT));
  }

  public Command getAutonomousCommand() {
    // Gets the selected autonomous command
    return m_chooser.getSelected();
  }
}
