// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



//import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.ManipulatorPIDCommand;
import frc.robot.commands.Tilt;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.TimedBasedAuto.ChargeStationNotEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationNotEngaged;
//import edu.wpi.first.wpilibj.XboxController.Button;

public class RobotContainer {
  // Encoder
  // public static Encoder elevatorEncoder = new Encoder(0, 1);
  // Subsystem Initalization
  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static Timer autoTimer;
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static Manipulator manipulator = new Manipulator();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command ChargedStationNotEngaged = new ChargeStationNotEngaged();
  private final Command ScoreMobilityChargeStationNotEngaged = new ScoreMobilityChargeStationNotEngaged();

  // Timed Auto Initaliztion



  public RobotContainer() {
    configureBindings();
    // Calling Arcade Drive Comment
    driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("ChargeStationNotEngaged", ChargedStationNotEngaged);
    m_chooser.addOption("ScoreMobilityChargeStationNotEngaged", ScoreMobilityChargeStationNotEngaged);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }



  public Command getAutonomousCommand() {
    // Gets the selected autonomous command
    return m_chooser.getSelected();

  }

  private void configureBindings() {

    // driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    // driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

    // leftBumper.onTrue(new IntakeWheels()).onFalse(m_TimedAutoCommand);
    // driverController.a().onTrue(wheels.test());
    // m_CommandXboxController.leftBumper().onTrue(new IntakeWheels());
    m_CommandXboxController.button(5).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(9).whileTrue(new ManipulatorPIDCommand(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(4).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT));
    m_CommandXboxController.button(3).whileTrue(new ElevatorPIDCommand(Constants.ELEVATOR_SETPOINT2));
    m_CommandXboxController.button(7).whileTrue(new Tilt(Constants.TILT_UP_SPEED));
    m_CommandXboxController.button(8).whileTrue(new Tilt(Constants.TILT_DOWN_SPEED));
  }

}
