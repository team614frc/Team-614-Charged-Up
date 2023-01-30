// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



//import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ManipulatorPID;
import frc.robot.commands.Tilt;
import frc.robot.commands.Autonomous.TimedBasedAuto.ChargeStationNotEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationEngaged;
import frc.robot.commands.Autonomous.TimedBasedAuto.ScoreMobilityChargeStationNotEngaged;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  // Encoder
  // public static Encoder elevatorEncoder = new Encoder(0, 1);
  // Subsystem Initalization
  public static DriveTrain driveTrain = new DriveTrain();
  public static Timer autoTimer;
  public static CommandXboxController m_CommandXboxController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static Manipulator manipulator = new Manipulator();
  public static Elevator elevator = new Elevator();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Command chargedStationNotEngaged = new ChargeStationNotEngaged();
  private final Command scoreMobilityChargeStationEngaged = new ScoreMobilityChargeStationEngaged();
  private final Command scoreMobilityChargeStationNotEngaged = new ScoreMobilityChargeStationNotEngaged();

  // Timed Auto Initaliztion

  public RobotContainer() {
    configureBindings();
    // Calling Arcade Drive Comment
    driveTrain.setDefaultCommand(new ArcadeDrive());

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Charge_Station_Not_Engaged", chargedStationNotEngaged);
    m_chooser.addOption("Score_Mobility_Charge_Station_Engaged", scoreMobilityChargeStationEngaged);
    m_chooser.addOption("Score_Mobility_Charge_Station_Not_Engaged", scoreMobilityChargeStationNotEngaged);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }


  private void configureBindings() {

    // driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());
    // driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

    // leftBumper.onTrue(new IntakeWheels()).onFalse(m_TimedAutoCommand);
    // driverController.a().onTrue(wheels.test());
    // m_CommandXboxController.leftBumper().onTrue(new IntakeWheels());
    m_CommandXboxController.button(5).whileTrue(new ManipulatorPID(Constants.MANIPULATOR_SETPOINT));
    m_CommandXboxController.button(9).whileTrue(new ManipulatorPID(Constants.MANIPULATOR_SETPOINT2));
    m_CommandXboxController.button(4).whileTrue(new ElevatorPID(Constants.ELEVATOR_SETPOINT));
    m_CommandXboxController.button(3).whileTrue(new ElevatorPID(Constants.ELEVATOR_SETPOINT2));
    m_CommandXboxController.button(7).whileTrue(new Tilt(Constants.TILT_UP_SPEED));
    m_CommandXboxController.button(8).whileTrue(new Tilt(Constants.TILT_DOWN_SPEED));
  }

  public Command getAutonomousCommand() {
    // Gets the selected autonomous command
    return m_chooser.getSelected();
  }
}
