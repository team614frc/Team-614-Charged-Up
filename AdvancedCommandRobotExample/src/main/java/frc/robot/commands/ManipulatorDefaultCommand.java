// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class ManipulatorDefaultCommand extends CommandBase {
  /** Creates a new ManipulatorDefaultCommand. */
  Manipulator m_manipulator;
  PowerDistribution pdh;
  double spikeThreshold = Constants.MANIPULATOR_THRESHOLD;

  public ManipulatorDefaultCommand(Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulator = manipulator;
    addRequirements(m_manipulator);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pdh.getCurrent(Constants.INTAKE_MOTOR) > spikeThreshold) {
      m_manipulator.set(Constants.MOTOR_REST_BACK);
      SmartDashboard.putBoolean("Picked Up game piece:", true);
    } else {
      SmartDashboard.putBoolean("Picked Up game piece:", false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
