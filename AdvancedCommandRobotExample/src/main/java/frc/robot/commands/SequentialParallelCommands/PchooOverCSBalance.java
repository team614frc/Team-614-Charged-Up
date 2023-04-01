// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SequentialParallelCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PathPlannerCommands.AutoBalance;
import frc.robot.commands.SimpleCommands.Intake;

public class PchooOverCSBalance extends SequentialCommandGroup {
  /** Creates a new PchooOverCSBalance. */
  public PchooOverCSBalance() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Intake(Constants.MANIPULATOR_SPEED_PCHOO).withTimeout(0.5),
        new AutoBalance());
  }
}
