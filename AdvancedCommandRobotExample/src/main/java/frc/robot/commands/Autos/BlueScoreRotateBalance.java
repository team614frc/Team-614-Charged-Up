// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.EventMap;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCone;

public class BlueScoreRotateBalance extends SequentialCommandGroup {
  public BlueScoreRotateBalance() {
    addCommands(
      new ScoreMidCone(),
        new PathPlannerLoadEventMapCommand("BlueScoreRotate", EventMap.ScoreRotateBalance()).withTimeout(15));
  }
}
