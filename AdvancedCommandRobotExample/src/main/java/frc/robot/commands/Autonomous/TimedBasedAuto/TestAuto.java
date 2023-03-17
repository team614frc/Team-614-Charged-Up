// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.TimedBasedAuto;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ForwardBalance;
import frc.robot.commands.PathPlannerLoadEventMapCommand;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new ForwardBalance());
    addCommands(
        new PathPlannerLoadEventMapCommand("pathplanner/generatedJSON/Marker1Path.wpilib.json", true, eventMap));
  }
}
