package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.EventMap;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;

public class BalanceAuto extends SequentialCommandGroup {
  public BalanceAuto() {
    addCommands(
        new PathPlannerLoadEventMapCommand("BBCMMCS", EventMap.ScoreAutoBalance()).withTimeout(15));
  }
}
