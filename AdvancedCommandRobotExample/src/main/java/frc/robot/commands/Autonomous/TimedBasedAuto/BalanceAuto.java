package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.EventMap;

public class BalanceAuto extends SequentialCommandGroup {
  public BalanceAuto() {
    addCommands(
        new PathPlannerLoadEventMapCommand("BBCMMCS", EventMap.ScoreAutoBalance()).withTimeout(15));
  }
}
