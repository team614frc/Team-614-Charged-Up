package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.getEventMap;

public class ScoreGrabBalance extends SequentialCommandGroup {
  public ScoreGrabBalance() {
    addCommands(
        new PathPlannerLoadEventMapCommand("ScoreGrabBalance", getEventMap.ScoreGrabBalance()).withTimeout(15));
  }
}
