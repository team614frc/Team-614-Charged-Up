package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LoadPathplannerTrajectoryCommand;

public class ChargeStationAuto extends SequentialCommandGroup {

  public ChargeStationAuto() {
    addCommands(
      new LoadPathplannerTrajectoryCommand("pathplanner/generatedJSON/inch.wpilib.json", true));
  }
}
