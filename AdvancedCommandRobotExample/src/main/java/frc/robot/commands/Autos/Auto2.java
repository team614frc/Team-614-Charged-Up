package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class Auto2 extends SequentialCommandGroup {
  public Auto2() {
    addCommands(
        new ScoreMidCubeAuto(),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/BTCMCS.wpilib.json", true));
  }
}
