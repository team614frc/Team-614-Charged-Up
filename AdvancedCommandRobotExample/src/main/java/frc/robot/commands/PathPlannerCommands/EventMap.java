package frc.robot.Commands.PathPlannerCommands;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Commands.PIDCommand.TiltPIDCommand;
import frc.robot.Commands.SequentialParallelCommands.GroundIntake;
import frc.robot.Commands.SequentialParallelCommands.PchooOverCSBalance;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCone;

public class EventMap {

  public EventMap() {
  }

  public static HashMap<String, Command> ScoreGrab() {
    HashMap<String, Command> ScoreGrab = new HashMap<>();
    ScoreGrab.put("intake", new GroundIntake());
    ScoreGrab.put("armup", new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
    ScoreGrab.put("score2", new ScoreMidCone());
    return ScoreGrab;
  }

  public static HashMap<String, Command> ScoreRotateBalance() {
    HashMap<String, Command> ScoreRotateBalance = new HashMap<>();
    ScoreRotateBalance.put("intake", new GroundIntake());
    ScoreRotateBalance.put("armup", new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
    ScoreRotateBalance.put("pchoobalance", new PchooOverCSBalance());
    return ScoreRotateBalance;
  }

  public static HashMap<String, Command> ScoreBalance() {
    HashMap<String, Command> ScoreBalance = new HashMap<>();
    ScoreBalance.put("balance", new AutoBalance());
    return ScoreBalance;
  }

  public static HashMap<String, Command> Grab1() {
    HashMap<String, Command> Grab1 = new HashMap<>();
    Grab1.put("intake", new GroundIntake());
    return Grab1;
  }
}
