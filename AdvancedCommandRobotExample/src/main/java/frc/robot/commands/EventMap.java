package frc.robot.commands;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.SimpleCommands.Intake;

public class EventMap {

  public EventMap() {
  }

  public static HashMap<String, Command> ScoreGrabBalance() {
    HashMap<String, Command> ScoreGrabBalance = new HashMap<>();
    ScoreGrabBalance.put("intake", new Intake(Constants.MANIPULATOR_SPEED_INTAKE));
    ScoreGrabBalance.put("balance", new ForwardBalance());
    return ScoreGrabBalance;
  }

  public static HashMap<String, Command> ScoreAutoBalance() {
    HashMap<String, Command> ScoreAutoBalanceBlue = new HashMap<>();
    ScoreAutoBalanceBlue.put("balance", new ForwardBalance());
    return ScoreAutoBalanceBlue;
  }
}
