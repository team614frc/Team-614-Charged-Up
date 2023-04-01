// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:AdvancedCommandRobotExample/src/main/java/frc/robot/commands/SequentialParallelCommands/PchooOverCS.java
package frc.robot.commands.SequentialParallelCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SimpleCommands.Intake;
========
package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCone;
>>>>>>>> e29c382f9cec7b608381b875253f7deb1d6ce36f:AdvancedCommandRobotExample/src/main/java/frc/robot/commands/Autos/Auto8.java

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
<<<<<<<< HEAD:AdvancedCommandRobotExample/src/main/java/frc/robot/commands/SequentialParallelCommands/PchooOverCS.java
public class PchooOverCS extends SequentialCommandGroup {
  /** Creates a new PchooOverCS. */
  public PchooOverCS() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new TiltPIDCommand(Constants.TILT_PCHOO_SETPOINT).withTimeout(0.5),
    new Intake(Constants.MANIPULATOR_SPEED_PCHOO).withTimeout(0.5));
========
public class Auto8 extends SequentialCommandGroup {
  /** Creates a new Auto8. */
  public Auto8() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreMidCone(),
      new PathPlannerLoadPathCommand("pathplanner/generatedJSON/RTCMM.wpilib.json", true)
    );
>>>>>>>> e29c382f9cec7b608381b875253f7deb1d6ce36f:AdvancedCommandRobotExample/src/main/java/frc/robot/commands/Autos/Auto8.java
  }
}