// package frc.robot.commands.PIDCommand;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;

// public class TiltPID extends PIDCommand {
//   /** Creates a new TiltPID. */
//   static double tiltSetpoint;
//   public TiltPID(double tiltSetpoint) { 
//     super(
//         // The controller that the command will use
//         new PIDController(Constants.P_kP, Constants.P_kI, Constants.P_kD),
//         // This should return the measurement
//         RobotContainer.tiltSubsystem::getHeight,
//         // This should return the setpoint (can also be a constant)
//         tiltSetpoint,
//         // This uses the output
//         RobotContainer.tiltSubsystem::set);
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.tiltSubsystem);
//     // Configure additional PID options by calling `getController` here.
//     getController().setTolerance(0.1);
//   }
// Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
