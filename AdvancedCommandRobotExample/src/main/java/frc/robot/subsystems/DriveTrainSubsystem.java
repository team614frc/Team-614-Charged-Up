package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrainSubsystem extends SubsystemBase {
  public CANSparkMax followerRightMotor = null;
  public CANSparkMax leaderRightMotor = null;
  public CANSparkMax followerLeftMotor = null;
  public CANSparkMax leaderLeftMotor = null;

  DifferentialDrive differentialDrive = null;
  public static AHRS navX;

  private final DifferentialDriveOdometry m_odometry;

  public DriveTrainSubsystem() {
    followerRightMotor = new CANSparkMax(Constants.DRIVETRAIN_FOLLOWER_RIGHT_MOTOR, MotorType.kBrushless);
    leaderRightMotor = new CANSparkMax(Constants.DRIVETRAIN_LEADER_RIGHT_MOTOR, MotorType.kBrushless);
    followerLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_FOLLOWER_LEFT_MOTOR, MotorType.kBrushless);
    leaderLeftMotor = new CANSparkMax(Constants.DRIVETRAIN_LEADER_LEFT_MOTOR, MotorType.kBrushless);

    leaderLeftMotor.getEncoder().setPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
    leaderRightMotor.getEncoder().setPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
    leaderLeftMotor.getEncoder().setVelocityConversionFactor(Constants.kLinearDistanceConversionFactor / 60);
    leaderRightMotor.getEncoder().setVelocityConversionFactor(Constants.kLinearDistanceConversionFactor / 60);

    leaderRightMotor.setInverted(true);

    followerRightMotor.follow(leaderRightMotor, false);
    followerLeftMotor.follow(leaderLeftMotor, false);

    followerRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    leaderRightMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    followerLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);
    leaderLeftMotor.setSmartCurrentLimit(Constants.MOTOR_CURRENT_LIMIT);

    differentialDrive = new DifferentialDrive(leaderLeftMotor, leaderRightMotor);

    navX = new AHRS(SPI.Port.kMXP);

    zeroHeading();
    resetEncoderValues();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeaderLeftEncoderPosition(),
        getLeaderRightEncoderPosition());

    m_odometry.resetPosition(navX.getRotation2d(), getLeaderLeftEncoderPosition(), getLeaderRightEncoderPosition(),
        new Pose2d());
  }

  public void setBreakMode() {
    leaderLeftMotor.setIdleMode(IdleMode.kBrake);
    leaderRightMotor.setIdleMode(IdleMode.kBrake);
    followerLeftMotor.setIdleMode(IdleMode.kBrake);
    followerRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    leaderLeftMotor.setIdleMode(IdleMode.kCoast);
    leaderRightMotor.setIdleMode(IdleMode.kCoast);
    followerLeftMotor.setIdleMode(IdleMode.kCoast);
    followerRightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    differentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  public double getEncoderPositionAverage() {
    double positionAverage = (Math.abs(leaderLeftMotor.getEncoder().getPosition())
        + Math.abs(leaderRightMotor.getEncoder().getPosition())) / 2.0;
    return positionAverage;
  }

  public void setSpeed(double val) {
    leaderRightMotor.set(val);
    leaderLeftMotor.set(val);
    SmartDashboard.putNumber("Drive Left Motor Subsystem Speed Value ", leaderLeftMotor.get());
    SmartDashboard.putNumber("Drivetrain Right Motor Subsystem Speed Value", val);
  }

  public void getSpeed() {
    leaderLeftMotor.get();
    leaderRightMotor.get();
  }

  public void resetEncoderValues() {
    leaderLeftMotor.getEncoder().setPosition(0.0);
    leaderRightMotor.getEncoder().setPosition(0.0);
  }

  public void rotateRight(double val) {
    leaderRightMotor.set(-val);
    leaderLeftMotor.set(val);
  }

  public void rotateLeft(double val) {
    leaderRightMotor.set(val);
    leaderLeftMotor.set(-val);
  }

  public double getLeaderLeftEncoderPosition() {
    return leaderLeftMotor.getEncoder().getPosition();
  }

  public double getLeaderRightEncoderPosition() {
    return leaderRightMotor.getEncoder().getPosition();
  }

  public double getLeaderRightEncoderVelocity() {
    return leaderRightMotor.getEncoder().getVelocity();
  }

  public double getLeaderLeftEncoderVelocity() {
    return leaderLeftMotor.getEncoder().getVelocity();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public static double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoderValues();
    m_odometry.resetPosition(navX.getRotation2d(), getLeaderLeftEncoderPosition(), getLeaderRightEncoderPosition(),
        pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeaderLeftEncoderVelocity(), getLeaderRightEncoderVelocity());
  }

  public void DifferentialDriveVolts(double leftVolts, double rightVolts) {
    leaderLeftMotor.setVoltage(leftVolts);
    leaderRightMotor.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }
  
  public static void zeroHeading() {
    navX.calibrate();
    navX.reset();
  }

  public Gyro getGyro() {
    return navX;
  }
  
  public double getPitch() {
    return navX.getPitch();
  }
  
  public double getRoll() {
    return navX.getRoll();
  }
  
  public double getYaw() {
    return navX.getYaw();
  }

  @Override
  public void periodic() {
    m_odometry.update(navX.getRotation2d(), getLeaderLeftEncoderPosition(), getLeaderRightEncoderPosition());

    SmartDashboard.putNumber("Left Encoder Value Meters", getLeaderLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Value Meters", getLeaderRightEncoderPosition());
    SmartDashboard.putNumber("Average Encoder Distance 2", getEncoderPositionAverage());
    SmartDashboard.putNumber("Gyro Heading", getHeading());
  }
}