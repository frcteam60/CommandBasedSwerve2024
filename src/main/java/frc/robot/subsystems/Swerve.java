// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;



public class Swerve extends SubsystemBase {
  // dimensions between wheels center-to-center
  public static final double length = 22.25;
  public static final double width = 22.25;

  // Gyro
  double gyro_radians;
  double gyro_degrees;
  double temp;
  double forward;
  double strafe;
  double turning;
  double diagonal;
  // Desired Position
  double desiredX = 0;
  double desiredY = 0;
  double desiredYaw = 0;
  double XError;
  double YError;
  double YawError;
  //
  double newX;
  double newY;
  double newRobotAngle;
  Rotation2d gyroRotation2d;
  Pose2d robotPose2d;
  // ***

  // Subtracts two angles
  public double angleSubtractor(double firstAngle, double secondAngle) {
    double result = (((firstAngle - secondAngle) + 360180) % 360) - 180;
    return result;
  }
  
  // ***
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  double yawOffset = 0;

  // ***
  double getGyroRobotYaw() {
    return angleSubtractor(yawOffset, gyro.getYaw());
  }

  void zeroGyro() {
    yawOffset = 0;
    gyro.zeroYaw();
  }

  void setYawOffset(double newYawOffset) {
    yawOffset = newYawOffset;
  }

  /** Creates a new ExampleSubsystem. */
  private final SwerveModule frontRightModule = new SwerveModule(
    OperatorConstants.frontRightDriveCANID, OperatorConstants.frontRightSteeringCANID, OperatorConstants.frontRightAbsoluteEncoderPort, OperatorConstants.frontRightAbsoluteEncoderOffset, OperatorConstants.frontRightDriveInvert, OperatorConstants.frontRightSteeringInvert);
  
  private final SwerveModule frontLefttModule = new SwerveModule(
    OperatorConstants.frontLeftDriveCANID, OperatorConstants.frontLeftSteeringCANID, OperatorConstants.frontLeftAbsoluteEncoderPort, OperatorConstants.frontLeftAbsoluteEncoderOffset, OperatorConstants.frontLeftDriveInvert, OperatorConstants.frontLeftSteeringInvert);
  
  private final SwerveModule backRightModule = new SwerveModule(
    OperatorConstants.backRightDriveCANID, OperatorConstants.backRightSteeringCANID, OperatorConstants.backRightAbsoluteEncoderPort, OperatorConstants.backRightAbsoluteEncoderOffset, OperatorConstants.backRightDriveInvert, OperatorConstants.backRightSteeringInvert);
  
  private final SwerveModule backLeftModule = new SwerveModule(
    OperatorConstants.backLeftDriveCANID, OperatorConstants.backLeftSteeringCANID, OperatorConstants.backLeftAbsoluteEncoderPort, OperatorConstants.backLeftAbsoluteEncoderOffset, OperatorConstants.backLeftDriveInvert, OperatorConstants.backLeftSteeringInvert);


  public Swerve() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
