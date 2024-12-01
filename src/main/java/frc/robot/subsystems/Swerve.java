// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;



public class Swerve extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SwerveModule frontRightModule = new SwerveModule(
    OperatorConstants.frontRightDriveCANID, OperatorConstants.frontRightSteeringCANID, OperatorConstants.frontRightDriveInvert, OperatorConstants.frontRightSteeringInvert);
  private final SwerveModule frontLefttModule = new SwerveModule(
    OperatorConstants.frontLeftDriveCANID, OperatorConstants.frontLeftSteeringCANID, OperatorConstants.frontLeftDriveInvert, OperatorConstants.frontLeftSteeringInvert);
  private final SwerveModule backRightModule = new SwerveModule(
    OperatorConstants.backRightDriveCANID, OperatorConstants.backRightSteeringCANID, OperatorConstants.backRightDriveInvert, OperatorConstants.backRightSteeringInvert);
  private final SwerveModule backLeftModule = new SwerveModule(
    OperatorConstants.backLeftDriveCANID, OperatorConstants.backLeftSteeringCANID, OperatorConstants.backLeftDriveInvert, OperatorConstants.backLeftSteeringInvert);


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
