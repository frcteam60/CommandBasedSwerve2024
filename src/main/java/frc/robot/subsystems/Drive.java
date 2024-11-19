// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drive extends SubsystemBase {
  // Drive motors
  private final CANSparkMax frontLeftDriveMotor = new CANSparkMax(OperatorConstants.frontLeftDriveCANID, MotorType.kBrushless);
  private final CANSparkMax frontRightDriveMotor = new CANSparkMax(OperatorConstants.frontRightDriveCANID, MotorType.kBrushless);
  private final CANSparkMax backRightDriveMotor = new CANSparkMax(OperatorConstants.backRightDriveCANID, MotorType.kBrushless);
  private final CANSparkMax backtLeftDriveMotor = new CANSparkMax(OperatorConstants.backLeftDriveCANID, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_DifferentialDrive = new DifferentialDrive(frontLeftDriveMotor::set, frontRightDriveMotor::set);

  /** Creates a new ExampleSubsystem. */
  public Drive() {
    backtLeftDriveMotor.follow(frontLeftDriveMotor);
    backRightDriveMotor.follow(frontRightDriveMotor);
  }
  
  public Command simpleDriveCommand(DoubleSupplier forward, DoubleSupplier rotation){
    // simple control controls both side wheels like westcoast not swerve
    return run(() -> m_DifferentialDrive.arcadeDrive(forward.getAsDouble(), 0))
        .withName("simpleDrive");
  }
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
