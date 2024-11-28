// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.OperatorConstants;

public class SwerveModule extends SubsystemBase {
  // Stuff here

  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorCANID, int twistMotorCANID, boolean driveInvert, boolean twistInvert) {
    //Drive Motor
    final CANSparkMax driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
    driveMotor.setInverted(driveInvert);
    driveMotor.setSmartCurrentLimit(OperatorConstants.AMPLimitDrive);
    //driveMotor.setClosedLoopRampRate(twistMotorCANID);
    //driveMotor.setIdleMode(null);
   
    // Steering Motor
    final CANSparkMax steeringMotor = new CANSparkMax(twistMotorCANID, MotorType.kBrushless);
    steeringMotor.setInverted(twistInvert);
    steeringMotor.setSmartCurrentLimit(OperatorConstants.AMPLimitSteering);
   

    // relative encoders on the drive/swerve motors
    final RelativeEncoder relativeSteeringEncoder = steeringMotor.getEncoder();
    relativeSteeringEncoder.setPositionConversionFactor();
    
    final RelativeEncoder relativeDriveEncoder = driveMotor.getEncoder();

    final SparkPIDController twistMotorPIDCOntroller = steeringMotor.getPIDController();
    final SparkPIDController driveMotorPIDCOntroller = driveMotor.getPIDController();
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
