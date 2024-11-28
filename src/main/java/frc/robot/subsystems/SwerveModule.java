// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
  private CANSparkMax driveMotor;
  private CANSparkMax twistMotor;
  private RelativeEncoder relativeTwistEncoder;
  private DutyCycleEncoder absoluteTwistEncoder;
  private RelativeEncoder relativeDriveEncoder;
  private SparkPIDController twistMotorPIDCOntroller;
  private SparkPIDController driveMotorPIDCOntroller;

  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorCANID, int twistMotorCANID, int absoluteTwistEncoderPort, double absoluteEncoderOffset, boolean driveInvert, boolean twistInvert) {
    //Drive Motor
    this.driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
    driveMotor.setInverted(driveInvert);
    driveMotor.setSmartCurrentLimit(OperatorConstants.AMPLimitDrive);
    //driveMotor.setClosedLoopRampRate(twistMotorCANID);
    //driveMotor.setIdleMode(null);
   
    // Twist Motor
    this.twistMotor = new CANSparkMax(twistMotorCANID, MotorType.kBrushless);
    twistMotor.setInverted(twistInvert);
    twistMotor.setSmartCurrentLimit(OperatorConstants.AMPLimitSteering);

    // relative twist encoder
    this.relativeTwistEncoder = twistMotor.getEncoder();
    relativeTwistEncoder.setPositionConversionFactor(OperatorConstants.SteeringMotorFactor);
    
    // absolute twist encoder
    this.absoluteTwistEncoder = new DutyCycleEncoder(absoluteTwistEncoderPort);
    //absoluteTwistEncoder.setDistancePerRotation(absoluteTwistEncoderPort);
    //absoluteTwistEncoder.setPositionOffset(absoluteTwistEncoderPort);

    // Sets twist encoder to correct angle based on absolute zero
    zeroEncoder(absoluteEncoderOffset);

    // relative drive encoder
    this.relativeDriveEncoder = driveMotor.getEncoder();
    relativeDriveEncoder.setPositionConversionFactor(OperatorConstants.driveMotorFactor);
    relativeDriveEncoder.setVelocityConversionFactor(OperatorConstants.driveMotorVelocityFactor);

    this.twistMotorPIDCOntroller = twistMotor.getPIDController();
    this.driveMotorPIDCOntroller = driveMotor.getPIDController();
    configPID();
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

  private void configAngleMotor(){
    
  }

  //TODO call this at beginning of match too
  private void zeroEncoder(double offset){
    relativeTwistEncoder.setPosition(absoluteTwistEncoder.getAbsolutePosition()* 360 - offset);  
  }   

  private void configPID(){
   /*
    driveMotorPIDCOntroller.setP(OperatorConstants.driveMotorP);
    driveMotorPIDCOntroller.setI(OperatorConstants.driveMotorI);
    driveMotorPIDCOntroller.setD(OperatorConstants.driveMotorD);
    driveMotorPIDCOntroller.setFF(OperatorConstants.driveMotorFF);*/
 
    twistMotorPIDCOntroller.setP(OperatorConstants.twistMotorP);
    twistMotorPIDCOntroller.setI(OperatorConstants.twistMotorI);
    twistMotorPIDCOntroller.setD(OperatorConstants.twistMotorD);
    twistMotorPIDCOntroller.setFF(OperatorConstants.twistMotorFF);
    twistMotorPIDCOntroller.setPositionPIDWrappingEnabled(true);
    twistMotorPIDCOntroller.setPositionPIDWrappingMaxInput(180);
    twistMotorPIDCOntroller.setPositionPIDWrappingMinInput(-180);
  }

  // Subtracts two angles
  private double angleSubtractor (double firstAngle, double secondAngle) {
    // 
     double result = ((firstAngle - secondAngle) + 360180)%360 - 180;
     return result;
    
  }

  double returnModuleSpeed(){
    // TODO convert module speed to meters per second
    double moduleSpeed = relativeDriveEncoder.getVelocity();
    moduleSpeed = 0;
    return moduleSpeed;
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
