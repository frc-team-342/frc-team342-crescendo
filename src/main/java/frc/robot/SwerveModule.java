// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  private CANSparkMax driveMotor;
  private CANSparkMax rotateMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder rotateEncoder;

  private PIDController rotateController;

  private AnalogInput absoluteEncoder;
  private boolean absEncoderReverse;

  private double encoderOffset;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int rotateID, int magEncoderPort, boolean invertRotate, boolean invertDrive, double encoderOffset) {
    
    System.out.println("Drive: " + driveID + " rotate: " + rotateID);
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(rotateID, MotorType.kBrushless);

    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setInverted(invertDrive);

    rotateMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setInverted(invertRotate);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION);
    driveEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION);

    rotateEncoder = rotateMotor.getEncoder();
    rotateEncoder.setPositionConversionFactor(DriveConstants.ROTATE_POSITION_CONVERSION);
    rotateEncoder.setVelocityConversionFactor(DriveConstants.ROTATE_VELOCITY_CONVERSION);

    absoluteEncoder = rotateMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);

    rotateController = new PIDController(DriveConstants.ROTATE_P_VALUE, DriveConstants.ROTATE_I_VALUE, DriveConstants.ROTATE_D_VALUE);
    rotateController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoder();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getRotatePosition() {
    return rotateEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getRotateVelocity() {
    return rotateEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= Math.PI;

    return angle * (absEncoderReverse ? -1.0 : 1.0);
  }

  public double getOffsets() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2 * Math.PI;

    return angle * (absEncoderReverse ? -1.0 : 1.0); 
  }

  public void resetEncoder() {
    driveEncoder.setPosition(0);
    rotateEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public void stop() {
    driveMotor.set(0);
    rotateMotor.set(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotatePosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAX_DRIVE_SPEED);
    rotateMotor.set(rotateController.calculate(getRotatePosition(), state.angle.getRadians()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
