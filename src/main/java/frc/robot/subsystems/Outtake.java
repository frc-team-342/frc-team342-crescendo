// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class Outtake extends SubsystemBase {
  
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private RelativeEncoder encoder;
  private SparkPIDController pidController;

  private double velocity;

  /** Creates a new Shooter. */
  public Outtake() {

    leftMotor = new CANSparkMax(OuttakeConstants.MOTOR_ONE_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(OuttakeConstants.MOTOR_TWO_ID, MotorType.kBrushless);

    leftMotor.setInverted(true);
    // rightMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);

    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);

    leftMotor.setSmartCurrentLimit(OuttakeConstants.CURRENT_LIMIT);
    rightMotor.setSmartCurrentLimit(OuttakeConstants.CURRENT_LIMIT);

    encoder = leftMotor.getEncoder();    
    pidController = leftMotor.getPIDController();

    //This didn't help
    pidController.setP(OuttakeConstants.P_VALUE);
    pidController.setFF(OuttakeConstants.FF_VALUE);
    pidController.setSmartMotionAllowedClosedLoopError(100, 0);
    pidController.setOutputRange(0, 5000);

    SmartDashboard.putNumber("setP", OuttakeConstants.P_VALUE);
    SmartDashboard.putNumber("setI", OuttakeConstants.I_VALUE);
    SmartDashboard.putNumber("setD", OuttakeConstants.D_VALUE);
    SmartDashboard.putNumber("setFF", OuttakeConstants.FF_VALUE);
  }

  public void shootPercent(double speed){
    leftMotor.set(speed);
    System.out.println("Shooting at " + speed);
  }

  public void stop(){
    leftMotor.set(0);
  }

  public void shootVelocity(double velocity){
    pidController.setReference(velocity, ControlType.kVelocity);
  }

  public boolean isUpToSpeed(double targetSpeed){
    return encoder.getVelocity() >= targetSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Desired Velocity", velocity);

    double p = SmartDashboard.getNumber("setP", 1);
    double i = SmartDashboard.getNumber("setI", 1);
    double d = SmartDashboard.getNumber("setD", 1);
    double ff = SmartDashboard.getNumber("setFF", 0);
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setFF(ff);
  }
}
