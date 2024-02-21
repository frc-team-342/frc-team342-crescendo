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
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class Outtake extends SubsystemBase {
  
  private CANSparkMax motorOne;
  private CANSparkMax motorTwo;

  private RelativeEncoder encoder;
  private SparkPIDController pidController;

  /** Creates a new Shooter. */
  public Outtake() {

    motorOne = new CANSparkMax(OuttakeConstants.MOTOR_ONE_ID, MotorType.kBrushless);
    motorTwo = new CANSparkMax(OuttakeConstants.MOTOR_TWO_ID, MotorType.kBrushless);

    motorTwo.follow(motorOne);
    motorTwo.setInverted(true);

    motorOne.setSmartCurrentLimit(OuttakeConstants.CURRENT_LIMIT);
    motorTwo.setSmartCurrentLimit(OuttakeConstants.CURRENT_LIMIT);

    encoder = motorOne.getEncoder();    
    pidController = motorOne.getPIDController();
    
    //This didn't help
    pidController.setP(6e-5);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0.000015);
    pidController.setSmartMotionAllowedClosedLoopError(100, 0);

    SmartDashboard.putNumber("setP", OuttakeConstants.P_VALUE);
    SmartDashboard.putNumber("setI", OuttakeConstants.I_VALUE);
    SmartDashboard.putNumber("setD", OuttakeConstants.D_VALUE);

    System.out.println("In Constructor");
  }

  public void shootPercent(double speed){
    motorOne.set(speed);
    System.out.println("Shooting at " + speed);
  }

  public void stop(){
    motorOne.set(0);
  }

  public void shootVelocity(double velocity){
    pidController.setReference(velocity, CANSparkBase.ControlType.kVelocity, 0);
  }

  public boolean isUpToSpeed(double targetSpeed){
    return encoder.getVelocity() >= targetSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("P Report",pidController.getP());
    SmartDashboard.putNumber("I Report", pidController.getI());
    SmartDashboard.putNumber("D Report", pidController.getD());

    double p = SmartDashboard.getNumber("setP", 1);
    double i = SmartDashboard.getNumber("setI", 1);
    double d = SmartDashboard.getNumber("setD", 1);
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);

    // System.out.println(encoder.getVelocity());

  }
}
