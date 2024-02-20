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

    encoder = motorOne.getEncoder();    
    pidController = motorOne.getPIDController();
    
    //This didn't help
    pidController.setP(0.005);

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
    pidController.setReference(velocity, CANSparkBase.ControlType.kVelocity);
  }

  public boolean isUpToSpeed(double targetSpeed){
    return encoder.getVelocity() >= targetSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
  }
}
