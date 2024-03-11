// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {

  private final CANSparkMax intake;
  
  private DigitalInput intakeSensor;
  private DutyCycleEncoder throughBore;
  private double velocity;
  
  /** Creates a new Intake. */
  public Intake(){
    intake = new CANSparkMax(INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);      
    intake.setIdleMode(IdleMode.kBrake);

    intakeSensor = new DigitalInput(5);

    velocity = 0.3;
    // SmartDashboard.putNumber("Set Velocity", 0.3);
  }


  //command version
  public Command spinIntake(){
    return runEnd( () -> {
      if(intakeSensor.get()){
        intake.set(-INTAKE_SPEED);
      }
      else {
        intake.set(0);
      }
    }, 
    
    () -> {intake.set(0);});
  }

  public Command outtake() {
    return runEnd(() -> {
      intake.set(velocity);
    }, () -> {intake.set(0);});
  }

  public void feedShooter(){
    intake.set(FEED_SHOOTER_SPEED);
  }

  public void hold(){
      intake.set(-0.4);   
  }


  public void stop(){
    intake.set(0);
  }

  /*
   * Returns the value from the intake Sensor
   */
  public boolean getIntakeSensor(){
    return intakeSensor.get();
  }

 

  public boolean isStuck() {
    return intake.getOutputCurrent() < IntakeConstants.DEFAULT_CURRENT; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
    public void initSendable(SendableBuilder sendableBuilder) {
    //  sendableBuilder.addBooleanProperty("Note Stuck", () -> isStuck(), null);
     sendableBuilder.addBooleanProperty("Note Detected", () -> !getIntakeSensor(), null);
    }
}
