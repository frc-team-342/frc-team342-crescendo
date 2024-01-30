// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;



public class Intake extends SubsystemBase {

  private final CANSparkMax intake;
  private final CANSparkMax wrist;
  private final CANSparkMax elevator;
  private final SparkPIDController pid_elevator;
  private DigitalInput sensor1;


  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(1, MotorType.kBrushless);
    wrist = new CANSparkMax(2, MotorType.kBrushless);
    elevator = new CANSparkMax(3, MotorType.kBrushless);
    pid_elevator = elevator.getPIDController();

    sensor1 = new DigitalInput(Constants.INTAKE_SENSOR_1);
  }


  public Command spinIntake(double speed){
    return runEnd( () -> {
      if(!sensor1.get()){

      intake.set(speed);

      }
      else {

        intake.set(0);
        
      }
    }, 
    
    () -> {intake.set(0);});


  }

  //change to use position not percent
  public void rotateWrist(double angle){
    wrist.set(.5);
  }

  public void raiseElevatorwithSpeed(double speed){
    elevator.set(.5);
  }

  public void raiseElevatorwithPosition(double pos){
    elevator.set(pos);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
