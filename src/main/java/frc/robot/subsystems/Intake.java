// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class Intake extends SubsystemBase {

  private final CANSparkMax intake;
  private final CANSparkMax wrist;
  private final CANSparkMax elevator;
  private final SparkPIDController pid_elevator;
  
  private DigitalInput sensor0;
  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private DigitalInput sensor3;


  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(1, MotorType.kBrushless);
    wrist = new CANSparkMax(2, MotorType.kBrushless);
    elevator = new CANSparkMax(3, MotorType.kBrushless);
    pid_elevator = elevator.getPIDController();

    sensor0 = new DigitalInput(INTAKE_SENSOR_0);
    sensor1 = new DigitalInput(INTAKE_SENSOR_1);
    sensor2 = new DigitalInput(INTAKE_SENSOR_2);
    sensor3 = new DigitalInput(INTAKE_SENSOR_3);
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

  public Command getSensors(){
    return runEnd( () -> {
      SmartDashboard.putBoolean("sensor0", sensor0.get());
      SmartDashboard.putBoolean("sensor1", sensor1.get());
      SmartDashboard.putBoolean("sensor2", sensor2.get());
      SmartDashboard.putBoolean("sensor3", sensor3.get());
    },

    () -> {});
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
    SmartDashboard.putNumber("Elevator Position",elevator.getEncoder().getPosition());
   
  }

  @Override
    public void initSendable(SendableBuilder sendableBuilder) {
      sendableBuilder.setSmartDashboardType("intake Values");
      sendableBuilder.addBooleanProperty("sensor0", () -> sensor0.get(), null);
      sendableBuilder.addBooleanProperty("sensor1", () -> sensor1.get(), null);
      sendableBuilder.addBooleanProperty("sensor2", () -> sensor2.get(), null);
      sendableBuilder.addBooleanProperty("sensor3", () -> sensor3.get(), null);
    }
}
