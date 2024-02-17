// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class Intake extends SubsystemBase {

  private final CANSparkMax intake;
  private final CANSparkMax wrist;
  private final CANSparkMax elevator_left;

  private final CANSparkMax elevator_right;

  private final SparkPIDController pid_elevator;
  
  private DigitalInput intakeSensor;
  private DigitalInput elevatorSwitchLow;
  private DigitalInput elevatorSwitchHigh;

  private AnalogInput throughBore;

  //private DigitalInput wristSwitchIn;
  //private DigitalInput wristSwitchOut;


  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(1, MotorType.kBrushless);
    wrist = new CANSparkMax(2, MotorType.kBrushless);
    elevator_left = new CANSparkMax(3, MotorType.kBrushless);
    elevator_right = new CANSparkMax(4, MotorType.kBrushless);

    throughBore = wrist.getAnalog(Mode.kAbsolute);
      
    pid_elevator = elevator_left.getPIDController();

    elevator_left.setIdleMode(IdleMode.kBrake);
    elevator_right.setIdleMode(IdleMode.kBrake);
    intake.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);

    //right elevaor will follow the left one 
    elevator_right.follow(elevator_left);

    intakeSensor = new DigitalInput(INTAKE_SENSOR);
    elevatorSwitchLow = new DigitalInput(ELEVATORSWITCHLOW);
    elevatorSwitchHigh = new DigitalInput(ELEVATORSWITCHHIGH); 
    
    
  }

  


  public Command spinIntake(){
    return runEnd( () -> {
      if(!intakeSensor.get()){

      intake.set(intakeSpeed);

      }
      else {

        intake.set(0);

      }
    }, 
    
    () -> {intake.set(0);});

  }

  public Command feedShooter(){
      return runEnd( () -> {
        intake.set(feedShooterSpeed);
      }
        
      ,  () -> {
        intake.set(0);
      });
  }

  public Command getSensors(){
    return runEnd( () -> {
      SmartDashboard.putBoolean("intakeSensor", intakeSensor.get());
      SmartDashboard.putBoolean("elevatorSwitchLow", elevatorSwitchLow.get());
      SmartDashboard.putBoolean("elevatorSwitchHigh", elevatorSwitchHigh.get());
      //SmartDashboard.putBoolean("sensor3", sensor3.get());
    },

    () -> {});
  }

  //change to use position not percent
  
  public void rotateWrist(double speed){
    wrist.set(speed);
  }

  public void raiseElevatorwithSpeed(double speed){
    elevator_left.set(speed);
  }

  public void raiseElevatorToPosition(double pos){
    pid_elevator.setReference(pos, ControlType.kPosition);
  }


  /*
   * Returns the value from the intake Sensor
   */
  public boolean getIntakeSenor(){
    return intakeSensor.get();
  }

  public boolean getElevatorSwitchLow(){
    return elevatorSwitchLow.get();
  }

  public boolean getElevatorSwitchHigh(){
    return elevatorSwitchHigh.get();
  }

  public double getElevatorEncoder(){
    return elevator_left.getEncoder().getPosition();
  }

  public AnalogInput getthroughBore(){
    return throughBore;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position",elevator_left.getEncoder().getPosition());
    SmartDashboard.putNumber("wrist", throughBore.getPosition());
  }

  @Override
    public void initSendable(SendableBuilder sendableBuilder) {
      sendableBuilder.setSmartDashboardType("intake Values");
      sendableBuilder.addBooleanProperty("intakeSensor", () -> intakeSensor.get(), null);
      sendableBuilder.addBooleanProperty("elevatorSwitchLow", () -> elevatorSwitchLow.get(), null);
      sendableBuilder.addBooleanProperty("elevatorSwitchHigh", () -> elevatorSwitchHigh.get(), null);
      //sendableBuilder.addBooleanProperty("sensor3", () -> sensor3.get(), null);
    }
}
