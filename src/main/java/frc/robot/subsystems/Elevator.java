// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {

  private final CANSparkMax elevator_left;
  private final CANSparkMax elevator_right;
  private final SparkPIDController pid_elevator;

  private final DigitalInput elevatorSwitchHigh;
  private final DigitalInput elevatorSwitchLow;

  /** Creates a new Elevator. */
  public Elevator() {

    elevator_left = new CANSparkMax(LEFT_ELEV_ID, MotorType.kBrushless);
    elevator_right = new CANSparkMax(RIGHT_ELEV_ID, MotorType.kBrushless);

    elevatorSwitchHigh = new DigitalInput(ELEVATOR_SWITCH_HIGH);
    elevatorSwitchLow = new DigitalInput(ELEVATOR_SWITCH_LOW);

    pid_elevator = elevator_left.getPIDController();

    elevator_left.setIdleMode(IdleMode.kBrake);
    elevator_right.setIdleMode(IdleMode.kBrake);
    
    elevator_right.follow(elevator_left);

  }

   public void raiseElevatorwithSpeed(double speed){
    elevator_left.set(ELEVATOR_SPEED);
  }

  public void raiseElevatorToPosition(double pos){
    pid_elevator.setReference(pos, ControlType.kPosition);
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


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position",elevator_left.getEncoder().getPosition());
  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addBooleanProperty("elevatorSwitchLow", () -> elevatorSwitchLow.get(), null);
    sendableBuilder.addBooleanProperty("elevatorSwitchHigh", () -> elevatorSwitchHigh.get(), null);
  }
}
