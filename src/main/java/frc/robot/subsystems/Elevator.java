// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class Elevator extends SubsystemBase {

  private final CANSparkMax elevator_left;
  private final CANSparkMax elevator_right;
  private final SparkPIDController pid_elevator;

  private boolean climbMode = false;

  private final RelativeEncoder encoder;
  
  /** Creates a new Elevator. */
  public Elevator() {

    elevator_left = new CANSparkMax(LEFT_ELEV_ID, MotorType.kBrushless);
    elevator_right = new CANSparkMax(RIGHT_ElEV_ID, MotorType.kBrushless);
    elevator_left.restoreFactoryDefaults();
    pid_elevator = elevator_left.getPIDController();

    elevator_left.setIdleMode(IdleMode.kBrake);
    elevator_right.setIdleMode(IdleMode.kBrake);

    elevator_left.setSmartCurrentLimit(30);
    elevator_right.setSmartCurrentLimit(30);

    elevator_left.setInverted(true);
    elevator_right.follow(elevator_left,true);
    
    encoder = elevator_left.getEncoder();
  }

  public Boolean getClimbMode(){
    return climbMode;
  }

  public Command toggleClimbMode(){
    return runEnd( () -> {}, ()-> {climbMode = !climbMode;});
  }

   public void raiseElevatorwithSpeed(double speed){
    elevator_left.set(speed);
  }

  public void raiseElevatorToPosition(double pos){
    pid_elevator.setReference(pos, ControlType.kPosition);
  }

  public void holdPosition (){
    pid_elevator.setReference(elevator_left.getEncoder().getPosition(), ControlType.kPosition);
  }


  public double getElevatorEncoder(){
    return elevator_left.getEncoder().getPosition();
  }


  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addBooleanProperty("Climb Mode", () -> climbMode, null);
  }
}
