// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Intake extends SubsystemBase {

  private final CANSparkMax intake;
  private final CANSparkMax wrist;
  private final CANSparkMax elevator;


  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(0, MotorType.kBrushless);
    wrist = new CANSparkMax(0, MotorType.kBrushless);
    elevator = new CANSparkMax(0, MotorType.kBrushless);

  }


  public Command spinIntake(double speed){
    return runEnd( () -> {intake.set(speed);}, () -> {intake.set(0);});


  }

  public void rotateWrist(double angle){
    wrist.set(.5);
  }

  public void raiseElevatorwithSpeed(double speed){
    elevator.set(speed);
  }

  public void raiseElevatorwithPosition(double pos){
    elevator.set(pos);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
