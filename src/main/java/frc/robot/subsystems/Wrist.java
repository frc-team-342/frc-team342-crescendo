// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.util.sendable.Sendable;



public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final CANSparkMax wrist;
  private final SparkPIDController wristController;
  
    private DutyCycleEncoder throughBore;


  public Wrist() {

    wrist = new CANSparkMax(WRIST_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    wristController = wrist.getPIDController();
    wristController.setP(0.01);
    wristController.setSmartMotionAllowedClosedLoopError(0.01, 0);
    wrist.setIdleMode(IdleMode.kBrake);

    throughBore = new DutyCycleEncoder(2);
  }


    public void rotateWrist(double speed){
        wrist.set(speed);
      }
    
    public void rotateWristToPosition(double position){
    wristController.setReference(position, ControlType.kPosition);
  }

  public DutyCycleEncoder getthroughBore(){
    return throughBore;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   @Override
    public void initSendable(SendableBuilder sendableBuilder) {
      // sendableBuilder.addBooleanProperty("Wrist sensor connected", () -> throughBore.isConnected(), null);
      sendableBuilder.addDoubleProperty("Wrist pos", () -> throughBore.getAbsolutePosition(), null);
    }
}
