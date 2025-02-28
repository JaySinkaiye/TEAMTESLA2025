// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // private final TalonFX rotateMotor = new TalonFX(60);
  private final TalonFX intakeMotor = new TalonFX(52);
  private final TalonFX secondIntakeMotor = new TalonFX(53);

  // private DutyCycleOut rotate = new DutyCycleOut(0);
  private DutyCycleOut intake = new DutyCycleOut(0);

  public Intake() {
    // applyRotateMotorConfigs(InvertedValue.Clockwise_Positive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void setRotateSpeed(double speed){
  //   rotate.Output = speed;
  //   rotateMotor.setControl(rotate);
  // }

  // public Command manualRotate(double speed){
  //   rotate.Output = speed;
  //   return runOnce(
  //     ()->{
  //         rotateMotor.setControl(rotate);
  //         }
  //     );
  // }

  public void setIntakeSpeed(double speed){
    intake.Output = speed;
    intakeMotor.setControl(intake);
    //secondIntakeMotor.setControl(intake);
  }

  // public void stopRotateMotor(){
  //   rotateMotor.set(0);
  // }

  // public void stopIntakeMotor(){
  //   intakeMotor.set(0);
  // }

  // public double getRotatePosition(){
  //   return rotateMotor.getPosition().getValueAsDouble();
  // }

  // public void resetRotatePosition(){
  //   rotateMotor.setPosition(0);
  // }

  // public void gotToPos(double pos){
  //   final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
  //   rotateMotor.setControl(request);
  // }

  // private void applyRotateMotorConfigs(InvertedValue inversion){
  //   TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
  //     talonConfigs.Slot0.kP = 10;
  //     talonConfigs.Slot0.kI = 0;
  //     talonConfigs.Slot0.kD = 0;
  //     talonConfigs.Slot0.kV = 0;
  //     talonConfigs.Slot0.kG = 0.29;
  //     talonConfigs.Slot0.kS = 0;
  //     talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

  //     var motionMagicConfigs = talonConfigs.MotionMagic;
  //     motionMagicConfigs.MotionMagicCruiseVelocity = 50;
  //     motionMagicConfigs.MotionMagicAcceleration = 100;
  //     motionMagicConfigs.MotionMagicJerk = 1000;

  //     talonConfigs.Feedback.FeedbackRemoteSensorID = rotateMotor.getDeviceID();
  //     talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

  //     rotateMotor.getConfigurator().apply(talonConfigs);

  //     MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
  //     motorOutputConfigs.Inverted = inversion;
  //     motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
  //     rotateMotor.getConfigurator().apply(motorOutputConfigs);
  // }
}
