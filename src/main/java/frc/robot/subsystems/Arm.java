// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
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

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(53);
  private final TalonFX intakeFollow = new TalonFX(62);
  private final WPI_VictorSPX wristMotor = new WPI_VictorSPX(50);
  private final TalonFX rotateMotor = new TalonFX(60);

  private DutyCycleOut intake = new DutyCycleOut(0);
  private DutyCycleOut wrist = new DutyCycleOut(0);
  private DutyCycleOut rotate = new DutyCycleOut(0);

  public Arm() {
    setRotationPosition(0);
    intakeFollow.setControl(new Follower(61, false));
    applyRotateMotorConfigs(InvertedValue.Clockwise_Positive);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotate pos", getRotatePosition());
  }

  public void setIntakeSpeed(double ispeed){
    intake.Output = ispeed;
    intakeMotor.setControl(intake);
  }

  public void setWristSpeed(double wspeed){
    wrist.Output = wspeed;
    wristMotor.set(wspeed);
  }

  public void setRotateSpeed(double rspeed){
    rotate.Output = rspeed;
    rotate.EnableFOC = true;
    rotateMotor.setControl(rotate);
  }

  public void stopIntakeMotor(){
    intakeMotor.set(0);
  }

  public void stopWristMotor(){
    wristMotor.set(0);
  }

  public void stopRotateMotor(){
    rotateMotor.set(0);
  }

  public double getRotatePosition(){
    return rotateMotor.getPosition().getValueAsDouble();
  }

  public void resetRotatePosition(){
    rotateMotor.setPosition(0);
  }

  public void setRotationPosition(double rPosition){
    rotateMotor.setPosition(rPosition);
  }

  public void rotateToPos(double pos){
    final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
    rotateMotor.setControl(request);
  }

  private void applyRotateMotorConfigs(InvertedValue inversion){
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    
    FeedbackConfigs fb = talonConfigs.Feedback;
    fb.SensorToMechanismRatio = 15;

    talonConfigs.Slot0.kP = 10;
    talonConfigs.Slot0.kI = 0;
    talonConfigs.Slot0.kD = 0;
    talonConfigs.Slot0.kV = 0;
    talonConfigs.Slot0.kG = 0.29;
    talonConfigs.Slot0.kS = 0;
    talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfigs = talonConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 50;
    motionMagicConfigs.MotionMagicAcceleration = 100;
    motionMagicConfigs.MotionMagicJerk = 1000;

    talonConfigs.Feedback.FeedbackRemoteSensorID = rotateMotor.getDeviceID();
    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    rotateMotor.getConfigurator().apply(talonConfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = inversion;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    rotateMotor.getConfigurator().apply(motorOutputConfigs);
  }
}
