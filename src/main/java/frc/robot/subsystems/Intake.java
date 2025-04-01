// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX rotateMotor = new TalonFX(54);
  private final TalonFX intakeMotor = new TalonFX(61);

  private DutyCycleOut rotate = new DutyCycleOut(0);
  private DutyCycleOut intake = new DutyCycleOut(0);

  public Intake() {
    setRotatePosition(-0.180664);
    applyRotateMotorConfigs(InvertedValue.Clockwise_Positive);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake rotate pos", rotateMotor.getPosition().getValueAsDouble());
    //This method will be called once per scheduler run
  }

  public void manualControl(BooleanSupplier up, BooleanSupplier down, double rspeed, BooleanSupplier left, BooleanSupplier right, double ispeed, double ispeed2){
    if (up.getAsBoolean() == true){
      rotate.Output = rspeed;
      rotate.EnableFOC = true;
      rotateMotor.setControl(rotate);
    } else if (down.getAsBoolean() == true){
      rotate.Output = -rspeed;
      rotate.EnableFOC = true;
      rotateMotor.setControl(rotate);
    } else{
      rotate.Output = 0;
      rotateMotor.setControl(rotate);
    }

    if (left.getAsBoolean()){
      intake.Output = ispeed;
      intakeMotor.setControl(intake);
    } else if (right.getAsBoolean()){
        intake.Output = -ispeed2;
        intakeMotor.setControl(intake);
    } else{
      intakeMotor.set(0);
    }
  }

  public void setRotateSpeed(double speed){
    rotate.Output = speed;
    rotateMotor.setControl(rotate);
  }

  public void setIntakeSpeed(double speed){
    intake.Output = speed;
    intakeMotor.setControl(intake);
  }

  public void stopRotateMotor(){
    rotateMotor.set(0);
  }

  public void stopIntakeMotor(){
    intakeMotor.set(0);
  }

  public double getRotatePosition(){
    return rotateMotor.getPosition().getValueAsDouble();
  }

  public void resetRotatePosition(){
    rotateMotor.setPosition(0);
  }

  public void setRotatePosition(double rPos){
    rotateMotor.setPosition(rPos);
  }

  public void gotToPos(double pos){
    final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
    rotateMotor.setControl(request);
  }

  private void applyRotateMotorConfigs(InvertedValue inversion){
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
      FeedbackConfigs fb = talonConfigs.Feedback;
      fb.SensorToMechanismRatio = 2;
      
      SoftwareLimitSwitchConfigs hw = talonConfigs.SoftwareLimitSwitch;
      hw.ForwardSoftLimitEnable = false;
      hw.ForwardSoftLimitThreshold = -0.18;
      hw.ReverseSoftLimitEnable = false;
      hw.ReverseSoftLimitThreshold = -0.88;

      talonConfigs.Slot0.kP = 15;
      talonConfigs.Slot0.kI = 7;
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
