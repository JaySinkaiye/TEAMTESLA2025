    package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX climbMotor = new TalonFX(51);
    private DutyCycleOut duty = new DutyCycleOut(0);

    public Climber(){
        climbMotor.setPosition(0);
        applyClimbConfigs();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Position", getClimberPosition());
    }

    public void setClimberSpeed(double speed){
        duty.Output = speed;
        climbMotor.setControl(duty);
    }

    public Command manualClimb(double speed){
        duty.Output = speed;
        return run(
            ()->{
                climbMotor.set(speed);
            }
        );
    }

    public void stopClimberMotor(){
        climbMotor.set(0);
    }

    public double getClimberPosition(){
        return climbMotor.getPosition().getValueAsDouble();
    }

    public void resetClimberPosition(){
        climbMotor.setPosition(0);
    }

    public void gotoPos(double pos){
        final MotionMagicDutyCycle request = new MotionMagicDutyCycle(pos);
        climbMotor.setControl(request);
    }
    
    private void applyClimbConfigs(){
        var talonConfigs = new TalonFXConfiguration();

        // gear ratio
        FeedbackConfigs fb = talonConfigs.Feedback;
        fb.SensorToMechanismRatio = 60;

        var slot0Configs = talonConfigs.Slot0;
        slot0Configs.kP = 10;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0;
        slot0Configs.kG = 0.29;
        slot0Configs.kS = 0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 5;
        motionMagicConfigs.MotionMagicAcceleration = 10;
        motionMagicConfigs.MotionMagicJerk = 30;

        talonConfigs.Feedback.FeedbackRemoteSensorID = climbMotor.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        MotorOutputConfigs motorOutputConfigs = talonConfigs.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        climbMotor.getConfigurator().apply(talonConfigs);
    }
}
