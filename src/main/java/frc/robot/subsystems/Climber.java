package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX climbMotor = new TalonFX(55);

    public Climber(){
        climbMotor.setPosition(0);
        applyClimbConfigs();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Position", getClimberPosition());
    }

    public void setClimberSpeed(double speed){
        climbMotor.set(speed);
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

    public Command gotToPos(double pos){
        final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
        return runOnce(
        ()->{
            climbMotor.setControl(request);
            }
        );
    }
    
    private void applyClimbConfigs(){
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = 10;
        talonConfigs.Slot0.kI = 0;
        talonConfigs.Slot0.kD = 0;
        talonConfigs.Slot0.kV = 0;
        talonConfigs.Slot0.kG = 0.29;
        talonConfigs.Slot0.kS = 0;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 5;
        motionMagicConfigs.MotionMagicAcceleration = 10;
        motionMagicConfigs.MotionMagicJerk = 30;

        talonConfigs.Feedback.FeedbackRemoteSensorID = climbMotor.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        climbMotor.getConfigurator().apply(motorOutputConfigs);
    }
}
