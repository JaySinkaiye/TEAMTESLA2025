package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.positionConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX leftElevatorMotor = new TalonFX(57);
    private final TalonFX follower = new TalonFX(58);
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public Elevator(){
        resetElevatorPosition();
        applyElevatorMotorConfigs(InvertedValue.Clockwise_Positive);
        follower.setControl(new Follower(57, true));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Encoder: ", getElevatorPosition());
        SmartDashboard.putString("A BUTTON", "L_1");
        SmartDashboard.putString("X BUTTON", "L_4");
        SmartDashboard.putString("Y BUTTON", "SAFE_TO_MOVE");
        SmartDashboard.putString("B BUTTON", "GROUND_INTAKE");
    }
    
    public void setElevatorMotorSpeed(double speed, CommandXboxController opController, BooleanSupplier a, BooleanSupplier x, BooleanSupplier y, BooleanSupplier b){

        if (a.getAsBoolean()){
            GoToPos(positionConstants.elevatorConstants.CORAL_L1_POSITION);
        } else if (x.getAsBoolean()){
            GoToPos(positionConstants.elevatorConstants.CORAL_L4_POSITION);
        } else if ( y.getAsBoolean()){
            GoToPos(positionConstants.elevatorConstants.ELEVATOR_SAFE_TO_MOVE_ZONE);
        } else if (b.getAsBoolean()){
            GoToPos(positionConstants.elevatorConstants.GROUND_INTAKE_POSITION);
        } else {
            duty.Output = speed;
            duty.EnableFOC = true;
            leftElevatorMotor.setControl(duty);
        }
    }

    public Command elevateManually(double speed){
        duty.Output = speed;
        duty.EnableFOC = true;
        return run(
            ()->{
                leftElevatorMotor.setControl(duty);
            }
        );
    }

    public void stopElevatorMotors(){
        leftElevatorMotor.set(0);
    }

    public double getElevatorPosition(){
        return leftElevatorMotor.getPosition().getValueAsDouble();
    }

    public void resetElevatorPosition(){
        leftElevatorMotor.setPosition(0);
    }

    public void GoToPos(double pos){
        final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
        leftElevatorMotor.setControl(request);
    }

    private void applyElevatorMotorConfigs(InvertedValue inversion){
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        FeedbackConfigs fb = talonConfigs.Feedback;
        fb.SensorToMechanismRatio = 15;

        SoftwareLimitSwitchConfigs sl = talonConfigs.SoftwareLimitSwitch;
        sl.ForwardSoftLimitEnable = true;
        sl.ForwardSoftLimitThreshold = 0;
        sl.ReverseSoftLimitEnable = true;
        sl.ReverseSoftLimitThreshold = -5.95; //-6.15

        talonConfigs.Slot0.kP = 30;
        talonConfigs.Slot0.kI = 5;
        talonConfigs.Slot0.kD = 0;
        talonConfigs.Slot0.kV = 0;
        talonConfigs.Slot0.kG = 0.29;
        talonConfigs.Slot0.kS = 0;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50;
        motionMagicConfigs.MotionMagicAcceleration = 100;
        motionMagicConfigs.MotionMagicJerk = 1000;

        talonConfigs.Feedback.FeedbackRemoteSensorID = leftElevatorMotor.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        
        leftElevatorMotor.getConfigurator().apply(talonConfigs);
        follower.getConfigurator().apply(talonConfigs);

        MotorOutputConfigs motorOutputConfigsRight = new MotorOutputConfigs();
        motorOutputConfigsRight.NeutralMode = NeutralModeValue.Brake;
        MotorOutputConfigs motorOutputConfigsLeft = new MotorOutputConfigs();
        motorOutputConfigsLeft.NeutralMode = NeutralModeValue.Brake;
        follower.getConfigurator().apply(motorOutputConfigsRight);
        leftElevatorMotor.getConfigurator().apply(motorOutputConfigsLeft);
  }
}
