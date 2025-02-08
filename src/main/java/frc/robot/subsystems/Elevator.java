package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
//     private TalonFX leftElevatorMotor = new TalonFX(51);
//     private TalonFX rightElevatorMotor = new TalonFX(52);

//     public Elevator(){
//         leftElevatorMotor.setPosition(0);
//         rightElevatorMotor.setPosition(0);
//         applyArmMotorConfigs(InvertedValue.Clockwise_Positive);
//     }

//     @Override
//     public void periodic(){
//         SmartDashboard.putNumber("Elevator Encoder: ", getElevatorPosition());
//     }
    
//     public void setElevatorMotorSpeed(double lspeed, double rspeed){
//         leftElevatorMotor.set(lspeed);
//         rightElevatorMotor.set(rspeed);
//     }

//     public void stopElevatorMotors(){
//         leftElevatorMotor.set(0);
//         rightElevatorMotor.set(0);
//     }

//     public double getElevatorPosition(){
//         return leftElevatorMotor.getPosition().getValueAsDouble();
//     }

//     public void resetElevatorPosition(){
//         leftElevatorMotor.setPosition(0);
//         rightElevatorMotor.setPosition(0);
//     }

//     public Command gotToPos(double pos){
//         final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(pos);
//         return runOnce(
//         ()->{
//             rightElevatorMotor.setControl(request);
//             leftElevatorMotor.setControl(request);
//         }
//         );
//   }

//     private void applyArmMotorConfigs(InvertedValue inversion){
//         TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
//         talonConfigs.Slot0.kP = 10;
//         talonConfigs.Slot0.kI = 0;
//         talonConfigs.Slot0.kD = 0;
//         talonConfigs.Slot0.kV = 0;
//         talonConfigs.Slot0.kG = 0.29;
//         talonConfigs.Slot0.kS = 0;
//         talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

//         var motionMagicConfigs = talonConfigs.MotionMagic;
//         motionMagicConfigs.MotionMagicCruiseVelocity = 5;
//         motionMagicConfigs.MotionMagicAcceleration = 10;
//         motionMagicConfigs.MotionMagicJerk = 30;

//         talonConfigs.Feedback.FeedbackRemoteSensorID = leftElevatorMotor.getDeviceID();
//         talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

//         leftElevatorMotor.getConfigurator().apply(talonConfigs);
//         rightElevatorMotor.getConfigurator().apply(talonConfigs);

//         MotorOutputConfigs motorOutputConfigsRight = new MotorOutputConfigs();
//         motorOutputConfigsRight.Inverted = inversion;
//         motorOutputConfigsRight.NeutralMode = NeutralModeValue.Brake;
//         MotorOutputConfigs motorOutputConfigsLeft = new MotorOutputConfigs();
//         motorOutputConfigsLeft.NeutralMode = NeutralModeValue.Brake;
//         rightElevatorMotor.getConfigurator().apply(motorOutputConfigsRight);
//         leftElevatorMotor.getConfigurator().apply(motorOutputConfigsLeft);
//   }
}
