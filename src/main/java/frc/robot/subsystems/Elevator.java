package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // private TalonFX leftElevatorMotor = new TalonFX(51);
    // private TalonFX rightElevatorMotor = new TalonFX(52);

    // private CANcoder elevatorEncoder = new CANcoder(1);

    // public Elevator(){

    // }

    // @Override
    // public void periodic(){
    //     SmartDashboard.putNumber("Elevator Encoder: ", getElevatorPosition());
    // }
    
    // public void setElevatorMotorSpeed(double lspeed, double rspeed){
    //     leftElevatorMotor.set(lspeed);
    //     rightElevatorMotor.set(rspeed);
    // }

    // public void stopElevatorMotors(){
    //     leftElevatorMotor.set(0);
    //     rightElevatorMotor.set(0);
    // }

    // public double getElevatorPosition(){
    //     return elevatorEncoder.getAbsolutePosition().getValueAsDouble();
    // }

    // public void ElevateHer(double yVal){
    //     leftElevatorMotor.set(0.8 * yVal);
    //     rightElevatorMotor.set(0.8 * yVal);
    // }
}
