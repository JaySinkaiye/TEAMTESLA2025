package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    // private TalonFX leftElevatorMotor = new TalonFX(51);
    // private TalonFX rightElevatorMotor = new TalonFX(52);

    // private CANcoder elevatorEncoder = new CANcoder(1);

    // public Arm(){

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
}
