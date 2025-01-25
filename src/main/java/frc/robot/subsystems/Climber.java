package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX climbMotor = new TalonFX(55);

    public Climber(){
        climbMotor.setPosition(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Position", getClimberPosition());
    }

    // public void climb(CommandXboxController operator){
    //     climbMotor.set(0.8 * operator.getLeftY());
    // }

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

}
