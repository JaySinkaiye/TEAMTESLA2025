package frc.robot.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class Limelight {

    double targetHeight;
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    PIDController pid = new PIDController(0.01, 0.002, 0);
    PIDController dPid = new PIDController(0.025, 0.004, 0.003);
    PIDController spid = new PIDController(0.005, 0, 0);
    double aprilTagID = LimelightHelpers.getFiducialID("limelight-front");

    public Limelight(){
    }


    //Ty FROM REEF: 4.4
    public double lockIn(double desiredDistance){
        dPid.setTolerance(0.1);
        dPid.setIntegratorRange(-0.15, 0.15);
        dPid.setIZone(1);
        return (dPid.calculate(LimelightHelpers.getTY("limelight-front"), desiredDistance)) * MaxSpeed;
    }

    public double slide(){
        spid.setTolerance(0.1);
        return spid.calculate(LimelightHelpers.getTXNC("limelight-front"), 0) * MaxSpeed;
    }

    public double limelight_aim_proportional()
    {    
        pid.setTolerance(1.5);
        double turn = pid.calculate(LimelightHelpers.getTX("limelight-front"), 0.2) * MaxAngularRate;
        return turn;
    }

    public Boolean isDone(){
        return (dPid.atSetpoint() && spid.atSetpoint()) || !LimelightHelpers.getTV("limelight-front");
    }

    public Boolean turnIsDone(){
        return pid.atSetpoint() || !LimelightHelpers.getTV("limelight-front");
    }

    public void printDeets(){
        SmartDashboard.putNumber("TX: ", LimelightHelpers.getTX("limelight-front"));
        SmartDashboard.putNumber("TY: ", LimelightHelpers.getTY("limelight-front"));
        SmartDashboard.putNumber("TXNC", LimelightHelpers.getTXNC("limelight-front"));
        SmartDashboard.putNumber("Turn Speed", limelight_aim_proportional());
        SmartDashboard.putNumber("Drive Speed", lockIn(4.4));
        SmartDashboard.putNumber("Slide Speed", slide());
        SmartDashboard.putBoolean("Locked in?", isDone());
        SmartDashboard.putBoolean("Turn locked in? ", turnIsDone());
    }

    public boolean blueSideReef(int id){
        if(id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22){
            return true;
        }
        return false;
    }

    public boolean redSideReef(int id){
        if(id == 6 || id == 7 || id == 8 || id == 9 || id == 10 || id == 11){
            return true;
        }
        return false;
    }

}