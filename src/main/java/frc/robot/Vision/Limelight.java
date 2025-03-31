package frc.robot.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class Limelight {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final double desiredDistance;

    PIDController rotationController = new PIDController(0.01, 0.002, 0);
    PIDController xTranslationController = new PIDController(0.025, 0.004, 0.003);
    PIDController YTranslationController = new PIDController(0.005, 0, 0);
    double aprilTagID = LimelightHelpers.getFiducialID("limelight-front");

    public Limelight(double desiredDistance){
        this.desiredDistance = desiredDistance;
    }

    //Ty FROM REEF: 4.4

    public double lockIn(){
        xTranslationController.setTolerance(0.1);
        xTranslationController.setIntegratorRange(-0.15, 0.15);
        xTranslationController.setIZone(1);
        return (xTranslationController.calculate(LimelightHelpers.getTY("limelight-front"), desiredDistance)) * MaxSpeed;
    }

    public double slide(){
        YTranslationController.setTolerance(0.1);
        return YTranslationController.calculate(LimelightHelpers.getTXNC("limelight-front"), 0) * MaxSpeed;
    }

    public double limelight_aim_proportional()
    {    
        rotationController.setTolerance(1.5);
        double turn = rotationController.calculate(LimelightHelpers.getTX("limelight-front"), 0.2) * MaxAngularRate;
        return turn;
    }

    public Boolean isDone(){
        return (xTranslationController.atSetpoint() && YTranslationController.atSetpoint()) || !LimelightHelpers.getTV("limelight-front");
    }

    public Boolean turnIsDone(){
        return rotationController.atSetpoint() || !LimelightHelpers.getTV("limelight-front");
    }

    public void printDeets(){
        SmartDashboard.putNumber("TX: ", LimelightHelpers.getTX("limelight-front"));
        SmartDashboard.putNumber("TY: ", LimelightHelpers.getTY("limelight-front"));
        SmartDashboard.putNumber("TXNC", LimelightHelpers.getTXNC("limelight-front"));
        SmartDashboard.putNumber("Turn Speed", limelight_aim_proportional());
        SmartDashboard.putNumber("Drive Speed", lockIn());
        SmartDashboard.putNumber("Slide Speed", slide());
        SmartDashboard.putBoolean("Locked in?", isDone());
        SmartDashboard.putBoolean("Turn locked in? ", turnIsDone());
    }

}
