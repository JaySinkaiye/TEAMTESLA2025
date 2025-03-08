package frc.robot.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class Limelight {

    // double targetHeight;
    // private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    // private final double desiredDistance;

    // PIDController pid = new PIDController(0.01, 0.002, 0);
    // PIDController dPid = new PIDController(0.025, 0.004, 0.003);
    // PIDController spid = new PIDController(0.005, 0, 0);
    // double aprilTagID = LimelightHelpers.getFiducialID("limelight-front");

    // public Limelight(double goalHeightInches, double desiredDistance){
    //     this.targetHeight = goalHeightInches;
    //     this.desiredDistance = desiredDistance;
    // }

    // public double getDistance(){
    //     double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-front");

    //     //how many degrees back the limelight is from being vertical
    //     double limilightMountAngleDegrees = 0;

    //     //distance from the center of the limelight lens to the floor
    //     double limelightLensHeightInches = 8;


    //     double angleToGoalDegrees = limilightMountAngleDegrees + targetOffsetAngle_Vertical;
    //     double angleToGoalRadians = angleToGoalDegrees * (Math.PI/180);

    //     //calculate distance
    //     double distanceFromLimelightToGoalInches = (targetHeight - limelightLensHeightInches)/ Math.tan(angleToGoalRadians);

    //     return distanceFromLimelightToGoalInches;
    // }

    // //Ty FROM REEF: 4.4

    // public double lockIn(){
    //     dPid.setTolerance(0.1);
    //     dPid.setIntegratorRange(-0.15, 0.15);
    //     dPid.setIZone(1);
    //     return (dPid.calculate(LimelightHelpers.getTY("limelight-front"), desiredDistance)) * MaxSpeed;
    // }

    // public double slide(){
    //     spid.setTolerance(0.1);
    //     return spid.calculate(LimelightHelpers.getTXNC("limelight-front"), 0) * MaxSpeed;
    // }

    // public double limelight_aim_proportional()
    // {    
    //     pid.setTolerance(1.5);
    //     double turn = pid.calculate(LimelightHelpers.getTX("limelight-front"), 0.2) * MaxAngularRate;
    //     return turn;
    // }

    // public Boolean isDone(){
    //     return (dPid.atSetpoint() && spid.atSetpoint()) || !LimelightHelpers.getTV("limelight-front");
    // }

    // public Boolean turnIsDone(){
    //     return pid.atSetpoint() || !LimelightHelpers.getTV("limelight-front");
    // }

    // public void printDeets(){
    //     SmartDashboard.putNumber("Distance to limelight: ", getDistance());
    //     SmartDashboard.putNumber("TX: ", LimelightHelpers.getTX("limelight-front"));
    //     SmartDashboard.putNumber("TY: ", LimelightHelpers.getTY("limelight-front"));
    //     SmartDashboard.putNumber("TXNC", LimelightHelpers.getTXNC("limelight-front"));
    //     SmartDashboard.putNumber("Turn Speed", limelight_aim_proportional());
    //     SmartDashboard.putNumber("Drive Speed", lockIn());
    //     SmartDashboard.putNumber("Slide Speed", slide());
    //     SmartDashboard.putBoolean("Locked in?", isDone());
    //     SmartDashboard.putBoolean("Turn locked in? ", turnIsDone());
    // }

}
