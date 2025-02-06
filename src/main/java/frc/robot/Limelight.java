package frc.robot;

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
    private final double desiredDistance;

    PIDController pid = new PIDController(0.01, 0, 0);
    PIDController dPid = new PIDController(0.01, 0.01, 0);
    PIDController spid = new PIDController(0.01, 0, 0);
    double aprilTagID = LimelightHelpers.getFiducialID("limelight-front");

    public Limelight(double goalHeightInches, double desiredDistance){
        this.targetHeight = goalHeightInches;
        this.desiredDistance = desiredDistance;
    }

    public double getDistance(){
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-front");

        //how many degrees back the limelight is from being vertical
        double limilightMountAngleDegrees = 0;

        //distance from the center of the limelight lens to the floor
        double limelightLensHeightInches = 6;


        double angleToGoalDegrees = limilightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI/180);

        //calculate distance
        double distanceFromLimelightToGoalInches = (targetHeight - limelightLensHeightInches)/ Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    public double limelight_range_proportional(double targetTY){    
        double kP = .1;
        double targetingForwardSpeed =  (LimelightHelpers.getTY("limelight-front") + targetTY) * kP;
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;

        return targetingForwardSpeed;
    }

    // does the same thing as above but using inches
    public double lockIn(){
        dPid.setTolerance(1);
        return (dPid.calculate(getDistance(), desiredDistance)) * -MaxSpeed;
    }

    public double slide(){
        spid.setTolerance(1);
        return spid.calculate(LimelightHelpers.getTXNC("limelight-front"), 0.1) * MaxSpeed;
    }

    public double limelight_aim_proportional()
    {    
        pid.setTolerance(0.1);
        double turn = pid.calculate(LimelightHelpers.getTX("limelight-front"), 0.2) * MaxAngularRate;
        return turn;
    }

    public Boolean isDone(){
        return (dPid.atSetpoint() && pid.atSetpoint()) || !LimelightHelpers.getTV("limelight-front");
    }

    public void printDeets(){
        SmartDashboard.putNumber("Distance to limelight: ", getDistance());
        SmartDashboard.putNumber("TX: ", LimelightHelpers.getTX("limelight-front"));
        SmartDashboard.putNumber("Turn Speed", limelight_aim_proportional());
        SmartDashboard.putNumber("Drive Speed", lockIn());
        SmartDashboard.putBoolean("Locked in?", isDone());
    }

}
