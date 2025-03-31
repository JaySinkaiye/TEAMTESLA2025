package frc.robot.commands.AprilTagPositions;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockInReef extends Command {

    private CommandSwerveDrivetrain swerve;
    private SwerveRequest m_Request;
    private final SwerveRequest.RobotCentric Drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double desiredDistance;

    public LockInReef(CommandSwerveDrivetrain swerve, double desiredDistance){
        this.swerve = swerve;
        this.desiredDistance = desiredDistance;
        addRequirements(swerve);

        //works for april tags 12 and 2 
        LimelightHelpers.setPipelineIndex("limelight-front", 0);
    }

    @Override
    public void execute(){
        double forwardSpeed = lockIn();
        double turnSpeed = limelight_aim_proportional();

        SmartDashboard.putNumber("Distance to limelight: ", getDistance());
        SmartDashboard.putNumber("TY: ", LimelightHelpers.getTY("limelight-front"));
        SmartDashboard.putNumber("TX: ", LimelightHelpers.getTX("limelight-front"));
        SmartDashboard.putNumber("Forward speed", forwardSpeed);
        SmartDashboard.putNumber("Turn speed", turnSpeed);

        m_Request = Drive.withVelocityX(forwardSpeed)
            .withVelocityY(0)
            .withRotationalRate(turnSpeed);

        swerve.setControl(m_Request);

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return (getDistance() >= desiredDistance - 1 && getDistance() <= desiredDistance + 1) && ((LimelightHelpers.getTX("limelight-front") < 0.1) && (LimelightHelpers.getTX("limelight-front")  > -0.9));
    }

    public static double getDistance(){
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-front");

        //how many degrees back the limelight is from being vertical
        double limilightMountAngleDegrees = 0;

        //distance from the center of the limelight lens to the floor
        double limelightLensHeightInches = 6;

        // distance from april tag to floor
        double goalHeightInches = 8.75;

        double angleToGoalDegrees = limilightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI/180);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/ Math.tan(angleToGoalRadians);

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
        double kP = 0.01;
        double targetingForwardSpeed = (getDistance() - desiredDistance) * kP;
        targetingForwardSpeed *= MaxSpeed;
        return targetingForwardSpeed;
    }

    double limelight_aim_proportional()
    {    
      double kP = .01;
  
      double targetingAngularVelocity = LimelightHelpers.getTX("limelight-front") * kP;
      targetingAngularVelocity *= MaxAngularRate;
      targetingAngularVelocity *= -1.0;
  
      return targetingAngularVelocity;
    }
}
