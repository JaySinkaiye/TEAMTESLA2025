package frc.robot.Vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Limelight {
    private CommandSwerveDrivetrain swerve;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final PIDController drive;
    private final PIDController slide;
    private final ProfiledPIDController theta;
    private final HolonomicDriveController sw;

    Pose2d goalPose;
    double aprilTagID = LimelightHelpers.getFiducialID("limelight-front");
    PoseEstimate poseEstimate;

    public Limelight(CommandSwerveDrivetrain swerve){
        LimelightHelpers.SetRobotOrientation("limelight-front", swerve.getPose().getRotation().getDegrees(), 0, 0,0, 0, 0);
        this.swerve = swerve;
        drive = new PIDController(0.25, 0.004, 0.003);
        slide = new PIDController(0.005, 0, 0);
        theta = new ProfiledPIDController(0.01, 0.002, 0, new TrapezoidProfile.Constraints(MaxSpeed, MaxAngularRate));
        sw = new HolonomicDriveController(drive, slide, theta);
    }

    public void updateDrivetrainPose(){

        //checking our alliance color
        DriverStation.getAlliance().ifPresent(allianceColor -> {
        if (allianceColor == Alliance.Red) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        } else {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-front");
        }
        });

        if (poseEstimate != null && LimelightHelpers.validPoseEstimate(poseEstimate)) {
            Pose2d visionPose = poseEstimate.pose;

            Matrix<N3, N1> visionStdDevs = VecBuilder.fill(
                0.1, // x standard deviation (meters)
                0.1, // y standard deviation (meters)
                0.1  // theta standard deviation (radians)
            );
            swerve.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds, visionStdDevs);
        
        }
    }

    public ChassisSpeeds lockingIn(double goalAngle){
        goalPose = aprilTags.aprilTagIDToPose((int) aprilTagID);
        return sw.calculate(swerve.getPose(), goalPose, MaxSpeed, Rotation2d.fromDegrees(goalAngle));
    }

    public boolean isDone(){
        return sw.atReference();
    }

    public void printDeets(){
        SmartDashboard.putNumber("x pose: ", swerve.getPose().getX());
        SmartDashboard.putNumber("y pose: ", swerve.getPose().getY());
        SmartDashboard.putNumber("rot pose: ", swerve.getPose().getRotation().getDegrees());

    }

}
