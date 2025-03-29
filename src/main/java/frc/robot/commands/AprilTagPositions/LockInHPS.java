package frc.robot.commands.AprilTagPositions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockInHPS extends Command {

    private CommandSwerveDrivetrain swerve;
    private final SwerveRequest.ApplyRobotSpeeds ApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private Limelight LL;
    private double aprilTagID;

    public LockInHPS(CommandSwerveDrivetrain swerve, double desiredDistance){
        this.swerve = swerve;
        addRequirements(swerve);

        //works for april tags 13 and 1 
        LimelightHelpers.setPipelineIndex("limelight-left", 0);
        LL = new Limelight(swerve);
        aprilTagID = LimelightHelpers.getFiducialID("limelight-left");
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        ChassisSpeeds speeds = LL.lockingIn(0, aprilTagID);
        swerve.setControl(ApplyRobotSpeeds.withSpeeds(speeds));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return LL.isDone();
    }

    public void tea(){
        LL.printDeets();
    }

}
