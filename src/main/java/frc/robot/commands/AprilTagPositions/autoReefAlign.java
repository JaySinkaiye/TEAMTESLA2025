package frc.robot.commands.AprilTagPositions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class autoReefAlign extends Command {
  private CommandSwerveDrivetrain swerve;
  private final SwerveRequest.ApplyRobotSpeeds ApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  private Limelight LL;  
  
  public autoReefAlign(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    //works for april tags 12 and 2 
    LimelightHelpers.setPipelineIndex("limelight-front", 0);
    LL = new Limelight(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ChassisSpeeds speeds = LL.lockingIn(0);
    swerve.setControl(ApplyRobotSpeeds.withSpeeds(speeds));
  }

  @Override
  public boolean isFinished() {
    return LL.isDone();
  }
}
