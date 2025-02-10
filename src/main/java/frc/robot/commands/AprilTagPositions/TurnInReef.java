// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagPositions;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnInReef extends Command {
  private CommandSwerveDrivetrain swerve;
  private SwerveRequest m_Request;
  private final SwerveRequest.RobotCentric Drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  private Limelight LL;

  public TurnInReef(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    //works for april tags 12 and 2 
    LimelightHelpers.setPipelineIndex("limelight-front", 0);
    LL = new Limelight(8.75, 0);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double turnSpeed = LL.limelight_aim_proportional();
    
    m_Request = Drive.withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(turnSpeed);

    swerve.setControl(m_Request);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return LL.turnIsDone();
  }
}
