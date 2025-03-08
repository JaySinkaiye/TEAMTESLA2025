package frc.robot.commands.AprilTagPositions;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.LimelightHelpers;
//import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockInReef extends Command {

    // private CommandSwerveDrivetrain swerve;
    // private SwerveRequest m_Request;
    // private final SwerveRequest.RobotCentric Drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    // private Limelight LL;

    // public LockInReef(CommandSwerveDrivetrain swerve, double desiredDistance){
    //     this.swerve = swerve;
    //     addRequirements(swerve);

    //     //works for april tags 12 and 2 
    //     LimelightHelpers.setPipelineIndex("limelight-front", 0);
    //     LL = new Limelight(8.75, desiredDistance);
    // }

    // @Override
    // public void initialize(){
    // }

    // @Override
    // public void execute(){
    //     double forwardSpeed = LL.lockIn();
    //     double cupidShuffle = LL.slide();

    //     m_Request = Drive.withVelocityX(forwardSpeed)
    //         .withVelocityY(cupidShuffle)
    //         .withRotationalRate(0);

    //      //when limelight is on the side of the robot
    //     // m_Request = Drive.withVelocityX(-cupidShuffle)
    //     //     .withVelocityY(forwardSpeed)
    //     //     .withRotationalRate(0);

    //     swerve.setControl(m_Request);
    // }

    // @Override
    // public void end(boolean interrupted) {}

    // @Override
    // public boolean isFinished() {
    //     return LL.isDone();
    // }

    // public void tea(){
    //     LL.printDeets();
    // }

}
