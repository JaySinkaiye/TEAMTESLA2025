package frc.robot.commands.AprilTagPositions;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockInProcessor extends Command {

    private CommandSwerveDrivetrain swerve;
    private SwerveRequest m_Request;
    private final SwerveRequest.RobotCentric Drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    private double desiredDistance;

    private Limelight LL;

    public LockInProcessor(CommandSwerveDrivetrain swerve, double desiredDistance){
        this.swerve = swerve;
        this.desiredDistance = desiredDistance;
        addRequirements(swerve);

        //works for april tags 16 and 3 
        LimelightHelpers.setPipelineIndex("limelight-front", 0);
    }

    @Override
    public void initialize(){
        LL = new Limelight(47.88, desiredDistance);

    }
    @Override
    public void execute(){
        double forwardSpeed = LL.lockIn();
        double turnSpeed = LL.limelight_aim_proportional();

        m_Request = Drive.withVelocityX(forwardSpeed)
            .withVelocityY(0)
            .withRotationalRate(turnSpeed);

        swerve.setControl(m_Request);

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
