package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AprilTagPositions.LockInLeftHPS;
import frc.robot.commands.AprilTagPositions.LockInProcessor;
import frc.robot.commands.AprilTagPositions.LockInReef;
import frc.robot.commands.AprilTagPositions.LockInRightHPS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDrive extends Command {

    private CommandSwerveDrivetrain swerve;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private SendableChooser<Double> m_speedChooser;

    private SwerveRequest m_Request;

    // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private CommandXboxController driverController;

    private double rotationVal, xVal, yVal;
    private SlewRateLimiter slewR, slewX, slewY;

    public SwerveDrive(CommandSwerveDrivetrain swerve, CommandXboxController driver){

        this.swerve = swerve;
        this.driverController = driver;
        addRequirements(swerve);

        slewR = new SlewRateLimiter(7);
        slewX = new SlewRateLimiter(7);
        slewY = new SlewRateLimiter(7);

        m_speedChooser = new SendableChooser<Double>();
        m_speedChooser.addOption("100%", 1.0);
        m_speedChooser.addOption("90%", 0.9);
        m_speedChooser.setDefaultOption("85%", 0.85);
        m_speedChooser.addOption("80%", 0.8);
        m_speedChooser.addOption("70%", 0.7);
        m_speedChooser.addOption("60%", 0.6);
        m_speedChooser.addOption("50%", 0.5);
        m_speedChooser.addOption("40%", 0.4);
        m_speedChooser.addOption("30%", 0.3);
        m_speedChooser.addOption("20%", 0.2);
        SmartDashboard.putData("Speed Percent", m_speedChooser);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        xVal = MathUtil.applyDeadband(-driverController.getLeftX() * m_speedChooser.getSelected(),0.2);
        yVal = MathUtil.applyDeadband(-driverController.getLeftY() * m_speedChooser.getSelected(), 0.2);
        rotationVal = MathUtil.applyDeadband(-driverController.getRightX() * m_speedChooser.getSelected(), 0.1);

        driverController.a().whileTrue(swerve.applyRequest(() -> brake));
        driverController.b().whileTrue(swerve.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));  
        driverController.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));
        
        m_Request = drive.withVelocityX(slewY.calculate(yVal * MaxSpeed))
        .withVelocityY(slewX.calculate(xVal * MaxSpeed))
        .withRotationalRate(slewR.calculate(rotationVal * MaxAngularRate));

        swerve.setControl(m_Request);

        SmartDashboard.putNumber("Distance to limelight: ", getDistance());
        SmartDashboard.putNumber("x speed", slewY.calculate(yVal * MaxSpeed));

        //april tag detection
        double aprilTagID = LimelightHelpers.getFiducialID("limelight-front");

        //placeholder distances 
        if (aprilTagID == 13 || aprilTagID == 1){
            //left HPS
            driverController.x().onTrue(new LockInLeftHPS(swerve, 50));
            System.out.println("left HPS");
        } else if(aprilTagID == 12 || aprilTagID == 2 ){
            //right HPS
            driverController.x().onTrue(new LockInRightHPS(swerve, 50));
            System.out.println("right HPS");
        }  else if (aprilTagID == 16 || aprilTagID == 3 ){
            // processor
            driverController.x().onTrue(new LockInProcessor(swerve, 50));
            System.out.println("processor");
        } else {
            driverController.x().onTrue(new LockInReef(swerve, 50));
            System.out.println("reef");
        }
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

        public static double getDistance(){
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-front");

        //how many degrees back the limelight is from being vertical
        double limilightMountAngleDegrees = 0;

        //distance from the center of the limelight lens to the floor
        double limelightLensHeightInches = 6;

        // distance from april tag to floor
        double goalHeightInches = 58.5;

        double angleToGoalDegrees = limilightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI/180);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/ Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }
}
