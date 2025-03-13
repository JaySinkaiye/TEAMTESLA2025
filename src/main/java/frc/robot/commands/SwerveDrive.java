package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.AprilTagPositions.LockInHPS;
import frc.robot.commands.AprilTagPositions.LockInProcessor;
import frc.robot.commands.AprilTagPositions.LockInReef;
import frc.robot.commands.AprilTagPositions.TurnInReef;
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

    private CommandXboxController driverController;

    private double rotationVal, xVal, yVal;
    private SlewRateLimiter slewR, slewX, slewY;

    private LockInHPS lihps19;
    private LockInReef lir4point4;
    private LockInProcessor lip20;

    //private Elevator elevator;

    public SwerveDrive(CommandSwerveDrivetrain swerve, CommandXboxController driver){

        this.swerve = swerve;
        this.driverController = driver;
        addRequirements(swerve);

        slewR = new SlewRateLimiter(8);
        slewX = new SlewRateLimiter(8);
        slewY = new SlewRateLimiter(8);

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

        lihps19 = new LockInHPS(swerve, 17);
        lir4point4 = new LockInReef(swerve, 4.4);
        lip20 = new LockInProcessor(swerve, 20);
    }

    
    // // public SwerveDrive(CommandSwerveDrivetrain swerve, CommandXboxController driver, Elevator elevator){

    // //     this.swerve = swerve;
    // //     this.driverController = driver;
    // //     this.elevator = elevator;
    // //     addRequirements(swerve);

    // //     slewR = new SlewRateLimiter(8);
    // //     slewX = new SlewRateLimiter(8);
    // //     slewY = new SlewRateLimiter(8);

    // //     m_speedChooser = new SendableChooser<Double>();
    // //     m_speedChooser.addOption("100%", 1.0);
    // //     m_speedChooser.addOption("90%", 0.9);
    // //     m_speedChooser.setDefaultOption("85%", 0.85);
    // //     m_speedChooser.addOption("80%", 0.8);
    // //     m_speedChooser.addOption("70%", 0.7);
    // //     m_speedChooser.addOption("60%", 0.6);
    // //     m_speedChooser.addOption("50%", 0.5);
    // //     m_speedChooser.addOption("40%", 0.4);
    // //     m_speedChooser.addOption("30%", 0.3);
    // //     m_speedChooser.addOption("20%", 0.2);
    // //     SmartDashboard.putData("Speed Percent", m_speedChooser);

    // //     lihps19 = new LockInHPS(swerve, 17);
    // //     lir4point4 = new LockInReef(swerve, 4.4);
    // //     lip20 = new LockInProcessor(swerve, 20);
    // // }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        xVal = MathUtil.applyDeadband(-driverController.getLeftX() * m_speedChooser.getSelected(),0.2);
        yVal = MathUtil.applyDeadband(-driverController.getLeftY() * m_speedChooser.getSelected(), 0.2);
        rotationVal = MathUtil.applyDeadband(-driverController.getRightX() * m_speedChooser.getSelected(), 0.1);

        driverController.rightBumper().whileTrue(swerve.applyRequest(() -> brake));
        driverController.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));
        
        // if elevator.getposition is below a certain amount then:
        m_Request = drive.withVelocityX(slewY.calculate(yVal * MaxSpeed))
        .withVelocityY(slewX.calculate(xVal * MaxSpeed))
        .withRotationalRate(slewR.calculate(rotationVal * MaxAngularRate));
        // else drive at 20 percent speed

        swerve.setControl(m_Request);

        //april tag detection
        double aprilTagID = LimelightHelpers.getFiducialID("limelight-front");

        //lock in apriltag
        if (aprilTagID == 13 || aprilTagID == 1 || aprilTagID == 12 || aprilTagID == 2){
            //left HPS
            driverController.x().onTrue(lihps19);
            lihps19.tea();       
        } else if (aprilTagID == 16 || aprilTagID == 3 ){
            // processor
            driverController.x().onTrue(lip20);
            lip20.tea();
        } else {
            driverController.x().onTrue(lir4point4);
            driverController.a().onTrue(new TurnInReef(swerve));
            lir4point4.tea();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
