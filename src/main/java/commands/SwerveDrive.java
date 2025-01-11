package commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDrive extends Command {

    private CommandSwerveDrivetrain swerve;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private SwerveRequest m_Request;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private SendableChooser<Double> m_speedChooser;

    private CommandXboxController driverController;

    private double rotationVal, xVal, yVal;

    public SwerveDrive(CommandSwerveDrivetrain swerve, CommandXboxController driver){

        this.swerve = swerve;
        this.driverController = driver;
        addRequirements(swerve);

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
    public void execute(){

        xVal = MathUtil.applyDeadband(driverController.getLeftX() * m_speedChooser.getSelected(),0.1);
        yVal = MathUtil.applyDeadband(-driverController.getLeftY() * m_speedChooser.getSelected(), 0.1);
        rotationVal = MathUtil.applyDeadband(driverController.getRightX() * m_speedChooser.getSelected(), MaxAngularRate * 0.1);

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));  
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        m_Request = drive.withVelocityX(yVal * MaxSpeed)
        .withVelocityX(xVal * MaxSpeed)
        .withRotationalRate(rotationVal * MaxSpeed);

        swerve.setControl(m_Request);
    }
}
