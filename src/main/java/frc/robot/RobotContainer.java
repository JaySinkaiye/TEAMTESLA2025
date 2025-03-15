// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.AprilTagPositions.LockInHPS;
import frc.robot.commands.AprilTagPositions.LockInProcessor;
import frc.robot.commands.AprilTagPositions.LockInReef;
import frc.robot.commands.AprilTagPositions.TurnInReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;


public class RobotContainer {
    //subsytems
    public final Climber climber = new Climber();
    public final Intake intake = new Intake();
    public final Arm arm = new Arm();
    public final Elevator elevator = new Elevator();

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> AutonChooser = new SendableChooser<>();
    
    public RobotContainer() {
        NamedCommands.registerCommand("Align to HPS", new LockInHPS(drivetrain, 50));
        NamedCommands.registerCommand("Align to Reef", new LockInReef(drivetrain, 4.4));
        NamedCommands.registerCommand("Turn To Reef", new TurnInReef(drivetrain));
        NamedCommands.registerCommand("Align to Processor", new LockInProcessor(drivetrain, 50));
        configureBindings();

        SmartDashboard.putData("AutonChooser", AutonChooser);
        AutonChooser.setDefaultOption("PID Test: ", new PathPlannerAuto("pidcontrols"));
        AutonChooser.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        AutonChooser.addOption("Simple Auto", new PathPlannerAuto("Simple R Auton"));
    }

    private void configureBindings() {
        //drive
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, driverController));

        SignalLogger.setPath("/media/sda1/ctre-logs/");
        driverController.leftBumper().and(driverController.a()).whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward)                
        .beforeStarting(() -> SignalLogger.start()) // Start logging
        .finallyDo(() -> SignalLogger.stop())
        );
        driverController.leftBumper().and(driverController.b()).whileTrue(drivetrain.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse)
        .beforeStarting(() -> SignalLogger.start()) // Start logging
        .finallyDo(() -> SignalLogger.stop())
        );
        driverController.rightBumper().and(driverController.a()).whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward)
        .beforeStarting(() -> SignalLogger.start()) // Start logging
        .finallyDo(() -> SignalLogger.stop())
        );
        driverController.rightBumper().and(driverController.b()).whileTrue(drivetrain.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse)
        .beforeStarting(() -> SignalLogger.start()) // Start logging
        .finallyDo(() -> SignalLogger.stop())
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        //climber
        // driverController.leftTrigger().onTrue(climber.run(()-> climber.setClimberSpeed(-MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1))));
        // driverController.rightTrigger().onTrue(climber.run(()-> climber.setClimberSpeed(MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1))));
        // driverController.b().onTrue(climber.runOnce(()->climber.gotoPos(-0.6)));

        // picking up and dropping intake
        // operatorController.leftBumper().onTrue(intake.run(()->intake.setRotateSpeed(1)));
        // operatorController.leftBumper().onFalse(intake.run(()->intake.setRotateSpeed(0)));
        // operatorController.rightBumper().onTrue(intake.run(()->intake.setRotateSpeed(-1)));
        // operatorController.rightBumper().onFalse(intake.run(()->intake.setRotateSpeed(0)));

        //intaking coral and possibly spitting it out
        // operatorController.leftTrigger().onTrue(intake.run(()->intake.setIntakeSpeed(0.7*operatorController.getLeftTriggerAxis())));
        // operatorController.rightTrigger().onTrue(intake.run(()->intake.setIntakeSpeed(-0.7*operatorController.getRightTriggerAxis())));


        //raising and lowering elevator
        // operatorController.leftTrigger().onTrue(elevator.run(()->elevator.setElevatorMotorSpeed(-0.7 * operatorController.getLeftTriggerAxis())));
        // operatorController.rightTrigger().onTrue(elevator.run(()->elevator.setElevatorMotorSpeed(0.7 * operatorController.getRightTriggerAxis())));

        //rotatting arm
        //operatorController.leftTrigger().onTrue(arm.run(()->arm.setRotateSpeed(operatorController.getLeftTriggerAxis())));
        
        //wrist pos
        // operatorController.b().onTrue(new WristCommand(arm, Position.ALGEA_L2));
        // operatorController.a().onTrue(new WristCommand(arm, Position.STOW));
        // //arm intake
        // operatorController.leftTrigger().onTrue(arm.run(()->arm.setIntakeSpeed(MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1))));
        // operatorController.rightTrigger().onTrue(arm.run(()->arm.setIntakeSpeed(-MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1))));


        //arm positions
        // operatorController.a().onTrue(new CoralL1(elevator, arm));
        // operatorController.b().onTrue(new CoralL2(elevator, arm));
        // operatorController.x().onTrue(new CoralL3(elevator, arm));
        // operatorController.y().onTrue(new CoralL4(elevator, arm));
        // operatorController.povUp().onTrue(new AlgeaBarge(elevator, arm));
        // operatorController.povDown().onTrue(new AlgeaProcesor(elevator, arm));
        // operatorController.povLeft().onTrue(new AlgeaL2(elevator, arm));
        // operatorController.povRight().onTrue(new AlgeaL3(elevator, arm));
    }

    public CommandXboxController getOperatorJoystick(){
        return driverController;
    }
    
    public CommandXboxController getDriverJoystick(){
        return operatorController;
    }

    public Command getAutonomousCommand() {
        return AutonChooser.getSelected();
    }
}
