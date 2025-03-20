// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.AprilTagPositions.LockInHPS;
import frc.robot.commands.AprilTagPositions.LockInProcessor;
import frc.robot.commands.AprilTagPositions.LockInReef;
import frc.robot.commands.AprilTagPositions.TurnInReef;
import frc.robot.commands.ArmPositions.AlgaeProccesor;
import frc.robot.commands.ArmPositions.AlgeaBarge;
import frc.robot.commands.ArmPositions.AlgeaL2;
import frc.robot.commands.ArmPositions.AlgeaL3;
import frc.robot.commands.ArmPositions.CoralL1;
import frc.robot.commands.ArmPositions.CoralL2;
import frc.robot.commands.ArmPositions.CoralL3;
import frc.robot.commands.ArmPositions.CoralL4;
import frc.robot.commands.ArmPositions.HumanPlayerStation;
import frc.robot.commands.ArmPositions.Stow;
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
        //auto align
        NamedCommands.registerCommand("Align to HPS", new LockInHPS(drivetrain, 50));
        NamedCommands.registerCommand("Align to Reef", new LockInReef(drivetrain, 4.4));
        NamedCommands.registerCommand("Turn To Reef", new TurnInReef(drivetrain));
        NamedCommands.registerCommand("Align to Processor", new LockInProcessor(drivetrain, 50));

        // arm positions
        NamedCommands.registerCommand("Algae Processor", new AlgaeProccesor(elevator, arm));
        NamedCommands.registerCommand("Algae Barge", new AlgeaBarge(elevator, arm));
        NamedCommands.registerCommand("Algae L2", new AlgeaL2(elevator, arm));
        NamedCommands.registerCommand("Algae L3", new AlgeaL3(elevator, arm));
        NamedCommands.registerCommand("Coral L1", new CoralL1(elevator, arm));
        NamedCommands.registerCommand("Coral L2", new CoralL2(elevator, arm));
        NamedCommands.registerCommand("Coral L2", new CoralL3(elevator, arm));
        NamedCommands.registerCommand("Coral L4", new CoralL4(elevator, arm));
        NamedCommands.registerCommand("Stow", new Stow(elevator, arm));
        NamedCommands.registerCommand("Human Player Station", new HumanPlayerStation(elevator, arm));

        NamedCommands.registerCommand("Outtake Coral", arm.run(()->arm.setIntakeSpeed(-0.8)));
        NamedCommands.registerCommand("Intake Coral", arm.run(()->arm.setIntakeSpeed(0.8)));

        configureBindings();

        SmartDashboard.putData("AutonChooser", AutonChooser);
        AutonChooser.setDefaultOption("PID Test: ", new PathPlannerAuto("pidcontrols"));
        AutonChooser.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        AutonChooser.addOption("Simple Auto", new PathPlannerAuto("Simple R Auton"));
    }

    private void configureBindings() {
        //drive
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, driverController, elevator));

        //rotate arm
        arm.setDefaultCommand(arm.run(()->arm.setRotateSpeed(MathUtil.applyDeadband(operatorController.getLeftX(), 0.1))));

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

        //intaking coral and spitting it out
        // operatorController.leftTrigger().onTrue(intake.run(()->intake.setIntakeSpeed(0.7*operatorController.getLeftTriggerAxis())));
        // operatorController.rightTrigger().onTrue(intake.run(()->intake.setIntakeSpeed(-0.7*operatorController.getRightTriggerAxis())));


        //raising and lowering elevator
        operatorController.leftTrigger().onTrue(elevator.run(()->elevator.setElevatorMotorSpeed(-0.7 * operatorController.getLeftTriggerAxis())));
        operatorController.rightTrigger().onTrue(elevator.run(()->elevator.setElevatorMotorSpeed(0.7 * operatorController.getRightTriggerAxis())));

        //rotating wrist
        operatorController.leftBumper().onTrue(arm.run(()->arm.setWristSpeed(0.2)));
        operatorController.leftBumper().onFalse(arm.run(()->arm.setWristSpeed(0)));
        operatorController.rightBumper().onTrue(arm.run(()->arm.setWristSpeed(-0.2)));
        operatorController.rightBumper().onFalse(arm.run(()->arm.setWristSpeed(0)));

        // operatorController.a().onTrue(arm.run(()->arm.wristGoToPos(0.2)));
        // operatorController.b().onTrue(arm.run(()->arm.wristGoToPos(0)));


        //arm intake
        operatorController.button(7).onTrue(arm.run(()->arm.setIntakeSpeed(0.8)));
        operatorController.button(7).onFalse(arm.run(()->arm.setIntakeSpeed(0)));
        operatorController.button(8).onTrue(arm.run(()->arm.setIntakeSpeed(-0.8)));
        operatorController.button(8).onFalse(arm.run(()->arm.setIntakeSpeed(0)));

        //arm positions
        operatorController.x().onTrue(new ElevatorCommand(elevator, Position.STOW));
        operatorController.b().onTrue(new ElevatorCommand(elevator, Position.CORAL_L1));
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
