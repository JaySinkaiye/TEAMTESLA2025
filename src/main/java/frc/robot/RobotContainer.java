// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.AprilTagPositions.LockInHPS;
import frc.robot.commands.AprilTagPositions.LockInProcessor;
import frc.robot.commands.AprilTagPositions.autoReefAlign;
import frc.robot.commands.ArmPositions.AlgaeProccesor;
import frc.robot.commands.ArmPositions.AlgeaBarge;
import frc.robot.commands.ArmPositions.AlgeaL2;
import frc.robot.commands.ArmPositions.AlgeaL3;
import frc.robot.commands.ArmPositions.CoralL1;
import frc.robot.commands.ArmPositions.CoralL2;
import frc.robot.commands.ArmPositions.CoralL3;
import frc.robot.commands.ArmPositions.CoralL4;
import frc.robot.commands.ArmPositions.HumanPlayerStation;
import frc.robot.commands.ArmPositions.SafeZone;
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
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> AutonChooser = new SendableChooser<>();
    private final SlewRateLimiter eLimiter = new SlewRateLimiter(0.6);

    
    public RobotContainer() {
        //auto align
        NamedCommands.registerCommand("Align to HPS", new LockInHPS(drivetrain, 50));
        NamedCommands.registerCommand("Align to Reef", new autoReefAlign(drivetrain));
        NamedCommands.registerCommand("Align to Processor", new LockInProcessor(drivetrain, 50));

        // arm positions
        // NamedCommands.registerCommand("Algae Processor", new AlgaeProccesor(elevator, arm));
        // NamedCommands.registerCommand("Algae Barge", new AlgeaBarge(elevator, arm));
        // NamedCommands.registerCommand("Algae L2", new AlgeaL2(elevator, arm));
        // NamedCommands.registerCommand("Algae L3", new AlgeaL3(elevator, arm));
        // NamedCommands.registerCommand("Coral L1", new CoralL1(elevator, arm));
        // NamedCommands.registerCommand("Coral L2", new CoralL2(elevator, arm));
        // NamedCommands.registerCommand("Coral L2", new CoralL3(elevator, arm));
        // NamedCommands.registerCommand("Coral L4", new CoralL4(elevator, arm));
        // NamedCommands.registerCommand("Stow", new SafeZone(elevator, arm));
        // NamedCommands.registerCommand("Human Player Station", new HumanPlayerStation(elevator, arm));

        // NamedCommands.registerCommand("Outtake Coral", arm.run(()->arm.setIntakeSpeed(-0.8)));
        // NamedCommands.registerCommand("Intake Coral", arm.run(()->arm.setIntakeSpeed(0.8)));

        configureBindings();

        SmartDashboard.putData("AutonChooser", AutonChooser);
        AutonChooser.addOption("Left Auto", new PathPlannerAuto("left"));
        AutonChooser.addOption("Right Auto", new PathPlannerAuto("right"));
        AutonChooser.addOption("Center Auto", new PathPlannerAuto("center"));
    }

    private void configureBindings() {
        //////DRIVER CONTROLS
        //drive
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, driverController, elevator));

        //climber
        driverController.leftTrigger().onTrue(climber.run(()-> climber.setClimberSpeed(-MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1))));
        driverController.rightTrigger().onTrue(climber.run(()-> climber.setClimberSpeed(MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1))));
        //driverController.b().onTrue(climber.runOnce(()->climber.gotoPos(-0.6)));

        ///// END OF DRIVER CONTROLS
        
        
    
        /// OPERATOR CONTROLS

        // rotate arm, left trigger == intake, right trigger == outake
        arm.setDefaultCommand(arm.run(()->arm.manualControl(MathUtil.applyDeadband(operatorController.getRightX(), 0.1), operatorController.leftTrigger(), operatorController.rightTrigger(), operatorController.getLeftTriggerAxis()*0.6, operatorController.getRightTriggerAxis()*0.6)));
        
        // rotate intake, left trigger == intake, right trigger == outake;
        intake.setDefaultCommand(intake.run(()->intake.manualControl(operatorController.leftBumper(), operatorController.rightBumper(), 0.3, operatorController.leftTrigger(), operatorController.rightTrigger(), operatorController.getLeftTriggerAxis()*0.2, operatorController.getRightTriggerAxis()*0.2)));

        // //raising and lowering elevator
        elevator.setDefaultCommand(elevator.run(()->elevator.setElevatorMotorSpeed(eLimiter.calculate(MathUtil.applyDeadband(operatorController.getLeftY(), 0.1)), driverController, operatorController.a(), operatorController.x(), operatorController.y(), operatorController.b())));

        // //rotate intake up and down
        // operatorController.leftBumper().onTrue(intake.run(()->intake.setRotateSpeed(0.2)));
        // operatorController.leftBumper().onFalse(intake.run(()->intake.setRotateSpeed(0)));
        // operatorController.rightBumper().onTrue(intake.run(()->intake.setRotateSpeed(-0.2)));
        // operatorController.rightBumper().onFalse(intake.run(()->intake.setRotateSpeed(0)));

        // //intake from intake
        // operatorController.povLeft().onTrue(intake.run(()->intake.setIntakeSpeed(0.3)));
        // operatorController.povLeft().onFalse(intake.run(()->intake.setIntakeSpeed(0)));
        // operatorController.povRight().onTrue(intake.run(()->intake.setIntakeSpeed(-0.3)));
        // operatorController.povRight().onFalse(intake.run(()->intake.setIntakeSpeed(0)));


        // operatorController.button(7).onTrue(new intakeToArm(arm, intake));

       
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
