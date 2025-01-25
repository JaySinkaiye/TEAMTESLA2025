// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climb;
import frc.robot.commands.LockIn;
import frc.robot.commands.SwerveDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    public static RobotContainer robotContainer = new RobotContainer();

    //subsytems
    public final Climber climber = new Climber();

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> AutonChooser = new SendableChooser<>();

    public RobotContainer() {

        NamedCommands.registerCommand("Align to April Tag 12", new LockIn(drivetrain, 50));
        configureBindings();

        SmartDashboard.putData("AutonChooser", AutonChooser);
        AutonChooser.setDefaultOption("Test: ", new PathPlannerAuto("test"));

        climber.setDefaultCommand(new Climb(climber, operatorController));
    }

    public static RobotContainer getInstance(){
        return robotContainer;
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new SwerveDrive(drivetrain, driverController));
        driverController.x().onTrue(new LockIn(drivetrain, 50));
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
