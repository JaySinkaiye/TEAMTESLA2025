// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Position;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class rotateArmCommand extends Command {
  private Arm arm;
  private Position position;

  private double rSetpoint = 0;

  private double rTolerance = 1;

  public rotateArmCommand(Arm arm, Position position) {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (position) {
      case ALGEA_BARGE:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case ALGEA_L2:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case ALGEA_L3:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case ALGEA_PROCESSOR:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L1:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L2:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L3:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L4:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
      case STOW:
      rSetpoint= 0;
      arm.rotateToPos(rSetpoint);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double rError = arm.getRotatePosition() - rSetpoint;
    return Math.abs(rError)< rTolerance;
  }
}
