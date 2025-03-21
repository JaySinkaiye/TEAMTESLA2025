// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Position;
import frc.robot.positionConstants;
import frc.robot.subsystems.Arm;

public class rotateArmCommand extends Command {
  private Arm arm;
  private Position position;

  private double rSetpoint;

  private double rTolerance = positionConstants.armRotationConstants.ARM_TOLERANCE;

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
      rSetpoint= positionConstants.armRotationConstants.ALGAE_BARGE_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case ALGEA_L2:
      rSetpoint= positionConstants.armRotationConstants.ALGAE_L2_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case ALGEA_L3:
      rSetpoint= positionConstants.armRotationConstants.ALGAE_L3_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case ALGEA_PROCESSOR:
      rSetpoint= positionConstants.armRotationConstants.ALGAE_PROCESSOR_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L1:
      rSetpoint= positionConstants.armRotationConstants.CORAL_L1_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L2:
      rSetpoint= positionConstants.armRotationConstants.CORAL_L2_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L3:
      rSetpoint= positionConstants.armRotationConstants.CORAL_L3_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case CORAL_L4:
      rSetpoint= positionConstants.armRotationConstants.CORAL_L4_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case STOW:
      rSetpoint= positionConstants.armRotationConstants.STOW_POSITION;
      arm.rotateToPos(rSetpoint);
        break;
      case HUMAN_PLAYER_STATION:
      rSetpoint = positionConstants.armRotationConstants.HUMAN_PLAYER_STATION_POSITION;
        arm.rotateToPos(rSetpoint);;
      default:
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
