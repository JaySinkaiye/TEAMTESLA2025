// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Position;
import frc.robot.subsystems.Arm;

public class WristCommand extends Command {
  private Arm arm;
  private Position position;

  private double setpoint;
  private double wTolerance = 0.005;

  public WristCommand(Arm arm, Position position) {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (position) {
      case ALGEA_BARGE:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case ALGEA_L2:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case ALGEA_L3:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case ALGEA_PROCESSOR:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case CORAL_L1:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case CORAL_L2:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case CORAL_L3:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case CORAL_L4:
      setpoint = 0.2;
      arm.wristGoToPos(setpoint);
        break;
      case STOW:
      setpoint = 0;
      arm.wristGoToPos(setpoint);
        break;
      case HUMAN_PLAYER_STATION:
      setpoint = 0;
        arm.wristGoToPos(setpoint);;
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
    double wError = arm.getWristPosition() - setpoint;
    return Math.abs(wError) < wTolerance;
  }
}
