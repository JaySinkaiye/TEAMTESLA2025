// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Position;
// import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristCommand extends Command {
  // private Arm arm;
  // private Position position;

  // private double wSetpoint;
  // private double wTolerance = 1;

  // public WristCommand(Arm arm, Position position) {
  //   this.arm = arm;
  //   this.position = position;
  //   addRequirements(arm);
  // }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   switch (position) {
  //     case ALGEA_BARGE:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case ALGEA_L2:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case ALGEA_L3:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case ALGEA_PROCESSOR:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case CORAL_L1:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case CORAL_L2:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case CORAL_L3:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case CORAL_L4:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //     case STOW:
  //     wSetpoint = 0;
  //     arm.wristGoToPos(wSetpoint);
  //       break;
  //   }
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   double wError = arm.getWristPosition() - wSetpoint;
  //   return Math.abs(wError) < wTolerance;
  // }
}
