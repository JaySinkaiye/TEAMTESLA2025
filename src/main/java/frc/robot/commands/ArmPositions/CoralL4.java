// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Position;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.rotateArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class CoralL4 extends SequentialCommandGroup {
  public CoralL4(Elevator elevator, Arm arm) {

    addCommands(
      new ParallelCommandGroup(      
        new ElevatorCommand(elevator, Position.CORAL_L4),
        new rotateArmCommand(arm, Position.CORAL_L4)
        ),
      new WristCommand(arm, Position.CORAL_L4)
    );
  }
}
