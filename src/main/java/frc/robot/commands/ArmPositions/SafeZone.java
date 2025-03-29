// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SafeZone extends SequentialCommandGroup {
  // public SafeZone(Elevator elevator, Arm arm) {
  //   addCommands(
  //           new ParallelCommandGroup(
  //       new ElevatorCommand(elevator, Position.SAFE_ZONE),
  //       new rotateArmCommand(arm, Position.SAFE_ZONE)
  //     )
  //   );
  // }
}
