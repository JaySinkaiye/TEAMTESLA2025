// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReefPositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Position;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.rotateArmCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralL4 extends SequentialCommandGroup {
  /** Creates a new CoralL4. */
  public CoralL4(Elevator elevator, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(      
        new ElevatorCommand(elevator, Position.CORAL_L1),
        new rotateArmCommand(arm, Position.CORAL_L1)
        ),
      new WristCommand(arm, Position.CORAL_L1)
    );
  }
}
