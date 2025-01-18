package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class Elevate extends Command {

    // private final Elevator elevator;
    // private DoubleSupplier ySup;
    // private double yVal;

    // public Elevate(Elevator elevator, DoubleSupplier ySup){
    //     this.elevator = elevator;
    //     this.ySup = ySup;
    //     addRequirements(elevator);
    // }

    // @Override
    // public void execute(){
    //     yVal = MathUtil.applyDeadband(ySup.getAsDouble(), 0.1);
    //     elevator.ElevateHer(yVal);
    // }

    // // Called once the command ends or is interrupted.
    // @Override
    // public void end(boolean interrupted) {
        
    // }

    // // Returns true when the command should end.
    // @Override
    // public boolean isFinished() {
    //     return false;
    // }

}
