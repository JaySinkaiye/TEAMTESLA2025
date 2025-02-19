package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Elevator;

public class Elevate extends Command {

    // private final Elevator elevator;
    // private CommandXboxController opController;
    // private double yVal;
    // private double negYVal;
    // private SlewRateLimiter ylimit;

    // public Elevate(Elevator elevator, CommandXboxController op){
    //     this.elevator = elevator;
    //     this.opController = op;
    //     addRequirements(elevator);

    //     ylimit = new SlewRateLimiter(0.5);
    // }

    // @Override
    // public void execute(){
    //     yVal = MathUtil.applyDeadband(opController.getLeftTriggerAxis(), 0.1);
    //     negYVal = -MathUtil.applyDeadband(opController.getRightTriggerAxis(), 0.1);

    //     if(yVal > 0.1){
    //         elevator.setElevatorMotorSpeed(ylimit.calculate(yVal));
    //     } else if(negYVal < -0.1){
    //         elevator.setElevatorMotorSpeed(ylimit.calculate(negYVal));
    //     }
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
