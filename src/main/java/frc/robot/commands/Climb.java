package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
    private final Climber climber;

    private CommandXboxController dController;

    private double climbValue;

    public Climb(Climber climber, CommandXboxController d){
        this.climber = climber;
        this.dController = d;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.resetClimberPosition();
    }

    @Override
    public void execute(){

        if (dController.getLeftTriggerAxis() >= 0.1){
            climbValue = MathUtil.applyDeadband(dController.getLeftTriggerAxis(), 0.1);
        } else if (dController.getRightTriggerAxis() >= 0.1){
            climbValue = MathUtil.applyDeadband(dController.getRightTriggerAxis(), 0.1) * -1;
        } else {
            climbValue = 0;
        }
        climber.setClimberSpeed(climbValue);

        dController.b().onTrue(climber.gotToPos(30));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
