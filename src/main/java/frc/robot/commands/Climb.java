package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
    private final Climber climber;
    private RobotContainer robotContainer;

    private CommandXboxController opController;

    private double climbValue;

    public Climb(Climber climber, CommandXboxController op){
        this.climber = climber;
        this.opController = op;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        robotContainer = RobotContainer.getInstance();
        climber.resetClimberPosition();
    }

    @Override
    public void execute(){

        climbValue = MathUtil.applyDeadband(opController.getLeftY(), 0.1);

        climber.setClimberSpeed(climbValue);
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
