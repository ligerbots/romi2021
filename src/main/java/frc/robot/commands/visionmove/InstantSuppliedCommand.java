package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public class InstantSuppliedCommand extends CommandBase {
    Command command;
    Supplier<Command> supplier;
    InstantSuppliedCommand(Supplier<Command> supplier, Subsystem... requirements){
        this.supplier = supplier;
        for(Subsystem requirement:requirements)
            addRequirements(requirement);
    }

    @Override
    public void initialize() {
        command=supplier.get();
        command.initialize();
    }
    @Override
    public void execute(){
        command.execute();

    }
    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);

    }
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
