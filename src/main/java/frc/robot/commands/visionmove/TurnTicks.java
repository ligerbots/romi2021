package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

class TurnTicks extends CommandBase {
    int ticks;
    int maxTicks;
    double turn;
    Drivetrain driveTrain;

    TurnTicks(int ticks, double turn, Drivetrain driveTrain) {
        this.maxTicks = ticks;
        this.turn = turn;
        this.driveTrain=driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        ticks = 0;
        driveTrain.arcadeDrive(0, turn);

    }

    @Override
    public void execute() {
        if(ticks<maxTicks) {
            driveTrain.arcadeDrive(0, turn);
            ticks++;
        }
    }
    @Override
    public void end(boolean interrupted){
        driveTrain.arcadeDrive(0, 0);

    }
    @Override
    public boolean isFinished() {
        return ticks>=maxTicks;
    }
}