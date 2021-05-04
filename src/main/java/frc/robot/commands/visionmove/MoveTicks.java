package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

class MoveTicks extends CommandBase {
    int ticks;
    int maxTicks;
    int rampTicks = 5;
    double speed;
    Drivetrain driveTrain;

    MoveTicks(int ticks, double speed, Drivetrain driveTrain) {
        this.maxTicks = ticks;
        this.speed = speed;
        this.driveTrain=driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        ticks = 0;
        driveTrain.arcadeDrive(0, 0);

    }

    @Override
    public void execute() {
        if(ticks<maxTicks) {
            driveTrain.arcadeDrive(speedAtTick(ticks), 0);
            ticks++;
        }
    }
    @Override
    public void end(boolean interrupted){
        driveTrain.arcadeDrive(0, 0);

    }
    public double speedAtTick(double tick){
        if(tick<rampTicks){
            return(tick*speed/rampTicks);
        }else if(tick<maxTicks-rampTicks){
            return(speed);
        }else{
            return((maxTicks-tick)*speed/rampTicks);
        }
    }
    @Override
    public boolean isFinished() {
        return ticks>=maxTicks;
    }
}