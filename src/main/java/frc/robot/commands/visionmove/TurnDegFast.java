package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.Drivetrain;

public class TurnDegFast {
    public static class TurnCalib{
        double positive_m,positive_b,negative_m,negative_b;
        TurnCalib(double positive_m, double positive_b, double negative_m, double negative_b){
            this.positive_m=positive_m;
            this.positive_b=positive_b;
            this.negative_m=negative_m;
            this.negative_b=negative_b;
        }
        int getTicksForAngle(Rotation2d angle){ //negative values mean turn in other dir
            double angleDeg = angle.getDegrees();
            if(angleDeg>0){
                int ticks = (int) Math.ceil((angleDeg-positive_b)/positive_m);
                if(ticks<0)return(0);
                return ticks;
            }else if(angleDeg<0){
                int ticks = (int) Math.ceil((angleDeg-negative_b)/negative_m);
                if(ticks<0)return(0);
                return -ticks;

            }else{
                return 0;
            }
        }
    }
    static TurnCalib calib = new TurnCalib(10.394095165546121, -6.051049335186291, -10.078563203078048, 1.8705840594951755);
    static TurnTicks getTurnCommand(Rotation2d angle, Drivetrain driveTrain){
        int signedTicks = calib.getTicksForAngle(angle);
        //System.out.println(angle+" -> "+signedTicks);
        if(signedTicks>=0) return(new TurnTicks(signedTicks,1,driveTrain));
        else return(new TurnTicks(-signedTicks,-1,driveTrain));
    }
}
