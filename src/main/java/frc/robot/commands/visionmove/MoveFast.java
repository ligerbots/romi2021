package frc.robot.commands.visionmove;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.Pair;
import frc.robot.subsystems.Drivetrain;

public class MoveFast {
    public static class MoveCalib{
        double distance_2, distance_1, distance_0,angle_2, angle_1, angle_0;
        MoveCalib(double distance_2, double distance_1, double distance_0, double angle_2, double angle_1, double angle_0){
            this.distance_2=distance_2;
            this.distance_1=distance_1;
            this.distance_0=distance_0;
            this.angle_2=angle_2;
            this.angle_1=angle_1;
            this.angle_0=angle_0;
        }
        Pair<Integer,Double> getTicksAndAngleForDistance(double distance){
            double ticks = distance_2*(distance*distance)+distance_1*distance+distance_0;
            double angle = angle_2*(ticks*ticks) + angle_1*ticks + angle_0;
            return(new Pair<>((int)ticks,angle));
        }
    }
    static MoveCalib calib_0p8 = new MoveCalib(-1.1622297299108804, 88.7287269931268, 4.145264104521038, -7.579167217986519e-06, -0.002266973422209879, -0.0730844019189155);
}
