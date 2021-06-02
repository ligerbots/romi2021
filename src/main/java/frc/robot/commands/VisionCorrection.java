package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class VisionCorrection {
    public double globalOffsetX;
    public double globalOffsetY;
    public double localOffsetX;
    public double localOffsetY;

    public VisionCorrection(String filename){
        try {
            Scanner in = new Scanner(new File(filename));
            globalOffsetX = in.nextDouble();
            globalOffsetY = in.nextDouble();
            localOffsetX = in.nextDouble();
            localOffsetY = in.nextDouble();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
    public Pose2d correct(Pose2d vision){
        Pose2d visionGlobalCorrection = new Pose2d(
                new Translation2d(vision.getX()+globalOffsetX,vision.getY()+globalOffsetY),
                vision.getRotation()
        );
        Pose2d visionLocalCorrection = visionGlobalCorrection.transformBy(new Transform2d(new Translation2d(
                localOffsetX,
                localOffsetY
        ),new Rotation2d(0)));
        return(visionLocalCorrection);
    }
}
