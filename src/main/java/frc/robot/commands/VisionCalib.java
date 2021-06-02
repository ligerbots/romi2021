package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class VisionCalib extends SequentialCommandGroup implements AutoCommandInterface {

    Drivetrain drivetrain;
    static final Map<String, Pose2d> poses = new TreeMap<>();
    Map<Pose2d, Pose2d> samples = new HashMap<>();
    public static Translation2d grid(double x, double y){
        return(new Translation2d(Units.inchesToMeters(x*15/2),Units.inchesToMeters(y*15/2)));
    }
    static {
        Map<String, Translation2d> positions = new HashMap<>();

        positions.put("Starting Circle 1", grid(1,3));
        positions.put("Starting Circle 2", grid(1,1));
        positions.put("Starting Circle 3", grid(11,5));
        Map<String, Rotation2d> rotations = new HashMap<>();
        rotations.put("up", Rotation2d.fromDegrees(90));
        rotations.put("left", Rotation2d.fromDegrees(180));
        rotations.put("right", Rotation2d.fromDegrees(0));
        rotations.put("down", Rotation2d.fromDegrees(-90));
        for (Map.Entry<String, Translation2d> position : positions.entrySet()) {
            for (Map.Entry<String, Rotation2d> rotation : rotations.entrySet()) {
                poses.put(position.getKey()+" "+rotation.getKey(),new Pose2d(position.getValue(),rotation.getValue()));
            }
        }

    }
    class Wait extends CommandBase{
        String instr;
        Wait(String instr){
            this.instr=instr;
        }
        @Override
        public void initialize() {
            SmartDashboard.putBoolean("Done Moving Romi",false);
            SmartDashboard.putString("Romi Target Position",this.instr);
            System.out.println(this.instr);
        }
        @Override
        public void end(boolean interrupted) {
        }
        @Override
        public boolean isFinished() {
            return SmartDashboard.getBoolean("Done Moving Romi",false);
        }
    }
    public VisionCalib(Drivetrain drivetrain){
        this.drivetrain=drivetrain;
        for (Map.Entry<String, Pose2d> pose : poses.entrySet()) {
            addCommands(
                    new Wait(pose.getKey()),
                    drivetrain.new WaitForVision((Pose2d result)->{
                        samples.put(pose.getValue(),result);
                        drivetrain.setPose(result);
                    })
            );
        }
        addCommands(new InstantCommand(this::computeAndSave));
    }
    void computeAndSave(){
        // vision -> real
        int numberSamples = samples.size();
        double globalOffsetX = 0;
        double globalOffsetY = 0;
        double localOffsetX = 0;
        double localOffsetY = 0;
        for (Map.Entry<Pose2d, Pose2d> sample : samples.entrySet()) {
            Pose2d vision = sample.getValue();
            Pose2d real = sample.getKey();
            globalOffsetX+= real.getX() - vision.getX();
            globalOffsetY+= real.getY() - vision.getY();

            Transform2d localOffset = new Transform2d(vision, real);
            localOffsetX+=localOffset.getX();
            localOffsetY+=localOffset.getY();
        }
        globalOffsetX/=numberSamples;
        globalOffsetY/=numberSamples;
        localOffsetX/=numberSamples;
        localOffsetY/=numberSamples;

        double avgError=0;
        double avgErrorGlobalCorrection=0;
        double avgErrorLocalCorrection=0;

        for (Map.Entry<Pose2d, Pose2d> sample : samples.entrySet()) {
            Pose2d vision = sample.getValue();
            Pose2d real = sample.getKey();
            avgError += vision.getTranslation().getDistance(real.getTranslation());
            Pose2d visionGlobalCorrection = new Pose2d(
                    new Translation2d(vision.getX()+globalOffsetX,vision.getY()+globalOffsetY),
                    vision.getRotation()
            );
            avgErrorGlobalCorrection += visionGlobalCorrection.getTranslation().getDistance(real.getTranslation());
            Pose2d visionLocalCorrection = visionGlobalCorrection.transformBy(new Transform2d(new Translation2d(
                    localOffsetX,
                    localOffsetY
            ),new Rotation2d(0)));
            avgErrorLocalCorrection += visionLocalCorrection.getTranslation().getDistance(real.getTranslation());
        }
        avgError/=numberSamples;
        avgErrorGlobalCorrection/=numberSamples;
        avgErrorLocalCorrection/=numberSamples;
        System.out.println("Average distance error: "+avgError);
        System.out.println("Average distance error w/ global correction: "+avgErrorGlobalCorrection);
        System.out.println("Average distance error w/ local correction: "+avgErrorLocalCorrection);
        try {
            PrintWriter out = new PrintWriter(new File("visioncalib.txt"));
            out.println(globalOffsetX);
            out.println(globalOffsetY);
            out.println(localOffsetX);
            out.println(localOffsetY);
            out.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }
    @Override
    public Pose2d getInitialPose() {
        return null;
    }
}
