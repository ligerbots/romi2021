package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;

public class Vision extends SubsystemBase {

    static final int port = 7000;
    static class Sample{
        float[] pose;
        long timestamp;
        Sample(long timestamp, float[] pose){
            this.pose=pose;
            this.timestamp=timestamp;
        }

        @Override
        public String toString() {
            return "Sample{" +
                    "pose=" + Arrays.toString(pose) +
                    ", timestamp=" + timestamp +
                    '}';
        }
    }
    float offsetX=0;
    float offsetY=0;
    public void initSmartDashboard() {
        SmartDashboard.getEntry("offsetX").addListener((EntryNotification e) -> offsetX = (float) e.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        SmartDashboard.putNumber("offsetX", offsetX);
        SmartDashboard.getEntry("offsetY").addListener((EntryNotification e) -> offsetY = (float) e.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        SmartDashboard.putNumber("offsetY", offsetY);
    }

    Queue<Sample> samples = new ConcurrentLinkedQueue<Sample>();

    Drivetrain drivetrain;

    public Vision(Drivetrain drivetrain){
        this.drivetrain=drivetrain;
        new Thread(new Server(samples)).start();
    }

    @Override
    public void periodic(){
        while(true){
            Sample sample = samples.poll();
            if(sample == null)break;
            double x = Units.inchesToMeters(sample.pose[0]+offsetX);
            double y = Units.inchesToMeters(sample.pose[2]+offsetY);
            Rotation2d rot = new Rotation2d(Units.degreesToRadians(180-sample.pose[3]));
            //drivetrain.addVisionSample(new Pose2d(x,y,rot), sample.timestamp);
            //drivetrain.m_field2d.setRobotPose(new Pose2d(x,y,rot));
        }
    }


    private static class Server implements Runnable {
        Queue<Sample> sampleQueue;
        Server(Queue<Sample> sampleQueue){
            this.sampleQueue=sampleQueue;
        }
        public void run() {
            ServerSocket server = null;

            try {
                server = new ServerSocket(port);
                server.setReuseAddress(true);

                while (true) {
                    Socket client = server.accept();

                    System.out.println("New client connected: "
                            + client.getInetAddress()
                            .getHostAddress());

                    new Thread(new ClientHandler(client, sampleQueue)).start();
                }
            } catch (IOException e) {
                e.printStackTrace();
            } finally {
                if (server != null) {
                    try {
                        server.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }
    private static class ClientHandler implements Runnable {
        private final Socket clientSocket;
        Queue<Sample> sampleQueue;
        public ClientHandler(Socket socket, Queue<Sample> sampleQueue)
        {
            this.clientSocket = socket;
            this.sampleQueue = sampleQueue;
        }

        public void run()
        {
            PrintWriter out = null;
            Scanner in = null;
            try {

                out = new PrintWriter(clientSocket.getOutputStream());

                in = new Scanner(clientSocket.getInputStream());

                while (in.hasNext()) {
                    String token = in.next();
                    switch(token) {
                        case "sync":
                            out.print(RobotController.getFPGATime()/1000);
                            out.flush();
                            break;
                        case "data":
                            long time = in.nextLong();
                            int N = in.nextInt();
                            float[] pose=new float[N];
                            for(int i=0;i<N;i++){
                                pose[i]=in.nextFloat();
                            }
                            if(sampleQueue.size()<10)sampleQueue.add(new Sample(time,pose));

                            break;
                        default:
                            System.out.println("Invalid token "+token);
                    }
                }
            }
            catch (IOException e) {
                e.printStackTrace();
            }
            finally {
                try {
                    if (out != null) {
                        out.close();
                    }
                    if (in != null) {
                        in.close();
                        clientSocket.close();
                    }
                }
                catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
