package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

// Static class for simple methods to plot a trajectory on the Field map

public class TrajectoryPlotter {
    private Field2d m_field2d;
    private int m_maxTrajectory = 0;
    private int m_maxWaypoints = 0;
    private int m_index = 0;
    public TrajectoryPlotter(Field2d field) {
        m_field2d = field;
    }

    public void clear() {
        // Seems to be the only way to clear the lists
        m_field2d.getObject("trajectory").setPoses(Collections.<Pose2d>emptyList());
        for (int i = 1; i <= m_maxTrajectory; i++)
            m_field2d.getObject("trajectory" + i).setPoses(Collections.<Pose2d>emptyList());
        m_field2d.getObject("waypoints").setPoses(Collections.<Pose2d>emptyList());
        for (int i = 1; i <= m_maxWaypoints; i++)
            m_field2d.getObject("waypoints" + i).setPoses(Collections.<Pose2d>emptyList());
        m_index=0;
    }

    public void plotTrajectory(Trajectory trajectory) {
        plotTrajectory(m_index++, trajectory);
    }

    public void plotTrajectory(int index, Trajectory trajectory) {
        String indexStr = "";
        if (index > 0) {
            indexStr = String.valueOf(index);
            m_maxTrajectory = Math.max(m_maxTrajectory, index);
        }
        m_field2d.getObject("trajectory" + indexStr)
                .setPoses(trajectory.getStates().stream()
                        .map(state -> state.poseMeters).collect(Collectors.toList()));
    }
    public void addTrajectory(Trajectory trajectory) {
        addTrajectory(m_index-1,trajectory);

    }
    public void addTrajectory(int index, Trajectory trajectory) {
        String indexStr = "";
        if (index > 0) {
            indexStr = String.valueOf(index);
        }
        m_field2d.getObject("trajectory" + indexStr)
                .setPoses(
                        Stream.concat(
                                m_field2d.getObject("trajectory" + indexStr).getPoses().stream(),
                                trajectory.getStates().stream().map(state -> state.poseMeters)
                        ).collect(Collectors.toList())
                );

    }

    public void plotWaypoints(List<Translation2d> waypoints) {
        plotWaypoints(0, waypoints);
    }

    public void plotWaypoints(int index, List<Translation2d> waypoints) {
        String indexStr = "";
        if (index > 0) {
            indexStr = String.valueOf(index);
            m_maxWaypoints = Math.max(m_maxWaypoints, index);
        }

        final Rotation2d rot = Rotation2d.fromDegrees(0);

        List<Pose2d> poses = new ArrayList<Pose2d>();
        for (Translation2d t : waypoints) {
            poses.add(new Pose2d(t, rot));
        }

        m_field2d.getObject("waypoints" + indexStr).setPoses(poses);
    }
}