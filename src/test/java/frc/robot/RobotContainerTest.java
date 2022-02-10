package frc.robot;

import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class RobotContainerTest {


    @Test
    public void testTrajectoryTime() {

        var config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared);

        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics);
        // Apply the voltage constraint
        config.addConstraint(DriveConstants.autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through interior waypoints
                List.of(/* new Translation2d(4.5, 4) */),
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

                // End 2 meters straight ahead of where we started, facing forward
                // new Pose2d(2, 0, new Rotation2d(0)),
                new Pose2d(1, 0, new Rotation2d(0)),
                // Pass config
                config);

        var totalTime = exampleTrajectory.getTotalTimeSeconds();

        assertTrue(totalTime > 3);
    }

}
