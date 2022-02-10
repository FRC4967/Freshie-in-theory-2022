package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.ComplexAuto;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveDistanceProfiled;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    // The robot's subsystems
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // A chooser for autonomous commands
    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    private final Command m_simpleAuto = new DriveDistanceProfiled(AutoConstants.kAutoDriveDistance, m_robotDrive);

    // A complex auto routine that drives forward, drops a hatch, and then drives
    // backward.
    private final Command m_complexAuto = new ComplexAuto(m_robotDrive);

    XboxController m_xboxController = null;  //= new XboxController(0);
    Joystick m_joystick = new Joystick(0);

    public RobotContainer() {

        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new DefaultDrive(m_robotDrive, this::getJoystickForward, this::getJoystickTurn));

        m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
        m_chooser.addOption("Complex Auto", m_complexAuto);

        // Put the chooser on the dashboard
        Shuffleboard.getTab("Autonomous").add(m_chooser);
    }

    public double getJoystickForward() {
        if (m_xboxController != null) {
            return m_xboxController.getLeftY();
        } else {
            return m_joystick.getRawAxis(1);
        }
    }

    public double getJoystickTurn() {
        if (m_xboxController != null) {
            return m_xboxController.getRightX();
        } else {
            return m_joystick.getRawAxis(0);
        }
    }

    public void simulationInit() {
        m_robotDrive.simulationInit();
    }

    private TrajectoryConfig createTrajectoryConfig(double maxVelocityMetersPerSecond,
            double maxAccelerationMetersPerSecondSq) {
        var config = new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics);
        // Apply the voltage constraint
        config.addConstraint(DriveConstants.autoVoltageConstraint);

        return config;

    }

    public Command createTrajectoryCommand() {

        // Create config for trajectory
        var config = createTrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        System.out.println("Total states " + exampleTrajectory.getStates().size());
        System.out.println("Expected total time: " + exampleTrajectory.getTotalTimeSeconds());

        System.out.println("Start time: " + Timer.getFPGATimestamp());

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(this::commandCompletion);
    }

    private void commandCompletion() {
        m_robotDrive.tankDriveVolts(0, 0);
        System.out.println("End time: " + Timer.getFPGATimestamp());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return createTrajectoryCommand();
        // return m_simpleAuto;
    }

    /**
     * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be
     * called on robot
     * disable to prevent integral windup.
     */
    public void disablePIDSubsystems() {

    }
}
