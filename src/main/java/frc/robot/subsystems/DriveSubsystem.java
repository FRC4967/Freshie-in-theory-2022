package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;
import frc.robot.utility.CTREConvert;

public class DriveSubsystem extends SubsystemBase {

    // The gyro sensor
    // public final Gyro m_gyro = new ADXRS450_Gyro();
    // public final ADXRS453Gyro m_gyro = new ADXRS453Gyro();

    public final AHRS m_gyro = new AHRS(I2C.Port.kOnboard);

    private final WPI_TalonSRX m_rightLeader;
    private final WPI_VictorSPX m_rightFollower;

    private final WPI_TalonSRX m_leftLeader;
    private final WPI_VictorSPX m_leftFollower;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    // The robot's drive
    private final DifferentialDrive m_drive;

    private final int pidIdx = 0; // Default PID index
    private final int slotIdx = 0;

    /**
     * Timeout value in ms to wait for config success and report an error if it
     * times out
     */
    private static final int kConfigTimeout = 50;

    private double lastLeftDistance = 0;
    private double lastRightDistance = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        m_rightLeader = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
        configureMotor(m_rightLeader);
        m_rightLeader.setSensorPhase(DriveConstants.kRightEncoderReversed);
        m_rightLeader.setInverted(DriveConstants.kRightMotorReversed);

        m_rightFollower = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);
        m_rightFollower.setInverted(DriveConstants.kRightEncoderReversed);
        m_rightFollower.follow(m_rightLeader);

        m_leftLeader = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
        configureMotor(m_leftLeader);
        m_leftLeader.setSensorPhase(DriveConstants.kLeftEncoderReversed);
        m_leftLeader.setInverted(DriveConstants.kLeftMotorReversed);

        m_leftFollower = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
        m_leftFollower.setInverted(DriveConstants.kLeftMotorReversed);
        m_leftFollower.follow(m_leftLeader);

        m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        resetEncoders();
    }

    private void configureMotor(WPI_TalonSRX motor) {

        // Factory Default to prevent unexpected behaviour
        motor.configFactoryDefault(kConfigTimeout);

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidIdx, kConfigTimeout);
        motor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1, 0x00, 0, kConfigTimeout);

        // Reset sensor position to 0
        motor.setSelectedSensorPosition(0, pidIdx, kConfigTimeout);
        motor.overrideLimitSwitchesEnable(false);
        motor.enableCurrentLimit(true);
        motor.configPeakCurrentLimit(0, kConfigTimeout);
        motor.configContinuousCurrentLimit(DriveConstants.kTalonContinuousCurrentLimit, kConfigTimeout);
        motor.setNeutralMode(NeutralMode.Brake);

        motor.configNominalOutputForward(0, kConfigTimeout);
        motor.configNominalOutputReverse(0, kConfigTimeout);
        motor.configPeakOutputForward(1, kConfigTimeout);
        motor.configPeakOutputReverse(-1, kConfigTimeout);
        motor.configAllowableClosedloopError(0, pidIdx, kConfigTimeout);

        motor.config_kD(slotIdx, DriveConstants.kP, kConfigTimeout);
        motor.config_kI(slotIdx, DriveConstants.kI, kConfigTimeout);
        motor.config_kD(slotIdx, DriveConstants.kD, kConfigTimeout);

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        var leftDistance = getLeftEncoderDistance();
        var rightDistance = getRightEncoderDistance();

        if (leftDistance != lastLeftDistance) {
            var rawDist = m_rightLeader.getSelectedSensorPosition();
            System.out.println(String.format("L Dist %.5f, raw %.5f", leftDistance, rawDist));
            lastLeftDistance = leftDistance;
        }

        if (rightDistance != lastRightDistance) {
            lastRightDistance = rightDistance;
        }

        m_odometry.update(m_gyro.getRotation2d(), leftDistance, rightDistance);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_rightLeader.setSelectedSensorPosition(0);
        m_leftLeader.setSelectedSensorPosition(0);
    }

    /**
     * Gets the distance traveled by the right motor
     * 
     * @return distance, in meters
     */
    public double getRightEncoderDistance() {
        var distance = CTREConvert.nativeUnitsToDistanceMeters(m_rightLeader.getSelectedSensorPosition());
        return distance;
    }

    /**
     * Gets the distance traveled by the left motor
     * 
     * @return distance, in meters
     */
    public double getLeftEncoderDistance() {
        var distance = CTREConvert.nativeUnitsToDistanceMeters(m_leftLeader.getSelectedSensorPosition());
        return distance;
    }

    /**
     * Gets the average distance of the TWO encoders.
     *
     * @return the average of the TWO encoder readings
     */
    public double getAverageEncoderDistance() {
        double right = getRightEncoderDistance();
        double left = getLeftEncoderDistance();
        return (right + left) / 2;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        var pose = m_odometry.getPoseMeters();
        System.out.println(String.format("Pose %.5f", pose.getX()));
        return pose;
    }

    public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
        var leftPos = CTREConvert.distanceToNativeUnits(left.position);
        System.out.println(String.format("Left POS %d", leftPos));
        m_leftLeader.set(ControlMode.Position, leftPos);

        var rightPos = CTREConvert.distanceToNativeUnits(right.position);
        m_rightLeader.set(ControlMode.Position, rightPos);
        m_drive.feed();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        var leftMetersPerSecond = CTREConvert
                .nativeUnitsToVelocityMetersPerSecond(m_leftLeader.getSelectedSensorVelocity());
        var rightMetersPerSecond = CTREConvert
                .nativeUnitsToVelocityMetersPerSecond(m_rightLeader.getSelectedSensorVelocity());
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftLeader.setVoltage(leftVolts);
        m_rightLeader.setVoltage(rightVolts);
        m_drive.feed();
    }

}
