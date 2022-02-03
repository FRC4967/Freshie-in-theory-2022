package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class DriveConstants {

    public static final double kP = 1.5;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int kRightMotor1Port = 0;
    public static final int kRightMotor2Port = 1;
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 3;

    public static final boolean kLeftMotorReversed = false;
    public static final boolean kRightMotorReversed = true;

    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = true;

    public static final boolean kGyroReversed = true;

    /** 
     * Counts per revolution for encoders
     * https://www.andymark.com/products/e4t-oem-miniature-optical-encoder-kit
    */
    public static final int kEncoderCPR = 360;

    /** 6" Wheel diameter, in meters */
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;

    /**
     * CIM Motor max RPM with free load
     * 
     * @see https://motors.vex.com
     */
    public static final double kMaxRPM = 5330;

    /**
     * Toughbox Mini 10.71:1
     * 
     * @see https://www.andymark.com/products/toughbox-mini-options
     */
    public static final double kDriveGearing = 10.71; // to 1

    /**
     * Distance between wheels, in meters
     */
    public static final double kTrackwidthMeters = 0.62;
    public static final int kTalonContinuousCurrentLimit = 40;

    public static final double ksVolts = 1.0644;
    public static final double kvVoltSecondsPerMeter = 1.1483;
    public static final double kaVoltSecondsSquaredPerMeter = 0.43839;

    /** Feedforward values from SysId tool */
    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

    /** P coefficient for velocity PID */
    public static final double kPDriveVel = 1.93;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    // Create a voltage constraint to ensure we don't accelerate too fast
    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            feedforward, kDriveKinematics, 10);

}
