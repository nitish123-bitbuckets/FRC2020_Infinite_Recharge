package frc.robot.config;

import frc.robot.config.Config.ShooterConfig.BallManagementConfig;
import frc.robot.utils.control.pidf.PIDF;

public class Config {

    //////////////////////////////////////////////////////////////////////////////
    // Motor IDs

    // Shooter
    public int AZIMUTH_MOTOR_ID = 12;
    public int SHOOTER_MOTOR_ID = 13;
    public int FEEDER_MOTOR_ID = 8;
    public int INTAKE_MOTOR_ID = 14;

    // Drive
    public int LEFT_DRIVE_IDS[] = { 1, 4 };
    public int RIGHT_DRIVE_IDS[] = { 2, 3 };

    //////////////////////////////////////////////////////////////////////////////
    // Vision

    //////////////////////////////////////////////////////////////////////////////
    // Motor Configs
    public static class ShooterConfig {
        public float azimuthGearRatio = 28f / 130f;
        public float shooterGearRatio = .48f / 1f;
        public float defaultTurnVelocityDeg = 10;

        public MotorConfig azimuth = new MotorConfig();
        public MotorConfig feeder = new MotorConfig();
        public MotorConfig shooter = new MotorConfig();

        public static class BallManagementConfig {
            public MotorConfig spinner = new MotorConfig();

            public BallManagementConfig() {
                spinner.encoderType = MotorConfig.EncoderType.None;
            }
        }

        public ShooterConfig() {
            shooter.encoderType = MotorConfig.EncoderType.Integrated;
        }
    }

    public static class IntakeConfig {

        public MotorConfig intake = new MotorConfig();
    }

    public static class DriveConfig {
        public int MOTORS_PER_SIDE = 2;
        public MotorConfig leftMotors[];
        public MotorConfig rightMotors[];

        public int[] leftIDs;
        public int[] rightIDs;

        public MotorConfig leftLeader;
        public MotorConfig rightLeader;

        public boolean leftInverted = true;
        public boolean rightInverted = false;

        public MotorConfig.EncoderType encoderType = MotorConfig.EncoderType.Integrated;

        public DriveConfig() {
        }

        /**
         * Based on the MOTORS_PER_SIDE
         */
        public void initMotorConfigArrays() {
            leftMotors = new MotorConfig[MOTORS_PER_SIDE];
            rightMotors = new MotorConfig[MOTORS_PER_SIDE];
            for (int i = 0; i < MOTORS_PER_SIDE; i++) {
                leftMotors[i] = new MotorConfig();
                leftMotors[i].id = leftIDs[i];
                leftMotors[i].encoderType = encoderType;
                leftMotors[i].inverted = leftInverted;

                rightMotors[i] = new MotorConfig();
                rightMotors[i].id = rightIDs[i];
                rightMotors[i].encoderType = encoderType;
                rightMotors[i].inverted = rightInverted;

                if (i > 0) {
                    leftMotors[i].followingID = leftIDs[0];
                    rightMotors[i].followingID = rightIDs[0];
                }
            }
            leftLeader = leftMotors[0];
            rightLeader = rightMotors[0];
        }
    }

    public ShooterConfig shooter = new ShooterConfig();
    public BallManagementConfig ballManagement = new BallManagementConfig();
    public DriveConfig drive = new DriveConfig();
    public IntakeConfig intake = new IntakeConfig();

    public Config() {

        //////////////////////////////////////////////////////////////////////////////
        // IDs (Again)
        shooter.azimuth.id = AZIMUTH_MOTOR_ID;
        shooter.feeder.id = FEEDER_MOTOR_ID;
        shooter.shooter.id = SHOOTER_MOTOR_ID;
        intake.intake.id = INTAKE_MOTOR_ID;

        drive.leftIDs = LEFT_DRIVE_IDS;
        drive.rightIDs = RIGHT_DRIVE_IDS;

        //////////////////////////////////////////////////////////////////////////////
        // PIDFs

        // Shooter
        shooter.azimuth.positionPIDF = new PIDF(//
                0.05, // P
                0, // I
                0, // D
                0 /// F
        );
        shooter.shooter.velocityPIDF = new PIDF(//
                0.08, // P
                0, // I
                0, // D
                0 /// F
        );
        shooter.feeder.velocityPIDF = new PIDF(//
                0.1, // P
                0, // I
                0, // D
                0 /// F
        );

        // Drive
        drive.initMotorConfigArrays();

        drive.leftLeader.velocityPIDF = new PIDF(//
                0.1 * 1023 / 1000 / 4, // P
                0.0, // I
                0.1 * 1023 / 1000 / 4 * 10, // D
                1023 / 21740f /// F
        );
        drive.leftLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );
        drive.rightLeader.velocityPIDF = new PIDF(//
                0.1 * 1023 / 1000 / 4, // P
                0.0, // I
                0.1 * 1023 / 1000 / 4 * 10, // D
                1023 / 21340f /// F
        );
        drive.rightLeader.positionPIDF = new PIDF(//
                0, // P
                0, // I
                0, // D
                0 /// F
        );

        //////////////////////////////////////////////////////////////////////////////
        // Ticks Per Revolution
        shooter.azimuth.ticksPerRevolution = 8192;
    }

}