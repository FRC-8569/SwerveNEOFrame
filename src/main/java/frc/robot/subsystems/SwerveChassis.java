package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DrivetrainConstants.Modules;

public class SwerveChassis extends SubsystemBase {
    public SwerveMod FrontLeft, FrontRight, BackLeft, BackRight;
    public AHRS gyro;
    public SwerveDriveOdometry odometry;
    public StructArrayPublisher<SwerveModuleState> CurrentState, IdealState;
    public StructPublisher<ChassisSpeeds> SpeedPublisher;

    public SwerveChassis() {
        FrontLeft = new SwerveMod(
                Modules.FrontLeft.DriveID,
                Modules.FrontLeft.SteerID,
                Modules.FrontLeft.CANCoderID,
                Modules.FrontLeft.CANCoderOffset);

        FrontRight = new SwerveMod(
                Modules.FrontRight.DriveID,
                Modules.FrontRight.SteerID,
                Modules.FrontRight.CANCoderID,
                Modules.FrontRight.CANCoderOffset);

        BackLeft = new SwerveMod(
                Modules.BackLeft.DriveID,
                Modules.BackLeft.SteerID,
                Modules.BackLeft.CANCoderID,
                Modules.BackLeft.CANCoderOffset);

        BackRight = new SwerveMod(
                Modules.BackRight.DriveID,
                Modules.BackRight.SteerID,
                Modules.BackRight.CANCoderID,
                Modules.BackRight.CANCoderOffset);

        gyro = new AHRS(NavXComType.kMXP_SPI);
        odometry = new SwerveDriveOdometry(Modules.kinematics, gyro.getRotation2d(), getPositions());
        CurrentState = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Drivetrain/CurrentState", SwerveModuleState.struct).publish();
        IdealState = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Drivetrain/IdealState", SwerveModuleState.struct).publish();
        SpeedPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Drivetrain/ChassisSpeeds", ChassisSpeeds.struct).publish();
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                FrontLeft.getPosition(),
                FrontRight.getPosition(),
                BackLeft.getPosition(),
                BackRight.getPosition()
        };
    }

    public SwerveModuleState[] getState() {
        return new SwerveModuleState[] {
                FrontLeft.getState(),
                FrontRight.getState(),
                BackLeft.getState(),
                BackRight.getState()
        };
    }

    public void drive(ChassisSpeeds speeds) {
        setStates(Modules.kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02)));
        SpeedPublisher.accept(speeds);
        IdealState.accept(Modules.kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02)));
    }

    public void setStates(SwerveModuleState... states) {
        FrontLeft.setState(states[0]);
        FrontRight.setState(states[1]);
        BackLeft.setState(states[2]);
        BackRight.setState(states[3]);
    }

    @Override
    public void periodic() {
        CurrentState.accept(getState());
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
    }

    public void autoInitial() {
        try {
            AutoBuilder.configure(
                    () -> odometry.getPoseMeters(),
                    this::resetPose,
                    () -> Modules.kinematics.toChassisSpeeds(getState()),
                    (speeds, ff) -> drive(speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(0, 0, 0),
                            new PIDConstants(0, 0, 0)),
                    RobotConfig.fromGUISettings(),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner Config mdfk", e.getStackTrace());
        }
    }
}
