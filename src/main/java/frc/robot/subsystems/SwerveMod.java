package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.DrivetrainConstants;

public class SwerveMod {
    public SparkMax DriveMotor, SteerMotor;
    public SparkClosedLoopController DrivePID;
    public PIDController SteerPID;
    public RelativeEncoder DriveEncoder, SteerEncoder;
    private SparkMaxConfig DriveConfig, SteerConfig;
    public CANcoder cancoder;
    private CANcoderConfiguration encoderConfig;

    public SwerveMod(int DriveID, int SteerID, int CANCoderID, Angle CANCoderOffset){
        DriveMotor = new SparkMax(DriveID, MotorType.kBrushless);
        SteerMotor = new SparkMax(SteerID, MotorType.kBrushless);
        cancoder = new CANcoder(CANCoderID);

        DriveConfig = new SparkMaxConfig();
        SteerConfig = new SparkMaxConfig();
        encoderConfig = new CANcoderConfiguration();

        DriveConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        DriveConfig.encoder
            .positionConversionFactor(DrivetrainConstants.DrivePositionConvertionFactor)
            .velocityConversionFactor(DrivetrainConstants.DriveVelocityConvertionFactor);
        
        SteerConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        SteerConfig.encoder
            .positionConversionFactor(DrivetrainConstants.SteerPositionConvertionFactor)
            .velocityConversionFactor(DrivetrainConstants.SteerVelocityConvertionFactor);

        encoderConfig.MagnetSensor
            .withMagnetOffset(CANCoderOffset)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        DriveConfig.apply(DrivetrainConstants.DrivePID);

        DriveMotor.configure(DriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SteerMotor.configure(SteerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        DrivePID = DriveMotor.getClosedLoopController();
        SteerPID = DrivetrainConstants.SteerPID;

        DriveEncoder = DriveMotor.getEncoder();
        SteerEncoder = SteerMotor.getEncoder();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            MetersPerSecond.of(DriveEncoder.getVelocity()),
            Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Degrees))
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Meters.of(DriveEncoder.getPosition()),
            Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Degrees))
        );
    }

    public void setState(SwerveModuleState state){
        state.optimize(getState().angle);

        DrivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        SteerMotor.set(SteerPID.calculate(getState().angle.getDegrees(), state.angle.getDegrees()));
    }
}
