package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DrivetrainConstants {

    public static final ClosedLoopConfig DrivePID = new ClosedLoopConfig()
        .pidf(0, 0, 0, 1.0/473);
    
    public static final PIDController SteerPID = new PIDController(0, 0, 0);

    public static final double DriveGearRatio = 6.12;
    public static final double SteerGearRatio = 150.0/7;

    public static final Distance WheelDistance = Inches.of(16.843424);
    public static final Distance WheelCirc = Inches.of(4).times(Math.PI);

    public static final LinearVelocity MaxVelocity = MetersPerSecond.of(RPM.of(5676).in(RotationsPerSecond)/DriveGearRatio*WheelDistance.in(Meters));
    public static final AngularVelocity MaxOmega = RadiansPerSecond.of(0.2/WheelDistance.in(Meters));

    public static final double DrivePositionConvertionFactor = 1/DriveGearRatio*WheelCirc.in(Meters);
    public static final double DriveVelocityConvertionFactor = DrivePositionConvertionFactor/60;
    public static final double SteerPositionConvertionFactor = 1/SteerGearRatio;
    public static final double SteerVelocityConvertionFactor = SteerPositionConvertionFactor/60;

    public static final int PDHID = 60;
    public class Modules {
        public class FrontLeft {
            public static final int DriveID = 11;
            public static final int SteerID = 12;
            public static final int CANCoderID = 1;
            public static final Angle CANCoderOffset = Degrees.of(0);
            public static final Translation2d place = new Translation2d(WheelDistance.times(-1), WheelDistance);
        }
        

        public class FrontRight {
            public static final int DriveID = 21;
            public static final int SteerID = 22;
            public static final int CANCoderID = 2;
            public static final Angle CANCoderOffset = Degrees.of(0);
            public static final Translation2d place = new Translation2d(WheelDistance, WheelDistance);
        }

        public class BackLeft {
            public static final int DriveID = 31;
            public static final int SteerID = 32;
            public static final int CANCoderID = 3;
            public static final Angle CANCoderOffset = Degrees.of(0);
            public static final Translation2d place = new Translation2d(WheelDistance.times(-1), WheelDistance.times(-1));
        }

        public class BackRight {
            public static final int DriveID = 41;
            public static final int SteerID = 42;
            public static final int CANCoderID = 4;
            public static final Angle CANCoderOffset = Degrees.of(0);
            public static final Translation2d place = new Translation2d(WheelDistance, WheelDistance.times(-1));
        }

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FrontLeft.place, FrontRight.place, BackLeft.place, BackRight.place);
    }

}