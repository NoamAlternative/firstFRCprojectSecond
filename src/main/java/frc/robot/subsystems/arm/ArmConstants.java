package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;

import java.util.Timer;

public class ArmConstants {
    static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS =
            new TrapezoidProfile.Constraints(10, 20);

    static final TrapezoidProfile PROFILE = new TrapezoidProfile(PROFILE_CONSTRAINTS);

    static final PIDController PID_CONTROLLER = new PIDController(1, 0, 0);
    static final ArmFeedforward FEED_FORWARD = new ArmFeedforward(1, 1, 1);

    static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);

    private static final int
            MOTOR_ID = 1,
            ENCODER_ID = 1;
    static final TalonFX MOTOR = new TalonFX(MOTOR_ID);
    static final CANcoder ENCODER = new CANcoder(ENCODER_ID);

    static final StatusSignal<Angle> ANGLE_STATUS_SIGNAL = ENCODER.getPosition();
    static final boolean FOC_ENABLE = true;

    static {
        configMotor();
        configEncoder();
    }

    private static void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = 1.5;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        MOTOR.getConfigurator().apply(config);
        MOTOR.optimizeBusUtilization();
    }

    private static void configEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.MagnetOffset = 0;
        ENCODER.getConfigurator().apply(config);
        ANGLE_STATUS_SIGNAL.setUpdateFrequency(100);
        ENCODER.optimizeBusUtilization();
    }

    public enum ArmState {
        HIGH_STATE(Rotation2d.fromDegrees(167)),
        LOW_STATE(Rotation2d.fromDegrees(67)),
        REST(Rotation2d.fromDegrees(0));

        public final Rotation2d targetAngle;

        ArmState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }
    }
}