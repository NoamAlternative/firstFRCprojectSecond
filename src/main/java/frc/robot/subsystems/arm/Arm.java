package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;

public class Arm extends SubsystemBase {
    private final TalonFX motor = ArmConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ArmConstants.FOC_ENABLE);
    private final PIDController pidController = ArmConstants.PID_CONTROLLER;

    private final TrapezoidProfile profile = ArmConstants.PROFILE;
    private final TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State initialState = new TrapezoidProfile.State();
    private final TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private final Timer timer = new Timer();

    public Arm() {
    }

    void setTargetState(ArmConstants.ArmState targetState) {
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        setTargetVoltage(calculatePIDOutput(targetAngle));
    }

    void stop() {
        motor.stopMotor();
    }

    private void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    private double calculatePIDOutput(Rotation2d targetAngle) {
        return ArmConstants.PID_CONTROLLER.calculate(getCurrentAngle().getRotations(), targetAngle.getRotations());
    }

    private Rotation2d getCurrentAngle(){
        double rotations = ArmConstants.ANGLE_STATUS_SIGNAL.refresh().getValueAsDouble();
        return Rotation2d.fromRotations(rotations);
    }



    /*

    startMotionProfile -
    1. the startUpState - the current angle, the current velocety probobly 0
    2. the goalState - the target angle you wanna get to like 90 degrees

    followMotionProfile -
     1. set the State and
     2. then make it Rotation2d
     3. and the use followSetPoint to get to the point

     followSetPoint
*/

    public void startMotionProfile(Rotation2d targetRotations) {
        initialState = new TrapezoidProfile.State(getCurrentAngle(), 0);
        goalState    = new TrapezoidProfile.State(targetRotations, 0.0);

        profile = new TrapezoidProfile(constraints, goalState, initialState);

        timer.reset();
        timer.start();
    }






}