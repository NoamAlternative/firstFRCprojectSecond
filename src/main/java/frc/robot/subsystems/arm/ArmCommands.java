package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;

public class ArmCommands {
    public static Command getSetTargetAngleCommand(ArmConstants.ArmState state) {
        return new FunctionalCommand(
                () -> RobotContainer.ARM.initializeMotionProfile(state.targetAngle),
                RobotContainer.ARM::followMotionProfile,
                (Interrupted) -> RobotContainer.ARM.stop(),
                () -> false,
                RobotContainer.ARM
        );
    }
}