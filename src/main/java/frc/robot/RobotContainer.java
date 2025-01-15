package frc.robot;

import frc.robot.subsystems.Motor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Motor motorSubsystem = new Motor(Constants.KRACKEN_MOTOR_CAN_ID);
    private final XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Toggle between Voltage and Torque control (LT)
        new Trigger(() -> xboxController.getLeftTriggerAxis() > 0.1)
            .onTrue(new RunCommand(motorSubsystem::toggleControlMode, motorSubsystem));

        // Move to Position 1 (0 rotations) while holding A
        new Trigger(() -> xboxController.getAButton())
            .whileTrue(new RunCommand(() -> motorSubsystem.moveToPosition(0), motorSubsystem));

        // Move to Position 2 (0.375 rotations) while holding Y
        new Trigger(() -> xboxController.getYButton())
            .whileTrue(new RunCommand(() -> motorSubsystem.moveToPosition(0.375), motorSubsystem));

        // Stop motor when Right Trigger is pressed
        new Trigger(() -> xboxController.getRightTriggerAxis() > 0.1)
            .onTrue(new RunCommand(motorSubsystem::stopMotor, motorSubsystem));

        // Joystick control for dynamic position
        new Trigger(() -> Math.abs(xboxController.getLeftY()) > 0.1)
            .whileTrue(new RunCommand(() -> {
                double joystickRotations = xboxController.getLeftY() * 10; // Scale joystick input
                motorSubsystem.moveToPosition(joystickRotations);
            }, motorSubsystem));
    }

    public Command getTeleopCommand() {
        return new RunCommand(() -> {
            double joystickRotations = xboxController.getLeftY() * 10; // Scale joystick input
            if (Math.abs(joystickRotations) > 0.1) {
                motorSubsystem.moveToPosition(joystickRotations);
            }
        }, motorSubsystem);
    }

    public Command getAutonomousCommand() {
        return new RunCommand(() -> motorSubsystem.moveToPosition(0.5), motorSubsystem)
            .andThen(() -> motorSubsystem.stopMotor())
            .withTimeout(Constants.AUTO_DURATION);
    }
}
