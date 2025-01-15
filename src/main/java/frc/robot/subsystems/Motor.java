package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
    private final TalonFX krakenMotor;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private final PositionTorqueCurrentFOC positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    private final NeutralOut brake = new NeutralOut();

    private boolean useTorqueControl = false; // Tracks the current control mode

    public Motor(int motorCANID) {
        krakenMotor = new TalonFX(motorCANID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure slot 0 for Voltage Control
        config.Slot0.kP = 2.0; // Example value, tune as needed
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Voltage.withPeakForwardVoltage(8).withPeakReverseVoltage(-8);

        // Configure slot 1 for Torque Control
        config.Slot1.kP = 60.0; // Example value, tune as needed
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 6.0;
        config.TorqueCurrent.withPeakForwardTorqueCurrent(120).withPeakReverseTorqueCurrent(-120);

        // Apply configurations with retries
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = krakenMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Motor configuration failed: " + status);
        }

        krakenMotor.setPosition(0); // Start at 0
    }

    public void toggleControlMode() {
        useTorqueControl = !useTorqueControl; // Toggle control mode
        System.out.println("Control Mode Toggled: " + (useTorqueControl ? "Torque" : "Voltage"));
    }

    public void moveToPosition(double rotations) {
        if (useTorqueControl) {
            System.out.println("Torque Control Activated: Target Position = " + rotations);
            krakenMotor.setControl(positionTorque.withPosition(rotations));
        } else {
            System.out.println("Voltage Control Activated: Target Position = " + rotations);
            krakenMotor.setControl(positionVoltage.withPosition(rotations));
        }
    }

    public void stopMotor() {
        System.out.println("Motor Stopped");
        krakenMotor.setControl(brake);
    }
}
