package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static final double MAX_POSITION = 2.7;
    private static final double MIN_POSITION = 0.0;
    private static final double DELTA_POSITION_DEADBAND = 0.001;
    private static final double MAX_SPEED = .5;

    private static final double L1_POSITION = 0.0d;
    private static final double L2_POSITION = 0.0d;
    private static final double L3_POSITION = 2.7d;
    private static final double L4_POSITION = 0.0d;

    private static final double heights[] = {L1_POSITION, L2_POSITION, L3_POSITION, L4_POSITION}

    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ElevatorIOTalonFX elevatorMotor;
    private final PositionVoltage command;
    
    // Encoder Offset Calculation Variables
    private boolean isCalculatingOffset = false; 
    private double lastPositionReading = 0.0d;
    private double targetPosition = 0.0;

    public Elevator() {
        elevatorMotor = new ElevatorIOTalonFX(2);
        command = new PositionVoltage(0).withSlot(0);
        elevatorMotor.setPosition(0);
        isCalculatingOffset = false; 
    }
    
    @Override
    public void periodic() {
        elevatorMotor.updateInputs(inputs);

        Logger.recordOutput("Elevator/Calibrating", isCalculatingOffset);
        Logger.recordOutput("Elevator/Position", inputs.positionValue);
        Logger.recordOutput("Elevator/TargetPosition", targetPosition);

        if (isCalculatingOffset == true) {
            // calculate the offsets of the encoders
            double positionDelta = lastPositionReading - inputs.positionValue;
            lastPositionReading = inputs.positionValue;
            elevatorMotor.setZeroVoltage();
            if (Math.abs(positionDelta) < DELTA_POSITION_DEADBAND) { // check to see that elevator stopped moving
                elevatorMotor.setPosition(0);
                targetPosition = 0;
                elevatorMotor.setBrakeMode(true);
                isCalculatingOffset = false;
            }
        }
        lastPositionReading = inputs.positionValue;
    }

    public void calculateOffset() {
        isCalculatingOffset = true;
        elevatorMotor.setBrakeMode(false); // switch to coast mode to make elevetor drop down to zero position
        elevatorMotor.setZeroVoltage();
        lastPositionReading = inputs.positionValue + DELTA_POSITION_DEADBAND * 2;
    }

    public void setPosition(double percentValue) {
        if (!isCalculatingOffset) { 
            // joystick is reversed
            targetPosition = MathUtil.clamp(-percentValue * MAX_POSITION, MIN_POSITION, MAX_POSITION);

            // Clamp the position at the min and max values, then add the encoder offset 
            elevatorMotor.applyPosition(command.withPosition(targetPosition)); 
        }
    }

    public void setPresetPos(double pos) {
        if (pos < 0 || pos > heights.length) {
            DriverStation.reportError("Invalid preset position", false);
        } else {
            elevatorMotor.applyPosition(command.withPosition(heights[pos]));
        }
    }

    public void changePosition(double percentSpeed) {
        if (!isCalculatingOffset) { 
            // joystick is reversed
            targetPosition = MathUtil.clamp(targetPosition + percentSpeed * MAX_SPEED,  MIN_POSITION, MAX_POSITION);

            // Clamp the position at the min and max values, then add the encoder offset 
            elevatorMotor.applyPosition(command.withPosition(targetPosition)); 
        }
    }
}
