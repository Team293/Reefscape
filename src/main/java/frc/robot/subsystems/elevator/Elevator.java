package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static final double MAX_POSITION = 2.87;
    private static final double MIN_POSITION = 0.0;
    private static final double DELTA_POSITION_DEADBAND = 0.005;
    
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ElevatorIOTalonFX elevatorMotor;
    private final PositionVoltage command;
    
    private double encoderOffset = 0.0d; 

    // Encoder Offset Calculation Variables
    private boolean isCalculatingOffset = false; 
    private double lastPositionReading = 0.0d;

    public Elevator() {
        elevatorMotor = new ElevatorIOTalonFX(0); // TODO: change to actual CAN ID
        command = new PositionVoltage(0).withSlot(0);
    }
    
    @Override
    public void periodic() {
        elevatorMotor.updateInputs(inputs);

        if (isCalculatingOffset = true) {
            // calculate the offsets of the encoders
            double positionDelta = lastPositionReading - inputs.positionValue;
            if (MathUtil.applyDeadband(positionDelta, DELTA_POSITION_DEADBAND) == 0) { // check to see that elevator stopped moving
                encoderOffset = inputs.positionValue;
                elevatorMotor.setBrakeMode(true);
                isCalculatingOffset = false;
            }
        }
    }

    public void calculateOffset() {
        isCalculatingOffset = true;
        elevatorMotor.setBrakeMode(false); // switch to coast mode to make elevetor drop down to zero position
    }

    public void setPosition(double percentValue) {
        if (!isCalculatingOffset) { 
            // joystick is reversed
            double newPosition = MathUtil.clamp(-percentValue * MAX_POSITION, MIN_POSITION, MAX_POSITION) + encoderOffset;

            // Clamp the position at the min and max values, then add the encoder offset 
            elevatorMotor.applyPosition(command.withPosition(newPosition)); 
        }
    }
}
