package frc.robot.subsystems.elevator;

import java.util.logging.Logger;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static final double MAX_POSITION = 184.22; //meters
    private static final double MIN_POSITION = 0;
    private static final double MAX_SPEED = 10d;
    private static final double POSITION_ERROR_THRESHOLD = 1;

    //Encoder units -- check pheonix tuner if belt tension changes for the positions again
    private static final double L1_POSITION = 0.0d; //0
    private static final double L2_POSITION = 20.85d; //20.85
    private static final double L3_POSITION = 92.07d; //92.07
    private static final double L4_POSITION = 181.22d; //184.22
    private static final double CORAL_STATION_POS = 0.0d;

    private static final double heights[] = {L1_POSITION, L2_POSITION, L3_POSITION, L4_POSITION, CORAL_STATION_POS};

    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final ElevatorIOTalonFX elevatorMotor;
    private final PositionVoltage command;

    private double targetPosition;
    private boolean isZeroing;

    public Elevator() {
        elevatorMotor = new ElevatorIOTalonFX(2);
        command = new PositionVoltage(0).withSlot(0);
        elevatorMotor.setPosition(0);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ElevatorIsAtTarget", isAtTarget());
        elevatorMotor.updateInputs(elevatorInputs);

        if (DriverStation.isDisabled()) {
            setPosition(elevatorInputs.positionValue);
        }

        if (isZeroing) {
            if (elevatorMotor.isAtZero()) {
                elevatorMotor.setPosition(0);
                isZeroing = false;
            } 
        }

        if (elevatorMotor.isAtZero()) {
            // calculate the offsets of the encoders
            elevatorMotor.setPosition(0);
        }
    }

    public boolean isAtTarget() {
        double diff = getPosition() - targetPosition;
        return diff < POSITION_ERROR_THRESHOLD && diff >= -1;
    }

    public boolean getIsZeroing() {
        return isZeroing;
    }


    public void setPercentage(double percentValue) {
        if (isZeroing) return;
        // joystick is reversed
        targetPosition = MathUtil.clamp(-percentValue * MAX_POSITION, MIN_POSITION, MAX_POSITION);

        // Clamp the position at the min and max values, then add the encoder offset 
        elevatorMotor.applyPosition(command.withPosition(targetPosition)); 
    }

    public void setPosition(double position) {
        if (isZeroing) return;

        targetPosition = MathUtil.clamp(position, MIN_POSITION, MAX_POSITION);

        elevatorMotor.applyPosition(command.withPosition(position)); 
    }

    public void setPresetPos(int pos) {
        if (isZeroing) return;

        targetPosition = heights[pos];
        if (pos < 0 || pos >= heights.length) {
            DriverStation.reportError("Invalid preset position", false);
        } else {
            elevatorMotor.applyPosition(command.withPosition(targetPosition));
        }
    }

    public void changePosition(double percentSpeed) {
        if (isZeroing) return;

        // joystick is reversed
        targetPosition = MathUtil.clamp(targetPosition + percentSpeed * MAX_SPEED,  MIN_POSITION, MAX_POSITION);

        // Clamp the position at the min and max values, then add the encoder offset
        setPosition(targetPosition);
    }

    public double getPosition() {
        return this.elevatorInputs.positionValue;
    }

    public void zero() {
        this.isZeroing = true;

        targetPosition = 0.0;
        this.elevatorMotor.runVelocity(-1);
    }

    @AutoLogOutput(key = "Elevator/IsZeroing")
    public boolean isZeroing() {
        return this.isZeroing;
    }

    public static double getTargetHeight(int level) {
        return heights[level];
    }
}
