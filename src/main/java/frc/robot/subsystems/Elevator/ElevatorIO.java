package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    void updateInputs(ElevatorIOInputs inputs);

    void setElevatorVoltage(double volts);

    void setElevatorTarget(double meters,boolean isGoingUp);

    void resetElevatorPosition();

    default double getElevatorHeight() {
        return 0.0;
    }

    default double getElevatorVelocity() {
        return 0.0;
    }

    @AutoLog
    class ElevatorIOInputs{
        public double currentPositionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double setpointMeters = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double motorVoltage = 0.0;
        public double tempCelsius = 0.0;
        // Dynamic Motion Magic
        public boolean isGoingUp = false;
        public double currentAcceleration = 0.0;
        public double currentCruiseVelocity = 0.0;
        public double currentJerk = 0.0;
    }
}
