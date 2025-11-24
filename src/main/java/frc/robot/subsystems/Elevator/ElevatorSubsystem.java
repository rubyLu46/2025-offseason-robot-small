package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import lombok.Getter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Elevator.ELEVATOR_ZEROING_FILTER_SIZE;
import static frc.robot.ElevatorCommonNT.*;

public class ElevatorSubsystem extends SubsystemBase {
    // @Getter
    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private LinearFilter currentFilter = LinearFilter.movingAverage(ELEVATOR_ZEROING_FILTER_SIZE);

    // @Getter
    @AutoLogOutput(key = "Elevator/setPoint")
    private double setPoint = 0.16;
    private double previousSetPoint = 0.16;

    // @Getter
    @AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;

    // @Getter
    @AutoLogOutput(key = "Elevator/isGoingUp")
    private boolean isGoingUp = false;

    // @Getter
    @AutoLogOutput(key = "Elevator/zeroing")
    public boolean setZero = false;

    @AutoLogOutput(key = "Elevator/currentFilterValue")
    public double currentFilterValue = 0.0;

    @AutoLogOutput(key = "Elevator/stopDueToLimit")
    private boolean stopDueToLimit = false;

    @AutoLogOutput(key = "Elevator/runningCharacterization")
    private boolean runningCharacterization = false;

    public ElevatorSubsystem(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;
    }

    @Override
    public void periodic(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator",inputs);
        
        SmartDashboard.putNumber("Elevator/currentPositionMeters",inputs.currentPositionMeters);
        SmartDashboard.putNumber("Elevator/setPoint",setPoint);
        SmartDashboard.putBoolean("Elevator/zeroing",setZero);

        if(setPoint>MAX_EXTENSION_METERS.getValue()){
            stopDueToLimit = true;
            System.out.println("Elevator setpoint " + setPoint + " exceeds maximum extension of " +
                    MAX_EXTENSION_METERS.getValue() + " meters");
        }
        else if(stopDueToLimit){
            stopDueToLimit = false;
        }

        if(!stopDueToLimit&&!setZero&&!runningCharacterization){
            atGoal=elevatorAtGoal(ELEVATOR_GOAL_TOLERANCE.getValue());
            if(setPoint != previousSetPoint){
                isGoingUp = setPoint > previousSetPoint;
                System.out.println("Elevator direction changed: " + (isGoingUp ? "UP" : "DOWN") +
                        " (from " + previousSetPoint + " to " + setPoint + ")");
                previousSetPoint = setPoint;
            }
            elevatorIO.setElevatorTarget(setPoint,isGoingUp);
        }
        else{
            atGoal = false;
        }

        // if(runningCharacterization){
        //     SignalLogger.writeDouble("elevator-motor-voltage", inputs.motorVoltage, "V");
        //     SignalLogger.writeDouble("elevator-position", inputs.currentPositionMeters, "m");
        //     SignalLogger.writeDouble("elevator-velocity", inputs.velocityMetersPerSec, "m/s");
        //     SignalLogger.writeDouble("elevator-applied-volts", inputs.appliedVolts, "V");
        //     SignalLogger.writeDouble("elevator-stator-current", inputs.statorCurrentAmps, "A");
        // }

        // LoggedTracer.record("Elevator");

        // SuperstructureVisualizer.getInstance().updateElevator(elevatorIO.getElevatorHeight());
    }

    public void setElevatorPosition(double position){
        setPoint=position;
    }

    public boolean elevatorAtGoal(double offset){
        return Math.abs(inputs.currentPositionMeters-setPoint)<offset;
    }

    public Command zeroElevator(){
        return Commands.startRun(
                () -> {
                    setZero = true;
                },
                () -> {
                    if(RobotBase.isReal()){
                        currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
                        if (currentFilterValue <= ELEVATOR_ZEROING_CURRENT.getValue()){
                            elevatorIO.setElevatorVoltage(-1);
                        }
                        if (currentFilterValue > ELEVATOR_ZEROING_CURRENT.getValue()){
                            elevatorIO.setElevatorVoltage(0);
                            elevatorIO.resetElevatorPosition();
                            setZero = false;
                        }
                    }
                    else{
                        elevatorIO.setElevatorTarget(0,false);
                        if (Math.abs(inputs.currentPositionMeters) < 0.01){
                            setZero = false;
                        }
                    }
                })
                .until(() -> !setZero)
                .finallyDo(() -> {
                    setZero = false;
        });
    }

}
