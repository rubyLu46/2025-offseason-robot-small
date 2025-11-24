package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.ElevatorCommonNT.ElevatorGainsClass;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.ElevatorCommonNT.*;
import static frc.robot.Ports.ELEVATOR_FOLLOWER;
import static frc.robot.Ports.ELEVATOR_MAIN;

public class ElevatorIOReal implements ElevatorIO{
    private final TalonFX leader = new TalonFX(ELEVATOR_MAIN.id,ELEVATOR_MAIN.bus);
    private final TalonFX follower = new TalonFX(ELEVATOR_FOLLOWER.id,ELEVATOR_FOLLOWER.bus);

    private final TalonFXConfigurator leaderConfigurator;
    private final TalonFXConfigurator followerConfigurator;

    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs;

    private final DynamicMotionMagicVoltage motionRequest = new DynamicMotionMagicVoltage(0.0, 100, 300, 0).withEnableFOC(true);

    private final StatusSignal<AngularVelocity> velocityLeft;
    private final StatusSignal<Angle> positionLeft;
    private final StatusSignal<Voltage> voltageLeft;
    private final StatusSignal<Current> statorLeft;
    private final StatusSignal<Current> supplyLeft;
    private final StatusSignal<Temperature> tempLeft;
    private double setpointMeters = 0;
    private boolean isGoingUp = false;

    public ElevatorIOReal() {
        this.leaderConfigurator = leader.getConfigurator();
        this.followerConfigurator = follower.getConfigurator();

        configureCurrentLimits();
        configureMotorOutputs();
        initializePositions();

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = motionAccelerationUp.getValue();
        motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocityUp.getValue();
        motionMagicConfigs.MotionMagicJerk = motionJerkUp.getValue();

        motionRequest.Velocity = motionCruiseVelocityUp.getValue();
        motionRequest.Acceleration = motionAccelerationUp.getValue();
        motionRequest.Jerk = motionJerkUp.getValue();

        slot0Configs = new Slot0Configs();
        slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.getValue();
        slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.getValue();
        slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.getValue();
        slot0Configs.kG = ElevatorGainsClass.ELEVATOR_KG.getValue();
        slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.getValue();
        slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.getValue();
        slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.getValue();

        leaderConfigurator.apply(slot0Configs);
        leaderConfigurator.apply(motionMagicConfigs);
        followerConfigurator.apply(slot0Configs);
        followerConfigurator.apply(motionMagicConfigs);

        leader.clearStickyFaults();
        follower.clearStickyFaults();

        velocityLeft = leader.getVelocity();
        positionLeft = leader.getPosition();
        voltageLeft = leader.getSupplyVoltage();
        statorLeft = leader.getStatorCurrent();
        supplyLeft = leader.getSupplyCurrent();
        tempLeft = leader.getDeviceTemp();

        follower.setControl(new Follower(leader.getDeviceID(), true));
    }

    private void configureCurrentLimits(){
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        leaderConfigurator.apply(currentLimitsConfigs);
        followerConfigurator.apply(currentLimitsConfigs);
    }

    private void configureMotorOutputs() {
        MotorOutputConfigs leaderMotorConfigs = new MotorOutputConfigs();
        leaderMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        leaderMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfigurator.apply(leaderMotorConfigs);

        MotorOutputConfigs followerMotorConfigs = new MotorOutputConfigs();
        followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        followerConfigurator.apply(followerMotorConfigs);
    }

    public void initializePositions(){
        leader.setPosition(heightToMotorRot(ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED));
        follower.setPosition(heightToMotorRot(ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityLeft,
                positionLeft,
                voltageLeft,
                statorLeft,
                supplyLeft,
                tempLeft
        );

        inputs.currentPositionMeters = motorRotToHeight(leader.getPosition().getValueAsDouble());
        inputs.setpointMeters = setpointMeters;
        inputs.velocityMetersPerSec = getElevatorVelocity();
        inputs.appliedVolts = voltageLeft.getValueAsDouble();
        inputs.statorCurrentAmps = statorLeft.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyLeft.getValueAsDouble();
        inputs.tempCelsius = tempLeft.getValueAsDouble();
        inputs.motorVoltage = leader.getMotorVoltage().getValueAsDouble();
        // Dynamic Motion Magic
        inputs.isGoingUp = isGoingUp;
        inputs.currentAcceleration = isGoingUp ? motionAccelerationUp.getValue() : motionAccelerationDown.getValue();
        inputs.currentCruiseVelocity = isGoingUp ? motionCruiseVelocityUp.getValue() : motionCruiseVelocityDown.getValue();
        inputs.currentJerk = isGoingUp ? motionJerkUp.getValue() : motionJerkDown.getValue();
        
        //update configs if needed
        if (ElevatorGainsClass.isAnyChanged()) {
            slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.getValue();
            slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.getValue();
            slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.getValue();
            slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.getValue();
            slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.getValue();
            slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.getValue();
            slot0Configs.kG = ElevatorGainsClass.ELEVATOR_KG.getValue();

            leaderConfigurator.apply(slot0Configs);
            followerConfigurator.apply(slot0Configs);

            // Update Dynamic Motion Magic
            if (isGoingUp) {
                motionRequest.Velocity = motionCruiseVelocityUp.getValue();
                motionRequest.Acceleration = motionAccelerationUp.getValue();
                motionRequest.Jerk = motionJerkUp.getValue();
            } else {
                motionRequest.Velocity = motionCruiseVelocityDown.getValue();
                motionRequest.Acceleration = motionAccelerationDown.getValue();
                motionRequest.Jerk = motionJerkDown.getValue();
            }
        }
    }

    @Override
    public void setElevatorVoltage(double volts) {
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElevatorTarget(double meters, boolean goingUp) {
        setpointMeters = meters;
        isGoingUp = goingUp;
        double targetPosition = heightToMotorRot(Math.min(meters, MAX_EXTENSION_METERS.getValue()));

        if (isGoingUp){
            motionRequest.Velocity = motionCruiseVelocityUp.getValue();
            motionRequest.Acceleration = motionAccelerationUp.getValue();
            motionRequest.Jerk = motionJerkUp.getValue();
        }
        else{
            motionRequest.Velocity = motionCruiseVelocityDown.getValue();
            motionRequest.Acceleration = motionAccelerationDown.getValue();
            motionRequest.Jerk = motionJerkDown.getValue();
        }

        leader.setControl(motionRequest.withPosition(targetPosition));
    }

    @Override
    public void resetElevatorPosition() {
        leader.setPosition(0.0);
        follower.setPosition(0.0);
    }

    @Override
    public double getElevatorVelocity(){
        return motorRotToHeight(leader.getVelocity().getValueAsDouble());
    }

    @Override
    public double getElevatorHeight(){
        return motorRotToHeight(leader.getPosition().getValueAsDouble());
    }

    public double heightToMotorRot(double heightMeters){
        return (heightMeters/(Math.PI*ELEVATOR_SPOOL_DIAMETER))*ELEVATOR_GEAR_RATIO;
    }

    public double motorRotToHeight(double rotations){
        return rotations*(Math.PI*ELEVATOR_SPOOL_DIAMETER)/ELEVATOR_GEAR_RATIO;
    }
}
