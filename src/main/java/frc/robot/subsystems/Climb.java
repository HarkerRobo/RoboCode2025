package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import harkerrobolib.motors.HSTalonFX;
import harkerrobolib.util.PIDConfig;

public class Climb extends SubsystemBase {
    private static Climb instance;

    private HSTalonFX master;

    private Climb() {
        master = new HSTalonFX(Constants.Climb.ID, HSTalonFX.makeDefaultConfig()
                .setInverted(true)
                .setBrakeMode()
                .setSensorToMechanismRatio(Constants.Climb.CLIMB_GEAR_RATIO)
                .setPIDConfig(0, new PIDConfig(Constants.Climb.kP, 0, 0))
                .setStatorCurrentLimit(Constants.Climb.STATOR_CURRENT_LIMIT)
                .setSupplyCurrentLimit(Constants.Climb.SUPPLY_CURRENT_LIMIT));
    }

    /**
     * @return rotations
     */
    public double getPosition() {
        return master.getSensorPosition();
    }

    public void moveToPosition(double desired) {
        master.setControl(new PositionVoltage(desired));
    }

    public void zeroClimb() {
        master.setSensorPosition(0);
    }

    public void setVelocity(double velocity) {
        master.setControl(new DutyCycleOut(velocity));
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }
}