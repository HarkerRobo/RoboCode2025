package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private static Climb instance;

    private TalonFX master;

    private Climb() {
        master = new TalonFX(Constants.Climb.ID, Constants.CAN_CHAIN);

        master.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = Constants.Climb.INVERTED;

        masterConfig.Feedback.SensorToMechanismRatio = Constants.Climb.CLIMB_GEAR_RATIO;
        
        masterConfig.CurrentLimits.StatorCurrentLimit = Constants.Climb.STATOR_CURRENT_LIMIT;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        masterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climb.SUPPLY_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(masterConfig);
    }

    public void setPower(double velocity) {
        master.setControl(new DutyCycleOut(velocity));
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }
}