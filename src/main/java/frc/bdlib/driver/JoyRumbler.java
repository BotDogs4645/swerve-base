package frc.bdlib.driver;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoyRumbler extends SubsystemBase {
    public enum RumblerType {
        LEFT_SHAKER,
        RIGHT_SHAKER
    }

    HashMap<RumblerType, ArrayList<BooleanSupplier>> shakers = new HashMap<RumblerType, ArrayList<BooleanSupplier>>();
    ControllerAIO xbox;
    boolean analysis_mode = true;

    BooleanSupplier specific_supplier;
    ToggleBooleanSupplier muter;

    public JoyRumbler(ControllerAIO xbox, ToggleBooleanSupplier muter) {
        this.xbox = xbox;
        this.muter = muter;
        for (RumblerType enu: RumblerType.values()) {
            shakers.put(enu, new ArrayList<BooleanSupplier>());
        }
        xbox.setRumble(RumbleType.kBothRumble, 1.0);
    }

    public void addRumbleScenario(BooleanSupplier suppl, RumblerType rumble_type) {
        shakers.get(rumble_type).add(suppl);
    }

    public void removeRumbleScenario(RumblerType type, BooleanSupplier suppl) {
        if (shakers.get(type).contains(suppl)) {
            shakers.get(type).remove(suppl);
        }
    }

    public void setSpecificRumble(BooleanSupplier suppl) {
        analysis_mode = false;
        if (this.getCurrentCommand() != null) {
            this.getCurrentCommand().cancel();
        }
        specific_supplier = suppl;
    }

    public void disableSpecificRumble() {
        analysis_mode = true;
        xbox.setRumble(RumbleType.kLeftRumble, 0.0);
        xbox.setRumble(RumbleType.kRightRumble, 0.0);
    }
    
    @Override
    public void periodic() {
        if (muter.getValue()) {
            if (analysis_mode) {
                boolean leave = false;
                for (BooleanSupplier bool: shakers.get(RumblerType.LEFT_SHAKER)) {
                    if (bool.getAsBoolean()) {
                        System.out.println("enabled!");
                        xbox.setRumble(RumbleType.kLeftRumble, 1);
                        leave = true;
                    }
                }
                if (!leave) {
                    xbox.setRumble(RumbleType.kLeftRumble, 0);
                }

                boolean leave2 = false;
                for (BooleanSupplier bool: shakers.get(RumblerType.RIGHT_SHAKER)) {
                    if (bool.getAsBoolean()) {
                        xbox.setRumble(RumbleType.kRightRumble, 1);
                        leave2 = true;
                    }
                }
                if (!leave2) {
                    xbox.setRumble(RumbleType.kRightRumble, 0);
                }
            } else {
                if (specific_supplier.getAsBoolean()) {
                    xbox.setRumble(RumbleType.kLeftRumble, 1);
                    xbox.setRumble(RumbleType.kRightRumble, 1);
                } else {
                    xbox.setRumble(RumbleType.kLeftRumble, 0);
                    xbox.setRumble(RumbleType.kRightRumble, 0);
                }
            }
        } else {
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }
    }
}
