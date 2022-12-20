package frc.bd_util.driver;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoyRumbler extends SubsystemBase {
    public enum RUMBLE_TYPE {
        SHAKER,
        PING
    }

    HashMap<RUMBLE_TYPE, ArrayList<BooleanSupplier>> shakers = new HashMap<RUMBLE_TYPE, ArrayList<BooleanSupplier>>();
    XboxController xbox;
    boolean analysis_mode = true;

    BooleanSupplier specific_supplier;

    public JoyRumbler(XboxController xbox) {
        this.xbox = xbox;
        for (RUMBLE_TYPE enu: RUMBLE_TYPE.values()) {
            shakers.put(enu, new ArrayList<BooleanSupplier>());
        }
    }

    public void addRumbleScenario(BooleanSupplier suppl, RUMBLE_TYPE rumble_type) {
        shakers.get(rumble_type).add(suppl);
    }

    public void removeRumbleScenario(RUMBLE_TYPE type, BooleanSupplier suppl) {
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
        if (analysis_mode) {
            for (BooleanSupplier bool: shakers.get(RUMBLE_TYPE.SHAKER)) {
                if (bool.getAsBoolean()) {
                    // somethings wrong
                    new ShakerRumble(this, xbox, bool).schedule();
                }
            }

            for (BooleanSupplier bool: shakers.get(RUMBLE_TYPE.PING)) {
                if (bool.getAsBoolean()) {
                    new PingRumble(this, xbox).schedule();
                }
            }
        } else {
            if (specific_supplier.getAsBoolean()) {
                xbox.setRumble(RumbleType.kLeftRumble, 1.0);
                xbox.setRumble(RumbleType.kRightRumble, 1.0);
            } else {
                xbox.setRumble(RumbleType.kLeftRumble, 0.0);
                xbox.setRumble(RumbleType.kRightRumble, 0.0);
            }
        }
    }
}
