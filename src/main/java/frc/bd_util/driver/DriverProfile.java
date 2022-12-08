package frc.bd_util.driver;

import java.util.HashMap;

import org.json.simple.JSONObject;

public class DriverProfile {
    public String name;
    public HashMap<String, String> cur_properties = new HashMap<String, String>();

    public DriverProfile(String name) {
        this.name = name;
        for (String i: DriverProfileManager.info_list.keySet()) {
            cur_properties.put(i, DriverProfileManager.info_list.get(i));
        }
        
    }

    public void changeName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public JSONObject encode() {
        return new JSONObject(cur_properties);
    }
}