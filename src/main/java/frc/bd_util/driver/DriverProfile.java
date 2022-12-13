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

    public DriverProfile(String name, HashMap<String, String> cur_properties) {
        this.name = name;
        this.cur_properties = cur_properties;
    }

    public void changeName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public HashMap<String, String> getSettings() {
        return cur_properties;
    }

    public void setSettings(HashMap<String, String> settings) {
        this.cur_properties = settings;
    }

    public JSONObject encode() {
        return new JSONObject(cur_properties);
    }
}