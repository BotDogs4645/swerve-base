// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.bd_util;

import java.util.HashMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class UTIL_CONSTANTS { 
        public static final class OPTIONS {
            public static final String[] RESOLUTIONS = {
                "1080P",
                "720P",
                "SXGA",
                "SVCD"
            };
            public static final String[] PTZ_TYPES = {
                "free_mode",
                "move_to_preset"
            };
            public static final String[] PRESET_POSITIONS = {
                "pos0",
                "pos1"
            };
        }

        public static final class CAMERA_DEFAULTS {
            public static final class NORMAL_DEFAULTS {
                public static final HashMap<String, Object[]> DEFAULT_INT_MAP;
                public static final HashMap<String, String[]> DEFAULT_STRING_MAP;
                public static final HashMap<String, Object[]> DEFAULT_BOOLEAN_MAP;
                
                public static final Object[] AVERAGE_JOE_SLIDER = new Object[] {50, 0, 100};

                static {
                    DEFAULT_INT_MAP = new HashMap<String, Object[]>();
                    DEFAULT_INT_MAP.put("brightness", AVERAGE_JOE_SLIDER);
                    DEFAULT_INT_MAP.put("saturation", AVERAGE_JOE_SLIDER);
                    DEFAULT_INT_MAP.put("hue", AVERAGE_JOE_SLIDER);
                    DEFAULT_INT_MAP.put("contrast", AVERAGE_JOE_SLIDER);
                    DEFAULT_INT_MAP.put("max_fps", new Object[] {30, 1, 30});
                    DEFAULT_INT_MAP.put("focus", new Object[] {0, 0, 1});

                    DEFAULT_STRING_MAP = new HashMap<String, String[]>();
                    DEFAULT_STRING_MAP.put("resolution", OPTIONS.RESOLUTIONS);
                    DEFAULT_STRING_MAP.put("ptz_type", OPTIONS.PTZ_TYPES);
                    DEFAULT_STRING_MAP.put("auto_position_preset", OPTIONS.PRESET_POSITIONS);

                    // default, on shuffle (true = on shuffle, no controller, false = on shuffle as view only but on controller)
                    DEFAULT_BOOLEAN_MAP = new HashMap<String, Object[]>();
                    DEFAULT_BOOLEAN_MAP.put("request_snap", new Object[] {false, false});
                    DEFAULT_BOOLEAN_MAP.put("request_reboot", new Object[] {false, true});
                    DEFAULT_BOOLEAN_MAP.put("record", new Object[] {false, true});
                }
            }

            public static final class SPECIAL_DEFAULTS {
                public static final int DEFAULT_MIN_BITRATE = 48;
                public static final int DEFAULT_MAX_BITRATE = 448;
                public static final int DEFAULT_X_AXIS_ROTATE = 0;
                public static final int DEFAULT_Y_AXIS_ROTATE = 0;
                public static final int DEFAULT_ZOOM = 0;
            }
        }
    }

    public static final class MISC {
        public static final String[] KEY_NAMES = {
            "null",
            "X",
            "A",
            "B",
            "Y",
            "LB",
            "RB",
            "back",
            "start"
        };
    }
}
