// Port of CommonParameters.h
package org.firstinspires.ftc.teamcode.auto.vision;

// Parameters extracted from XML files.
public class VisionParameters {

    // In c++ these are structs but here we'll make all of the fields final.
    // Use public static nested classes for "packaging convenience".
    // See https://stackoverflow.com/questions/253492/static-nested-class-in-java-why
    // Create by Outer.Nested instance = new Outer.Nested();

    // From the image_parameters element of any XML file.
    public static class ImageParameters {
        public final String file_name;
        public final int resolution_width;
        public final int resolution_height;
        public final FTCRect image_roi;

        public ImageParameters(String pFileName, int pWidth, int pHeight, FTCRect pImageROI) {
            file_name = pFileName;
            resolution_width = pWidth;
            resolution_height = pHeight;
            image_roi = pImageROI;
        }
    }

    // Sorry about this but we need a container for an optional image roi -
    // in RobotAction.xml. If a project does not need OpenCV then if we use
    // OpenCv's Rect for this container we have to pull in all of OpenCv
    // to avoid compilation errors. This simple repackaging avoids this.
    public static class FTCRect {
        public final int x;
        public final int y;
        public final int width;
        public final int height;

        // Order of parameters follows OpenCV Rect.
        public FTCRect(int pX, int pY, int pWidth, int pHeight) {
            x = pX;
            y = pY;
            width = pWidth;
            height = pHeight;
        }
    }

    // From the gray_parameters element of any XML file.
    public static class GrayParameters {
        public final int target; // normalization target
        public final int low_threshold; // for binary thresholding

        public GrayParameters(int pTarget, int pLowThreshold) {
            target = pTarget;
            low_threshold = pLowThreshold;
        }
    }

    // From the hsv_parameters element of any XML file.
    public static class HSVParameters {
        public final String hue_name;
        public final int hue_low;
        public final int hue_high;
        public final int saturation_target; // normalization target
        public final int saturation_low_threshold; // for inRange thresholding
        public final int value_target; // normalization target
        public final int value_low_threshold; // for inRange thresholding

        public HSVParameters(String pHueName, int pHueLow, int pHueHigh,
                             int pSaturationTarget, int pSaturationLowThreshhold,
                             int pValueTarget, int pValueLowThreshhold) {
            hue_name = pHueName;
            hue_low = pHueLow;
            hue_high = pHueHigh;
            saturation_target = pSaturationTarget;
            saturation_low_threshold = pSaturationLowThreshhold;
            value_target = pValueTarget;
            value_low_threshold = pValueLowThreshhold;
        }
    }

}
