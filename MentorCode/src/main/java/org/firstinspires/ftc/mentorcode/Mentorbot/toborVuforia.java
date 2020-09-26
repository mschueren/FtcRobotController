/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.mentorcode.Mentorbot;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


public abstract class toborVuforia {

    //enumeration of the images on the field
    public enum images{wheels, legos, tools, gears}
    //variabe to keep track of the most recently seen image
    public static images lastSeen = images.wheels;

    //declaration of matrices to keep track of the last available data based on each image
    public static OpenGLMatrix lastGearsLocation = OpenGLMatrix.identityMatrix();
    public static OpenGLMatrix lastToolsLocation = OpenGLMatrix.identityMatrix();
    public static OpenGLMatrix lastLegosLocation = OpenGLMatrix.identityMatrix();
    public static OpenGLMatrix lastWheelsLocation = OpenGLMatrix.identityMatrix();

    //variables specific to each image
    public static VuforiaTrackable tBRwheels; //object for vuforia to keep track of the image
    //matrix to store the location of the robot based on each image
    public static OpenGLMatrix location_tBRwheels;
    //declaration of a variabe to place each image on the field coordinate system
    public static OpenGLMatrix wheelsLocation = OpenGLMatrix.identityMatrix();
    //variable to indicate whether each image is visible
    public static boolean wheelsVisible;

    //each variable is repeated for each image

    public static VuforiaTrackable tBClegos;
    public static OpenGLMatrix location_tBClegos;
    public static OpenGLMatrix legosLocation = OpenGLMatrix.identityMatrix();
    public static boolean legosVisible;
    public static VuforiaTrackable tRCtools;
    public static OpenGLMatrix location_tRCtools;
    public static OpenGLMatrix toolsLocation = OpenGLMatrix.identityMatrix();
    public static boolean toolsVisible;
    public static VuforiaTrackable tRRgears;
    public static OpenGLMatrix location_tRRgears;
    public static OpenGLMatrix gearsLocation = OpenGLMatrix.identityMatrix();
    public static boolean gearsVisible;

    //variable for storing information used to identify each image
    public static VuforiaTrackables FTCImages;
    //declaration
    public static OpenGLMatrix phoneLocationOnRobot;

    public static VuforiaLocalizer.Parameters parameters;
    public static VuforiaLocalizer vuforia;

    public static void init() {

        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AWc/BOX/////AAAAGQoPIOSnQ0W/pZoS+pEUrjx6IMYJtACLfKyLmGUj4pXGU6hbY2eTijVGycLFLg99x7vNP9kVMx+++gFx016Z4Hza3nQdX2JnlvlIfzJmm6kpi+d54NzsjjS1zQY+Ayk130zfHEKSuSJR+3OrLkRCl0o+ZEtIXkQW+xl1SJZiGwOcl3quTBxxmx6GJMVvh9ZiwMXbomadQM3apGhGAG2uUhVY15ogzXEw3D0RWlnTHMgnkznKOkpT5yqmkKJ3Rv707kFRATprvX9Ncb2bmcs3kfpC+en3UHxwek/jf/E2hzE4snteX4NCRdCSzbuE4MAIxR5IpGkgoGX6KO2yjSblZ0fjU5cPAhYjmaspgc+J5h+N";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        FTCImages = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        tBRwheels = FTCImages.get(0);
        tBRwheels.setName("tBRwheels");

        tRCtools = FTCImages.get(1);
        tRCtools.setName("tRCtools");

        tBClegos = FTCImages.get(2);
        tBClegos.setName("tBClegos");

        tRRgears = FTCImages.get(3);
        tRRgears.setName("tRRgears");

        float fieldHalfDimension = 1794;
        float shortImageCenterlineDimension = 305;
        float longImageCenterlineDimension = 915;
        float imageHeight = 140;

        location_tRRgears = OpenGLMatrix
                .translation(-fieldHalfDimension, -shortImageCenterlineDimension, imageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 90));
        tRRgears.setLocation(location_tRRgears);

        location_tRCtools = OpenGLMatrix
                .translation(-fieldHalfDimension, longImageCenterlineDimension, imageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 90));
        tRCtools.setLocation(location_tRCtools);

        location_tBClegos = OpenGLMatrix
                .translation(-longImageCenterlineDimension, fieldHalfDimension, imageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));// 8 and a eighth
        tBClegos.setLocation(location_tBClegos);

        location_tBRwheels = OpenGLMatrix
                .translation(shortImageCenterlineDimension, fieldHalfDimension, imageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        tBRwheels.setLocation(location_tBRwheels);

        phoneLocationOnRobot = OpenGLMatrix.translation(-182,156,1).multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES,0,-90,-180
        ));
                /*.translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));*/

        ((VuforiaTrackableDefaultListener) tRRgears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) tRCtools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) tBClegos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) tBRwheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        FTCImages.activate();
    }
    public static OpenGLMatrix matrixLocation()
    {
        imageUpdate();
        if (wheelsVisible){lastSeen=images.wheels; return lastWheelsLocation;}
        else if (legosVisible){lastSeen=images.legos; return lastLegosLocation;}
        else if (toolsVisible){lastSeen=images.tools; return lastToolsLocation;}
        else if (gearsVisible){lastSeen=images.gears; return lastGearsLocation;}
        else{return null;}
    }
    public static double getHeading()
    {
        OpenGLMatrix tempMatrix = matrixLocation();
        double h = 0;
        if (tempMatrix==null)
        {
            switch (lastSeen)
            {
                case wheels:
                    h = Orientation.getOrientation(lastWheelsLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                case legos:
                    h = Orientation.getOrientation(lastLegosLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                case tools:
                    h = Orientation.getOrientation(lastToolsLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                case gears:
                    h = Orientation.getOrientation(lastGearsLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            }
        }
        else
        {
            h = Orientation.getOrientation(tempMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        }
        if (h<=0)
        {
            return 180+h;
        }
        else
        {
            return -180+h;
        }
    }
    public static double[] getLocation()
    {
        OpenGLMatrix tempMatrix = matrixLocation();
        if (tempMatrix==null)
        {
            switch (lastSeen)
            {
                case wheels:
                    return new double[] {lastWheelsLocation.getTranslation().get(0), lastWheelsLocation.getTranslation().get(1),0};
                case legos:
                    return new double[] {lastLegosLocation.getTranslation().get(0), lastLegosLocation.getTranslation().get(1),0};
                case tools:
                    return new double[] {lastToolsLocation.getTranslation().get(0), lastToolsLocation.getTranslation().get(1),0};
                case gears:
                    return new double[] {lastGearsLocation.getTranslation().get(0), lastGearsLocation.getTranslation().get(0),0};
            }
        }
        else
        {
            return new double[] {tempMatrix.getTranslation().get(0),tempMatrix.getTranslation().get(1),1};
        }
        return null;
    }
    public static void imageUpdate() {
        wheelsLocation = ((VuforiaTrackableDefaultListener) tBRwheels.getListener()).getUpdatedRobotLocation();
        wheelsVisible = ((VuforiaTrackableDefaultListener) tBRwheels.getListener()).isVisible();
        if (wheelsLocation != null) {
            lastWheelsLocation = wheelsLocation;
        }
        //telemetry.addData("wheels loc", lastWheelsLocation == null || !wheelsVisible ? " unknown" : format(lastWheelsLocation));
        legosLocation = ((VuforiaTrackableDefaultListener) tBClegos.getListener()).getUpdatedRobotLocation();
        legosVisible = ((VuforiaTrackableDefaultListener) tBClegos.getListener()).isVisible();
        if (legosLocation != null) {
            lastLegosLocation = legosLocation;
        }
        //telemetry.addData("legos loc", lastLegosLocation == null || !legosVisible ? " unknown" : format(lastLegosLocation));
        toolsLocation = ((VuforiaTrackableDefaultListener) tRCtools.getListener()).getUpdatedRobotLocation();
        toolsVisible = ((VuforiaTrackableDefaultListener) tRCtools.getListener()).isVisible();
        if (toolsLocation != null) {
            lastToolsLocation = toolsLocation;
        }
        //telemetry.addData("tools loc", lastToolsLocation == null || !toolsVisible ? " unknown" : format(lastToolsLocation));
        gearsLocation = ((VuforiaTrackableDefaultListener) tRRgears.getListener()).getUpdatedRobotLocation();
        gearsVisible = ((VuforiaTrackableDefaultListener) tRRgears.getListener()).isVisible();
        if (gearsLocation != null) {
            lastGearsLocation = gearsLocation;
        }
        //telemetry.addData("gears loc", lastGearsLocation == null || !gearsVisible ? " unknown" : format(lastGearsLocation));

    }

    public static String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
