/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.examplecode.ComputerVision;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name = "Capture Image to File", group = "Vuforia")
//@Disabled

public class ExampleCaptureImageToFile extends OpMode {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables velocityVortexImages;
    private VuforiaTrackable wheelsTarget;
    private VuforiaTrackableDefaultListener wheelsListener;
    public static final String TAG = "Vuforia Navigation Sample";

    private boolean buttonPressed = false;
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;


    @Override
    public void init() {
        vuforiaInit();

        vuforia.enableConvertFrameToBitmap();

        /** @see #captureFrameToFile() */
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        velocityVortexImages.activate();//Start tracking the data sets we care about.
    }


    @Override
    public void loop() {
        if (gamepad1.a && !buttonPressed) {
            captureFrameToFile();
        }
        buttonPressed = gamepad1.a;

        // send the info back to driver station using telemetry function.
        telemetry.addData(wheelsTarget.getName(), wheelsListener.isVisible() ? "Visible" : "Not Visible");
    }


    @Override
    public void stop() {

    }

    private void vuforiaInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        velocityVortexImages = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        wheelsTarget = velocityVortexImages.get(0);
        wheelsTarget.setName("Wheels");

        wheelsListener = (VuforiaTrackableDefaultListener) wheelsTarget.getListener();
    }

    /**
     * Sample one frame from the Vuforia stream and write it to a .PNG image file on the robot
     * controller in the /sdcard/FIRST/data directory. The images can be downloaded using Android
     * Studio's Device File Explorer, ADB, or the Media Transfer Protocol (MTP) integration into
     * Windows Explorer, among other means. The images can be useful during robot design and calibration
     * in order to get a sense of what the camera is actually seeing and so assist in camera
     * aiming and alignment.
     */
    void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }

}