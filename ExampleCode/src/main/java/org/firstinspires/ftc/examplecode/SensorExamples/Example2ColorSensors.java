/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.examplecode.SensorExamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Demonstrates how to setup and use 2 MR color sensors
 */
@Autonomous(name = "Read 2 Color Sensors", group = "SensorExamples")
@Disabled
public class Example2ColorSensors extends OpMode {

    ColorSensor colorSensor1, colorSensor2;

    @Override
    public void init() {
        colorSensor1 = hardwareMap.colorSensor.get("colorsensor1");
        colorSensor2 = hardwareMap.colorSensor.get("colorsensor2");

        /*
        Note that the address on each color sensor must be changed to the addresses below
        before plugging them in together.  The best way to change the address on a Modern Robotics
        sensor is to use the Core Device Discovery program found on their website.
         */
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3C));
        colorSensor2.setI2cAddress(I2cAddr.create8bit(0x32));

        colorSensor1.enableLed(false);
        colorSensor2.enableLed(true);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() { }


    @Override
    public void loop() {
        telemetry.addData("Color1 Red", colorSensor1.red());
        telemetry.addData("Color1 Green", colorSensor1.green());
        telemetry.addData("Color1 Blue", colorSensor1.blue());

        telemetry.addData("Color2 Red", colorSensor2.red());
        telemetry.addData("Color2 Green", colorSensor2.green());
        telemetry.addData("Color2 Blue", colorSensor2.blue());
    }

    @Override
    public void stop() {}
}
