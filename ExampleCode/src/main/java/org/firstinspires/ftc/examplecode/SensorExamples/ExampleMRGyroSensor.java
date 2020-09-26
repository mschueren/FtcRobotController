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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Demonstrates how to setup and use 2 MR color sensors
 */
@Autonomous(name = "Read MR Gyro Sensor", group = "SensorExamples")
@Disabled
public class ExampleMRGyroSensor extends OpMode {

    ModernRoboticsI2cGyro gyroSensor;
    boolean lastResetState = false;
    boolean curResetState  = false;

    @Override
    public void init() {
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyrosensor");

        gyroSensor.setI2cAddress(I2cAddr.create8bit(0x20));
    }


    @Override
    public void init_loop() {
        if(gyroSensor.getIntegratedZValue() != 0) {
            gyroSensor.calibrate();
        }

        if(gyroSensor.isCalibrating()) {
            telemetry.addData("Status", "Calibrating Do Not Move!");
        } else {
            telemetry.addData("Status", "Initialized");
        }
    }


    @Override
    public void start() { }


    @Override
    public void loop() {
        // If the A and B buttons are pressed just now, reset Z heading.
        curResetState = (gamepad1.a && gamepad1.b);
        if (curResetState && !lastResetState) {
            gyroSensor.resetZAxisIntegrator();
        }
        lastResetState = curResetState;


        telemetry.addData("0", "Press A & B on Gamepad1 to reset Z Axis");
        telemetry.addData("1", "Heading: " + gyroSensor.getHeading());
        telemetry.addData("2", "Integrated Z Value: " + gyroSensor.getIntegratedZValue());
    }

    @Override
    public void stop() {}
}
