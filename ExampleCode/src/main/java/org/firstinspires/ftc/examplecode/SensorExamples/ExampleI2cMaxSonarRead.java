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

package org.firstinspires.ftc.examplecode.SensorExamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Read Max Sonar sensor", group = "SensorExamples")
@Disabled

public class ExampleI2cMaxSonarRead extends OpMode {
    int distance = 200;   //Variable for Distance data
    int sonarDistance = 0;   //Variable for combined sonar Distance reading

    byte[] sonarCache;         //The read will return an array of bytes.  They are stored in this variable

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static final int SONAR_ADDRESS = 224;           //Default I2C address for Max Sonar
    public static final int SONAR_REG_START = 0x00;        //Register to start reading
    public static final int SONAR_READ_LENGTH = 2;         //Number of byte to read
    public static final int SONAR_COMMAND_REGISTER = 0x00; //Register to issue commands to the sonar sensor
    public static final byte SONAR_COMMAND = 0x51;         //Command to send a sonar pulse

    private I2cAddr sonarAddr;
    public I2cDevice sonarSensor;
    public I2cDeviceSynch sonarSynch;


    @Override
    public void init() {
        sonarSensor = hardwareMap.i2cDevice.get("maxsonar");
        sonarAddr = I2cAddr.create8bit(SONAR_ADDRESS);
        sonarSynch = new I2cDeviceSynchImpl(sonarSensor, sonarAddr, false);
        sonarSynch.engage();

        timer.reset();
    }

    @Override
    public void init_loop() {
        //Trigger a read to the sonar sensor.  You should hear a clicking sound!
        if (timer.time() > 0.1) {
            sonarSynch.write8(SONAR_COMMAND_REGISTER, SONAR_COMMAND);
            timer.reset();
        }
        else {
            sonarCache = sonarSynch.read(SONAR_REG_START, SONAR_READ_LENGTH);
            telemetry.addData("MSB", sonarCache[0]);
            telemetry.addData("LSB", sonarCache[1]);
        }
    }


    @Override
    public void loop() {

        if (timer.time() > 0.2) {
            sonarSynch.write8(SONAR_COMMAND_REGISTER, SONAR_COMMAND);
            timer.reset();
        }
        else if (timer.time() > .08) {
            sonarCache = sonarSynch.read(SONAR_REG_START, SONAR_READ_LENGTH);

            //One byte only goes to 256 values.  Two bytes must be combined to get the total distance
            sonarDistance = 0X000000FF & (int) sonarCache[0];
            sonarDistance = (sonarDistance << 8 | (0X000000FF & (int) sonarCache[1]));

            distance = sonarDistance > 625 ? distance : sonarDistance;
        }

       // send the info back to driver station using telemetry function.

        telemetry.addData("MSB", sonarCache[0]);
        telemetry.addData("LSB", sonarCache[1]);
        telemetry.addData("DIST", distance );

    }


    @Override
    public void stop() {

    }
}