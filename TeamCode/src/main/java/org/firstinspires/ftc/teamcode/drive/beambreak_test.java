package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="beambreak", group="Iterative OpMode")
public class beambreak_test extends OpMode {
    private DigitalChannel beambreak;

    @Override
    public void init() {
        beambreak = hardwareMap.get(DigitalChannel.class, "beambreak");
        beambreak.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        telemetry.addData("beambreak",  beambreak.getState());

    }
}
