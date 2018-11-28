package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOPLifterTest", group = "REVTrixbot")
public class REVTrixbotLifterTeleOpTest extends ModularRobotIterativeOpMode {
    REVTrixbot robot  = new REVTrixbot();
    @Override
    public void init() {
        robot.roverRuckusRevTrixBotLift.init(hardwareMap);
    }

    @Override
    public void loop() {
        //robot.roverRuckusRevTrixBotLift.setBraking(false);
        robot.roverRuckusRevTrixBotLift.teleOpMove(gamepad1.a, gamepad1.b, 0.15);
        telemetry.addData("Lift Rotations", robot.roverRuckusRevTrixBotLift.getCurrentPosition());
    }
}
