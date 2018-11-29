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
        robot.roverRuckusRevTrixBotLift.teleOpMove(gamepad1.a, gamepad1.b, 0.1);
        robot.roverRuckusRevTrixBotLift.teleOpMoveToMaxPos(gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.dpad_up, 0.1); //must all be pressed to prevent accidental retractions
        robot.roverRuckusRevTrixBotLift.teleOpMoveToMinPos(gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.dpad_down, 0.1);
        //msStuckDetectLoop = //TODO make method to calculate wait time for watch dog
        telemetry.addData("Lift Rotations", robot.roverRuckusRevTrixBotLift.getCurrentPosition());

    }


}
