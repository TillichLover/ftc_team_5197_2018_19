package org.firstinspires.ftc.teamcode;

public class MultithreadedREVTrixbotIterativeAutonomousOpmode extends ModularRobotIterativeOpMode {
    REVTrixbot robot = new REVTrixbot();
    @Override
    public void init() {
        robot.driveTrain = new REVTrixbot.DriveTrain(){
            /**
             * If this thread was constructed using a separate
             * <code>Runnable</code> run object, then that
             * <code>Runnable</code> object's <code>run</code> method is called;
             * otherwise, this method does nothing and returns.
             * <p>
             * Subclasses of <code>Thread</code> should override this method.
             *
             * @see #start()
             * @see #stop()

             */
            @Override
            public void run() {
                //super.run();
                encoderDrive(1, 4, 4);
            }
        };

        robot.driveTrain.init(hardwareMap);

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    @Override
    public void start() {
        super.start();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {

    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    @Override
    public void stop() {
        super.stop();

        robot.driveTrain.interrupt(); //may or maynot need to catch exception?  https://stackoverflow.com/questions/10961714/how-to-properly-stop-the-thread-in-java

    }
}
