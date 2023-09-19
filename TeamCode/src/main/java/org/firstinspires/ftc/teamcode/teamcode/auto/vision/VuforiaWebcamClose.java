package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.IOException;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;

public class VuforiaWebcamClose {

    private final CompletableFuture<Void> futureVuforiaWebcamClose;

    public VuforiaWebcamClose(VuforiaLocalizer pVuforiaLocalizer) {
        // Start up the shutdown of the Vuforia webcam as a CompletableFuture.
        futureVuforiaWebcamClose = Threading.launchAsync(new VuforiaWebcamCloseCallable(pVuforiaLocalizer));
     }

    // Because closing the Vuforia webcam may be time-consuming (up to 5 sec. from the logs)
    // run it in a separate thread.
    private static class VuforiaWebcamCloseCallable implements Callable<Void> {

        private final VuforiaLocalizer vuforiaLocalizer;

        VuforiaWebcamCloseCallable(VuforiaLocalizer pVuforiaLocalizer) {
            vuforiaLocalizer = pVuforiaLocalizer;
        }

        public Void call() {
            vuforiaLocalizer.getCamera().close();
            return null;
        }
    }

    // Wait for the Vuforia webcam close to complete.
    public void waitForVuforiaWebcamClose() throws IOException, InterruptedException {
        Threading.getFutureCompletion(futureVuforiaWebcamClose);
    }

}
