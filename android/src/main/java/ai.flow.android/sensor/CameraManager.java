package ai.flow.android.sensor;

import ai.flow.app.OnRoadScreen;
import ai.flow.common.ParamsInterface;
import ai.flow.common.transformations.Camera;
import ai.flow.definitions.Definitions;
import ai.flow.modeld.ModelExecutor;
import ai.flow.modeld.messages.MsgFrameData;
import ai.flow.sensor.SensorInterface;
import ai.flow.sensor.messages.MsgFrameBuffer;

import android.annotation.SuppressLint;
import android.content.Context;
import android.hardware.camera2.CaptureRequest;
import android.os.Build;
import android.util.Log;
import android.util.Range;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.OptIn;
import androidx.annotation.RequiresApi;
import androidx.camera.camera2.interop.Camera2Interop;
import androidx.camera.camera2.interop.ExperimentalCamera2Interop;
import androidx.camera.core.*;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import com.google.common.util.concurrent.ListenableFuture;

import messaging.ZMQPubHandler;

import org.opencv.core.Core;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;

import static ai.flow.android.sensor.Utils.fillYUVBuffer;

public class CameraManager implements SensorInterface {
    private static final String TAG = "CameraManager";
    private ImageAnalysis.Analyzer analyzer;
    public ProcessCameraProvider cameraProvider;
    public String frameDataTopic = "wideRoadCameraState",
            frameBufferTopic = "wideRoadCameraBuffer";
    public ZMQPubHandler ph;
    public boolean running = false;
    public int W = Camera.frameSize[0];
    public int H = Camera.frameSize[1];
    public MsgFrameData msgFrameData, msgFrameRoadData;
    public MsgFrameBuffer msgFrameBuffer, msgFrameRoadBuffer;
    public int frameID = 0;
    public Context context;
    public ParamsInterface params = ParamsInterface.getInstance();
    public Fragment lifeCycleFragment;
    int cameraType = Camera.CAMERA_TYPE_WIDE;
    CameraControl cameraControl;
    ByteBuffer yuvBuffer;
    androidx.camera.core.Camera camera;

    public CameraSelector getCameraSelector() {
        OnRoadScreen.CamSelected = Camera.UseCameraID;
        List<CameraInfo> availableCamerasInfo = cameraProvider.getAvailableCameraInfos();
        return availableCamerasInfo.get(OnRoadScreen.CamSelected).getCameraSelector();
    }

    public CameraManager(Context context){
        msgFrameData = new MsgFrameData(cameraType);
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        this.context = context;

        msgFrameBuffer = new MsgFrameBuffer(W * H * 3/2, cameraType);
        yuvBuffer = msgFrameBuffer.frameBuffer.getImage().asByteBuffer();
        msgFrameBuffer.frameBuffer.setEncoding(Definitions.FrameBuffer.Encoding.YUV);
        msgFrameBuffer.frameBuffer.setFrameHeight(H);
        msgFrameBuffer.frameBuffer.setFrameWidth(W);

        ph = new ZMQPubHandler();
        ph.createPublishers(Arrays.asList(frameDataTopic, frameBufferTopic));
    }

    public void setLifeCycleFragment(Fragment lifeCycleFragment){
        this.lifeCycleFragment = lifeCycleFragment;
    }

    public void start() {
        if (running)
            return;
        running = true;

        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(context);
        cameraProviderFuture.addListener(() -> {
            analyzer = new ImageAnalysis.Analyzer() {
                @SuppressLint("RestrictedApi")
                @ExperimentalCamera2Interop @OptIn(markerClass = ExperimentalGetImage.class) @RequiresApi(api = Build.VERSION_CODES.N)
                @Override
                public void analyze(@NonNull ImageProxy image) {
                    long startTimestamp = System.currentTimeMillis();
                    fillYUVBuffer(image, yuvBuffer);

                    ImageProxy.PlaneProxy yPlane = image.getPlanes()[0];

                    msgFrameBuffer.frameBuffer.setYWidth(W);
                    msgFrameBuffer.frameBuffer.setYHeight(H);
                    msgFrameBuffer.frameBuffer.setYPixelStride(yPlane.getPixelStride());
                    msgFrameBuffer.frameBuffer.setUvWidth(W /2);
                    msgFrameBuffer.frameBuffer.setUvHeight(H /2);
                    msgFrameBuffer.frameBuffer.setUvPixelStride(image.getPlanes()[1].getPixelStride());
                    msgFrameBuffer.frameBuffer.setUOffset(W * H);
                    if (image.getPlanes()[1].getPixelStride() == 2)
                        msgFrameBuffer.frameBuffer.setVOffset(W * H +1);
                    else
                        msgFrameBuffer.frameBuffer.setVOffset(W * H + W * H /4);
                    msgFrameBuffer.frameBuffer.setStride(yPlane.getRowStride());

                    msgFrameData.frameData.setFrameId(frameID);

                    ModelExecutor.instance.ExecuteModel(
                            msgFrameData.frameData.asReader(),
                            msgFrameBuffer.frameBuffer.asReader(),
                            startTimestamp);

                    ph.publishBuffer(frameDataTopic, msgFrameData.serialize(true));
                    ph.publishBuffer(frameBufferTopic, msgFrameBuffer.serialize(true));

                    frameID += 1;
                    image.close();
                }
            };

            try {
                cameraProvider = cameraProviderFuture.get();
                bindUseCases(cameraProvider);
            } catch (InterruptedException | ExecutionException e) {
                Log.w(TAG, "Camera provider was interrupted: "+ e);
            }
        }, ContextCompat.getMainExecutor(context));
    }

    @SuppressLint({"RestrictedApi", "UnsafeOptInUsageError"})
    private void bindUseCases(@NonNull ProcessCameraProvider cameraProvider) {
        ImageAnalysis.Builder builder = new ImageAnalysis.Builder();
        builder.setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST);
        Size ims = new Size(W, H);
        builder.setDefaultResolution(ims);
        builder.setMaxResolution(ims);
        builder.setTargetResolution(ims);

        Camera2Interop.Extender<ImageAnalysis> CameraRequests = new Camera2Interop.Extender<>(builder);
        CameraRequests.setCaptureRequestOption(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
        CameraRequests.setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
        CameraRequests.setCaptureRequestOption(CaptureRequest.COLOR_CORRECTION_MODE, CaptureRequest.COLOR_CORRECTION_MODE_FAST);
        CameraRequests.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(20, 20));
//        CameraRequests.setCaptureRequestOption(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
//        CameraRequests.setCaptureRequestOption(CaptureRequest.LENS_FOCUS_DISTANCE, 0f);
        ImageAnalysis imageAnalysis = builder.build();
        imageAnalysis.setAnalyzer(ContextCompat.getMainExecutor(context), analyzer);

        // f3 uses wide camera.
        CameraSelector cameraSelector = getCameraSelector();

        camera = cameraProvider.bindToLifecycle(lifeCycleFragment.getViewLifecycleOwner(), cameraSelector, imageAnalysis);

        cameraControl = camera.getCameraControl();
        cameraControl.setZoomRatio(Camera.digital_zoom_apply);
    }

    @SuppressLint("RestrictedApi")
    @Override
    public void stop() {
        // TODO: add pause/resume functionality
        if (!running)
            return;
        cameraProvider.unbindAll();
        running = false;
    }

    @Override
    public void dispose(){
        stop();
        ph.releaseAll();
    }
}