package com.tina.tinaopencv;

import android.hardware.Camera;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

public class MainActivity extends AppCompatActivity implements Camera.PreviewCallback, SurfaceHolder.Callback{

    static {
        System.loadLibrary("native-lib");
    }
    private CameraHelper cameraHelper;
    int cameraId = Camera.CameraInfo.CAMERA_FACING_FRONT;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        SurfaceView surfaceView = findViewById(R.id.surfaceView);
        surfaceView.getHolder().addCallback(this);
        cameraHelper = new CameraHelper(cameraId);
        cameraHelper.setPreviewCallback(this);
        cameraHelper.setPreviewDisplay(surfaceView.getHolder());

        Utils.copyAssets(this, "lbpcascade_frontalface.xml");
    }

    @Override
    protected void onResume() {
        super.onResume();
        //初始化跟踪器
        init("/sdcard/lbpcascade_frontalface.xml");
        cameraHelper.startPreview();
    }

    @Override
    protected void onStop() {
        super.onStop();
        //释放跟踪器
        release();
        cameraHelper.stopPreview();
    }

    @Override
    public void onPreviewFrame(byte[] data, Camera camera) {
        //传输数据
        postData(data, CameraHelper.WIDTH, CameraHelper.HEIGHT, cameraId);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        setSurface(holder.getSurface());
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }

    /**
     * 初始化 追踪器
     * @param model
     */
    native void init(String model);

    /**
     * 设置画布
     *  ANativeWindow
     * @param surface
     */
    native void setSurface(Surface surface);

    /**
     * 处理摄像头数据
     * @param data
     * @param w
     * @param h
     * @param cameraId
     */
    native void postData(byte[] data, int w, int h, int cameraId);
    /**
     * 释放
     */
    native void release();


    public void switchCamera(View view) {
        cameraHelper.switchCamera();
        cameraId = cameraHelper.getCameraId();
    }
}
