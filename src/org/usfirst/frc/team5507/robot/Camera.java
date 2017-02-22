package org.usfirst.frc.team5507.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
	
	public Object imgLock = new Object();
	private Thread visionThread;
	private GripPipeline pipeline;
	private double centerX = 0.0;
	private double centerY = 0.0;
	private double rectangleArea = 0.0;
	private int imgWidth = 320;
	private int imgHeight = 240;
	public static double[] queueX = new double[5];
	public static double[] queueY = new double[5];
	
	/**
	 * Constructor for a camera object
	 */
	public Camera() {
		enableVisionThread();
	}
	
	/**
	 * method that calculated avg center x value based on an array. Replaces last place int, moves everything down, and adds new number
	 * @param values array of center x values
	 * @param x new center x value
	 * @return center x average
	 */
	public double averageCenter (double[] values, double x){
		for(int i = 3; i>=0; i--){
			values[i+1] = values[i];
		}
		values[0] = x;
		return (values[0]+values[1]+values[2]+values[3]+values[4])/5;
	}
	
	/**
	 * method to initialize camera
	 */
	public void enableVisionThread() {
		pipeline = new GripPipeline();
		AxisCamera camera = CameraServer.getInstance().addAxisCamera("10.55.7.37");

		CvSink cvSink = CameraServer.getInstance().getVideo(camera); 
		CvSource outputStream = CameraServer.getInstance().putVideo("Stream", imgWidth, imgHeight);
		Mat mat = new Mat();
		
		visionThread = new Thread(() -> {
			
			while(!Thread.interrupted()) {
				cvSink.grabFrame(mat);				
				pipeline.process(mat);
				
				if(pipeline.filterContoursOutput().size() > 1)
				{
					Rect biggestRect = new Rect(0, 0, 0, 0);
					Rect secondBiggestRect = new Rect(0, 0, 0, 0);
					for(int i = 0; i<pipeline.filterContoursOutput().size(); i++){
						Rect rectI = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
						if(rectI.area()>biggestRect.area()){
							secondBiggestRect = biggestRect.clone();
							biggestRect = rectI.clone();
						}
						else if(rectI.area()>secondBiggestRect.area()){
							secondBiggestRect = rectI.clone();
						}
					}
					centerX = (biggestRect.x+biggestRect.width/2 + secondBiggestRect.x+secondBiggestRect.width/2)/2;
					centerY = (biggestRect.y+biggestRect.height/2 + secondBiggestRect.y+secondBiggestRect.height/2)/2;
					Imgproc.rectangle(mat, new Point(secondBiggestRect.x,secondBiggestRect.y), new Point(secondBiggestRect.x+secondBiggestRect.width,secondBiggestRect.y+secondBiggestRect.height), new Scalar(255, 255, 255), 2);
					Imgproc.rectangle(mat, new Point(biggestRect.x,biggestRect.y), new Point(biggestRect.x+biggestRect.width,biggestRect.y+biggestRect.height), new Scalar(255, 255, 255), 2);					//Rect rectOne = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0)); //get the first MatOfPoint (contour), calculate bounding rectangle
				}
				Imgproc.rectangle(mat, new Point(centerX-1,centerY-1), new Point(centerX+1,centerY+1), new Scalar(0, 50, 220), 5);		
				outputStream.putFrame(mat); //give stream (and CameraServer) a new frame
				SmartDashboard.putNumber("numberOfContoursFound", pipeline.filterContoursOutput().size());
				//SmartDashboard.putNumber("CenterX", this.getCenterX());
				//SmartDashboard.putNumber("CenterY", this.getCenterY());
				
				continue;
			}
		});	
		visionThread.setDaemon(true);
		visionThread.start();
	}
	
	/**
	 * method to get the area
	 * @return a double representing area
	 */
	public double getArea(){
		synchronized(imgLock) {
			return rectangleArea;
		}
	}
	
	/**
	 * method to get the y value 
	 * @return a double representing center y
	 */
	public double getCenterY() {
		synchronized(imgLock) {
			return centerY;
		}
	}

	/**
	 * method to get center x value
	 * @return a double representing center x
	 */
	public double getCenterX() {
		synchronized(imgLock) {
			return centerX;
		}
	}
	
	/**
	 * method to get number of contours found
	 * @return an int representing number of contours found
	 */
	public int getContoursFound() {
		return pipeline.filterContoursOutput().size();
	}
	
	/**
	 * method to get width of image
	 * @return an int representing image width
	 */
	public int getImgWidth(){
		return imgWidth;
	}
	
	/**
	 * method to get height of image
	 * @return an int representing image height
	 */
	public int getImgHeight(){
		return imgHeight;
	}
}