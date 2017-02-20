package org.usfirst.frc.team5507.robot;

// This is really awesome camera code.

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
	
	public Object imgLock = new Object();
	private Thread visionThread;
	private GripPipeline pipeline;
	private double centerX = 0.0;
	private double centerY = 0.0;
	private double centerXAvg = 0.0;
	private double centerYAvg = 0.0;
	private double rectangleArea = 0.0;
	private int imgWidth = 320;
	private int imgHeight = 240;
	public static double[] queueX = new double[5];
	public static double[] queueY = new double[5];
	
	public Camera() {
		enableVisionThread(); //outputs a processed feed to the dashboard (overlays the found boiler tape)
	}
	
	public double averageCenter (double[] values, double x){
		for(int i = 3; i>=0; i--){
			values[i+1] = values[i];
		}
		values[0] = x;
		return (values[0]+values[1]+values[2]+values[3]+values[4])/5;
	}
	
	public void enableVisionThread() {
		pipeline = new GripPipeline();
		//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		AxisCamera camera = CameraServer.getInstance().addAxisCamera("10.55.7.37");
		
//		camera.setBrightness(0);
//		camera.setExposureManual(0);
//		camera.setWhiteBalanceManual(3000);
		
		//camera.setResolution(imgWidth, imgHeight);
		//UsbCamera camera = CameraServer.getInstance().
		//CameraServer.getInstance().addCamera(camera);
		
		//CameraServer.getInstance().addServer("Stream");
		CvSink cvSink = CameraServer.getInstance().getVideo(camera); //capture mats from camera
		CvSource outputStream = CameraServer.getInstance().putVideo("Stream", imgWidth, imgHeight); //send stream to CameraServer
		Mat mat = new Mat(); //define mat in order to reuse it
		
		visionThread = new Thread(() -> {
			
			while(!Thread.interrupted()) { //this should only be false when thread is disabled
				cvSink.grabFrame(mat);				
				pipeline.process(mat); //process the mat (this does not change the mat, and has an internal output to pipeline)
				
				// Write information about the contours back to the smart dashboard
				
				// Try to get the smart dashboard to display the filtered mat correctly, although this is weird
				// -> Are there colorspace issues?
				
				//Mat mat2 = pipeline.hsvThresholdOutput();
				if(pipeline.filterContoursOutput().size() > 1)
				{
					Rect biggestRect = new Rect(0, 0, 0, 0);
					Rect secondBiggestRect = new Rect(0, 0, 0, 0);
					for(int i = 0; i<pipeline.filterContoursOutput().size(); i++){
						Rect rectI = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
						//Imgproc.rectangle(mat, new Point(rectI.x,rectI.y), new Point(rectI.x+rectI.width,rectI.y+rectI.height), new Scalar(255, 255, 255), 2);
						if(rectI.area()>biggestRect.area()){
							biggestRect = rectI;
						}
						else if(rectI.area()>secondBiggestRect.area()){
							secondBiggestRect = rectI;
						}
					}
					centerX = (biggestRect.x+biggestRect.width/2 + secondBiggestRect.x+secondBiggestRect.width/2)/2;
					centerY = (biggestRect.y+biggestRect.height/2 + secondBiggestRect.y+secondBiggestRect.height/2)/2;
					centerXAvg = this.averageCenter(queueX, centerX);
					centerYAvg = this.averageCenter(queueY, centerY);
					//Imgproc.rectangle(mat, new Point(centerXAvg-1,40), new Point(centerXAvg+1,60), new Scalar(0, 50, 220), 5);
						Imgproc.rectangle(mat, new Point(biggestRect.x,biggestRect.y), new Point(biggestRect.x+biggestRect.width,biggestRect.y+biggestRect.height), new Scalar(255, 255, 255), 2);
						Imgproc.rectangle(mat, new Point(secondBiggestRect.x,secondBiggestRect.y), new Point(secondBiggestRect.x+secondBiggestRect.width,secondBiggestRect.y+secondBiggestRect.height), new Scalar(255, 255, 255), 2);
					//Rect rectOne = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0)); //get the first MatOfPoint (contour), calculate bounding rectangle
					//Imgproc.rectangle(mat, new Point(rectOne.x,rectOne.y), new Point(rectOne.x+rectOne.width,rectOne.y+rectOne.height), new Scalar(255, 0, 0), 5);
				}
				Imgproc.rectangle(mat, new Point(centerXAvg-1,centerYAvg-1), new Point(centerXAvg+1,centerYAvg+1), new Scalar(0, 50, 220), 5);
				//Imgproc.rectangle(mat2, new Point(50,50), new Point(100,100), new Scalar(255, 0, 0), 5);			
				outputStream.putFrame(mat); //give stream (and CameraServer) a new frame
				SmartDashboard.putNumber("numberOfContoursFound", pipeline.filterContoursOutput().size());
				SmartDashboard.putNumber("CenterX", this.getCenterX());
				SmartDashboard.putNumber("CenterY", this.getCenterY());
				
				continue;
			}
		});	
		visionThread.setDaemon(true);
		visionThread.start();
	}
	
	public double getArea(){
		synchronized(imgLock) {
			return rectangleArea;
		}
	}
	
	public double getCenterY() {
		synchronized(imgLock) {
			return centerYAvg;
		}
	}

	public double getCenterX() {
		synchronized(imgLock) {
			return centerXAvg;
		}
	}
	
	public int getContoursFound() {
		return pipeline.filterContoursOutput().size();
	}
	
	public int getImgWidth(){
		return imgWidth;
	}
	
	public int getImgHeight(){
		return imgHeight;
	}
	
	
//Reference Code__________________________________________________________________________________________________________________
	
//	if(runProcessing) {		
//		pipeline.process(mat); //process the mat (this does not change the mat, and has an internal output to pipeline)
//		int contoursFound = pipeline.filterContoursOutput().size();
//		
//		if(contoursFound>=2) {
//			
//			Rect rectOne = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0)); //get the first MatOfPoint (contour), calculate bounding rectangle
//			Rect rectTwo = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1)); //get the second MatOfPoint (contour)
//			Rect rectThree = null;
//			
//			if(contoursFound>2) {
//				
//				SmartDashboard.putString("More Vision State", "Saw Three Contours");
//				rectThree = Imgproc.boundingRect(pipeline.filterContoursOutput().get(2));
//				ArrayList<Rect> orderedRectangles= new ArrayList<Rect>();
//				orderedRectangles.add(rectOne);
//				if(rectTwo.area()>rectOne.area()){
//					orderedRectangles.add(0,rectTwo);
//				} else {
//					orderedRectangles.add(rectTwo);
//				} 
//				
//				if(rectThree.area()>orderedRectangles.get(0).area()){
//					orderedRectangles.add(0,rectThree);
//				} else if(rectThree.area()>orderedRectangles.get(1).area()){
//					orderedRectangles.add(1,rectThree);
//				} else {
//					orderedRectangles.add(rectThree);
//				}
//				
//				Rect topRect = (orderedRectangles.get(2).y>orderedRectangles.get(1).y) ? orderedRectangles.get(1) : orderedRectangles.get(2); 
//				Rect bottomRect = (orderedRectangles.get(2).y>orderedRectangles.get(1).y) ? orderedRectangles.get(2) : orderedRectangles.get(1);
//				Rect mergedRect = new Rect(topRect.x, topRect.y, (int)(bottomRect.br().x-topRect.tl().x), (int)(bottomRect.br().y-topRect.tl().y));
//				
//				if(rectOne==orderedRectangles.get(1) || rectOne==orderedRectangles.get(2)){
//					rectOne = mergedRect;
//					rectTwo = orderedRectangles.get(0);
//				} else {
//					rectOne = orderedRectangles.get(0);
//					rectTwo = mergedRect;
//				}
//			} 
//			else {
//				SmartDashboard.putString("More Vision State", "Saw Two Contours");			
//			}
//			
//			//rect.x is the left edge as far as I can tell
//			//rect.y is the top edge as far as I can tell
//			centerXOne = rectOne.x + (rectOne.width/2); //returns the center of the bounding rectangle
//			centerYOne = rectOne.y + (rectOne.height/2); //returns the center of the bounding rectangle
//			centerXTwo = rectTwo.x + (rectTwo.width/2);
//			centerYTwo = rectTwo.y + (rectTwo.height/2);
//			double width=rectTwo.x-(rectOne.x+rectOne.width);
//			double height=rectOne.y-(rectTwo.y+rectTwo.height);
//			
//			synchronized (imgLock){
//				rectangleArea=width*height;
//				centerYAvg = (centerYOne + centerYTwo)/2;
//				centerXAvg = (centerXOne + centerXTwo)/2;
//			}
//			
//			//scalar(int, int, int) is in BGR color space
//			//the points are the two corners of the rectangle as far as I can tell
//			Imgproc.rectangle(mat, new Point(rectOne.x, rectOne.y), new Point(rectTwo.x + rectTwo.width, rectTwo.y + rectTwo.height), new Scalar(0, 0, 255), 2); //draw rectangle of the detected object onto the image
//			Imgproc.rectangle(mat, new Point(centerXAvg-3,centerYAvg-3), new Point(centerXAvg+3,centerYAvg+3), new Scalar(255, 0, 0), 5);
//		
//			SmartDashboard.putString("Vision State", "Executed overlay!");
//		}
//		else {
//			SmartDashboard.putString("Vision State", "Did not find goal, found " + pipeline.filterContoursOutput().size() + " contours");
//		}
//		
//		Rect rectangle = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0)); //get the first MatOfPoint (contour), calculate bounding rectangle
//		Imgproc.rectangle(mat, new Point(rectangle.x, rectangle.y), new Point(rectangle.x + rectangle.width, rectangle.y + rectangle.height), new Scalar(0, 0, 255), 2); //draw rectangle of the detected object onto the image
//		SmartDashboard.putNumber("Center X", centerXAvg);
//	}
//	outputStream.putFrame(mat); //give stream (and CameraServer) a new frame
	
}