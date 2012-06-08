package net.ferrus.ReCo;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.ShapeDrawable;
import android.graphics.drawable.shapes.OvalShape;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

public class StatusView extends View {
	private final String TAG = "StatusView";

	ControlClient client;
	
	private boolean gestureValid = false;
	private float maxSpeed = 1.0f;
	private float maxRotsp = 1.0f;
	
	private float touchXX  = 0.25f;
	private float touchYY  = 0.25f;
	private double lastSP  = 0.0;
	private double lastTU  = 0.0;
	
	public StatusView(Context context) {
		super(context);

		client = ControlClient.getInstance();
		maxSpeed = client.getMaxSpeed();
		maxRotsp = client.getMaxRotsp();
	}

	private float sign(float xx) {
		if (xx >= 0)
			return +1.0f;
		else
			return -1.0f;
	}
	
	private void sendCommand(double sp, double tu) {
		client.setCommand(sp, tu);
		lastSP = sp;
		lastTU = tu;
	}
	
	@Override
	public boolean onTouchEvent(MotionEvent ev) {
		float WW = getWidth();
		float HH = getHeight();
		
		int count = ev.getPointerCount();
		
		if (count > 1) {
			gestureValid = false;
			Log.v(TAG, "Gesture end: second touch");
			sendCommand(0.0, 0.0);
			return true;
		}
		
		int   aa = ev.getAction() & MotionEvent.ACTION_MASK;
		touchXX = ev.getX(0) / WW;
		touchYY = ev.getY(0) / HH;
		invalidate();

		if (aa == MotionEvent.ACTION_DOWN) {
			if (Math.hypot(touchXX - 0.5, touchYY - 0.5) < 0.1) {
				gestureValid = true;
				Log.v(TAG, "Gesture start");
			}
			sendCommand(0.0, 0.0);
		}

		if (aa == MotionEvent.ACTION_MOVE) {
			float dx = touchXX - 0.5f;
			float dy = touchYY - 0.5f;

			if (Math.hypot(dx, dy) < 0.05f) {
				dx = 0.0f;
				dy = 0.0f;				
			}
			else {
				dx -= 0.025f * sign(dx);
				dy -= 0.025f * sign(dy);
			}

			float speed = maxSpeed * (2.0f * -dy);
			float rotsp = maxRotsp * (2.0f * -dx);

			if (gestureValid)
				sendCommand(speed, rotsp);
		}

		if (aa == MotionEvent.ACTION_UP) {
			gestureValid = false;
			Log.v(TAG, "Gesture end: touch ended");
			sendCommand(0.0, 0.0);
		}

		return true;
	}
	
	@Override
	public void onDraw(Canvas canvas) {
		float WW = getWidth();
		float HH = getHeight();
		
		canvas.scale(WW, HH, 0.0f, 0.0f);
		Paint paint = new Paint();
		
		// Clear the canvas.
		canvas.drawColor(Color.BLACK);
		
		// Draw the robot.
		paint.setColor(Color.WHITE);
		canvas.drawCircle(0.5f, 0.5f, 0.1f, paint);
		paint.setColor(Color.GRAY);
		canvas.drawCircle(0.5f, 0.5f, 0.05f, paint);
		
		// Show current commands
		paint.setColor(Color.WHITE);
		paint.setTextSize(10.0f);
		canvas.drawText(String.format("%.02f %.02f", lastSP, lastTU), 0.1f, 0.1f, paint);
		
		// Draw the gesture.
		if(gestureValid) {
			paint.setColor(Color.BLUE);			
		}
		else {
			paint.setColor(Color.RED);
		}
		
		canvas.drawCircle(touchXX, touchYY, 0.025f, paint);
		
		Log.v(TAG, "onDraw");
	}
}
