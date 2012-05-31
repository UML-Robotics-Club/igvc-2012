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

	private boolean gestureValid = false;
	private float maxSpeed = 1.0f;
	private float maxRotsp = 1.0f;
	
	public StatusView(Context context) {
		super(context);
	}

	@Override
	public boolean onTouchEvent(MotionEvent ev) {
		float WW = getWidth();
		float HH = getHeight();
		
		ControlClient client = ControlClient.getInstance();
		
		int count = ev.getPointerCount();
		
		if (count > 1) {
			gestureValid = false;
			client.setCommand(false, 0, 0);
			return true;
		}
		
		int   aa = ev.getAction() & MotionEvent.ACTION_MASK;
		float xx = ev.getX(0) / WW;
		float yy = ev.getY(0) / HH;

		if (aa == MotionEvent.ACTION_DOWN) {
			if (Math.hypot(xx - 0.5, yy - 0.5) < 0.1) {
				gestureValid = true;
			}
			client.setCommand(false, 0, 0);
		}

		if (aa == MotionEvent.ACTION_MOVE) {
			float dx = xx - 0.5f;
			float dy = yy - 0.5f;
			
			float speed = maxSpeed * (2.0f * -dy);
			float rotsp = maxRotsp * (2.0f * -dx);
			
			if (gestureValid)
				client.setCommand(false, speed, rotsp);
		}
				
		if (aa == MotionEvent.ACTION_UP) {
			gestureValid = false;
			client.setCommand(false, 0, 0);
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

		Log.v(TAG, "onDraw");
	}
}
