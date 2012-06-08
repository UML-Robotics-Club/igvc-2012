package net.ferrus.ReCo;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;

public class ControlActivity extends Activity {
	private static final String TAG = "ControlActivity";
	
	StatusView view;
	
	ControlClient client;

	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        
        client = ControlClient.getInstance();
        
        view = new StatusView(this);
        setContentView(view);
        
    	Log.v(TAG, "onCreate");
	}
	
    @Override
    public void onStart() {
    	super.onStart();
    	
    	Log.v(TAG, "onStart");
    }

    public void onPause() {
    	super.onPause();
    	killProcess();
    }
    
    public void onDestroy() {
    	super.onDestroy();
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main, menu);
        return true;
    }
    
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
       switch (item.getItemId()) {
       case R.id.menuShowConfig:
    	   startActivity(new Intent(ControlActivity.this, ConnectActivity.class));
    	   return true;
       case R.id.menuExit:
    	   killProcess();
    	   return true;    	   
       default:
    	   Log.v(TAG, "Unknown menu item");
    	   return true;
       }
    }
    
    public void killProcess() {
 	   finish();
 	   int pid = android.os.Process.myPid();
 	   android.os.Process.killProcess(pid); 
    }
}    
