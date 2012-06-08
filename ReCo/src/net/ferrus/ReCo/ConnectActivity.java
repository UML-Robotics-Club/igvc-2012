package net.ferrus.ReCo;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.TextView;

public class ConnectActivity extends Activity {
	private static final String TAG = "ConnectActivity";

	@Override
    public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
        setContentView(R.layout.connect);

    	Button button = (Button) findViewById(R.id.connectNowButton);
        button.setOnClickListener(new OnClickListener() {
        	public void onClick(View vv) {
        		Log.v(TAG, "Connect...");
        		
        		TextView editHost = (TextView) findViewById(R.id.editHost);
        		TextView editPort = (TextView) findViewById(R.id.editPort);
        		TextView editMaxSp = (TextView) findViewById(R.id.editMaxSpeed);
        		TextView editMaxTu = (TextView) findViewById(R.id.editMaxRotsp);
        		CheckBox checkAssist = (CheckBox) findViewById(R.id.enableDriverAssist);
        		
        		ControlClient client = ControlClient.getInstance();
        		client.config(
        				editHost.getText().toString(),
        				editPort.getText().toString(),
        				editMaxSp.getText().toString(),
        				editMaxTu.getText().toString(),
        				checkAssist.isChecked());
        		
        		startActivity(new Intent(ConnectActivity.this, ControlActivity.class));
        	}
        });
        
    	Log.v(TAG, "onCreate");
	}
	
	
}
