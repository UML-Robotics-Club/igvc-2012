package net.ferrus.ReCo;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class ConnectActivity extends Activity {
	private static final String TAG = "ConnectActivity";

	@Override
    public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
        setContentView(R.layout.connect);

    	Button button = (Button)findViewById(R.id.connectButton);
        button.setOnClickListener(new OnClickListener() {
        	public void onClick(View vv) {
        		TextView editHost = (TextView) findViewById(R.id.editHost);
        		TextView editPort = (TextView) findViewById(R.id.editPort);
        		
        		ControlClient client = ControlClient.getInstance();
        		client.connect(editHost.getText().toString(), editPort.getText().toString());
        		
        		startActivity(new Intent(Intent.ACTION_RUN));
        	}
        });
        
    	Log.v(TAG, "onCreate");
	}
	
	
}
