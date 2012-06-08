package net.ferrus.ReCo;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.TextView;

public class ConfigActivity extends Activity {
	private static final String TAG = "ConfigActivity";
	
	  public void onCreate(Bundle savedInstanceState) {
			super.onCreate(savedInstanceState);
	        setContentView(R.layout.config);
	        
	        Button button = (Button)findViewById(R.id.configButton); 
	        button.setOnClickListener(new OnClickListener() {
	        	public void onClick(View vv) {
	        		TextView editMaxSp = (TextView) findViewById(R.id.editMaxSpeed);
	        		TextView editMaxTu = (TextView) findViewById(R.id.editMaxRotsp);
	        		CheckBox enAssist  = (CheckBox) findViewById(R.id.enableDriverAssist); 
	        		
	        		ControlClient client = ControlClient.getInstance();
	        		client.updateSettings(
	        				editMaxSp.getText().toString(), 
	        				editMaxTu.getText().toString(),
	        				enAssist.isChecked());
	        		startActivity(new Intent(ConfigActivity.this, ControlActivity.class));
	        	}
	        });
	  }
}