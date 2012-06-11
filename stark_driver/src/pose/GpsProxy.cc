#include <iostream>
#include <iomanip>

using std::cout;
using std::cerr;
using std::endl;

#include "GpsProxy.hh"
#include "CalcUTM.hh"

GpsProxy::GpsProxy()
{
    gps = new gpsmm;
    gps->open("localhost", DEFAULT_GPSD_PORT);
    gps->stream(WATCH_ENABLE|WATCH_JSON);
    
    gps0_ekf.SetProcGain(10.2, 10.2, 10.2);
    gps0_ekf.SetProcVar(0.01, 0.01, 0.01);
    gps0_ekf.SetMeasGain(1.0, 1.0, 1.0);
    gps0_ekf.SetMeasVar(8.06, 8.06, 0.2);

    gps1_ekf.SetProcGain(10.2, 10.2, 10.2);
    gps1_ekf.SetProcVar(0.01, 0.01, 0.01);
    gps1_ekf.SetMeasGain(1.0, 1.0, 1.0);
    gps1_ekf.SetMeasVar(8.21, 8.21, 0.2);

    fout_gps.open("raw_gps.txt");
    fout_kf_gps.open("filt_gps.txt");
    fout_encoder_pos.open("encoder_pos.txt");
    fout_gps << std::setprecision(9);
    fout_kf_gps << std::setprecision(9);
    fout_encoder_pos << std::setprecision(9);

    encoder_x_pos = 0;
    encoder_y_pos = 0;
}

GpsProxy::~GpsProxy()
{
    gps->close();
    delete gps;
    fout_gps.close();
    fout_kf_gps.close();
    fout_encoder_pos.close();
}

void
GpsProxy::update()
{
    static ros::Time last_gps0 = ros::Time::now();
    static ros::Time last_gps1 = ros::Time::now();

    struct gps_data_t* msg;
    RobotEKF::Vector u(3);
    RobotEKF::Vector z(3);
    RobotEKF::Vector x(3);
    RobotEKF* ekf = NULL;

    u(3) = 0; // no compass
    z(3) = 0;
    msg = gps->poll();
    z(1) = utm_east(msg->fix.latitude, msg->fix.longitude); 
    z(2) = utm_north(msg->fix.latitude, msg->fix.longitude);
    std::string path(msg->dev.path);
    if(!path.compare("/dev/ttyGPS0"))
      ekf = &gps0_ekf;
    else if(!path.compare("/dev/ttyGPS1"))
      ekf = &gps1_ekf;
    fout_gps << path << " " << z(1) << " " << z(1) << endl;
    if(ekf != NULL && !(z(1) != z(1) || z(2) != z(2))){ // test for nan (will not work with fastmath enabled)
      ros::Time new_time = ros::Time::now();
      ros::Duration dt;
      tf::StampedTransform odom_tf;

      try{
	if(ekf == &gps0_ekf){
	  if((new_time - ros::Duration(3.0)) > last_gps0)
	    last_gps0 = new_time - ros::Duration(3.0);
	  
	  tf_listener.waitForTransform("/base_link", last_gps0,
				       "/base_link", new_time,
				       "/odom", ros::Duration(1.0));
	  tf_listener.lookupTransform("/base_link", last_gps0,
				      "/base_link", new_time,
				      "/odom", odom_tf);
	  dt = new_time - last_gps0;
	  last_gps0 = new_time;	
	}
	else{
	  if((new_time - ros::Duration(3.0)) > last_gps1)
	    last_gps1 = new_time - ros::Duration(3.0);
	  
	  tf_listener.waitForTransform("/base_link", last_gps1,
				       "/base_link", new_time,
				       "/odom", ros::Duration(1.0));
	  tf_listener.lookupTransform("/base_link", last_gps1,
				      "/base_link", new_time,
				      "/odom", odom_tf);
	  dt = new_time - last_gps1;
	  last_gps1 = new_time;
	}
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }
      
      u(1) = odom_tf.getOrigin().x();
      u(2) = odom_tf.getOrigin().y();
      if(!(u(1) != u(1) || u(2) != u(2))){
	if(ekf->isInitialized()){
	  ekf->step(u,z);
	  x = ekf->getX();
	  fout_kf_gps << path << " " << x(1) << " " << z(1) 
		      << " " << x(2) << " " << z(2) << std::endl;
	}
	else
	  ekf->InitVals(z(1), z(2), z(3), u(1), u(2), u(3));
	fout_encoder_pos << dt.toSec() << " " << u(1) << " " 
			 << u(2) << std::endl;
      }

      /*

      try{
	if(!path.compare("/dev/ttyGPS0")){
	  
	  // New gps0 data.
	  ros::Time new_time = ros::Time::now();
	  if((new_time - ros::Duration(3.0)) > last_gps0)
	    last_gps0 = new_time - ros::Duration(3.0);
	  
	  tf_listener.waitForTransform("/base_link", last_gps0,
				       "/base_link", new_time,
				       "/odom", ros::Duration(1.0));
	  tf_listener.lookupTransform("/base_link", last_gps0,
				      "/base_link", new_time,
				      "/odom", odom_tf);
	  last_gps0 = new_time;				       
	  
	  u(1) = odom_tf.getOrigin().x();
	  u(2) = odom_tf.getOrigin().y();
	  if(!(u(1) != u(1) || u(2) != u(2))){
	    if(gps0_ekf.isInitialized()){
	      gps0_ekf.step(u,z); // correct
	      //gps0_ekf.step(z,z); // incorrect
	      x = gps0_ekf.getX();
	      fout_kf_gps << path << " " << x(1) << " " << z(1) 
			  << " " << x(2) << " " << z(2) << std::endl;
	      encoder_x_pos += u(1); // fubar
	      encoder_y_pos += u(2);
	      fout_encoder_pos << u(1) << " " 
			       << u(2) << std::endl;
	    }
	    else{
	      gps0_ekf.InitVals(z(1), z(2), z(3), u(1), u(2), u(3));
	      encoder_x_pos = z(1);
	      encoder_y_pos = z(2);
	      fout_encoder_pos << u(1) << " " 
			       << u(2) << std::endl;
	    }
	  }
	  else if(!path.compare("/dev/ttyGPS1")){
	    
	    // new GPS1 data
	    ros::Time new_time = ros::Time::now();
	    if((new_time - ros::Duration(3.0)) > last_gps1)
	      last_gps1 = new_time - ros::Duration(3.0);
	    
	    tf_listener.waitForTransform("/base_link", last_gps1,
					 "/base_link", new_time,
					 "/odom", ros::Duration(1.0));
	    tf_listener.lookupTransform("/base_link", last_gps1,
					 "/base_link", new_time,
					"/odom", odom_tf);
	    last_gps1 = new_time;
	    
	    u(1) = odom_tf.getOrigin().x();
	    u(2) = odom_tf.getOrigin().y();
	    if(!(u(1) != u(1) || u(2) != u(2))){
	      if(gps1_ekf.isInitialized()){
		gps1_ekf.step(u,z); // correct
		//gps1_ekf.step(z,z); // incorrect
		x = gps1_ekf.getX();
		fout_kf_gps << path << " " << x(1) << " " << z(1) 
			    << " " << x(2) << " " << z(2) << std::endl;
		if(gps0_ekf.isInitialized()){
		  encoder_x_pos += u(1); // fubar
		  encoder_y_pos += u(2);
		  fout_encoder_pos << u(1) << " " 
				   << u(2) << std::endl;
		}
	      }
	      else
		gps1_ekf.InitVals(z(1), z(2), z(3), u(1), u(2), u(3));
	    }
	  }
	}
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	}*/
    }
#if 0
    cout << "Polled dev: " << path
         << "; status =  " << msg->status 
         << "; sats = " << msg->satellites_used <<endl;
#endif

    if (msg->status && path != "") {
        if (data.find(path) == data.end()) {
            cout << "New device: " << path << endl;
        }

        data[path] = *msg;
    }
    else {
        cout << "No lock for device: " << path << endl;
    }
}

gps_pos_t
GpsProxy::position()
{
    gps_pos_t pos;

    int total_sats = 0;
    double lat = 0.0;
    double lon = 0.0;

    std::map<std::string,struct gps_data_t>::iterator it;
    for (it = data.begin(); it != data.end(); ++it) {
        struct gps_data_t dd = it->second;
        int sats = dd.satellites_used;

        if (sats > 0) {
            total_sats += sats;
            lat += sats * dd.fix.latitude;
            lon += sats * dd.fix.longitude;
        }
    }

    lat /= total_sats;
    lon /= total_sats;
    
    if(gps0_ekf.isInitialized() && gps1_ekf.isInitialized()){
      RobotEKF::Vector x0(3);
      RobotEKF::Vector x1(3);
      x0 = gps0_ekf.getX();
      x1 = gps1_ekf.getX();
      pos.utm_e = (x0(1) + x1(1)) / 2.0;
      pos.utm_n = (x0(2) + x1(2)) / 2.0;
    }
    else{
      pos.utm_n = utm_north(lat, lon);
      pos.utm_e = utm_east(lat, lon);
    }
    pos.lat   = lat;
    pos.lon   = lon;

    return pos;
}

