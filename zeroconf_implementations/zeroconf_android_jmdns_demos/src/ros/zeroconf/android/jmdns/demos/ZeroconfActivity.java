package ros.zeroconf.android.jmdns.demos;

import java.lang.Thread;
import java.util.List;

import javax.jmdns.ServiceInfo;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;
import ros.zeroconf.android.jmdns.Zeroconf;

//private class DiscoveryTask extends AsyncTask<Zeroconf, String, void> {
//
//	protected void doInBackground(Zeroconf zeroconf) {
//        int count = urls.length;
//        long totalSize = 0;
//        for (int i = 0; i < count; i++) {
//            totalSize += Downloader.downloadFile(urls[i]);
//            publishProgress((int) ((i / (float) count) * 100));
//        }
//    }
//
//    protected void onProgressUpdate(Integer... progress) {
//        setProgressPercent(progress[0]);
//    }
//
//    protected void onPostExecute(Long result) {
//        showDialog("Downloaded " + result + " bytes");
//    }
//}
/**
 * This test does : 
 * 
 * 1) listens for _ros-master._tcp services
 * 2) every second for 10 seconds it will display currently found services
 * 3) deactivate listeners
 * 4) publish a _ros-master._tcp service (DudeMaster)
 * 5) wait 10 seconds
 * 6) remove all services
 * 
 * Best way to use it is to use avahi to publish/browse on the other end, in oen shell:
 * 
 * > avahi-publish -s ConcertMaster _ros-master._tcp 8883
 * 
 * In another window:
 * 
 * > avahi-browse -r _ros-master._tcp
 * 
 */
public class ZeroconfActivity extends Activity {

	private Zeroconf zeroconf;
	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
    	// gui
        super.onCreate(savedInstanceState);
        TextView tv = new TextView(this);
		tv.setText("Dude is babbling.");
        setContentView(tv);
        
        zeroconf = new Zeroconf();
        
        // adb logcat zeroconf:I *:S
        android.util.Log.i("zeroconf","*********** Zeroconf Listener Test **************");
        // adb logcat System.out:I *:S
		// System.out.println("************ Zerconf Listener Test ************");
        zeroconf.addListener("_ros-master._tcp","local");
        int i = 0;
        while( i < 10 ) {
    		try {
    			android.util.Log.i("zeroconf","************ Discovered Services ************");
    			// System.out.println("************ Discovered Services ************");
    			List<ServiceInfo> service_infos = zeroconf.listDiscoveredServices();
    			for ( ServiceInfo service_info : service_infos ) {
	        		zeroconf.display(service_info);
    			}
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
    		++i;
        }
        zeroconf.removeListener("_ros-master._tcp","local");
        
        android.util.Log.i("zeroconf","*********** Zeroconf Publisher Test **************");
        //System.out.println("*********** Zeroconf Publisher Test **************");
        zeroconf.addService("DudeMaster", "_ros-master._tcp", "local", 8888, "Dude's test master");
    }
    
    @Override
    public void onDestroy() {
    	android.util.Log.i("zeroconf","*********** Zeroconf Destroy **************");
    	//System.out.println("*********** Zeroconf Destroy **************");
		zeroconf.removeAllServices();
		super.onDestroy();
    }
}
