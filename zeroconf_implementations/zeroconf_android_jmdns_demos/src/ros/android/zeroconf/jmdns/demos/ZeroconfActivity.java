package ros.android.zeroconf.jmdns.demos;


import java.io.IOException;
import java.lang.Thread;
import javax.jmdns.JmmDNS;
import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public class ZeroconfActivity extends Activity {

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
    	// gui
        super.onCreate(savedInstanceState);
        TextView tv = new TextView(this);
		tv.setText("Dude is babbling.");
        setContentView(tv);

        // zeroconf
        Zeroconf browser = new Zeroconf(JmmDNS.Factory.getInstance());
		try {
    		Thread.sleep(1000L);
	    } catch (InterruptedException e) { e.printStackTrace(); }
        while(true) {
    		try {
    			browser.listDiscoveredServices();
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
        }
    }
    

}
