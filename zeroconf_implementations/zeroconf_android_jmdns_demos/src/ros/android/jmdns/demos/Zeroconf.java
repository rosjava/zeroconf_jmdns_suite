package ros.android.jmdns.demos;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public class Zeroconf extends Activity
{
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
//        TextView tv = (TextView) findViewById(R.id.mytextview);
//		tv.setText("Dude is babbling.");
        TextView tv = new TextView(this);
		tv.setText("Dude is babbling.");
        setContentView(tv);
    }
}
