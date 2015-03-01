/*
 * Copyright (C) 2014 Kenji Hata.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
package listener;

import listener.RosListenerWrap;
import listener.Talker;

import java.util.Arrays;

public class TestRunnable {

	public static void main(String args[]) {
		/*
		String listenerArgs[] = {"listener.Listener"};
		String talkerArgs[] = {"listener.Talker"};
		RosListenerWrap wrapListener = new RosListenerWrap(listenerArgs);
		RosListenerWrap wrapTalker = new RosListenerWrap(talkerArgs);

		wrapListener.run();
		wrapTalker.run();

		Talker.waitUntilStarted();

		System.out.println("ready");

		MsgData.setNumJoints(5);

		double[] c_pos = {900, 901, 902};
		double[] c_ori = {1,2,3,4,5,6,7,8,9};
		double[] c_force = {8,7,6,5,4,3};
		double[] c_stiff = {2000,2001,2002,200,201,202};

		double[] j_pos = {5,10,15,20,25};
		double[] j_tor = {6,12,18,24,30};


		double[] c_pos_in = new double[3];
		double[] c_ori_in = new double[9];
		double[] c_stiff_in = new double[6];

		double[] j_pos_in = new double[5];

		boolean isJointControl = true;

		int num = 0;

		try {
			while(true) {
				num++;

				Talker.publish(true,c_pos,c_ori,c_force,c_stiff,j_pos,j_tor,System.nanoTime());
				System.out.println("published");
				Thread.sleep(1000);
				MsgData.getValues(isJointControl,c_pos_in, c_ori_in, c_stiff_in,j_pos_in);
				System.out.println(isJointControl);
				System.out.println(Arrays.toString(c_pos_in));
				System.out.println(Arrays.toString(c_ori_in));
				System.out.println(Arrays.toString(c_stiff_in));
				System.out.println(Arrays.toString(j_pos_in));
			}
		}
		catch(InterruptedException e)
		{System.out.println("exception occured");}
		*/
	}
}
