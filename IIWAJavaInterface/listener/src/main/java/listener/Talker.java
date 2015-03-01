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

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.message.Time;

import IIWA.IIWAMsg;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class Talker extends AbstractNodeMain {

	private static boolean has_started = false;

	private static Publisher<IIWAMsg> publisher;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("iiwa/talker");
  }

	public static boolean waitUntilStarted() {
		try {
			synchronized(Talker.class) {
				if(!has_started) Talker.class.wait(5000);
			}
		}
		catch (InterruptedException e)
		{
			return false;
		}
		if(!has_started) return false;
		return true;
	}

	public static void publish(boolean bJointControl,
                                 double[] c_pos,
                                 double[] c_force,
                                 double[] c_posStiff,
                                 double[] c_ori,
								 double[] c_moment,
								 double[] c_oriStiff,
                                 double[] j_ang,
                                 double[] j_torque,
                                 double[] j_stiff,
								 long time) {
		if(has_started)
		{
			IIWAMsg msg = publisher.newMessage();

			msg.setIsJointControl(bJointControl);
			msg.setCartPosition(c_pos);
			msg.setCartForces(c_force);
			msg.setCartPositionStiffness(c_posStiff);
			
			msg.setCartOrientation(c_ori);
			msg.setCartMoments(c_moment);
			msg.setCartOrientationStiffness(c_oriStiff);
			

			msg.setJointAngles(j_ang);
			msg.setJointTorques(j_torque);
			msg.setJointStiffness(j_stiff);

			publisher.publish(msg);
		}
	}

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    publisher = connectedNode.newPublisher("iiwa/state", IIWAMsg._TYPE);
		synchronized(Talker.class) {
			has_started = true;
			Talker.class.notify();
		}
  }
}
