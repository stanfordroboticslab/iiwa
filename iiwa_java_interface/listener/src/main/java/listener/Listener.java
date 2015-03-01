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

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import java.lang.StringBuilder;

import IIWA.IIWAMsg;
import listener.MsgData;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class Listener extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("iiwa/listener");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    Subscriber<IIWAMsg> subscriber = connectedNode.newSubscriber("iiwa/command", IIWAMsg._TYPE);
    subscriber.addMessageListener(new MessageListener<IIWAMsg>() {
      @Override
	      public void onNewMessage(IIWAMsg message) {
					MsgData.setValues(message.getIsJointControl(),
	                                  message.getCartPosition(),
									  message.getCartOrientation(),
									  message.getJointAngles(),
									  message.getCartPositionStiffness(),
									  message.getCartOrientationStiffness(),
									  message.getJointStiffness());
      }
    });
  }
}
