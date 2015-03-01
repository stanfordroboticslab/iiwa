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

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.ros.exception.RosRuntimeException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import listener.Listener;

public class RosListenerWrap implements Runnable {

	CommandLineLoader loader;
	NodeMain nodeMain;
	NodeMainExecutor nodeMainExecutor;

	public RosListenerWrap(String argv[])
	{
		loader = new CommandLineLoader(Lists.newArrayList(argv));
		nodeMain = null;
		nodeMainExecutor = null;
	}

	public void run() {
		String nodeClassName = loader.getNodeClassName();

		System.out.println("Loading node class: " + loader.getNodeClassName());
		NodeConfiguration nodeConfiguration = loader.build();

		nodeMain = null;
		try {
			nodeMain = loader.loadClass(nodeClassName);
		} catch (ClassNotFoundException e) {
			throw new RosRuntimeException("Unable to locate node: " + nodeClassName, e);
		} catch (InstantiationException e) {
			throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName, e);
		} catch (IllegalAccessException e) {
			throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName, e);
		}

		Preconditions.checkState(nodeMain != null);
		nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
		nodeMainExecutor.execute(nodeMain, nodeConfiguration);
	}

	public void shutdown() {
		if(nodeMainExecutor != null) {
			if(nodeMain != null) {
				nodeMainExecutor.shutdownNodeMain(nodeMain);
			}
		}
	}
}
