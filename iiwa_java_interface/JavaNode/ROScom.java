package ROScomPkg;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp; 

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.IRotation;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.DirectServo;
import com.kuka.roboticsAPI.motionModel.IServoRuntime;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

import listener.RosListenerWrap;
import listener.Talker;
import listener.MsgData;

/**************************************************************************
***   Copyright (c) 2014 K. Go and M. Khansari, Stanford Robotics Lab,  ***
***                      Stanford University, USA                       ***
***************************************************************************
*
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Stanford University nor the name of the author may
#       be used to endorse or promote products derived from this software without
#       specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY KEEGAN GO and MOHAMMAD KHANSARI ''AS IS'' AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL MOHAMMAD KHANSARI BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* To get latest upadate of the software please visit:
*                          http://cs.stanford.edu/people/khansari
*
* Please send your feedbacks or questions to:
* 							keegango_at_stanford.edu
*                           khansari_at_cs.stanford.edu
***************************************************************************/

public class ROScom extends RoboticsAPIApplication
{
	static int nbJointsIIWA = 7;
	
	class RobotState {
		boolean is_joint_control;
		double[] position;
		double[] force;
		double[] posStiffness;
		double[] orientation;
		double[] moment;
		double[] oriStiffness;
		double[] jointAngles;
		double[] jointTorques;
	    double[] jointStiffness;
	    
	    double[] posStiff_last;
	    double[] oriStiff_last;
	    double[] jointStiff_last;
	    
	    int		 nbJoints;
	    double	 time;
	    
	    public RobotState(int num_joints) {
	    	nbJoints = num_joints;
	    	
	        position = new double[3];
	        force = new double[3];
	        posStiffness = new double[3];
	        
	        orientation = new double[9];
	        moment = new double[3];
	        oriStiffness = new double[3];
	        
	        // Joint position and forces
	        jointAngles = new double[num_joints];
	        jointTorques = new double[num_joints];
	        jointStiffness = new double[num_joints];
	        
	        // Default for robot is {2000,2000,2000,200,200,200}
	        for(int i = 0; i < 3; ++i) posStiffness[i] = 2000;
	        for(int i = 0; i < 3; ++i) oriStiffness[i] = 200;
	        for(int i = 0; i < num_joints; ++i) jointStiffness[i] = 1000;
	        
	        posStiff_last = new double[3];
	        oriStiff_last = new double[3];
	        jointStiff_last = new double[num_joints];
	        System.arraycopy(posStiffness, 0, posStiff_last, 0, 3);
	        System.arraycopy(oriStiffness, 0, oriStiff_last, 0, 3);
	        System.arraycopy(jointStiffness, 0, jointStiff_last, 0, num_joints);
	    }
	}
	
	RobotState currentData = new RobotState(nbJointsIIWA);
	RobotState receiveData = new RobotState(nbJointsIIWA);
	
    // members
    private LBR _theLbr;
    private PhysicalObject _toolAttachedToLBR;
    private Tool 		tool;
    
    private RosListenerWrap wrapTalker;
	private RosListenerWrap wrapListener;
    
	/*
	 * SETTINGS
	 */
    boolean do_interpolation = false;
    double max_cart_offset_allowed = 100;
    // Set communication addresses (CHANGE AS NEEDED)
	String my_ip = "192.168.168.2";
	String my_master = "http://192.168.168.1:11311";
    
    /*
     * The rate at which stiffness should be updated
     * The rate is given as the number of update ticks
     */
    final int stiffness_update_rate = 5;

    @Override
    public void initialize()
    {
    	
        // Locate the "first" Lightweight Robot in the system
        _theLbr = ServoMotionUtilities.locateLBR(getContext());

        // FIXME: Set proper Weights or use the plugin feature
        double translationOfTool[] = { 0, 0, 0 };
        /*//these values are identified for the allegro hand
        double mass = 1.2;
        double centerOfMassInMillimeter[] = { 3, 15, 237 };*/
        /*double mass = 0.0;
        double centerOfMassInMillimeter[] = { 0, 0, 0 };*/

        double mass = 0.0;
        double centerOfMassInMillimeter[] = { 0, 0, 0 };
        _toolAttachedToLBR = ServoMotionUtilities.createTool(_theLbr,
                    "SimpleJointMotionSampleTool", translationOfTool, mass,
                    centerOfMassInMillimeter);

        /*
        tool = getApplicationData().createFromTemplate("MetallicGripper");
        tool.attachTo(_theLbr.getFlange());
        */

		// Set the data to read the correct number of joints
        MsgData.setNumJoints(nbJointsIIWA);
        
        /*
		 * Start ros nodes
		 */
		
		String talkerArgs[] = {
				"listener.Talker",
				"__ip:=" + my_ip,
				"__master:=" + my_master
		};
		String listenerArgs[] = {
				"listener.Listener",
				"__ip:=" + my_ip,
				"__master:=" + my_master
		};
		wrapListener = new RosListenerWrap(listenerArgs);
		wrapListener.run();
		wrapTalker = new RosListenerWrap(talkerArgs);
		wrapTalker.run();
		
		// Sleeps until talker is ready, or 5 seconds has passed
		// (should have a check for the second case)
		Talker.waitUntilStarted();
		System.out.println("Registered and ready");
    }

    /**
     * Move to an initial Position 
     * WARNING: MAKE SURE THAT the pose is collision free
     */
    public void moveToInitialPosition()
    {
        _toolAttachedToLBR.move(
                ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));

        if (DirectServo.validateForImpedanceMode(_toolAttachedToLBR) != true)
        {
            System.out.println("Validation of Torque Model failed - correct your mass property settings");
            System.out.println("DirectServo will be available for position controlled mode only, until validation is performed");
        }
    }

    /*
     * Methods to convert between transform and array
     */
    private void fillOriArray(Matrix mat, double[] arr) {
    	for(int i = 0; i < 9; ++i) arr[i] = mat.get(i/3, i%3);
    }
    private void fillPosArray(Vector vec, double[] arr) {
    	for(int i = 0; i < 3; ++i) arr[i] = vec.get(i);
    }
    private Matrix extractOriArray(double[] arr) {
    	return Matrix.ofRowFirst(arr[0], arr[1], arr[2],
			       arr[3], arr[4], arr[5],
			       arr[6], arr[7], arr[8]);
    }
    private Vector extractPosArray(double[] arr) {
    	return Vector.of(arr[0], arr[1], arr[2]);
    }
    
    /*
     * Methods to send and receive data
     */
    double[] empty_arr3 = new double[3];
    double[] empty_arrJoint = new double[nbJointsIIWA];
    private void publishData(Frame frame, double[] jointPosition, double[] jointTorques, double[] tcpForce) {
    	Transformation cur_transform = frame.transformationFromWorld();
    	Matrix msrRot = cur_transform.getRotationMatrix();
        Vector msrPos = cur_transform.getTranslation();
        fillOriArray(msrRot,currentData.orientation);
        fillPosArray(msrPos,currentData.position);
        
        Talker.publish(receiveData.is_joint_control,
        		currentData.position,
        		tcpForce,
        		receiveData.posStiffness, //why?
        		currentData.orientation,
        		empty_arr3, //currentData.moment,
        		receiveData.oriStiffness, //why?
        		jointPosition,
        		jointTorques,
        		empty_arrJoint,
        		System.nanoTime());
    }
    
    private void receiveData()
    {
    	Vector received_position_last = extractPosArray(receiveData.position);
    	
    	receiveData.is_joint_control = MsgData.getValues(receiveData.is_joint_control,
    			receiveData.position,
    			receiveData.orientation,
    			receiveData.jointAngles,
    			receiveData.posStiffness,
    			receiveData.oriStiffness,
    			receiveData.jointStiffness);
    	
    	Vector received_position_cur = extractPosArray(receiveData.position);
    	
    	// If we are not doing interpolation, do a check to ensure the
    	// commanded position is not too far from the current
    	if(!do_interpolation)
    	{
    		// Check if received_position is far from current position
        	// If it is too far, reset to the last commanded position
        	if(received_position_cur.subtract(received_position_last).length() > max_cart_offset_allowed) {
        		fillPosArray(received_position_last, receiveData.position);
        	}
    	}
    }
    
    /*
     * Check if receive_stiffness is different from the last one set
   	 * Only update if is different
     */
    private boolean shouldSetStiffness() {
    	boolean is_same = true;
    	for(int i = 0; i < 3; ++i) {
    		if(receiveData.posStiffness[i] != receiveData.posStiff_last[i]) {
    			is_same = false;
    			break;
    		}
    	}
    	
    	for(int i = 0; i < 3; ++i) {
    		if(receiveData.oriStiffness[i] != receiveData.oriStiff_last[i]) {
    			is_same = false;
    			break;
    		}
    	}
    	
    	for(int i = 0; i < receiveData.nbJoints; ++i) {
    		if(receiveData.jointStiffness[i] != receiveData.jointStiff_last[i]) {
    			is_same = false;
    			break;
    		}
    	}
    	
    	if(is_same) return false;
    	else {
    		System.arraycopy(receiveData.posStiffness, 0, receiveData.posStiff_last, 0, 3);
	        System.arraycopy(receiveData.oriStiffness, 0, receiveData.oriStiff_last, 0, 3);
	        System.arraycopy(receiveData.jointStiffness, 0, receiveData.jointStiff_last, 0, receiveData.nbJoints);
    		return true;
    	}
    }
    
    /*
     * Update stiffness and damping
     */
    private void setStiffness(final CartesianImpedanceControlMode cartImp, 
    		IServoRuntime theDirectServoRuntime) {
    	
    	if (cartImp instanceof CartesianImpedanceControlMode)
        {
    		//System.out.println(receive_stiff[0] + " " + receive_stiff[1] + " " + receive_stiff[2]);
        	// Linear
            cartImp.parametrize(CartDOF.X).setStiffness(receiveData.posStiffness[0]);
            cartImp.parametrize(CartDOF.Y).setStiffness(receiveData.posStiffness[1]);
            cartImp.parametrize(CartDOF.Z).setStiffness(receiveData.posStiffness[2]);
            
            //cartImp.parametrize(CartDOF.X).setDamping(0.5);
            //cartImp.parametrize(CartDOF.Y).setDamping(0.5);
            //cartImp.parametrize(CartDOF.Z).setDamping(0.5);
            
            // Rotational
            cartImp.parametrize(CartDOF.A).setStiffness(receiveData.oriStiffness[0]);
            cartImp.parametrize(CartDOF.B).setStiffness(receiveData.oriStiffness[1]);
            cartImp.parametrize(CartDOF.C).setStiffness(receiveData.oriStiffness[2]);
            
            //cartImp.setNullSpaceStiffness(0.0);
            
            // Send the new Stiffness settings down to the controller
            theDirectServoRuntime.changeControlModeSettings(cartImp);
        }
    }

    /**
     * Main Application Routine
     */
    public void run()
    {
        moveToInitialPosition();
        System.out.println("Robot was moved to the initial configuration.");

        DirectServo aDirectServoMotion = new DirectServo(_theLbr.getCurrentJointPosition());
        aDirectServoMotion.setMinimumTrajectoryExecutionTime(8e-3);

        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        // Start up with impedance control
        _toolAttachedToLBR.moveAsync(aDirectServoMotion.setMode(cartImp));
        
        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime exists AFTER motion command was issued
        IServoRuntime theDirectServoRuntime = aDirectServoMotion.getRuntime();
        
        /*
        NOTE: From Sunrise.OS 1.5 on a new class IDirectServoRuntime was introduced,
        it inherits from IServoRutime, but overloads of setDestination() were moved to this new class.
        Uncomment the following line to fix errors using Sunrise.OS >= 1.5
        */
        //IDirectServoRuntime theDirectServoRuntime = aDirectServoMotion.getRuntime();
        
        /*
        //an attempt to revive joint impedance, but failed
        DirectServo aDirectServoMotionJoint = new DirectServo(_theLbr.getCurrentJointPosition());
        aDirectServoMotionJoint.setMinimumTrajectoryExecutionTime(8e-3);
        final JointImpedanceControlMode    jointImp = new JointImpedanceControlMode(50.0,50.0,50.0,50.0,50.0,50.0,50.0);
        _toolAttachedToLBR.moveAsync(aDirectServoMotionJoint.setMode(jointImp));
        IServoRuntime theDirectServoRuntimeJoint = aDirectServoMotionJoint.getRuntime();
        */

        // Write starting values into memory
        Frame aFrame = theDirectServoRuntime.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
        Transformation cur_transform = aFrame.transformationFromWorld();
        // Get the starting position
        Matrix start_rot = cur_transform.getRotationMatrix();
        Vector start_pos = cur_transform.getTranslation();
        
        double[] joint_start = theDirectServoRuntime.getAxisQMsrOnController().getInternalArray();
        // Set these initial values in the data structures
        fillOriArray(start_rot, currentData.orientation);
        fillPosArray(start_pos, currentData.position);
        fillOriArray(start_rot, receiveData.orientation);
        fillPosArray(start_pos, receiveData.position);
        for(int i = 0; i < nbJointsIIWA; ++i) receiveData.jointAngles[i] = joint_start[i];
        JointPosition j_desired = new JointPosition(theDirectServoRuntime.getAxisQMsrOnController());
        
        MsgData.setValues(false,
        		currentData.position,
        		currentData.orientation,
        		currentData.jointAngles,
        		receiveData.posStiffness, //why?
        		receiveData.oriStiffness, //why?
        		receiveData.jointStiffness //why?
        		);
        
        // Init stiffness
        setStiffness(cartImp, theDirectServoRuntime);
        
        // Print a message of the starting coordinates
        System.out.println("Starting position");
        System.out.println(start_pos);
        System.out.println(start_rot);

        try
        {
        	long time_counter = 0;
        	int loop_counter = 0;
        	
            //long startTimeStamp = System.nanoTime();
        	
            while(true)
            {
            	long time_start_loop = System.nanoTime();
            	
                // Read and format ROS data
                receiveData();

                //check if we are doing joint control
                if(receiveData.is_joint_control)
                {
                	if(receiveData.jointAngles.length == nbJointsIIWA) {
                		for(int i = 0; i < nbJointsIIWA; ++i) j_desired.set(i, receiveData.jointAngles[i]);
                	}
                	theDirectServoRuntime.setDestination(j_desired);
                }
                else //we are doing cartesian impedance control
                {
                	if(shouldSetStiffness()) setStiffness(cartImp, theDirectServoRuntime);
                    
                    Matrix received_orientation = extractOriArray(receiveData.orientation);
                    Vector received_position = extractPosArray(receiveData.position);
                    
                    MatrixTransformation received_transform;
                    
                    Frame curPose = theDirectServoRuntime.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
                    Transformation tToWorld = curPose.transformationFromWorld(); 
                	Vector curPos = tToWorld.getTranslation();
                	
	                if(do_interpolation) {
	                	
		            	double[] delta_pos = new double[3];
		            	double max_delta = 10; 
		            	for(int i = 0; i < 3; ++i) {
		            		double delta = received_position.get(i) - curPos.get(i);
		            		if(delta > max_delta) {
		            			delta = max_delta;
		            		} else if(delta < -max_delta) {
		            			delta = -max_delta;
		            		}
		            		delta_pos[i] = delta; 
		            	}
		            	
		            	Vector delta_vect = extractPosArray(delta_pos);
		            	Vector new_cmd_pos = curPos.add(delta_vect);
		                
		                received_transform = MatrixTransformation.of(
		                		new_cmd_pos,MatrixRotation.of(received_orientation));
	            	}
	                else {
	                	received_transform = MatrixTransformation.of(
		                	received_position,MatrixRotation.of(received_orientation));
	                }
	
	                theDirectServoRuntime.setDestination(received_transform);
                }
                
                // Send update of current pose and sensed torques
                Frame msrPose = theDirectServoRuntime.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
                double tcpForce[] = theDirectServoRuntime.getExtForceMsr();
            	publishData(msrPose,
            			theDirectServoRuntime.getAxisQMsrOnController().getInternalArray(),
            			theDirectServoRuntime.getExtForceMsr(), tcpForce);
                
                loop_counter++;
                
                time_counter += System.nanoTime() - time_start_loop;
                if(loop_counter % 1000 == 0) {
                	
                	/*
                	double extForce[] = theDirectServoRuntime.getExtForceMsr();
                	System.out.println("["+extForce[0]+" "+extForce[1]+" "+extForce[2]+"]");
                	*/

                	time_counter = 0;
                }
            }
        }
        catch (Exception e)
        {
            System.out.println(e);
            e.printStackTrace();
        }

        // /////////////////////////////////////////////////
        // Do or die: print statistics and parameters of the motion

        System.out.println("EXITED!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" + theDirectServoRuntime.toString());
        // Stop the motion
        //theDirectServoRuntime.stopMotion(); //this seems to causes some problems

    }

    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        ROScom app = new ROScom();

        app.runApplication();

    }
}
