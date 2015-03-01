package listener;

import java.util.ArrayList;
import java.util.Arrays;

/*
 * MsgData is the interface for passing recieved values between a ros listener
 * thread and the main control loop. These tend to be commanded values.
 */
public class MsgData {
	public static int nJoints = 7;

	public static boolean isJointControl = false;
	public static double[] cart_position = new double[3];
	public static double[] cart_orientation = new double[9];
	public static double[] cart_positionStiffness = new double[3];
	public static double[] cart_orientationStiffness = new double[3];
	public static double[] joint_angle = new double[nJoints];
	public static double[] joint_stiffness = new double[nJoints];

	public static void setNumJoints(int num) {
		if(num < 1) return;
		synchronized(MsgData.class) {
			nJoints = num;
			joint_angle = new double[nJoints];
			joint_stiffness = new double[nJoints];
		}
	}

	public static void setValues(boolean is_j_control, double[] c_pos, double[] c_ori, double[] j_ang, double[] c_posStiff,
                              double[] c_oriStiff, double[] j_stiff) {
		synchronized(MsgData.class) {
			isJointControl = is_j_control;
			if(c_pos != null && c_pos.length == 3) System.arraycopy(c_pos, 0, cart_position, 0, 3);
			if(c_ori != null && c_ori.length == 9) System.arraycopy(c_ori, 0, cart_orientation, 0, 9);
			if(c_posStiff != null && c_posStiff.length == 3) System.arraycopy(c_posStiff, 0, cart_positionStiffness, 0, 3);
			if(c_oriStiff != null && c_oriStiff.length == 3) System.arraycopy(c_oriStiff, 0, cart_orientationStiffness, 0, 3);

			if(j_ang != null && j_ang.length == nJoints) System.arraycopy(j_ang, 0, joint_angle, 0, nJoints);
			if(j_stiff != null && j_stiff.length == nJoints) System.arraycopy(j_stiff, 0, joint_stiffness, 0, nJoints);
		}
	}

	public static boolean getValues(boolean is_j_control, double[] c_pos, double[] c_ori, double[] j_ang, 
                              double[] c_posStiff, double[] c_oriStiff, double[] j_stiff) {
		synchronized(MsgData.class) {
			is_j_control = isJointControl;
			if(c_pos.length == 3) System.arraycopy(cart_position, 0, c_pos, 0, 3);
			if(c_ori.length == 9) System.arraycopy(cart_orientation, 0, c_ori, 0, 9);
			if(c_posStiff.length == 3) System.arraycopy(cart_positionStiffness, 0, c_posStiff, 0, 3);
			if(c_oriStiff.length == 3) System.arraycopy(cart_orientationStiffness, 0, c_oriStiff, 0, 3);

			if(j_ang.length == nJoints) System.arraycopy(joint_angle, 0, j_ang, 0, nJoints);
			if(j_stiff.length == nJoints) System.arraycopy(joint_stiffness, 0, j_stiff, 0, nJoints);
		}
		return isJointControl;
	}
}
