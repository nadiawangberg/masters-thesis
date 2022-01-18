import numpy as np

#Quaterion functions
def quaternion_to_euler(quaternion):
    quaternion_squared = quaternion ** 2
    phi = np.arctan2(2*quaternion[3]*quaternion[2]+2*quaternion[0]*quaternion[1],quaternion_squared[0] - quaternion_squared[1] - quaternion_squared[2] + quaternion_squared[3])
    theta = np.arcsin(2*(quaternion[0]*quaternion[2]-2*quaternion[1]*quaternion[3]))
    psi =  np.arctan2(2*quaternion[1]*quaternion[2]+2*quaternion[0]*quaternion[3],quaternion_squared[0] + quaternion_squared[1] - quaternion_squared[2] - quaternion_squared[3])

    euler_angles = np.array([phi,theta,psi])
    return euler_angles

def ros_quat_to_yaw(quat): 
    #assumes rot around z only
    q_np = np.array([quat.w, quat.x, quat.y, quat.z])
    euler_angles = quaternion_to_euler(q_np)
    return euler_angles[2]

def angleaxis_to_quat(axis, angle):
    ax,ay,az = axis
    qx = ax*np.sin(angle/2)
    qy = ay*np.sin(angle/2)
    qz = az*np.sin(angle/2)
    qw = np.cos(angle/2)

    return [qw,qx,qy,qz]