import time
import rospy
import rosservice
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

def rosMover():
	service_list=rosservice.get_service_list()
	print service_list
	rospy.wait_for_service('/gazebo/get_model_state');
	print "finished waiting for getter service..."
	get_model_srv=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
	rospy.wait_for_service('/gazebo/set_model_state')
	print "finished waiting for setter service..."
	set_model=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
	req=SetModelStateRequest()
	req.model_state.model_name='piano2'
	req.model_state.pose.position.x=4
	req.model_state.pose.position.y=9
	req.model_state.pose.position.z=2
	set_model(req)
	return

def rosViewer():
	service_list=rosservice.get_service_list()
	print service_list
	rospy.wait_for_service('/gazebo/get_model_state');
	print "finished waiting for service..."
	get_model_srv=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
	model=GetModelStateRequest()
	model.model_name='piano2'
	while not rospy.is_shutdown():
		result=get_model_srv(model)
		print '[',result.pose.position.x,',',result.pose.position.y,',',result.pose.position.z,']'
		time.sleep(1)
	

if __name__=="__main__":
	print "starting..."
	rosMover()

