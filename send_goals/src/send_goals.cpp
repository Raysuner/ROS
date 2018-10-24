#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<std_msgs/String.h>
#include<string>
#include<sstream>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SendGoals{
private:
ros::NodeHandle n_;
ros::Publisher tts2voice_pub_;
ros::Subscriber keyword_sub_;
MoveBaseClient ac;
move_base_msgs::MoveBaseGoal kitchengoal;
move_base_msgs::MoveBaseGoal livingroomgoal;
move_base_msgs::MoveBaseGoal bedroomgoal;
move_base_msgs::MoveBaseGoal diningroomgoal;

public:
SendGoals():ac("move_base",true){
tts2voice_pub_=n_.advertise<std_msgs::String>("/speak_string" ,5);
keyword_sub_=n_.subscribe("/cmd_output",1,&SendGoals::sendGoalsCallback,this);

kitchengoal.target_pose.header.frame_id = "/map";
kitchengoal.target_pose.header.stamp = ros::Time::now();
kitchengoal.target_pose.pose.position.x = 4.5;
kitchengoal.target_pose.pose.position.y = 0.83;
kitchengoal.target_pose.pose.orientation.w = 1.0;
kitchengoal.target_pose.pose.orientation.z = 0.8;
kitchengoal.target_pose.pose.orientation.x = 0;
kitchengoal.target_pose.pose.orientation.y = 0;


livingroomgoal.target_pose.header.frame_id = "/map";
livingroomgoal.target_pose.header.stamp = ros::Time::now();
livingroomgoal.target_pose.pose.position.x = -1.75;
livingroomgoal.target_pose.pose.position.y = 1.5;
livingroomgoal.target_pose.pose.orientation.w = 1.0;


bedroomgoal.target_pose.header.frame_id = "/map";
bedroomgoal.target_pose.header.stamp = ros::Time::now();
bedroomgoal.target_pose.pose.position.x = 1.27;
bedroomgoal.target_pose.pose.position.y = 1.68;
bedroomgoal.target_pose.pose.orientation.w = 1.0;


diningroomgoal.target_pose.header.frame_id = "/map";
diningroomgoal.target_pose.header.stamp = ros::Time::now();
diningroomgoal.target_pose.pose.position.x = -1.75;
diningroomgoal.target_pose.pose.position.y = 4.38;
diningroomgoal.target_pose.pose.orientation.w = 1.0;

}



void sendGoalsCallback(const std_msgs::String cmdin){

move_base_msgs::MoveBaseGoal firstgoal;
move_base_msgs::MoveBaseGoal secondgoal;
string goal=cmdin.data;
char p[8];
int i;
for( i=0;i<goal.length();i++)
p[i] = goal[i];


    std_msgs::String error1_msg;
    std_msgs::String ok1_msg;
    std_msgs::String ok2_msg;
        
        
        
        
        std_msgs::String ss4_msg;
        std_msgs::String ss5_msg;
        std_msgs::String ss6_msg;
        std_msgs::String ss7_msg;
        std_msgs::String ss8_msg;
        std_msgs::String ss9_msg;
        std_msgs::String ss10_msg;
        std_msgs::String ss11_msg;


    std::stringstream ss1;
    std::stringstream ss2;
    std::stringstream ss3;
    std::stringstream ss4;
    std::stringstream ss5;
    std::stringstream ss6;
    std::stringstream ss7;
    std::stringstream ss8;
    std::stringstream ss9;
    std::stringstream ss10;
    std::stringstream ss11;
    
    
    ss1 << "对不起，我没明白你的意思，您要我干什么 " ;
    ss2 << "很高兴为您服务" ;
    ss3<<"好的";
    ss4<<"我将先去厨房";
    ss5<<"我将先去卧室";
    ss6<<"我将先去餐厅";
    ss7<<"我将先去客厅";
    ss8<<"现在我将去厨房";
     ss9<<"现在我将去卧室";
     ss10<<"现在我将去餐厅";
     ss11<<"现在我将去客厅";
     
        
    ok1_msg.data = ss2.str();
    ok2_msg.data = ss3.str();
    error1_msg.data = ss1.str();
    ss4_msg.data=ss4.str();
    ss5_msg.data=ss5.str();
    ss6_msg.data=ss6.str();
    ss7_msg.data=ss7.str();
    ss8_msg.data=ss8.str();
    ss9_msg.data=ss9.str();
    ss10_msg.data=ss10.str();
    ss11_msg.data=ss11.str();


if(p[0]=='F') tts2voice_pub_.publish(error1_msg);
else{
tts2voice_pub_.publish(ok2_msg);
if(p[0]=='k'&&p[1]=='i'&&p[2]=='t') {firstgoal=kitchengoal; tts2voice_pub_.publish(ss4_msg);}
if(p[0]=='b'&&p[1]=='e'&&p[2]=='d') {firstgoal=bedroomgoal; tts2voice_pub_.publish(ss5_msg);}
if(p[0]=='d'&&p[1]=='i'&&p[2]=='n') {firstgoal=diningroomgoal; tts2voice_pub_.publish(ss6_msg);}
if(p[0]=='l'&&p[1]=='i'&&p[2]=='v') {firstgoal=livingroomgoal; tts2voice_pub_.publish(ss7_msg);}

if(p[4]=='k'&&p[5]=='i'&&p[6]=='t') {secondgoal=kitchengoal; tts2voice_pub_.publish(ss8_msg);}
if(p[4]=='b'&&p[5]=='e'&&p[6]=='d') {secondgoal=bedroomgoal; tts2voice_pub_.publish(ss9_msg);}
if(p[4]=='d'&&p[5]=='i'&&p[6]=='n') {secondgoal=diningroomgoal; tts2voice_pub_.publish(ss10_msg);}
if(p[4]=='l'&&p[5]=='i'&&p[6]=='v') {secondgoal=livingroomgoal; tts2voice_pub_.publish(ss11_msg);}

//while(!ac.waitForServer(ros::Duration(5.0))){
   // ROS_INFO("Waiting for the move_base action server to come up");
 // }
}
ac.sendGoal(firstgoal);
ac.waitForResult();
if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
tts2voice_pub_.publish(ok1_msg);

//ac.sendGoal(secondgoal);
//ac.waitForResult();
 // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   // tts2voice_pub_.publish(ok1_msg);
}
}

};


int main(int argc, char** argv){
ros::init(argc,argv,"move_cmd");
SendGoals goal;
ros::spin();
  return 0;
}
