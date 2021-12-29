#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

bool state = true;	// current robot's motors state (default is true)

void stateChanged(const std_msgs::Bool::ConstPtr& pt){
	state = pt->data;
	if (state) ROS_INFO("[estop] robot's motors have been ENABLED");
	else ROS_INFO("[estop] robot's motors have been DISABLED");
}

static struct termios olds, news;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &olds); /* grab old terminal i/o settings */
  news = olds; /* make new settings same as old settings */
  news.c_lflag &= ~ICANON; /* disable buffered i/o */
  news.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &news); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &olds);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}

bool kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return true;
  }
 
  return false;
}

int main(int argc, char** argv){
  
	ros::init(argc, argv, "estop");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	ros::Rate rate(50.0);
	char c;
	bool r;

	// ROS subscriber
	ros::Subscriber motors_st_sub;
	motors_st_sub  = n.subscribe<std_msgs::Bool>("/motors_state", 50, stateChanged);

	// ROS clients
	ros::ServiceClient clEnable = n.serviceClient<std_srvs::Empty>("enable_motors");
	ros::ServiceClient clDisable = n.serviceClient<std_srvs::Empty>("disable_motors");
	std_srvs::Empty req;

	// node's main loop
	while (n.ok() && c != 'Q') {
		if (kbhit()) {
			c = getch();
			ROS_INFO("[estop] The key \'%c\' has been hitted", c);
			if (state) {
				ROS_INFO("[estop] Call service disable_motors");
				r = clDisable.call(req);
				if (!r) ROS_WARN("[estop] Service returned an error");
			}
			else if (c == 'g' || c == 'G') {
				ROS_INFO("[estop] Call service enable_motors");
				r = clEnable.call(req);
				if (!r) ROS_WARN("[estop] Service returned an error");
			}

			if (c == 'Q') ROS_INFO("[estop] Exit key (\'Q\') has been hitted!"); 
		}
	
		ros::spinOnce();	// trigger callbacks once
		rate.sleep();		// sleep some time to attain desired processing rate
	}

	return(0);
}

