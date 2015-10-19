#include <ros/ros.h>
#include <quadrivio_msgs/dcled.h>

using namespace std;

class queueManager {

    public:

    queueManager(int nQueue, int speed, bool pre) {
        mSpeed = speed;
        mPreamble = pre;
        queues = vector<list<string> >(nQueue, list<string>());
        queuesSize = queues.size();
    }

    void dcledCallback (const quadrivio_msgs::dcled::ConstPtr& msg) {
        queues[msg -> priority].push_back(msg -> message);
    }

    void showMessage() {
        stringstream ss;
        string message;
        for(int i = 0; i < queuesSize; i++) {
            if(!queues[i].empty()) {
                message = queues[i].front();
                queues[i].pop_front();
                break;
            }
        }
        if(!message.empty()) {
            ss << "cd ~/dcled && ./dcled";
            if(mPreamble)
                ss << " -p 2";
            ss << " -s " << mSpeed;
            ss << " -m '" << message << "'";
            system(ss.str().c_str());
        }
    }

    private:
    int queuesSize;
    int mSpeed;
    bool mPreamble;
    vector<list<string> > queues;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dcled");
  ros::NodeHandle nHandle;
  ros::Rate loopRate(50);
  int queues, speed;
  bool pre;

  ros::NodeHandle pNodeHandle("~");
  pNodeHandle.param("time", speed, int(20));
  pNodeHandle.param("queues", queues, int(3));
  pNodeHandle.param("preamble", pre, bool(0));

  queueManager *qMan = new queueManager(queues, speed, pre);

  ros::Subscriber sub = nHandle.subscribe("dcled", 1000, &queueManager::dcledCallback, qMan);

  while (ros::ok()) {
      ros::spinOnce();
      qMan -> showMessage();
      loopRate.sleep();
  }

  return 0;
}
