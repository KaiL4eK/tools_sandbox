#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <svserver.h>

#include <QCoreApplication>
#include <QObject>

class ROSServer : public QThread
{
	Q_OBJECT

	SVServer *m_server;

public:
    ROSServer(SVServer *server)
    {
    	m_server = server;

    	QObject::connect( this, SIGNAL(sendData(DataPackage const&)), server, SLOT(sendData(DataPackage const&)) );
    }

	void topicCallback(const std_msgs::Float32& msg)
	{
		ROS_INFO("Received potentiometer data: %g", msg.data);
		
		DataPackage data( DataPackage::State::WAIT );
		data.m_steeringAngle = msg.data;

		emit sendData( data );
	}

	void run()
	{
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("steering", 1000, &ROSServer::topicCallback, this);
		
		while ( ros::ok() )
		{
			ros::spinOnce();
		}

		Q_EMIT rosShutdown();
	}

Q_SIGNALS:
    void rosShutdown();
    void sendData(DataPackage const& data);
};
