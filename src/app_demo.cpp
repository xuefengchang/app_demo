/**
Software License Agreement (BSD)
\file      app_demo.cpp
\authors Xuefeng Chang <changxuefengcn@163.com>
\copyright Copyright (c) 2016, the micROS Team, HPCL (National University of Defense Technology), All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of micROS Team, HPCL, nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "app_demo/app_demo.h"

// Register the application
PLUGINLIB_EXPORT_CLASS(app_demo::AppDemo, micros_swarm_framework::Application)

namespace app_demo{

    struct XY
    {
        float x;
        float y;
    };

    AppDemo::AppDemo()
    {
    }

    AppDemo::~AppDemo()
    {
    }

    void AppDemo::init()
    {
        //set parameters
        delta = 7;
        epsilon = 150;
    }

    float AppDemo::force_mag(float dist)
    {
        return -(epsilon/(dist+0.1)) *(pow(delta/(dist+0.1), 4) - pow(delta/(dist+0.1), 2));
    }

    XY AppDemo::force_sum(micros_swarm_framework::NeighborBase n, XY &s)
    {
        micros_swarm_framework::Base l=base();
        float xl=l.x;
        float yl=l.y;

        float xn=n.x;
        float yn=n.y;

        float dist=sqrt(pow((xl-xn),2)+pow((yl-yn),2));

        float fm = force_mag(dist)/1000;
        if(fm>0.5) fm=0.5;

        float fx=(fm/(dist+0.1))*(xn-xl);
        float fy=(fm/(dist+0.1))*(yn-yl);

        s.x+=fx;
        s.y+=fy;
        return s;
    }

    XY AppDemo::direction()
    {
        XY sum;
        sum.x=0;
        sum.y=0;

        micros_swarm_framework::Neighbors<micros_swarm_framework::NeighborBase> n(true);
        //n.print();
        boost::function<XY(micros_swarm_framework::NeighborBase, XY &)> bf=boost::bind(&AppDemo::force_sum, this, _1, _2);
        sum=n.reduce(bf, sum);

        return sum;
    }

    void AppDemo::publish_cmd(const ros::TimerEvent&)
    {
        XY v=direction();
        geometry_msgs::Twist t;
        t.linear.x=v.x;
        t.linear.y=v.y;

        pub.publish(t);
    }

    void AppDemo::motion()
    {
        ros::NodeHandle nh;
        timer = nh.createTimer(ros::Duration(0.1), &AppDemo::publish_cmd, this);
    }

    void AppDemo::baseCallback(const nav_msgs::Odometry& lmsg)
    {
        float x=lmsg.pose.pose.position.x;
        float y=lmsg.pose.pose.position.y;

        float vx=lmsg.twist.twist.linear.x;
        float vy=lmsg.twist.twist.linear.y;

        micros_swarm_framework::Base l(x, y, 0, vx, vy, 0);
        set_base(l);
    }

    void AppDemo::start()
    {
        init();

        ros::NodeHandle nh;
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        sub = nh.subscribe("base_pose_ground_truth", 1000, &AppDemo::baseCallback, this, ros::TransportHints().udp());

        motion();
    }
};

