/**
* @file     relocalization_recovery.cpp
* @brief    自己位置初期化クラスRelocalizationRecoveryのソースファイル
* @author   S.Kumada
* @date     2023/9/19
* @note     外部似て認識した位置情報を取得し、自己位置のリセットを行うクラスの実装
*/

#include "relocalization_recovery/relocalization_recovery.h"


//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(relocalization_recovery::RelocalizationRecovery, nav_core::RecoveryBehavior)

namespace relocalization_recovery 
{
    RelocalizationRecovery::RelocalizationRecovery():local_costmap_(NULL), initialized_(false), world_model_(NULL)
    {
#if DEBUG
        ROS_INFO("!!!!!!!!! RelocalizationRecovery : RUN Constructor !!!!!!!!!");
#endif
        is_position_received_ = false;
       
    }

    void RelocalizationRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
        costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
    {
        if (!initialized_)
        {
#if DEBUG
            ROS_INFO("!!!!!!!!! RelocalizationRecovery : RUN Initialize !!!!!!!!!");
            ROS_INFO("name : %s", name.c_str());
#endif
            // get Param
            ros::NodeHandle private_nh("~/" + name);

            // 物体認識システムで認識したロボットの位置がRDRへ格納されるまでの更新間隔時間
            std::string position_update_interval_time;
            getParam(private_nh, "position_update_interval_time", position_update_interval_time_, ROS_TIME_30S);
            
            // 位置情報取得要求の配信トピック名 
            std::string get_position_topic_name;
            getParam(private_nh, "pub_get_position_topic", get_position_topic_name, std::string("/robot_bridge/get_position_data"));
            
            // 位置情報取得結果の受信トピック名
            std::string sub_position_topic_name;
            getParam(private_nh, "sub_get_position_result_topic", sub_position_topic_name, std::string("/initialpose"));
            
            // 位置情報取得のタイムアウト時間
            getParam(private_nh, "get_position_timeout", timeout_, DEF_TIME_OUT_TIME);

            // 位置情報取得要求のパブリッシャ
            pub_get_position_data_ = private_nh.advertise<uoa_poc5_msgs::r_get_position_data>(get_position_topic_name, ROS_QUEUE_SIZE_1, true);
            sub_get_position_data_ = private_nh.subscribe(sub_position_topic_name, ROS_QUEUE_SIZE_1, &RelocalizationRecovery::recvPositionDataCB, this);
        }
        else
        {
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    void RelocalizationRecovery::recvPositionDataCB(const geometry_msgs::PoseWithCovarianceStamped& msg)
    {
#if DEBUG
        ROS_INFO("!!!!!!!!! RelocalizationRecovery : Receive Position Data !!!!!!!!!");
#endif  
        is_position_received_ = true;
    }

    void RelocalizationRecovery::runBehavior()
    {
#if DEBUG
        ROS_INFO("!!!!!!!!! RelocalizationRecovery : RUN Behavior !!!!!!!!!");
#endif  
        // 物体認識システム側の位置情報更新待ち
        ROS_INFO("wait for %lf [sec] ...", position_update_interval_time_);
        sleepFunc(position_update_interval_time_);

        // 位置情報取得を行う
        uoa_poc5_msgs::r_get_position_data pub_msg;
        pub_msg.header.stamp    = ros::Time::now();
        pub_msg.retry           = 5;
        pub_msg.wait_interval   = 1.0;

        pub_get_position_data_.publish(pub_msg);    // 配信
        
        // 受信フラグオフ
        is_position_received_ = false;

        // 位置情報を取得するまで待機
	    ros::Time t_start = ros::Time::now();
        ros::Rate rate(ROS_RATE_30HZ);

        while(ros::ok && !is_position_received_)
        {   // 位置情報受信完了まで
            ros::spinOnce();
            rate.sleep();

            // タイムアウト確認
            if(timeout_ < (ros::Time::now() - t_start).toSec()) break;

#if DEBUG
		    std::cout << "DEBUG : Passed time[s] = " << (ros::Time::now() - t_start).toSec() << std::endl;
#endif
        }

    }

    RelocalizationRecovery::~RelocalizationRecovery()
    {
#if DEBUG
        ROS_INFO("!!!!!!!!! RelocalizationRecovery : RUN Destructor !!!!!!!!!");
#endif
    }

};