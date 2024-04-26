/**
* @file     relocalization_recovery.cpp
* @brief    自己位置初期化クラスRelocalizationRecoveryのヘッダファイル
* @author   S.Kumada
* @date     2023/9/19
* @note     外部似て認識した位置情報を取得し、自己位置のリセットを行うクラスの定義
*/

#ifndef RELOCALIZATION_RECOVERY_H_
#define RELOCALIZATION_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


#include "uoa_poc5_msgs/r_get_position_data.h"

#include "utilities.h"

#define DEF_TIME_OUT_TIME 10.0 // 10秒

namespace relocalization_recovery
{
    /**
     * @brief 再自己位置取得のリカバリー処理クラス
     * @details RDR/Planner側とのhttp通信を行うためのブリッジプログラム。
     */
    class RelocalizationRecovery : public nav_core::RecoveryBehavior 
    {
      public:
        /**
        * @brief   RelocalizationRecoveryクラスのコンストラクタ
        * @details 初期化を行う
        */
        RelocalizationRecovery();

        /**
        * @brief   RelocalizationRecoveryクラスのデストラクタ
        * @details オブジェクトの破棄を行う
        */
        ~RelocalizationRecovery();

        /**
        * @brief        ナビゲーション起動時の初期化関数
        * @param[in]    name std::string型 リカバリー処理名
        * @param[in]    tf tf2_ros::Buffer*型 tfリスナーへのポインタ
        * @param[in]    global_costmap costmap_2d::Costmap2DROS*型 global_costmap へのポインタ
        * @param[in]    local_costmap costmap_2d::Costmap2DROS*型 local_costmap へのポインタ
        * @return       void
        * @details      RecoveryBehaviorクラスの仮想関数の実装
        */
        void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

        /**
        * @brief        リカバリー処理の実行関数
        * @param[in]    void
        * @return       void
        * @details      RecoveryBehaviorクラスの仮想関数の実装
        */
        void runBehavior();

      private:
        /**
        * @brief        位置情報の受信コールバック関数
        * @param[in]    msg geometry_msgs/PoseWithCovarianceStamped型のメッセージデータ
        * @return       void
        * @details      外部から取得したロボットの位置の受信処理
        */
        void recvPositionDataCB(const geometry_msgs::PoseWithCovarianceStamped& msg);

        costmap_2d::Costmap2DROS* local_costmap_; // ローカルコストマップのポインタ（未使用）
        bool initialized_;  // 初期化済みフラグ
        bool is_position_received_; // 補正位置の取得フラグ
        double position_update_interval_time_; // 位置更新の間隔時間
        double timeout_; // 位置情報取得のタイムアウト時間
        base_local_planner::CostmapModel* world_model_; // グローバルモデル
        ros::Publisher pub_get_position_data_;
        ros::Subscriber sub_get_position_data_;
    };
};
#endif
