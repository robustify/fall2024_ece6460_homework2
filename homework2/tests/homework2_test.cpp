#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <avs_lecture_msgs/Trajectory.h>

ros::NodeHandle* node;
gps_common::GPSFix g_fix;
bool received_fix;

avs_lecture_msgs::Trajectory g_traj;
bool received_traj;

static constexpr double G_TIME_RESOLUTION = 0.01;
static constexpr double G_TOPIC_TIMEOUT = 10.0;
static constexpr double G_TEST_DURATION = 42.0;

static constexpr double G_LAT_ERROR_TOL = 0.3;

bool foundMessages()
{
  return received_fix && received_traj;
}

double tand(double ang_deg)
{
  return tan(M_PI / 180.0 * ang_deg);
}

double sind(double ang_deg)
{
  return sin(M_PI / 180.0 * ang_deg);
}

double computeLatError()
{
  std::string utm_zone;
  geometry_msgs::Point utm;
  gps_common::LLtoUTM(g_fix.latitude, g_fix.longitude, utm.y, utm.x, (char*)utm_zone.data());

  int zone_number;
  char zone_letter;
  std::sscanf(utm_zone.c_str(), "%d%c", &zone_number, &zone_letter);

  double central_meridian = 6 * (zone_number - 1) - 177;
  double utm_heading = M_PI_2 - M_PI / 180.0 * g_fix.track + atan(tand(g_fix.longitude - central_meridian) * sind(g_fix.latitude));

  double min_dist2 = INFINITY;
  double lat_error = 0;
  for (size_t i = 0; i < g_traj.points.size(); i++) {
    double dx = g_traj.points[i].point.x - utm.x;
    double dy = g_traj.points[i].point.y - utm.y;

    // avs_lecture_msgs::PointWithSpeed local_p;
    tf2::Vector3 local_p(0, 0, 0);
    local_p.setX( dx * cos(utm_heading) + dy * sin(utm_heading));
    local_p.setY(-dx * sin(utm_heading) + dy * cos(utm_heading));

    double d2 = local_p.length2();
    if (d2 < min_dist2) {
      min_dist2 = d2;
      lat_error = std::abs(local_p.y());
    }
  }

  return lat_error;
}

void runCheck()
{
  int num_tolerance_violations = 0;
  double max_error = 0.0;
  double max_speed = 0.0;
  ros::Time timeout_t = ros::Time::now() + ros::Duration(G_TEST_DURATION);
  while (!ros::isShuttingDown() && ((timeout_t - ros::Time::now()).toSec() > 0)) {
    if (received_traj) {
      received_traj = false;

      double lat_error = computeLatError();
      if (lat_error > max_error) {
        max_error = lat_error;
      }

      if (g_fix.speed > max_speed) {
        max_speed = g_fix.speed;
      }

      if (lat_error > G_LAT_ERROR_TOL) {
        num_tolerance_violations++;
      }

      ASSERT_LE(num_tolerance_violations, 3) << "Lateral error " << max_error << " exceeded tolerance of " << G_LAT_ERROR_TOL;
    }

    ros::spinOnce();
    ros::Duration(G_TIME_RESOLUTION).sleep();
  }

  if ((timeout_t - ros::Time::now()).toSec() > 0) {
    FAIL() << "Test was stopped prematurely";
  }

  ASSERT_GE(max_speed, 2.27) << "Vehicle did not move";
}

void recvFix(const gps_common::GPSFixConstPtr& msg)
{
  g_fix = *msg;
  received_fix = true;
}

void recvTrajectory(const avs_lecture_msgs::TrajectoryConstPtr& msg)
{
  g_traj = *msg;
  received_traj = true;
}

TEST(Homework2, test_tracking_accuracy)
{
  ros::Time timeout_t = ros::Time::now() + ros::Duration(G_TOPIC_TIMEOUT);
  while(!ros::isShuttingDown() && !foundMessages() && ((timeout_t - ros::Time::now()).toSec() > 0.0)) {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  EXPECT_TRUE(received_traj) << "Test did not receive trajectory message";
  EXPECT_TRUE(received_fix) << "Test did not receive fix message";
  ASSERT_TRUE(foundMessages()) << "Missing messages";
  runCheck();

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "homework2_tests");

  node = new ros::NodeHandle();

  ros::Subscriber sub_fix = node->subscribe("/vehicle/perfect_gps/enhanced_fix", 1, recvFix);
  ros::Subscriber sub_trajectory = node->subscribe("/lookahead_trajectory", 1, recvTrajectory);

  received_traj = false;
  received_fix = false;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  node->shutdown();
  delete node;

  return 0;
}
