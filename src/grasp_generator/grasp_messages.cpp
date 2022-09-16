#include <grasp_generator/grasp_messages.h>

grasp_generator::GraspConfigList GraspMessages::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header)
{
  grasp_generator::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

grasp_generator::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  grasp_generator::GraspConfig msg;
  tf::pointEigenToMsg(hand.getPosition(), msg.position);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);

  return msg;
}
