#include "pose_graph.h"
#include "utils.h"
using namespace rgbd_tutor;

/**
 * @brief PoseGraph::tryInsertKeyFrame
 * @param frame
 * @return
 * TODO: Problem: when inserting large number of keyframes, the mapping thread will block for long time, causing even more key-frames to be processed.
 */
bool PoseGraph::tryInsertKeyFrame(RGBDFrame::Ptr& frame)
{
    if ( keyframes.size() == 0 )
   