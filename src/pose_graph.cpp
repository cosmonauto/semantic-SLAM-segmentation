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
    {
        // 图是空的，直接加入原始点
        unique_lock<mutex> lck(keyframes_mutex);
        keyframes.push_back(frame);
        refFrame = frame;
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId( frame->id );
        v-