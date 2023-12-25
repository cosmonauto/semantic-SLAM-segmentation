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
        v->setEstimate( frame->T_f_w );
        v->setFixed(true);
        optimizer.addVertex( v );
        vertexIdx.push_back( frame->id );
        return true;
    }

    // 计算 frame 和 refFrame 之间的位移差
    Eigen::Isometry3d delta = frame->getTransform().inverse() * refFrame->getTransform();
    if ( norm_translate( delta ) > keyframe_min_translation ||
         norm_rotate( delta ) > keyframe_min_rotation )
    {
        // 离keyframe够远
        // 在key frames中进行插入，并在图中生成对应节点和边
        unique_lock<mutex> lck(keyframes_mutex);
        cout<<YELLOW<<"adding keyframe "<<frame->id<<" with ref to "<<refFrame->id<<", n_t="<<norm_translate( delta )<<",n_r="<<norm_rotate(delta)<<RESET<<endl;
        newFrames.push_back( frame );
        
        //  add the vertex
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId( frame->id );
        v->setEstimate( frame->getTransf