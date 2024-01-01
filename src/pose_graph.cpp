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
        v->setEstimate( frame->getTransform() );
        v->setFixed(false);
        optimizer.addVertex( v );
        vertexIdx.push_back( frame->id );
        keyframes.push_back( frame );

        // and the edge with refframe
        // 这里直接根据refFrame和currentFrame的位姿差生成一个边
        // 因为位姿差是tracker估计出来的，我们认为这是比较准的
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 注意边的赋值有些绕，详见EdgeSE3的误差计算方式
        g2o::VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( refFrame->id ));
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( frame->id ));
        edge->setVertex(0, v1);
        edge->setVertex(1, v0);
        // because the state is estimated from tracker
        edge->setMeasurementFromState();
        edge->setInformation( Eigen::Matrix<double,6,6>::Identity() * 100);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );

        EdgeID id;
        id[refFrame->id] = frame->id;
        edges[ id ] = edge;
        optimizer.addEdge( edge );
        
        // set ref frame to current
        refFrame = frame;

        keyframe_updated.notify_one();
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief PoseGraph::mainLoop
 */
void PoseGraph::mainLoop()
{
    cout<<"starting pose graph thread..."<<endl;
    double  loopAccumulatedError = 0.0; //回环的累积误差
    double  localAccumulatedError = 0.0; //回环的累积误差
    while(1)
    {
        if (shutDownFlag == true)
        {
            break;
        }
        unique_lock<mutex> lck_update_keyframe(keyf