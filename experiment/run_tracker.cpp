#include "common_headers.h"
#include "rgbdframe.h"
#include "track.h"
#include "readGTPose.h"
#include "vo_stereo.hpp"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running tracker..."<<endl;
    ParameterReader para;
    FrameReader frameReader( para );

    VisualOdometryStereo::parameters voparam; 
 