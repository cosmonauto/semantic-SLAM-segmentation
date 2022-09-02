#include "rgbdframe.h"

using namespace rgbd_tutor;

int main()
{    
    ParameterReader para;
    FrameReader     fr(para);

    int i = 1 ;
    while( RGBDFrame::Ptr frame = fr.next() )
    {
        char _filename[256];
   