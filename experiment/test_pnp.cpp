#include "pnp.h"
#include "common_headers.h"
#include "readGTPose.h"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running test pnp"<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
   