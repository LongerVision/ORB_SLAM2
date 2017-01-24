#ifndef ORBEXTRACTOR_GPU_H
#define ORBEXTRACTOR_GPU_H

#include <vector>
#include "ORBextractor.h"


namespace ORB_SLAM2
{

class ORBextractorGPU: public ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractorGPU(int nfeatures, 
                    float scaleFactor, 
                    int nlevels,
                    int iniThFAST, 
                    int minThFAST);

    virtual ~ORBextractorGPU();
    
    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()(cv::InputArray image, 
                    cv::InputArray mask,
                    std::vector<cv::KeyPoint>& keypoints,
                    cv::OutputArray descriptors);

    
protected:
    cv::Ptr<cv::cuda::ORB> d_orb;
    cv::cuda::GpuMat d_src;
    int d_src_width;
    int d_src_height;
};

} //namespace ORB_SLAM

#endif  // ORBEXTRACTOR_GPU_H
