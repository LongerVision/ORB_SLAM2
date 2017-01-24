
#include "opencv2/core/cuda.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudawarping.hpp"

#include "ORBextractorGPU.h"


namespace ORB_SLAM2
{

ORBextractorGPU::ORBextractorGPU(   int _nfeatures,
                                    float _scaleFactor,
                                    int _nlevels,
                                    int _iniThFAST,
                                    int _minThFAST) : ORBextractor(_nfeatures, _scaleFactor, _nlevels, _iniThFAST, _minThFAST)
{
    d_orb = cv::cuda::ORB::create(  nfeatures,
                                    scaleFactor,
                                    nlevels,
                                    ORB_SLAM2::EDGE_THRESHOLD,
                                    0,
                                    2,
                                    cv::ORB::HARRIS_SCORE,
                                    ORB_SLAM2::PATCH_SIZE,
                                    20,
                                    true);
}


ORBextractorGPU::~ORBextractorGPU()
{

}


void ORBextractorGPU::operator()(   cv::InputArray _image,
                                    cv::InputArray _mask,
                                    std::vector<cv::KeyPoint>& _keypoints,
                                    cv::OutputArray _descriptors)
{ 
    if(_image.empty())
        return;

    cv::Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // upload
    d_src.upload(image);
    
    // initialize image info
    d_src_width = image.cols;
    d_src_height = image.rows;
    
    // Pre-compute the scale pyramid
    // ComputePyramid(d_src);
    
    // Compute the descriptors
    cv::cuda::GpuMat gpu_keypoints;
    cv::cuda::GpuMat gpu_descriptors;
    d_orb->detectAndComputeAsync(d_src, 
                                 cv::cuda::GpuMat(), 
                                 gpu_keypoints,
                                 gpu_descriptors);
    d_orb->convert(gpu_keypoints, _keypoints);
    gpu_descriptors.download(_descriptors);
}


/***
 * Description: For this specific function, we don't use mvInvScaleFactor
 * URL: http://answers.opencv.org/question/91398/pyramid-laplace-cuda/
**/ 


}
