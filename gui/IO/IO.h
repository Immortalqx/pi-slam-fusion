#pragma once
#include <GSLAM/core/GImage.h>

#if defined(HAS_OPENCV)
#include <opencv2/highgui/highgui.hpp>

namespace GSLAM {

inline GImage imread(const std::string& filepath,int flags=1)
{
    cv::Mat img= cv::imread(filepath,flags);
    return img;
}

}

#elif defined(HAS_QT)

#include <QImage>

namespace GSLAM {

inline GImage imread(const std::string& filepath,int flags=1)
{
    GSLAM::GImage result;
    QImage qimage(filepath.c_str());
    if(qimage.format()==QImage::Format_RGB32)
    {
        result=GImage(qimage.height(),qimage.width(),
                      GImageType<uchar,4>::Type,qimage.bits(),true);// BGRA
    }
    else if(qimage.format()==QImage::Format_RGB888){
        qimage=qimage.rgbSwapped();
        result=GImage(qimage.height(),qimage.width(),
                      GImageType<uchar,3>::Type,qimage.bits(),true);// BGR
    }
    else if(qimage.format()==QImage::Format_Indexed8) // GRAY
    {
        result=GImage(qimage.height(),qimage.width(),
                      GImageType<uchar,1>::Type,qimage.bits(),true);
    }
    return result;
}

}

#else

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#define STB_IMAGE_INLINE
#include <GSLAM/core/stb_image.h>


namespace GSLAM {

inline GImage cvtrgb2bgr(GImage src){
    if(src.type()==GImageType<uchar,3>::Type||src.type()==GImageType<uchar,4>::Type){
        int    channels=src.channels();
        uchar *ele=src.data;
        uchar *eleEnd=ele+src.total()*channels;
        for(;ele<eleEnd;ele+=channels){
            uchar tmp=ele[0];
            ele[0]=ele[2];
            ele[2]=tmp;
        }
    }
    return src;
}

inline GImage imread(const std::string& filepath,int flags=1)
{
    int x=0,y=0,channels=0;
    auto data=stbi_load(filepath.c_str(),&x,&y,&channels,STBI_default);
    if(!data){
        return GImage();
    }
    int type=(GElement<uchar>::Type&0x7)+((channels-1)<<3);
    auto result= GImage(y,x,type,data,true);
    stbi_image_free(data);
    return cvtrgb2bgr(result);
//    cv::Mat img= cv::imread(filepath,flags);
//    return img;
}
}
#endif
