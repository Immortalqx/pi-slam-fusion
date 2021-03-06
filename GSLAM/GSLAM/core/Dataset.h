#ifndef __GSLAM_DATASET_H__
#define __GSLAM_DATASET_H__

#include "GSLAM.h"
#include "Svar.h"
#include "SharedLibrary.h"

namespace GSLAM{

#define REGISTER_DATASET(D,E) \
    extern "C" SPtr<GSLAM::Dataset> createDataset##E(){ return SPtr<GSLAM::Dataset>(new D());}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::DatasetFactory::instance()._ext2creator.insert(#E,createDataset##E);\
}}D##E##_instance;

/// create
class Dataset;
typedef SPtr<Dataset> DatasetPtr;
typedef SPtr<Dataset> (*funcCreateDataset)();

// A dataset configuration file : DatasetName.DatasetType --eg. Sequence1.kitti desk.tumrgbd
class Dataset : public GSLAM::GObject
{
public:
    typedef std::vector<std::string> StrVec;
    Dataset():_name("Untitled"){}
    virtual ~Dataset(){}

    std::string         name() const{if(_impl) return _impl->type();else return _name;}
    virtual std::string type() const{if(_impl) return _impl->type();else return "Dataset";}
    virtual bool        isOpened(){if(_impl) return _impl->isOpened();else return false;}
    virtual FramePtr    grabFrame(){if(_impl) return _impl->grabFrame();else return FramePtr();}

    virtual bool        open(const std::string& dataset);
    virtual bool        close(void);

protected:
    std::string _name;
    DatasetPtr  _impl;
};

class DatasetFactory
{
public:
    typedef SPtr<Dataset> DatasetPtr;

    static DatasetFactory& instance(){
        static SPtr<DatasetFactory> inst(new DatasetFactory());
        return *inst;
    }

    static DatasetPtr create(std::string dataset);

    SvarWithType<funcCreateDataset>        _ext2creator;
};

inline bool Dataset::open(const std::string& dataset){
    //TODO 这里要好好看看
    DatasetPtr impl=DatasetFactory::create(dataset);
    //St10shared_ptrIN5GSLAM7DatasetEE
    //std::cout<< typeid(impl).name()<<std::endl;
    if(impl) {_impl=impl;return _impl->open(dataset);}
    return false;
}

inline bool Dataset::close(void)
{
    if( _impl ) _impl.reset();
    return true;
}


inline DatasetPtr DatasetFactory::create(std::string dataset)
{
    std::string extension;
    // The input path could be dataset configuration file or just a folder path
    //从后往前找到第一个'.'的位置
    size_t dotPosition=dataset.find_last_of('.');
    if(dotPosition!=std::string::npos)// call by extension
    {
        extension=dataset.substr(dotPosition+1);
    }
    if(extension.empty()) return DatasetPtr();

    //extension是扩展名，rtm
    if(!instance()._ext2creator.exist(extension))//False
    {
        SharedLibraryPtr plugin=Registry::get("libgslamDB_"+extension);
        if(!plugin.get()) return DatasetPtr();
        funcCreateDataset createFunc=(funcCreateDataset)plugin->getSymbol("createDataset"+extension);
        if(createFunc) return createFunc();
    }

    if(!instance()._ext2creator.exist(extension)) return DatasetPtr();//False
    funcCreateDataset createFunc=instance()._ext2creator.get_var(extension,NULL);
    if(!createFunc) return DatasetPtr();//False

    //这里实际上就是根据文件的后缀来指定一个类来读取数据！
    //比如NPU数据集用的是rtm格式的，这里返回的就是RTMapper类！
    return createFunc();//代码最后其实走到了这里！这里最后给出了一个什么？？？
}

} // end of namespace GSLAM

#endif // end of __GSLAM_DATASET_H__
