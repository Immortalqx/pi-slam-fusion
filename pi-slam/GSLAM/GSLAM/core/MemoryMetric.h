#ifndef __MEMORYMETRIC_H__
#define __MEMORYMETRIC_H__

#include <unordered_map>
#include <vector>
#include <mutex>


#define MEMORYUSAGE_TRACEDEPTH 4


namespace GSLAM {


struct MemoryUsageItem
{
public:
    size_t      memsize;
    size_t      mallocCaller[MEMORYUSAGE_TRACEDEPTH];

public:
    MemoryUsageItem() :
        memsize(0)
    {
        for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++) mallocCaller[i] = 0;
    }

    MemoryUsageItem(size_t _size, size_t* _mallocCaller) :
        memsize(_size)
    {
        if( _mallocCaller )
            for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++) mallocCaller[i] = _mallocCaller[i];
        else
            for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++) mallocCaller[i] = 0;
    }

    ~MemoryUsageItem() {}
};

struct MemoryCallerItem
{
public:
    size_t      callerCount;
    size_t      totalMemSize;
    size_t      mallocCaller[MEMORYUSAGE_TRACEDEPTH];

public:
    MemoryCallerItem():
        callerCount(0), totalMemSize(0)
    {
        for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++) mallocCaller[i] = 0;
    }
    MemoryCallerItem(size_t cn, size_t ms, size_t* mc) :
        callerCount(cn), totalMemSize(ms)
    {
        for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++) mallocCaller[i] = mc[i];
    }

    ~MemoryCallerItem() {}
};


bool MemoryCallerItemCompCount(MemoryCallerItem& a, MemoryCallerItem& b)
{
    return a.callerCount > b.callerCount;
}

bool MemoryCallerItemCompSize(MemoryCallerItem& a, MemoryCallerItem& b)
{
    return a.totalMemSize > b.totalMemSize;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class MemoryMetric {
public:
    MemoryMetric():_usage(0),_enabled(false),_shouldIgnore(false){}
    ~MemoryMetric(){_enabled=false;}

    static MemoryMetric& instanceCPU(){
        static MemoryMetric inst;
        return inst;
    }

    bool isEnabled()const{return _enabled;}

    void enable(){_enabled=true;}

    size_t usage()const{
        if(_enabled)
            return _usage;
        else return processUsage();
    }

    size_t count()const{return _allocated_sizes.size();}

    void AddAllocation(void* ptr, size_t size, size_t* callerAddr=0){
        if(!_enabled) return;
        if(_shouldIgnore) return;

        {
            std::unique_lock<std::mutex> lock(_mutex);
            _shouldIgnore=true;
            _allocated_sizes[ptr] = MemoryUsageItem(size, callerAddr);
            _usage+=size;
            _shouldIgnore=false;
        }
    }

    void FreeAllocation(void* ptr){
        if(!_enabled) return;
        if(_shouldIgnore) return;
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _shouldIgnore=true;

            auto it=_allocated_sizes.find(ptr);
            if(it!=_allocated_sizes.end())
            {
                _usage -= it->second.memsize;
                _allocated_sizes.erase(it);
            }

            _shouldIgnore=false;
        }
    }

    operator bool(){return isEnabled();}

    static size_t processUsage();

    std::unordered_map<void*, MemoryUsageItem>& getMemMap(){
        return _allocated_sizes;
    }

private:
    std::unordered_map<void*, MemoryUsageItem>  _allocated_sizes;

    std::mutex          _mutex;
    size_t              _usage;
    bool                _enabled;
    volatile bool       _shouldIgnore;
};



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class MemoryMallocAnalysis
{
public:
    typedef std::unordered_map<size_t, std::string>         MapAddr2Line;
    typedef std::unordered_map<size_t, MemoryCallerItem>    MapMemoryCaller;


    enum MemoryUsageSortMehtod {
        SORT_CALLCOUNT = 0,
        SORT_MEMSIZE,
    };

    MemoryMallocAnalysis();

    void analysisMemoryUsage(void);
    void dumpMemoryUsage(const std::string& fname, MemoryUsageSortMehtod sortMethod=SORT_CALLCOUNT);

    MapAddr2Line getAddressMap(std::unordered_map<void*, MemoryUsageItem>& memStastic);
    void convertAddr2Funcs(std::vector<size_t> arrAddr, int tid);
    std::string parseBacktraceSymbols(const std::string& symline);
    size_t addressHash(size_t *addr, int len);


protected:
    int                             addr2line_threads;
    std::string                     programExe;
    std::vector<MapAddr2Line>       arrMapAddr2Line;
    MapAddr2Line                    mapAddrFuncs;
    MapMemoryCaller                 mapMemCaller;

};


} // end of namespace GSLAM

#endif // end of __MEMORYMETRIC_H__

