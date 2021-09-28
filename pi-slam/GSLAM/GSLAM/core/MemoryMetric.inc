
#include <math.h>

#include "Svar.h"
#include "SharedLibrary.h"
#include "Utils.h"
#include "MemoryMetric.h"


#if (defined(_WIN32) || defined(_WIN64)) && !defined(__CYGWIN__) // Windows

// currently do not support Windows

namespace GSLAM {

size_t MemoryMetric::processUsage()
{
    return 0;
}


MemoryMallocAnalysis::MemoryMallocAnalysis()
{
    char **argv = (char**) svar.GetPointer("argv");
    programExe = argv[0];

    addr2line_threads = 16;
}

void MemoryMallocAnalysis::analysisMemoryUsage(void)
{
}


void MemoryMallocAnalysis::dumpMemoryUsage(const std::string& fname, MemoryMallocAnalysis::MemoryUsageSortMehtod sortMethod)
{
}

MemoryMallocAnalysis::MapAddr2Line MemoryMallocAnalysis::getAddressMap(std::unordered_map<void*, MemoryUsageItem>& memStastic)
{
    std::unordered_map<size_t, std::string> addrFuncs;
    return addrFuncs;
}

void MemoryMallocAnalysis::convertAddr2Funcs(std::vector<size_t> arrAddr, int tid)
{
}

std::string MemoryMallocAnalysis::parseBacktraceSymbols(const std::string& symline)
{
    return "??:0";
}

size_t MemoryMallocAnalysis::addressHash(size_t *addr, int len)
{
    return 0;
}

} // end of namespace GSLAM



#else // Linux/Unix

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <mutex>

#include <unistd.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <errno.h>
#include <cxxabi.h>


#ifdef ANDROID
#define MALLOC_SUFFIX
#define FREE_SUFFIX
#define REALLOC_SUFFIX
#define CALLOC_SUFFIX
#elif __linux
#define MALLOC_SUFFIX throw()
#define FREE_SUFFIX throw()
#define REALLOC_SUFFIX throw()
#define CALLOC_SUFFIX throw()
#else
#define MALLOC_SUFFIX __result_use_check __alloc_size(1)
#define FREE_SUFFIX 
#define REALLOC_SUFFIX __result_use_check __alloc_size(2)
#define CALLOC_SUFFIX __result_use_check __alloc_size(1,2)
#endif


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

extern "C" {

static std::recursive_mutex gpu_alloc_lock;
typedef std::lock_guard<decltype(gpu_alloc_lock)> gpu_lock_guard_t;

static std::recursive_mutex alloc_lock;
typedef std::lock_guard<decltype(alloc_lock)> lock_guard_t;

// We provide our own malloc/calloc/realloc/free, which are then used
// by whatever we link against (i.e., SLAM implementations).

static void *load_and_call_libc_malloc(size_t);
static void load_and_call_libc_free(void*);
static void *load_and_call_libc_calloc(size_t, size_t);
static void *load_and_call_libc_realloc(void*, size_t);

typedef void *(*malloc_t)(size_t);
typedef void (*free_t)(void*);
typedef void *(*realloc_t)(void*, size_t);
typedef void *(*calloc_t)(size_t, size_t);

// TODO: make this thread-safe

static malloc_t libc_malloc = load_and_call_libc_malloc;
static free_t libc_free = load_and_call_libc_free;
static calloc_t libc_calloc = load_and_call_libc_calloc;
static realloc_t libc_realloc = load_and_call_libc_realloc;

static void* load_and_call_libc_malloc(size_t t) {
    libc_malloc = (malloc_t)dlsym(RTLD_NEXT, "malloc");
    return libc_malloc(t);
}
static void load_and_call_libc_free(void *t) {
    libc_free = (free_t)dlsym(RTLD_NEXT, "free");
    libc_free(t);
}

static void* load_and_call_libc_calloc(size_t nmemb, size_t size) {
    libc_calloc = (calloc_t)dlsym(RTLD_NEXT, "calloc");
    return libc_calloc(nmemb, size);
}

static void *load_and_call_libc_realloc(void *t, size_t size) {
    libc_realloc = (realloc_t)dlsym(RTLD_NEXT, "realloc");
    return libc_realloc(t, size);
}

//define a scratch space for calloc
static char calloc_scratch[4096];
static char* calloc_ptr = calloc_scratch;
static volatile bool backtrace_called = false;


void first_backtrace(void)
{
    void *addrlist[MEMORYUSAGE_TRACEDEPTH];
    backtrace( addrlist, MEMORYUSAGE_TRACEDEPTH);

    backtrace_called = true;
}

static inline void getCallerAddress(size_t *addr)
{
    void *addrlist[MEMORYUSAGE_TRACEDEPTH+1];
    if( backtrace_called )
    {
        backtrace( addrlist, MEMORYUSAGE_TRACEDEPTH+1);
        for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++)
            addr[i] = (size_t) addrlist[1+i];
    }
}



/*
    References:
        Avoid deadlock in malloc on backtrace https://patchwork.ozlabs.org/patch/442890/
        How Backtrace works on Linux x86_64? https://stackoverflow.com/questions/8724814/how-backtrace-works-on-linux-x86-64?rq=1
 */

void *malloc(size_t size) MALLOC_SUFFIX
{
    size_t caller[MEMORYUSAGE_TRACEDEPTH];
    getCallerAddress(caller);

    void *ptr = NULL;

    {
        lock_guard_t    lock(alloc_lock);

        ptr = libc_malloc(size);

        if(!GSLAM::MemoryMetric::instanceCPU())
            return ptr;
        GSLAM::MemoryMetric::instanceCPU().AddAllocation(ptr, size, caller);
    }

    return ptr;
}

void *malloc_withCaller(size_t size, size_t* caller) MALLOC_SUFFIX
{
    lock_guard_t    lock(alloc_lock);

    void *ptr = libc_malloc(size);

    if(!GSLAM::MemoryMetric::instanceCPU())
        return ptr;
    GSLAM::MemoryMetric::instanceCPU().AddAllocation(ptr, size, caller);

    return ptr;
}

void free(void* ptr) FREE_SUFFIX
{
    lock_guard_t lock(alloc_lock);
    GSLAM::MemoryMetric::instanceCPU().FreeAllocation(ptr);
    libc_free(ptr);
}

void *realloc(void *ptr, size_t newsize) REALLOC_SUFFIX {
    lock_guard_t lock(alloc_lock);

    auto newptr = libc_realloc(ptr, newsize);

    GSLAM::MemoryMetric::instanceCPU().FreeAllocation(ptr);
    GSLAM::MemoryMetric::instanceCPU().AddAllocation(newptr, newsize);

    return newptr;
}

void *calloc(size_t nmemb, size_t size) CALLOC_SUFFIX {
    lock_guard_t lock(alloc_lock);

    // need to be careful with calloc since dlsym will use it! need to do this
    // even if the profiler is disabled.
    static bool reentered = false;
    if(reentered) {
        void *ptr = calloc_ptr;
        calloc_ptr += nmemb*size;
        calloc_ptr += 16-(((uintptr_t)calloc_ptr) % 16);
        if(calloc_ptr >= (calloc_scratch + sizeof(calloc_scratch))) abort();
        return ptr;
    }
    reentered = true;

    void *result = libc_calloc(nmemb, size);
    GSLAM::MemoryMetric::instanceCPU().AddAllocation(result, nmemb * size);
    reentered = false;
    return result;
}

} // end of extern "C"



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void *operator new(std::size_t size) {
    size_t caller[MEMORYUSAGE_TRACEDEPTH];
    getCallerAddress(caller);

    {
        lock_guard_t lock(alloc_lock);

        void *mem = malloc_withCaller(size, caller);

        if(mem == nullptr) {
            throw std::bad_alloc();
        } else {
            return mem;
        }
    }
}

void *operator new(std::size_t size, const std::nothrow_t &nothrow_value) MALLOC_SUFFIX {
    (void)nothrow_value;

    size_t caller[MEMORYUSAGE_TRACEDEPTH];
    getCallerAddress(caller);

    try {
        lock_guard_t lock(alloc_lock);

        void *mem = malloc_withCaller(size, caller);

        if(mem == nullptr) {
            throw std::bad_alloc();
        } else {
            return mem;
        }
    } catch (std::exception &e) {
        return nullptr;
    }
}


void *operator new[](std::size_t size) {
    size_t caller[MEMORYUSAGE_TRACEDEPTH];
    getCallerAddress(caller);

    {
        lock_guard_t lock(alloc_lock);

        void *mem = malloc_withCaller(size, caller);

        if(mem == nullptr) {
            throw std::bad_alloc();
        } else {
            return mem;
        }
    }
}

void *operator new[](std::size_t size, const std::nothrow_t &nothrow_value) MALLOC_SUFFIX {
    (void)nothrow_value;

    size_t caller[MEMORYUSAGE_TRACEDEPTH];
    getCallerAddress(caller);

    try {
        lock_guard_t lock(alloc_lock);

        void *mem = malloc_withCaller(size, caller);

        if(mem == nullptr) {
            throw std::bad_alloc();
        } else {
            return mem;
        }
    } catch (std::exception &e) {
        return nullptr;
    }
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace GSLAM {

size_t MemoryMetric::processUsage()
{
#ifdef __linux
    char file_name[64]={0};
    FILE *fd;
    char line_buff[512]={0};
    sprintf(file_name,"/proc/%d/status",getpid());

    fd =fopen(file_name,"r");
    if(nullptr == fd){
        return 0;
    }

    char name[64];
    int vmrss;
    const int VMRSS_LINE=17;
    for (int i=0; i<VMRSS_LINE;i++){
        if(!fgets(line_buff,sizeof(line_buff),fd)) return 0;
    }

    sscanf(line_buff,"%s %d",name,&vmrss);
    fclose(fd);
    return vmrss;
#else
    return 0;
#endif
}


MemoryMallocAnalysis::MemoryMallocAnalysis()
{
    char **argv = (char**) svar.GetPointer("argv");
    programExe = argv[0];

    addr2line_threads = 16;
}

void MemoryMallocAnalysis::analysisMemoryUsage(void)
{
    // get memory stastics
    std::unordered_map<void*, MemoryUsageItem>& memStastic = GSLAM::MemoryMetric::instanceCPU().getMemMap();

    // conver address to functions/line/file
    mapAddrFuncs = getAddressMap(memStastic);

    // change basic stastic -> caller stastic
    mapMemCaller.clear();

    for(auto it : memStastic)
    {
        size_t *addr = it.second.mallocCaller;
        size_t addHash = addressHash(addr, MEMORYUSAGE_TRACEDEPTH);

        auto it2 = mapMemCaller.find(addHash);
        if(it2 != mapMemCaller.end())
        {
            it2->second.totalMemSize += it.second.memsize;
            it2->second.callerCount  ++;
        }
        else
        {
            mapMemCaller[addHash] = MemoryCallerItem(1, it.second.memsize, it.second.mallocCaller);
        }
    }
}

void MemoryMallocAnalysis::dumpMemoryUsage(const std::string& fname,
                                           MemoryMallocAnalysis::MemoryUsageSortMehtod sortMethod)
{
    // do memory caller analysis
    if( mapMemCaller.size() == 0 || mapAddrFuncs.size() == 0 )
        analysisMemoryUsage();

    // sort by call count or memory size
    std::vector<MemoryCallerItem> arrCaller;
    arrCaller.reserve(mapMemCaller.size());
    for(auto it : mapMemCaller)
        arrCaller.push_back(it.second);

    if( sortMethod == 0 )
        std::sort(arrCaller.begin(), arrCaller.end(), MemoryCallerItemCompCount);
    else
        std::sort(arrCaller.begin(), arrCaller.end(), MemoryCallerItemCompSize);

    // resolve caller's function name & file name, line number
    FILE *fp = fopen(fname.c_str(), "wt");
    if( !fp )
    {
        fprintf(stderr, "Can not open log file for write: %s\n", fname.c_str());
        return;
    }

    for(auto it : arrCaller)
    {
        fprintf(fp, ">>> caller address: 0x%016zx, caller count: %6zd, totalMem: %8zd\n",
                it.mallocCaller[0], it.callerCount, it.totalMemSize);

        for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++)
            fprintf(fp, "%s", mapAddrFuncs[it.mallocCaller[i]].c_str());
        fprintf(fp, "\n\n");
    }

    fclose(fp);
}

MemoryMallocAnalysis::MapAddr2Line MemoryMallocAnalysis::getAddressMap(std::unordered_map<void*, MemoryUsageItem>& memStastic)
{
    // collect all caller address
    std::set<size_t> callAddrSet;
    for(auto it : memStastic)
    {
        size_t* addr = it.second.mallocCaller;

        for(int i=0; i<MEMORYUSAGE_TRACEDEPTH; i++)
        {
            size_t addi = addr[i];

            auto it2 = callAddrSet.find(addi);
            if(it2 == callAddrSet.end())
            {
                callAddrSet.insert(addi);
            }
        }
    }

    std::vector<size_t> callAddrArray;
    callAddrArray.resize(callAddrSet.size());
    int i = 0;
    for(auto it : callAddrSet)
    {
        callAddrArray[i++] = it;
    }

    // convert address to function/file name/line number
    int naddr = callAddrArray.size();
    double naddr_perThead = naddr * 1.0 / addr2line_threads;

    std::vector<std::thread> arrThreads;
    arrMapAddr2Line.clear();
    arrMapAddr2Line.resize(addr2line_threads);

    for(int j=0; j<addr2line_threads; j++)
    {
        int i1 = round(j*naddr_perThead);
        int i2 = round((j+1)*naddr_perThead);

        std::vector<size_t> subCallArray(callAddrArray.begin()+i1, callAddrArray.begin()+i2);

        std::thread tid(&MemoryMallocAnalysis::convertAddr2Funcs, this, subCallArray, j);
        arrThreads.push_back(std::move(tid));
    }

    for(std::thread &tid : arrThreads)
    {
        if( tid.joinable() ) tid.join();
    }

    std::unordered_map<size_t, std::string> addrFuncs;
    for(auto ri : arrMapAddr2Line)
    {
        for(auto it : ri) addrFuncs.insert(it);
    }

    return addrFuncs;
}

void MemoryMallocAnalysis::convertAddr2Funcs(std::vector<size_t> arrAddr, int tid)
{
    char cmd[256];
    char addr[256];

    MapAddr2Line &mapAddrFuncs = arrMapAddr2Line.at(tid);

    for(auto it : arrAddr)
    {
        size_t addi = it;
        GSLAM::utils::StringArray sa;

        sprintf(cmd, "addr2line -e %s -C -f 0x%016zx", programExe.c_str(), addi);
        std::string addrline = utils::exec_cmd(cmd);

        sprintf(addr, " [0x%016zx]", addi);

        if( addrline.size() < 2 )
        {
            sa.resize(2);
            sa[0] = std::string("?? ") + addr;
            sa[1] = "    ??:0\n";
        }
        else
        {
            if( addrline[0] == '?' && addrline[1] == '?' )
            {
                void* addrlist[1] = {(void*) addi};
                char** symbollist = backtrace_symbols( addrlist, 1 );

                sa.resize(2);
                sa[0] = parseBacktraceSymbols(symbollist[0]);
                sa[1] = "    ??:0\n";

                free(symbollist);
            }
            else
            {
                sa = utils::split_text(addrline, "\n\r");
                if( sa.size() > 0 ) sa[0] = sa[0] + addr;
                if( sa.size() > 1 ) sa[1] = "    " + sa[1];
            }
        }

        std::string res = utils::join_text(sa, "\n");
        mapAddrFuncs.insert(std::make_pair(addi, res));
    }
}

std::string MemoryMallocAnalysis::parseBacktraceSymbols(const std::string& symline)
{
    using namespace GSLAM::utils;

    std::string n_ld, n_fun, n_off, n_addr;

    utils::StringArray sa1 = utils::split_text(symline, "(");
    if( sa1.size() > 1 )
    {
        n_ld = utils::trim(sa1[0]);
        std::string tem = utils::trim(sa1[1]);

        utils::StringArray sa2 = utils::split_text(tem, ")");
        if( sa2.size() > 1 )
        {
            std::string tem2 = utils::trim(sa2[0]);
            n_addr = utils::trim(sa2[1]);

            utils::StringArray sa3 = utils::split_text(tem2, "+");
            if( sa3.size() > 1 )
            {
                n_fun = utils::trim(sa3[0]);
                n_off = utils::trim(sa3[1]);

                size_t  funcnamesize = 1024;
                char    funcname[1024];
                int     status = 0;

                char* ret = abi::__cxa_demangle( n_fun.c_str(),
                                                 funcname, &funcnamesize,
                                                 &status );
                char* fname = (char*) n_fun.c_str();
                if ( status == 0 ) fname = ret;

                std::string res = n_ld + " (" + fname + " + " + n_off + " ) " + n_addr;
                return res;
            }
        }
    }

    return "??:0";
}

size_t MemoryMallocAnalysis::addressHash(size_t *addr, int len)
{
    size_t h = 0;
    int shift_max = sizeof(size_t)*8;

    for(int i=0; i<len; i++)
        h ^= addr[i] << (i*16 % shift_max);

    return h;
}

} // end of namespace GSLAM


#endif // end of Linux/Unix
