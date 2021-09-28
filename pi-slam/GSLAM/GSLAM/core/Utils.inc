
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <bitset>
#include <array>
#include <chrono>
#include <algorithm>


#if _WIN32 // WIN
    #if _MSC_VER >=1400
        #include <intrin.h>
    #endif

    #include <io.h>
    #include <Winsock2.h>
    #include <Iptypes.h>
    #include <wincrypt.h>
    #include <iphlpapi.h>
    #include <windows.h>

#else
    #include <linux/hdreg.h>
    #include <sys/fcntl.h>
    #include <sys/ioctl.h>
    #include <sys/stat.h>
    #include <net/if.h>
    #include <unistd.h>
    #include <netinet/in.h>
    #include <dirent.h>
    #include <pwd.h>
#endif

#include "Utils.h"

using namespace std;

namespace GSLAM {
namespace utils {


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline int char_is_delims(char c, char d)
{
    if( c == d )
        return 1;
    else
        return 0;
}

StringArray split_text(const std::string &intext, const std::string &delims)
{
    StringArray         r;

    int                 st;
    int                 n, nd, i, j, k, dd;
    char                *buf;
    const char          *pb, *pd;


    n = intext.size();
    nd = delims.size();

    pb = intext.c_str();
    pd = delims.c_str();

    buf = new char[n+10];

    st = 0;
    i = 0;
    k = 0;
    buf[0] = 0;

    while( i<n ) {
        for(dd = 0, j=0; j<nd; j++) dd += char_is_delims(pb[i], pd[j]);

        if( dd > 0 ) {
            buf[k] = 0;
            r.push_back(buf);

            k = 0;
            st = 1;
        } else {
            buf[k++] = pb[i];
            st = 0;
        }

        i++;
    }

    // process last character
    if( st == 0 ) {
        buf[k] = 0;
        r.push_back(buf);
    } else {
        buf[0] = 0;
        r.push_back(buf);
    }

    delete [] buf;

    return r;
}

StringArray split_text_noempty(const std::string &intext, const std::string &delims)
{
    StringArray         r;

    int                 st;
    int                 n, nd, i, j, k, dd;
    char                *buf;
    const char          *pb, *pd;


    n = intext.size();
    nd = delims.size();

    pb = intext.c_str();
    pd = delims.c_str();

    buf = new char[n+10];

    st = 0;
    i = 0;
    k = 0;
    buf[0] = 0;

    while( i<n ) {
        for(dd = 0, j=0; j<nd; j++) dd += char_is_delims(pb[i], pd[j]);

        if( dd > 0 ) {
            buf[k] = 0;
            if( strlen(buf) > 0 ) r.push_back(buf);

            k = 0;
            st = 1;
        } else {
            buf[k++] = pb[i];
            st = 0;
        }

        i++;
    }

    // process last character
    if( st == 0 ) {
        buf[k] = 0;
        r.push_back(buf);
    } else {
        //buf[0] = 0;
        //r.push_back(buf);
    }

    delete [] buf;

    return r;
}

std::string join_text(const StringArray& sa, const std::string& delims)
{
    std::string s;

    for(int i=0; i<sa.size(); i++) {
        if( i == 0 ) s = sa[i];
        else         s = s + delims + sa[i];
    }

    return s;
}


std::string& str_toupper(std::string &s)
{
    for(size_t i=0; i < s.size(); i++) {
        s[i] = toupper(s[i]);
    }

    return s;
}

std::string& str_tolower(std::string &s)
{
    for(size_t i=0; i < s.size(); i++) {
        s[i] = tolower(s[i]);
    }

    return s;
}


int str_to_int(const string &s)
{
    return atoi(s.c_str());
}


std::string int_to_str(const int &i)
{
    ostringstream ost;
    ost<<i;
    return ost.str();
}




std::string trim(const std::string &s)
{
    std::string              delims = " \t\n\r",
                             r;
    std::string::size_type   i, j;

    i = s.find_first_not_of(delims);
    j = s.find_last_not_of(delims);

    if( i == std::string::npos ) {
        r = "";
        return r;
    }

    if( j == std::string::npos ) {
        r = "";
        return r;
    }

    r = s.substr(i, j-i+1);
    return r;
}



std::string& trim2(std::string& s)
{
    if (s.empty()) {
        return s;
    }

    std::string& r = s;
    std::string::iterator c;

    // Erase whitespace before the string
    for (c = r.begin(); c != r.end() && isspace(*c++););
    r.erase(r.begin(), --c);

    // Erase whitespace after the string
    for (c = r.end(); c != r.begin() && isspace(*--c););
    r.erase(++c, r.end());

    return r;
}


std::string read_textFile(const std::string& fname)
{
    std::ifstream ifs;
    std::string txt;

    ifs.open(fname);
    if( !ifs.is_open() ) return "";

    if( 0 ) {
        char *p;

        int nlen;
        ifs.seekg(0, std::ios_base::end);
        nlen = ifs.tellg();
        ifs.seekg(0);

        txt.resize(nlen+1);
        p = (char*) txt.data();
        ifs.read(p, nlen);
        p[nlen] = 0;
        ifs.close();
    } else {
        ifs >> txt;
        ifs.close();
    }

    return txt;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::string path_getFileName(const std::string& fname)
{
    size_t found;
    found = fname.find_last_of("/\\");
    return fname.substr(found+1);
}

std::string path_getPathName(const std::string& fname)
{
    size_t found;
    found = fname.find_last_of("/\\");
    return fname.substr(0,found);
}

std::string path_getFileBase(const std::string& fname)
{
    size_t found = fname.find_last_of(".");
    if( found != std::string::npos ) {
        if( found == 0 ) return "";
        else return fname.substr(0, found);
    } else {
        return fname;
    }
}

std::string path_getFileExt(const std::string& fname)
{
    size_t found = fname.find_last_of(".");
    if( found != std::string::npos ) {
        return fname.substr(found+1);
    } else {
        return "";
    }
}

#ifdef _WIN32

BOOL IsDots(const char* str)
{
    if(strcmp(str,".") && strcmp(str,"..")) return FALSE;
    return TRUE;
}

int path_lsdir(const std::string &dir_name, StringArray &dl, int sortFiles)
{
    // generate search pattern
    std::string dir;
    int ns = dir_name.size();
    if( ns > 1 && (dir_name[ns-1] == '\\' || dir_name[ns-1] == '/') )
    {
        dir = dir_name.substr(0, ns-1) + "\\*";
    }
    else
    {
        dir = dir_name + "\\*";
    }

    dl.clear();

    // find files
    HANDLE hFind;  // file handle
    WIN32_FIND_DATA data;

    if ( (hFind = FindFirstFileA(dir.c_str(), &data)) != INVALID_HANDLE_VALUE)
    {
        do {
            if( FALSE == IsDots(data.cFileName) )
                dl.push_back(data.cFileName);
        } while (FindNextFileA(hFind, &data) != 0);

        FindClose(hFind);
    }

    // sort all file name
    if( sortFiles ) std::sort(dl.begin(), dl.end());

    return 0;
}

BOOL DeleteDirectory(const char* sPath)
{
    HANDLE hFind;  // file handle
    WIN32_FIND_DATA FindFileData;

    char DirPath[MAX_PATH];
    char FileName[MAX_PATH];

    strcpy(DirPath,sPath);
    strcat(DirPath,"\\*");    // searching all files
    strcpy(FileName,sPath);
    strcat(FileName,"\\");

    hFind = FindFirstFileA(DirPath,&FindFileData); // find the first file
    if(hFind == INVALID_HANDLE_VALUE) return FALSE;
    strcpy(DirPath,FileName);

    bool bSearch = true;
    while(bSearch) { // until we finds an entry
        if(FindNextFileA(hFind,&FindFileData)) {
            if(IsDots(FindFileData.cFileName)) continue;
            strcat(FileName,FindFileData.cFileName);
            if((FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {

                // we have found a directory, recurse
                if(!DeleteDirectory(FileName)) {
                    FindClose(hFind);
                    return FALSE; // directory couldn't be deleted
                }
                RemoveDirectoryA(FileName); // remove the empty directory
                strcpy(FileName,DirPath);
            }
            else {
                if(FindFileData.dwFileAttributes & FILE_ATTRIBUTE_READONLY)
                    _chmod(FileName, _S_IWRITE); // change read-only file mode
                if(!DeleteFileA(FileName)) {  // delete the file
                    FindClose(hFind);
                    return FALSE;
                }
                strcpy(FileName,DirPath);
            }
        }
        else {
            if(GetLastError() == ERROR_NO_MORE_FILES) // no more files there
            bSearch = false;
            else {
                // some error occured, close the handle and return FALSE
                FindClose(hFind);
                return FALSE;
            }
        }
    }
    FindClose(hFind);  // closing file handle

    return RemoveDirectoryA(sPath); // remove the empty directory
}

int path_rmdir(const std::string &path)
{
    if( TRUE == DeleteDirectory(path.c_str()) )
        return 0;
    else
        return -1;
}

int path_rmfile(const std::string& path)
{
    // remove a file object
    if ( !DeleteFileA(path.c_str()) ) {
        fprintf(stderr, "Can`t remove a file: %s\n", path.c_str());
        return -1;
    }

    return 0;
}

#else

int path_lsdir(const std::string &dir_name, StringArray &dl, int sortFiles)
{
    DIR             *dir;
    struct dirent   *dp;

    // open directory
    dir = opendir(dir_name.c_str());
    if( dir == NULL ) {
        fprintf(stderr, "Failed to open dir: %s\n", dir_name.c_str());
        return -1;
    }

    // get each items
    dl.clear();
    for(dp=readdir(dir); dp!=NULL; dp=readdir(dir)) {
        // skip .
        if( strlen(dp->d_name) == 1 && dp->d_name[0] == '.' )
            continue;

        // skip ..
        if( strlen(dp->d_name) == 2 && dp->d_name[0] == '.' && dp->d_name[1] == '.' )
            continue;

        // add to list
        dl.push_back(dp->d_name);
    }

    closedir(dir);

    // sort all file name
    if( sortFiles ) std::sort(dl.begin(), dl.end());

    return 0;
}

int path_rmdir(const std::string& path)
{
    size_t path_len;
    char *full_path;
    DIR *dir;
    struct stat stat_path, stat_entry;
    struct dirent *entry;

    // stat for the path
    stat(path.c_str(), &stat_path);

    // if path does not exists or is not dir - exit with status -1
    if (S_ISDIR(stat_path.st_mode) == 0) {
        fprintf(stderr, "ERR: not a directory: %s\n", path.c_str());
        return -1;
    }

    // if not possible to read the directory for this user
    if ((dir = opendir(path.c_str())) == NULL) {
        fprintf(stderr, "ERR: Can`t open directory: %s\n", path.c_str());
        return -2;
    }

    // the length of the path
    path_len = path.size();

    // iteration through entries in the directory
    while ((entry = readdir(dir)) != NULL) {

        // skip entries "." and ".."
        if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, ".."))
            continue;

        // determinate a full path of an entry
        full_path = (char*) calloc(path_len + strlen(entry->d_name) + 1, sizeof(char));
        strcpy(full_path, path.c_str());
        strcat(full_path, "/");
        strcat(full_path, entry->d_name);

        // stat for the entry
        stat(full_path, &stat_entry);

        // recursively remove a nested directory
        if (S_ISDIR(stat_entry.st_mode) != 0) {
            path_rmdir(full_path);
            continue;
        }

        // remove a file object
        if (unlink(full_path) != 0)
            fprintf(stderr, "Can`t remove a file: %s\n", full_path);
    }

    // remove the devastated directory and close the object of it
    if (rmdir(path.c_str()) != 0)
        fprintf(stderr,"ERR: Can`t remove a directory: %s\n", path.c_str());

    closedir(dir);

    return 0;
}

int path_rmfile(const std::string& path)
{
    // remove a file object
    if ( unlink(path.c_str()) != 0 ) {
        fprintf(stderr, "Can`t remove a file: %s\n", path.c_str());
        return -1;
    }

    return 0;
}


#endif

int path_rename(const std::string &pathOld, const std::string &pathNew)
{
    return rename(pathOld.c_str(), pathNew.c_str());
}




#ifdef _WIN32
std::string systemImpl()
{
    char buffer[MAX_PATH];
    DWORD n = GetSystemDirectoryA(buffer, sizeof(buffer));
    if (n > 0 && n < sizeof(buffer))
    {
        std::string result(buffer, n);
        if (result[n - 1] != '\\')
            result.append("\\");
        return result;
    }
    else throw "Cannot get system directory";
}

std::string getImpl(const std::string& name)
{
    DWORD len = GetEnvironmentVariableA(name.c_str(), 0, 0);
//    if (len == 0) throw NotFoundException(name);
    char* buffer = new char[len];
    GetEnvironmentVariableA(name.c_str(), buffer, len);
    std::string result(buffer);
    delete [] buffer;
    return result;
}


bool hasImpl(const std::string& name)
{
    DWORD len = GetEnvironmentVariableA(name.c_str(), 0, 0);
    return len > 0;
}
#endif

std::string homeImpl(int allUser=0)
{
    // FIXME:
    //  How to get all-user writeable folder to save global settings
    //

#if defined(linux) || defined(__linux) || defined(__linux__) || defined(__TOS_LINUX__)
    std::string path;
    struct passwd* pwd = getpwuid(getuid());
    if (pwd)
        path = pwd->pw_dir;
    else
    {
        pwd = getpwuid(geteuid());
        if (pwd)
            path = pwd->pw_dir;
    }
    std::string::size_type n = path.size();
    if (n > 0 && path[n - 1] != '/') path.append("/");
    return path;
#else
    // Home and all-user setting folder
    //  https://en.wikipedia.org/wiki/Environment_variable#Default_values
    //

    std::string userProfile = "USERPROFILE";
    if( allUser ) userProfile = "ALLUSERSPROFILE";

    std::string result;
    if ( hasImpl(userProfile) )
    {
        result = getImpl(userProfile);
    }
    else if (hasImpl("HOMEDRIVE") && hasImpl("HOMEPATH"))
    {
        result = getImpl("HOMEDRIVE");
        result.append(getImpl("HOMEPATH"));
    }
    else
    {
        result = systemImpl();
    }

    std::string::size_type n = result.size();
    if (n > 0 && result[n - 1] != '\\')
        result.append("\\");
    return result;
#endif
}

std::string path_getUserSettingsPath(void)
{
#if defined(linux) || defined(__linux) || defined(__linux__) || defined(__TOS_LINUX__)
    return homeImpl() + ".config";
#else
    return homeImpl() + "AppData\\Local";
#endif

}

std::string path_getAllUserSettingsPath(void)
{
#if defined(linux) || defined(__linux) || defined(__linux__) || defined(__TOS_LINUX__)
    return homeImpl() + ".config";
#else
    return homeImpl(1);
#endif

}


#ifdef _WIN32
bool pathExist(const std::string& path)
{
    if( path.size() == 0 ) return false;

    DWORD attr = GetFileAttributesA(path.c_str());
    if (attr == INVALID_FILE_ATTRIBUTES)
    {
        switch (GetLastError())
        {
        case ERROR_FILE_NOT_FOUND:
        case ERROR_PATH_NOT_FOUND:
        case ERROR_NOT_READY:
        case ERROR_INVALID_DRIVE:
            return false;
        default:
            //handleLastErrorImpl(_path);
            return false;
        }
    }

    return true;
}

int _mkdir(const std::string& p)
{
    if ( CreateDirectoryA(p.c_str(), 0) )
        return 1;
    else
        return 0;
}
#endif

int path_mkdir(const std::string& path)
{
#ifdef _WIN32
    if( path.size() == 0 ) return false;

        StringArray sa = split_text(path, "/\\");
        int n = sa.size();
        if( n < 1 ) return false;

        // try to create dir
        std::string p = sa[0];
        for(int i=1; i<n; i++) {
            p = p + "\\" + sa[i];
            if( pathExist(p) )
                continue;
            else {
                if( !_mkdir(p) ) {
                    return false;
                }
            }
        }

        return true;
#else
    char            cmds[2048];
    sprintf(cmds, "mkdir -p '%s'", path.c_str());
    return system(cmds)==0?0:-1;
#endif
}

std::string path_join(const std::string &p1, const std::string &p2)
{
#ifdef _WIN32
    char path_sep[] = "\\";
#else
    char path_sep[] = "/";
#endif

    string      p;
    int         l;

    p = p1;

    l = p.size();
    if( p[l-1] == '/' || p[l-1] == '\\' )
        p = p.substr(0, l-1);

    p = p + path_sep + p2;
    return p;
}

std::string path_join(const std::string &p1, const std::string &p2, const std::string &p3)
{
    std::string     p;

    p = path_join(p1, p2);
    return path_join(p, p3);
}


StringArray path_split(const std::string &fname)
{
    size_t      found = -1;
    StringArray r;

    r.clear();

    /* find / or \ */
    found = fname.find_last_of("/\\");

    if( found == std::string::npos ) {
        r.push_back("");
        r.push_back(fname);
        return r;
    }

    // folder
    r.push_back(fname.substr(0, found));
    // file
    r.push_back(fname.substr(found+1));

    return r;
}

StringArray path_splitext(const std::string &fname)
{
    size_t      found;
    StringArray r, ff;

    r.clear();

    // get folder & file name
    ff = path_split(fname);

    // find .
    found = ff[1].find_last_of(".");
    if( found == std::string::npos ) {
        r.push_back(path_join(ff[0],ff[1]));
        r.push_back("");
        return r;
    }

    // filename
    r.push_back(path_join(ff[0],ff[1].substr(0, found)));
    // extname
    r.push_back(ff[1].substr(found));

    return r;
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifndef _WIN32

/*
 *
*/
/**
 * @brief Execute a command and get the stdout strings
 *      ref: https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-output-of-command-within-c-using-posix
 *
 * @param cmd               - [in] command string
 *
 * @return
 *      stdout string
 *
 */
std::string exec_cmd(const char* cmd)
{
    char buffer[128];
    std::string result = "";

    FILE* pipe = popen(cmd, "r");
    if ( !pipe ) return "";

    try {
        while (fgets(buffer, sizeof buffer, pipe) != NULL) {
            result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        pipe = NULL;
    }

    if( pipe ) pclose(pipe);

    return result;
}

#else

std::string exe_cmd(const char* cmd)
{
    return "";
}

#endif


double timestamp()
{
    return std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


int64_t time_utc(struct tm *tm);

#ifdef _WIN32 // WIN

#include <windows.h>
#include <io.h>
#include <time.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>

uint64_t tm_get_millis(void)
{
    return GetTickCount();
}

uint64_t tm_get_ms(void)
{
    return GetTickCount();
}

uint64_t tm_get_us(void)
{
    FILETIME        t;
    uint64_t        t_ret;

    // get UTC time
    GetSystemTimeAsFileTime(&t);

    t_ret = 0;

    t_ret |= t.dwHighDateTime;
    t_ret <<= 32;
    t_ret |= t.dwLowDateTime;

    // convert 100 ns to [ms]
    return t_ret/10;
}

double tm_getTimeStamp(void)
{
    FILETIME        t;
    uint64_t        t_ret;
    double          ts;

    // get UTC time
    GetSystemTimeAsFileTime(&t);

    t_ret = 0;

    t_ret |= t.dwHighDateTime;
    t_ret <<= 32;
    t_ret |= t.dwLowDateTime;

    // convert 100 ns to second
    ts = 1.0 * t_ret / 1e7;

    return ts;
}


uint32_t tm_getTimeStampUnix(void)
{
    FILETIME        t;
    uint64_t        t_ret;
    uint32_t        ts;

    // get UTC time
    GetSystemTimeAsFileTime(&t);

    t_ret = 0;

    t_ret |= t.dwHighDateTime;
    t_ret <<= 32;
    t_ret |= t.dwLowDateTime;

    // convert 100 ns to second
    ts = t_ret / 10000000;

    return ts;
}

void tm_sleep(uint32_t t)
{
    Sleep(t);
}

void tm_sleep_us(uint64_t t)
{
    HANDLE timer;
    LARGE_INTEGER ft;

    // Convert to 100 nanosecond interval, negative value indicates relative time
    ft.QuadPart = -(10*t);

    timer = CreateWaitableTimer(NULL, TRUE, NULL);
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
    WaitForSingleObject(timer, INFINITE);
    CloseHandle(timer);
}

const char * strp_weekdays[] =
{ "sunday", "monday", "tuesday", "wednesday", "thursday", "friday", "saturday"};

const char * strp_monthnames[] =
{ "january", "february", "march", "april", "may", "june", "july", "august", "september", "october", "november", "december"};

bool strp_atoi(const char * & s, int & result, int low, int high, int offset)
{
    bool worked = false;
    char * end;
    unsigned long num = strtoul(s, & end, 10);
    if (num >= (unsigned long)low && num <= (unsigned long)high)
    {
        result = (int)(num + offset);
        s = end;
        worked = true;
    }
    return worked;
}

char * strptime(const char *s, const char *format, struct tm *tm)
{
    bool working = true;
    while (working && *format && *s)
    {
        switch (*format)
        {
        case '%':
        {
            ++format;
            switch (*format)
            {
            case 'a':
            case 'A': // weekday name
                tm->tm_wday = -1;
                working = false;
                for (size_t i = 0; i < 7; ++ i)
                {
                    size_t len = strlen(strp_weekdays[i]);
                    if (!strnicmp(strp_weekdays[i], s, len))
                    {
                        tm->tm_wday = i;
                        s += len;
                        working = true;
                        break;
                    }
                    else if (!strnicmp(strp_weekdays[i], s, 3))
                    {
                        tm->tm_wday = i;
                        s += 3;
                        working = true;
                        break;
                    }
                }
                break;
            case 'b':
            case 'B':
            case 'h': // month name
                tm->tm_mon = -1;
                working = false;
                for (size_t i = 0; i < 12; ++ i)
                {
                    size_t len = strlen(strp_monthnames[i]);
                    if (!strnicmp(strp_monthnames[i], s, len))
                    {
                        tm->tm_mon = i;
                        s += len;
                        working = true;
                        break;
                    }
                    else if (!strnicmp(strp_monthnames[i], s, 3))
                    {
                        tm->tm_mon = i;
                        s += 3;
                        working = true;
                        break;
                    }
                }
                break;
            case 'd':
            case 'e': // day of month number
                working = strp_atoi(s, tm->tm_mday, 1, 31, 0);
                break;
            case 'D': // %m/%d/%y
            {
                const char * s_save = s;
                working = strp_atoi(s, tm->tm_mon, 1, 12, -1);
                if (working && *s == '/')
                {
                    ++ s;
                    working = strp_atoi(s, tm->tm_mday, 1, 31, 0);
                    if (working && *s == '/')
                    {
                        ++ s;
                        working = strp_atoi(s, tm->tm_year, 0, 99, 0);
                        if (working && tm->tm_year < 69)
                            tm->tm_year += 100;
                    }
                }
                if (!working)
                    s = s_save;
            }
                break;
            case 'H': // hour
                working = strp_atoi(s, tm->tm_hour, 0, 23, 0);
                break;
            case 'I': // hour 12-hour clock
                working = strp_atoi(s, tm->tm_hour, 1, 12, 0);
                break;
            case 'j': // day number of year
                working = strp_atoi(s, tm->tm_yday, 1, 366, -1);
                break;
            case 'm': // month number
                working = strp_atoi(s, tm->tm_mon, 1, 12, -1);
                break;
            case 'M': // minute
                working = strp_atoi(s, tm->tm_min, 0, 59, 0);
                break;
            case 'n': // arbitrary whitespace
            case 't':
                while (isspace((int)*s))
                    ++s;
                break;
            case 'p': // am / pm
                if (!strnicmp(s, "am", 2))
                { // the hour will be 1 -> 12 maps to 12 am, 1 am .. 11 am, 12 noon 12 pm .. 11 pm
                    if (tm->tm_hour == 12) // 12 am == 00 hours
                        tm->tm_hour = 0;
                    s += 2;
                }
                else if (!strnicmp(s, "pm", 2))
                {
                    if (tm->tm_hour < 12) // 12 pm == 12 hours
                        tm->tm_hour += 12; // 1 pm -> 13 hours, 11 pm -> 23 hours
                    s += 2;
                }
                else
                    working = false;
                break;
            case 'r': // 12 hour clock %I:%M:%S %p
            {
                const char * s_save = s;
                working = strp_atoi(s, tm->tm_hour, 1, 12, 0);
                if (working && *s == ':')
                {
                    ++ s;
                    working = strp_atoi(s, tm->tm_min, 0, 59, 0);
                    if (working && *s == ':')
                    {
                        ++ s;
                        working = strp_atoi(s, tm->tm_sec, 0, 60, 0);
                        if (working && isspace((int)*s))
                        {
                            ++ s;
                            while (isspace((int)*s))
                                ++s;
                            if (!strnicmp(s, "am", 2))
                            { // the hour will be 1 -> 12 maps to 12 am, 1 am .. 11 am, 12 noon 12 pm .. 11 pm
                                if (tm->tm_hour == 12) // 12 am == 00 hours
                                    tm->tm_hour = 0;
                            }
                            else if (!strnicmp(s, "pm", 2))
                            {
                                if (tm->tm_hour < 12) // 12 pm == 12 hours
                                    tm->tm_hour += 12; // 1 pm -> 13 hours, 11 pm -> 23 hours
                            }
                            else
                                working = false;
                        }
                    }
                }
                if (!working)
                    s = s_save;
            }
                break;
            case 'R': // %H:%M
            {
                const char * s_save = s;
                working = strp_atoi(s, tm->tm_hour, 0, 23, 0);
                if (working && *s == ':')
                {
                    ++ s;
                    working = strp_atoi(s, tm->tm_min, 0, 59, 0);
                }
                if (!working)
                    s = s_save;
            }
                break;
            case 'S': // seconds
                working = strp_atoi(s, tm->tm_sec, 0, 60, 0);
                break;
            case 'T': // %H:%M:%S
            {
                const char * s_save = s;
                working = strp_atoi(s, tm->tm_hour, 0, 23, 0);
                if (working && *s == ':')
                {
                    ++ s;
                    working = strp_atoi(s, tm->tm_min, 0, 59, 0);
                    if (working && *s == ':')
                    {
                        ++ s;
                        working = strp_atoi(s, tm->tm_sec, 0, 60, 0);
                    }
                }
                if (!working)
                    s = s_save;
            }
                break;
            case 'w': // weekday number 0->6 sunday->saturday
                working = strp_atoi(s, tm->tm_wday, 0, 6, 0);
                break;
            case 'Y': // year
                working = strp_atoi(s, tm->tm_year, 1900, 65535, -1900);
                break;
            case 'y': // 2-digit year
                working = strp_atoi(s, tm->tm_year, 0, 99, 0);
                if (working && tm->tm_year < 69)
                    tm->tm_year += 100;
                break;
            case '%': // escaped
                if (*s != '%')
                    working = false;
                ++s;
                break;
            default:
                working = false;
            }
        }
            break;
        case ' ':
        case '\t':
        case '\r':
        case '\n':
        case '\f':
        case '\v':
            // zero or more whitespaces:
            while (isspace((int)*s))
                ++ s;
            break;
        default:
            // match character
            if (*s != *format)
                working = false;
            else
                ++s;
            break;
        }
        ++format;
    }
    return (working?(char *)s:0);
}

#else // UNIX

#define _XOPEN_SOURCE
#include <time.h>
#include <sys/time.h>
#include <sys/timeb.h>


uint64_t tm_get_millis(void)
{
    struct timeval  tm_val;
    uint64_t        v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);
    v = tm_val.tv_sec*1000 + tm_val.tv_usec/1000;

    return v;
}

uint64_t tm_get_ms(void)
{
    struct timeval  tm_val;
    uint64_t        v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);
    v = tm_val.tv_sec*1000 + tm_val.tv_usec/1000;

    return v;
}

uint64_t tm_get_us(void)
{
    struct timeval  tm_val;
    uint64_t        v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);
    v = tm_val.tv_sec*1000000 + tm_val.tv_usec;

    return v;
}

double tm_getTimeStamp(void)
{
    struct timeval  tm_val;
    double          v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);
    v = tm_val.tv_sec + 1.0*tm_val.tv_usec/1e6;

    return v;
}


uint32_t tm_getTimeStampUnix(void)
{
    struct timeval  tm_val;
    uint32_t        v;
    int             ret;

    ret = gettimeofday(&tm_val, NULL);
    v = tm_val.tv_sec;

    return v;
}

void tm_sleep(uint32_t t)
{
    struct timespec tp;

    tp.tv_sec  = t / 1000;
    tp.tv_nsec = ( t % 1000 ) * 1000000;

    while( nanosleep(&tp, &tp) );
}

void tm_sleep_us(uint64_t t)
{
    struct timespec tp;

    tp.tv_sec  = t / 1000000;
    tp.tv_nsec = ( t % 1000000 ) * 1000;

    while( nanosleep(&tp, &tp) );
}


#endif // end of _WIN32



int is_leap(int y) {
    y += 1900;
    return (y % 4) == 0 && ((y % 100) != 0 || (y % 400) == 0);
}

int64_t time_utc(struct tm *tm)
{
    static const unsigned ndays[2][12] = {
        {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
        {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
    };

    int64_t res = 0;
    int i;

    for (i = 70; i < tm->tm_year; ++i)
        res += is_leap(i) ? 366 : 365;

    for (i = 0; i < tm->tm_mon; ++i)
        res += ndays[is_leap(tm->tm_year)][i];

    res += tm->tm_mday - 1;
    res *= 24;

    res += tm->tm_hour;
    res *= 60;

    res += tm->tm_min;
    res *= 60;

    res += tm->tm_sec;
    return res;
}

double tm_getTimeStamp(const char* dtStr, const char* fmt)
{
    struct tm tsTime;

    strptime(dtStr, fmt, &tsTime);
    double ts = time_utc(&tsTime);

    return ts;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool get_cpu_id_by_asm(std::string & cpu_id)
{
//    cpu_id.clear();
    int a = 0;
    int d = 0;

#ifdef _MSC_VER
    std::array<int, 4> cpui;
    __cpuid(cpui.data(), 1);
    d=cpui[3];
    a=cpui[0];
#else
    asm volatile
    (
        "movl $0x01, %%eax; \n\t"
        "xorl %%edx, %%edx; \n\t"
        "cpuid; \n\t"
        "movl %%edx, %0; \n\t"
        "movl %%eax, %1; \n\t"
        : "=m"(d), "=m"(a)
    );
#endif
    if (0 == a && 0 == d)
    {
        return(false);
    }

    char cpu[32] = { 0 };
    snprintf(cpu, sizeof(cpu), "%08X%08X", d, a);
    std::string(cpu).swap(cpu_id);

    return(true);
}

int get_macAddress(std::vector<std::string>& macs)
{
#ifdef __linux
    struct ifreq ifr;
    struct ifconf ifc;
    char buf[2048];

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock == -1) {
        printf("socket error\n");
        return -1;
    }

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) {
        printf("ioctl error\n");
        return -1;
    }

    struct ifreq* it = ifc.ifc_req;
    const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));
    char szMac[64];
    int count = 0;
    for (; it != end; ++it) {
        strcpy(ifr.ifr_name, it->ifr_name);
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
            if (! (ifr.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
                if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
                    count ++ ;
                    unsigned char * ptr ;
                    ptr = (unsigned char  *)&ifr.ifr_ifru.ifru_hwaddr.sa_data[0];
                    snprintf(szMac, 64, "%02X%02X%02X%02X%02X%02X",
                             *ptr,*(ptr+1),*(ptr+2),*(ptr+3),*(ptr+4),*(ptr+5));
                    macs.push_back(szMac);
                }
            }
        }else{
            printf("get mac info error\n");
            return -1;
        }
    }

#else
    IP_ADAPTER_INFO AdapterInfo[16];
    DWORD dwBuflen = sizeof(AdapterInfo);

    DWORD dwStatus = GetAdaptersInfo(AdapterInfo, &dwBuflen);
    if( dwStatus == NO_ERROR ) {
        PIP_ADAPTER_INFO pAdapterInfo = AdapterInfo;

        do {
            BYTE* MACData=pAdapterInfo->Address;
            char szMac[64];
            sprintf(szMac,"%02X%02X%02X%02X%02X%02X",
                    MACData[0], MACData[1],MACData[2],MACData[3],MACData[4],MACData[5]);
            macs.push_back(szMac);
            pAdapterInfo = pAdapterInfo->Next;
        } while(pAdapterInfo);
    }
#endif

    // if do not find an adaptor, then insert a default MAC address "FFFFFFFFFFFF"
    if( macs.size() < 1 ) {
        macs.push_back("FFFFFFFFFFFF");
    }

    return -1;
}

/**
 * @brief Get disk UUID
 *
 * @param features      - [out] uuid strings
 *
 * @return
 *      0               - Success
 *      other           - Failed
 *
 * Note:
 *  use `lsblk -f` is better
 *
 */
int get_diskNumber(std::vector<std::string>& features)
{
#ifdef __linux

    std::string res;
    std::string diskRoot = "";

    // get "/" disk
    res = exec_cmd("df");
    StringArray lns = split_text_noempty(res, "\n");
    for(int i=0; i<lns.size(); i++) {
        std::string l = lns[i];
        StringArray sa = split_text_noempty(trim(l), " ");
        if( sa.size() > 5 ) {
            if( trim(sa[5]) == "/" ) diskRoot = trim(sa[0]);
        }
    }

    // get uuid of the disk
    char buf[1024];
    sprintf(buf, "lsblk %s -o UUID", diskRoot.c_str());
    res = exec_cmd(buf);
    lns = split_text_noempty(res, "\n");
    if( lns.size() > 1 && trim(lns[0]) == "UUID" ) {
        string uuid = trim(lns[1]);
        features.push_back(uuid);
    }


    // get disk uuid
    // FIXME: how to get `/` disk uuid
    if( 0 )
    {
        std::vector<std::string> dUUIDs;
        path_lsdir("/dev/disk/by-uuid", dUUIDs, 0);
        for(std::string dUUID:dUUIDs)
        {
            features.push_back(dUUID);
        }
    }

    return 0;
#endif


#ifdef _WIN32
    char id[128];
    char Name[MAX_PATH];
    DWORD serno;
    DWORD length;
    DWORD FileFlag;
    char FileName[MAX_PATH];
    BOOL Ret;
    Ret = GetVolumeInformation("c:\\", Name, MAX_PATH, &serno, &length, &FileFlag, FileName, MAX_PATH);
    if (Ret)
    {
        sprintf(id,"%x",serno);
        features.push_back(id);
        return 0;
    }
    return -1;
#endif
}

int get_computerFeatures(std::vector<std::string>& features)
{

    //cpuid, mac address, disk id
    std::string cpuid;
    if(get_cpu_id_by_asm(cpuid))
        features.push_back(cpuid);

    get_macAddress(features);
    get_diskNumber(features);

    return features.size()?0:-1;
}


int net_gethostname(std::string& hn)
{
#ifdef _WIN32
    WORD versionWanted = MAKEWORD(1, 1);
    WSADATA wsaData;
    WSAStartup(versionWanted, &wsaData);
#endif

    char hostname[1024];
    int ret = gethostname(hostname, 1024);
    hn = trim(hostname);
    if( hn.size() < 1 ) hn = "DEFAULTCOMPUTER";
    for(int i=0; i<hn.size(); i++) if( hn[i] == '|' ) hn[i] = '_';

    return ret;
}


} } // end of namespace GSLAM::utils


